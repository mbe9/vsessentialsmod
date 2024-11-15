using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Config;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Datastructures;
using System.Runtime.CompilerServices;

namespace Vintagestory.GameContent
{
    using AngleConstraint = SoftBody.AngleConstraint;
    using DistanceConstraint = SoftBody.DistanceConstraint;
    using LengthConstraint = SoftBody.LengthConstraint;
    using LineSegemntCollisionConstraint = SoftBody.LineSegemntCollisionConstraint;
    using ConstraintComparison = SoftBody.ConstraintComparison;

    [ProtoContract]
    public enum EnumClothType
    {
        Rope,
        Cloth
    }

    [ProtoContract]
    public class ClothPointPacketXAxis
    {
        public struct Offset
        {
            public ushort PointOffset;
            public byte PointIdx;
        }

        public double AnchorPosX;
        public double OffsetMult;

        public Offset[] pointOffsets;

        public ushort FrameNum;
    }

    public class TimeStepData
    {
        public double StepTime;
        public double InvStepTime;
        public double InvStepTimeSqr;

        public double SubstepTime;
        public double InvSubstepTime;
        public double InvSubstepTimeSqr;

        public TimeStepData(double tickTime, int substepCount)
        {
            this.StepTime = tickTime;
            this.InvStepTime = 1.0 / tickTime;
            this.InvStepTimeSqr = 1.0 / (tickTime * tickTime);

            this.SubstepTime = tickTime / substepCount;
            this.InvSubstepTime = 1.0 / (tickTime * substepCount);
            this.InvSubstepTimeSqr = 1.0 / (tickTime * tickTime * substepCount * substepCount);
        }
    }


    // How to synchronize this over the network?
    // Idea: We run the simulation client and server, but server 
    [ProtoContract]
    public class ClothSystem
    {
        public const double DistMult = 0.001;
        const int MinSegmentCount = 2;
        const int MaxSegmentCount = 64;
        const ushort RopeSegmentLength = 500;
        const ushort MinRopeLength = RopeSegmentLength * MinSegmentCount;
        const ushort MaxRopeLength = RopeSegmentLength * MaxSegmentCount;

        public const int SubstepCount = 100;

        const ushort MinBufferSize = MinSegmentCount + 1;
        const ushort MaxBufferSize = MaxSegmentCount + 1;

        ushort RopeLength = 0;
        float RopeWidth = 0.1f;

        [ProtoMember(5)]
        float DistanceCompliance = 10000;
        float DistanceDamping = 0;

        float AngleCompliance = 10000;
        float AngleDamping = 0;
        float AngleTarget = 0;

        float LengthCompliance = 1000;
        float LengthDamping = 0;

        ushort bufferHead = 0;
        ushort bufferSize = 0;

        ushort bufferFirst => bufferHead;

        ushort bufferLastPoint => (ushort)((bufferHead + bufferSize - 1) % MaxBufferSize);
        ushort bufferLastDist => (ushort)((bufferHead + bufferSize - 2) % MaxBufferSize);
        ushort bufferLastAngle => (ushort)((bufferHead + bufferSize - 3) % MaxBufferSize);

        ushort bufferEmptyRightPoint => (ushort)((bufferHead + bufferSize) % MaxBufferSize);
        ushort bufferEmptyRightDist => (ushort)((bufferHead + bufferSize - 1) % MaxBufferSize);
        ushort bufferEmptyRightAngle => (ushort)((bufferHead + bufferSize - 2) % MaxBufferSize);
        ushort bufferEmptyLeft => (ushort)((bufferHead + MaxBufferSize - 1) % MaxBufferSize);

        [ProtoMember(1)]
        public int ClothId;
        [ProtoMember(2)]
        EnumClothType clothType;

        ClothPoint[] Points = new ClothPoint[MaxBufferSize];
        public ClothPointData[] PointsData = new ClothPointData[MaxBufferSize];

        DistanceConstraint[] DistanceConstraints = new DistanceConstraint[MaxBufferSize];
        ushort[] SegmentDistances = new ushort[MaxBufferSize];

        AngleConstraint[] AngleConstraints = new AngleConstraint[MaxBufferSize];
        FastVec3f[] Forces = new FastVec3f[MaxBufferSize];

        List<LineSegemntCollisionConstraint> CollisionConstraints = new ();

        [ProtoMember(3)]
        public ClothPinStorage PinStorage = new();
        [ProtoMember(4)]
        public bool Active { get; set; }

        /// <summary>
        /// 10 joints per meter
        /// </summary>
        public static float Resolution = 2;

        public float ForceWarn = 1000.0f;
        public float ForceRip = 2000.0f;

        public bool LineDebug=false;
        protected ICoreClientAPI capi;
        public ICoreAPI api;        
        public Vec3d windSpeed = new Vec3d();
        public IBlockAccessor BlockAccess;
        protected TimeStepData timeStepData;
        protected NormalizedSimplexNoise noiseGen;
        protected float[] tmpMat = new float[16];
        protected Vec3f distToCam = new Vec3f();
        protected AssetLocation ropeSectionModel;
        protected MeshData debugUpdateMesh;
        protected MeshRef debugMeshRef;

        public float secondsOverStretched;


        public bool PinnedAnywhere => PinStorage.Count > 0;

        // FIXME
        public double MaxExtension => 0;//Constraints.Count == 0 ? 0 : Constraints.Max(c => c.Extension);

        public (float, ClothPoint) MaxForce
        {
            get
            {
                float maxForceSqr = 0;
                ClothPoint maxForcePoint = null;

                for (int i = 0; i < bufferSize; i++)
                {
                    int idx = (bufferHead + i) % MaxBufferSize;

                    ref var force = ref Forces[idx];
                    ref var point = ref Points[idx];

                    float forceSqr = force.LengthSq();

                    if (forceSqr > maxForceSqr)
                    {
                        maxForceSqr = forceSqr;
                        maxForcePoint = point;
                    }
                }

                return ((float)Math.Sqrt(maxForceSqr), maxForcePoint);
            }
        }

        public Vec3d CenterPosition
        {
            get
            {
                if (bufferSize == 0) return Vec3d.Zero;

                Vec3d pos = new Vec3d();

                double mult = 1.0 / (double)bufferSize;

                for (int i = 0; i < bufferSize; i++)
                {
                    int idx = (bufferHead + i) % MaxBufferSize;

                    ref var point = ref PointsData[idx];

                    pos.Add(point.Pos.X * mult, point.Pos.Y * mult, point.Pos.Z * mult);
                }

                return pos;
            }
        }

        public int SegmentCount => ((bufferSize - 1) >= 0) ? (bufferSize - 1) : 0;

        public ClothPoint FirstPoint => Points[bufferFirst];
        public ClothPoint LastPoint => Points[bufferLastPoint];

        public ClothPoint[] Ends => new ClothPoint[] { FirstPoint, LastPoint };

        // public int Width => Points2d.Count;
        // public int Length => Points2d[0].Points.Count;


        public static ClothSystem CreateCloth(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end)
        {
            return new ClothSystem(api, cm, start, end, EnumClothType.Cloth);
        }

        public static ClothSystem CreateRope(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end, AssetLocation clothSectionModel)
        {
            return new ClothSystem(api, cm, start, end, EnumClothType.Rope, clothSectionModel);
        }


        private ClothSystem() { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort nextIdx(ushort idx, ushort offset)
        {
            return (ushort)((idx + offset) % MaxBufferSize);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort prevIdx(ushort idx, ushort offset)
        {
            return (ushort)((idx + MaxBufferSize - offset) % MaxBufferSize);
        }

        void RecalculateRopeLength()
        {
            RopeLength = 0;

            for (int i = 0; i < bufferSize - 1; i++)
            {
                int index = (bufferHead + i) % MaxBufferSize;

                RopeLength += DistanceConstraints[index].TargetDistance;
            }
        }

        public ushort IncreaseRopeLength(ClothPoint point, ushort changeLength)
        {
            // Limit change value by max rope length
            if ((MaxRopeLength < changeLength) || (MaxRopeLength - changeLength < RopeLength))
            {
                changeLength = MaxRopeLength - RopeLength;
            }

            ushort remainingLength = changeLength;

            // Changing length at the start of the rope
            if (point.InternalIndex == bufferHead)
            {
                {
                    ushort addLength = changeLength;

                    if (RopeSegmentLength - SegmentDistances[bufferFirst] < addLength)
                    {
                        addLength = RopeSegmentLength - SegmentDistances[bufferFirst];
                    }

                    SegmentDistances[bufferFirst] += addLength;
                    remainingLength -= addLength;
                }

                while (remaingingLength > 0)
                {
                    if (bufferSize == MaxBufferSize)
                    {
                        break;
                    }

                    bufferHead = bufferEmptyLeft;
                    bufferSize += 1;

                    ushort addLength = Math.Min(remainingLength, RopeSegmentLength);

                    SegmentDistances[bufferHead] = addLength;
                    remainingLength -= addLength;

                    DistanceConstraints[bufferHead].PointIndex1 = bufferHead;
                    DistanceConstraints[bufferHead].PointIndex2 = nextIdx(bufferHead, 1);
                    DistanceConstraints[bufferHead].Comparison = ConstraintComparison.Equal;

                    if (bufferSize >= 3)
                    {
                        AngleConstraints[bufferHead] = new AngleConstraint {
                            TargetAngleCos = AngleTarget,
                            PointIndex1 = bufferHead,
                            PointIndex2 = nextIdx(bufferHead, 1),
                            PointIndex3 = nextIdx(bufferHead, 2),
                            Comparison = ConstraintComparison.Equal,
                        };
                    }

                    ushort secondIdx = nextIdx(bufferHead, 1);

                    Points[bufferHead] = Points[secondIdx];
                    Points[bufferHead].InternalIndex = bufferHead;
                    Points[secondIdx] = new ClothPoint(this, secondIdx);

                    PointsData[bufferHead] = PointsData[secondIdx];
                    PointsData[secondIdx] = new ClothPointData {
                        Pos = PointsData[bufferHead].Pos,
                        PrevPos = PointsData[bufferHead].PrevPos,
                        InvMass = PointsData[bufferHead].InvMass,
                    };
                }
            }
            // Changing length at the end of the rope
            else if (point.InternalIndex == bufferLastPoint)
            {
                {
                    ushort addLength = changeLength;

                    if (RopeSegmentLength - SegmentDistances[bufferLastDist] < addLength)
                    {
                        addLength = RopeSegmentLength - SegmentDistances[bufferLastDist];
                    }

                    SegmentDistances[bufferLastDist] += addLength;
                    remainingLength -= addLength;
                }

                while (remainingLength > 0)
                {
                    if (bufferSize == MaxBufferSize)
                    {
                        break;
                    }

                    bufferSize += 1;

                    ushort addLength = Math.Min(remainingLength, RopeSegmentLength);

                    SegmentDistances[bufferLastDist] = addLength;
                    remainingLength -= addLength;

                    DistanceConstraints[bufferLastDist].PointIndex1 = prevIdx(bufferLastPoint, 1);
                    DistanceConstraints[bufferLastDist].PointIndex2 = bufferLastPoint;
                    DistanceConstraints[bufferLastDist].Comparison = Comparison.Equals;

                    if (bufferSize >= 3)
                    {
                        AngleConstraints[bufferLastAngle] = new AngleConstraint {
                            TargetAngleCos = AngleTarget,
                            PointIndex1 = prevIdx(bufferLastPoint, 2),
                            PointIndex2 = prevIdx(bufferLastPoint, 1),
                            PointIndex3 = bufferLastPoint,
                            Comparison = ConstraintComparison.Equal,
                        };
                    }

                    ushort secondIdx = prevIdx(bufferLastPoint, 1);

                    Points[bufferLastPoint] = Points[secondIdx];
                    Points[bufferLastPoint].InternalIndex = bufferLastPoint;
                    Points[secondIdx] = new ClothPoint(this, secondIdx);

                    PointsData[bufferLastPoint] = PointsData[secondIdx];
                    PointsData[secondIdx] = new ClothPointData {
                        Pos = PointsData[bufferLastPoint].Pos;
                        PrevPos = PointsData[bufferLastPoint].PrevPos;
                        InvMass = PointsData[bufferLastPoint].InvMass;
                    };
                }
            }
            else
            {
                // Disallow changing length on any of the other points,
                // because it would insanely complicate both game mechanics and the code
                return 0;
            }

            ushort changeLength = changeLength - remainingLength;
            RopeLength += changeLength;

            genDebugMesh();

            return changeLength;
        }

        public ushort ReduceRopeLength(ClothPoint point, ushort changeLength)
        {
            // Limit change value by min rope length
            if (changeLength > RopeLength - MinRopeLength)
            {
                changeLength = RopeLength - MinRopeLength;
            }

            ushort remainingLength = changeLength;

            // Changing length at the start of the rope
            if (point.InternalIndex == bufferHead)
            {
                while (remainingLength > 0)
                {
                    if (SegmentDistances[bufferFirst] < remainingLength)
                    {
                        remainingLength -= SegmentDistances[bufferFirst];

                        if (bufferSize == MinBufferSize)
                        {
                            break;
                        }

                        bufferHead = nextIdx(bufferHead, 1);
                        bufferSize -= 1;

                        // Remove any pins on points we removing
                        Points[bufferHead].UnPin();

                        Points[bufferHead] = Points[prevIdx(bufferHead, 1)];
                        Points[bufferHead].InternalIndex = bufferHead;
                        Points[prevIdx(bufferHead, 1)] = null;
                    }
                    else
                    {
                        SegmentDistances[bufferFirst] -= removeLength;
                        remainingLength -= removeLength;
                        break;
                    }
                }
            }
            // Changing length at the end of the rope
            else if (point.InternalIndex == bufferLastPoint)
            {
                while (remainingLength > 0)
                {
                    if (SegmentDistances[bufferLastDist] < remainingLength)
                    {
                        remainingLength -= SegmentDistances[bufferLastDist];

                        if (bufferSize == MinBufferSize)
                        {
                            break;
                        }

                        bufferSize -= 1;

                        // Remove any pins on points we removing
                        Points[bufferLastPoint].UnPin();

                        Points[bufferLastPoint] = Points[nextIdx(bufferLastPoint, 1)];
                        Points[bufferLastPoint].InternalIndex = bufferHead;
                        Points[nextIdx(bufferLastPoint, 1)] = null;
                    }
                    else
                    {
                        SegmentDistances[bufferLastDist] -= removeLength;
                        remainingLength -= removeLength;
                        break;
                    }
                }
            }
            else
            {
                // Disallow changing length on any of the other points,
                // because it would insanely complicate both game mechanics and the code
                return 0;
            }

            ushort changeLength = changeLength - remainingLength;
            RopeLength -= changeLength;

            genDebugMesh();

            return changeLength;
        }

        private ClothSystem(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end, EnumClothType clothType, AssetLocation ropeSectionModel = null)
        {
            this.clothType = clothType;
            this.ropeSectionModel = ropeSectionModel;

            Init(api, cm);

            var delta = end - start;
            double length = delta.Length();

            if (length < MinRopeLength)
            {
                // TODO: this solution is kinda hardcoded and works only for MinSegmentCount = 2
                bufferHead = 0;
                bufferSize = 3;

                Points[bufferHead] = new ClothPoint(this, bufferHead);
                PointsData[bufferHead] = new ClothPointData(start, start, 1.0); // TODO mass

                var pos = start + (end - start) * 0.5;
                Points[nextIdx(bufferHead, 1)] = new ClothPoint(this, nextIdx(bufferHead, 1));
                PointsData[nextIdx(bufferHead, 1)] = new ClothPointData(pos, pos, 1.0); // TODO mass

                Points[nextIdx(bufferHead, 2)] = new ClothPoint(this, nextIdx(bufferHead, 2));
                PointsData[nextIdx(bufferHead, 2)] = new ClothPointData(end, end, 1.0); // TODO mass

                DistanceConstraints[bufferHead] = new DistanceConstraint {
                    TargetDistance = RopeSegmentLength,
                    Compliance = DistanceCompliance,
                    Damping = DistanceDamping,
                    PointIndex1 = bufferHead,
                    PointIndex2 = nextIdx(bufferHead, 1),
                    Comparison = ConstraintComparison.Equal,
                };

                DistanceConstraints[nextIdx(bufferHead, 1)] = new DistanceConstraint {
                    TargetDistance = RopeSegmentLength,
                    Compliance = DistanceCompliance,
                    Damping = DistanceDamping,
                    PointIndex1 = nextIdx(bufferHead, 1),
                    PointIndex2 = nextIdx(bufferHead, 2),
                    Comparison = ConstraintComparison.Equal,
                };

                AngleConstraints[bufferHead] = new AngleConstraint {
                    TargetAngleCos = AngleTarget,
                    Compliance = AngleCompliance,
                    Damping = AngleDamping,
                    PointIndex1 = bufferHead,
                    PointIndex2 = nextIdx(bufferHead, 1),
                    PointIndex3 = nextIdx(bufferHead, 2),
                    Comparison = ConstraintComparison.Equal,
                };

                RopeLength = MinRopeLength;
            }
            else
            {
                double remLength = length;

                bufferHead = 0;
                bufferSize = 0;

                while (remLength > RopeSegmentLength)
                {
                    double t = 1.0 - remLength / length;
                    var pos = start + (end - start) * t;

                    bufferSize += 1;

                    Points[bufferLastPoint] = new ClothPoint(this, bufferLastPoint);
                    PointsData[bufferLastPoint] = new ClothPointData(pos, pos, 1.0); // TODO mass

                    if (bufferSize >= 2)
                    {
                        DistanceConstraints[bufferLastDist] = new DistanceConstraint {
                            TargetDistance = RopeSegmentLength,
                            Compliance = DistanceCompliance,
                            Damping = DistanceDamping,
                            PointIndex1 = prevIdx(bufferLastPoint, 1),
                            PointIndex2 = bufferLastPoint,
                            Comparison = ConstraintComparison.Equal,
                        };
                    }

                    if (bufferSize >= 3)
                    {
                        AngleConstraints[bufferLastAngle] = new AngleConstraint {
                            TargetAngleCos = AngleTarget,
                            Compliance = AngleCompliance,
                            Damping = AngleDamping,
                            PointIndex1 = prevIdx(bufferLastPoint, 2),
                            PointIndex2 = prevIdx(bufferLastPoint, 1),
                            PointIndex3 = bufferLastPoint,
                            Comparison = ConstraintComparison.Equal,
                        };
                    }

                    remLength -= RopeSegmentLength;
                }

                bufferSize += 1;

                Points[bufferLastPoint] = new ClothPoint(this, bufferLastPoint);
                PointsData[bufferLastPoint] = new ClothPointData(end, end, 1.0); // TODO mass

                if (bufferSize >= 2)
                {
                    DistanceConstraints[bufferLastDist] = new DistanceConstraint {
                        TargetDistance = (float)remLength,
                        Compliance = DistanceCompliance,
                        Damping = DistanceDamping,
                        PointIndex1 = prevIdx(bufferLastPoint, 1),
                        PointIndex2 = bufferLastPoint,
                        Comparison = ConstraintComparison.Equal,
                    };
                }

                if (bufferSize >= 3)
                {
                    AngleConstraints[bufferLastAngle] = new AngleConstraint {
                        TargetAngleCos = AngleTarget,
                        Compliance = AngleCompliance,
                        Damping = AngleDamping,
                        PointIndex1 = prevIdx(bufferLastPoint, 2),
                        PointIndex2 = prevIdx(bufferLastPoint, 1),
                        PointIndex3 = bufferLastPoint,
                        Comparison = ConstraintComparison.Equal,
                    };
                }

                RopeLength = (float)length;
            }
        }

        public void genDebugMesh()
        {
            if (capi == null) return;

            debugMeshRef?.Dispose();
            debugUpdateMesh = new MeshData(20, 15, false, false, true, true);

            int vertexIndex = 0;

            for (int i = 0; i < bufferSize - 1; i++)
            {
                int idx1 = (bufferHead + i) % MaxBufferSize;
                int idx2 = (bufferHead + i + 1) % MaxBufferSize;

                ref var p1 = ref PointsData[idx1];
                ref var p2 = ref PointsData[idx2];

                int color = (i % 2) > 0 ? ColorUtil.WhiteArgb : ColorUtil.BlackArgb;

                // FIXME this probably should use point coords?
                debugUpdateMesh.AddVertexSkipTex(0, 0, 0, color);
                debugUpdateMesh.AddVertexSkipTex(0, 0, 0, color);

                debugUpdateMesh.AddIndex(vertexIndex++);
                debugUpdateMesh.AddIndex(vertexIndex++);
            }


            debugUpdateMesh.mode = EnumDrawMode.Lines;
            debugMeshRef = capi.Render.UploadMesh(debugUpdateMesh);

            debugUpdateMesh.Indices = null;
            debugUpdateMesh.Rgba = null;
        }

        public void Init(ICoreAPI api, ClothManager cm)
        {
            this.api = api;
            this.capi = api as ICoreClientAPI;
            this.BlockAccess = api.World.BlockAccessor;
            this.timeStepData = new TimeStepData(GlobalConstants.PhysicsFrameTime, SubstepCount);

            noiseGen = NormalizedSimplexNoise.FromDefaultOctaves(4, 100, 0.9, api.World.Seed + CenterPosition.GetHashCode());
        }

        public void WalkPoints(Action<ClothPoint> onPoint)
        {
            for (int i = 0; i < bufferSize; i++)
            {
                int index = (bufferHead + i) % MaxBufferSize;

                onPoint(Points[index]);
            }
        }


        public int UpdateMesh(MeshData updateMesh, float dt)
        {
            var cfloats = updateMesh.CustomFloats;
            Vec3d campos = capi.World.Player.Entity.CameraPos;
            int basep = cfloats.Count;

            for (int i = 0; i < bufferSize - 1; i++)
            {
                int idx1 = (bufferHead + i) % MaxBufferSize;
                int idx2 = (bufferHead + i + 1) % MaxBufferSize;

                ref FastVec3d p1 = ref PointsData[idx1].Pos;
                ref FastVec3d p2 = ref PointsData[idx2].Pos;

                double dX = p1.X - p2.X;
                double dY = p1.Y - p2.Y;
                double dZ = p1.Z - p2.Z;

                float yaw = (float)Math.Atan2(dX, dZ) + GameMath.PIHALF;
                float pitch = (float)Math.Atan2(Math.Sqrt(dZ * dZ + dX * dX), dY) + GameMath.PIHALF;

                double nowx = p1.X + (p1.X - p2.X) / 2;
                double nowy = p1.Y + (p1.Y - p2.Y) / 2;
                double nowz = p1.Z + (p1.Z - p2.Z) / 2;

                distToCam.Set(
                    (float)(nowx - campos.X),
                    (float)(nowy - campos.Y),
                    (float)(nowz - campos.Z)
                );

                Mat4f.Identity(tmpMat);

                Mat4f.Translate(tmpMat, tmpMat, 0, 1 / 32f, 0);

                Mat4f.Translate(tmpMat, tmpMat, distToCam.X, distToCam.Y, distToCam.Z);
                Mat4f.RotateY(tmpMat, tmpMat, yaw);
                Mat4f.RotateZ(tmpMat, tmpMat, pitch);

                float roll = i / 5f;
                Mat4f.RotateX(tmpMat, tmpMat, roll);

                float length = GameMath.Sqrt(dX*dX+dY*dY+dZ*dZ);

                Mat4f.Scale(tmpMat, tmpMat, new float[] { length, 1, 1 }); // + (float)Math.Sin(api.World.ElapsedMilliseconds / 1000f) * 0.1f
                Mat4f.Translate(tmpMat, tmpMat, -1.5f, -1 / 32f, -0.5f); // not sure why the -1.5 here instead of -0.5

                var midPoint = new Vec3d(
                    (p1.X + p2.X) / 2,
                    (p1.Y + p2.Y) / 2,
                    (p1.Z + p2.Z) / 2
                );

                Vec4f lightRgba = api.World.BlockAccessor.GetLightRGBs(midPoint.AsBlockPos);

                int j = basep + i * 20;
                cfloats.Values[j++] = lightRgba.R;
                cfloats.Values[j++] = lightRgba.G;
                cfloats.Values[j++] = lightRgba.B;
                cfloats.Values[j++] = lightRgba.A;

                for (int k = 0; k < 16; k++)
                {
                    cfloats.Values[j + k] = tmpMat[k];
                }
            }

            return SegmentCount;
        }


        /// <summary>
        /// Instantly update all cloth contraints center render pos. This is used to reduce jerkiness on new rope in its initial state
        /// </summary>
        public void setRenderCenterPos()
        {
            // for (int i = 0; i < Constraints.Count; i++)
            // {
            //     ClothConstraint cc = Constraints[i];

            //     Vec3d start = cc.Point1.Pos;
            //     Vec3d end = cc.Point2.Pos;

            //     double nowx = start.X + (start.X - end.X) / 2;
            //     double nowy = start.Y + (start.Y - end.Y) / 2;
            //     double nowz = start.Z + (start.Z - end.Z) / 2;

            //     cc.renderCenterPos.X = nowx;
            //     cc.renderCenterPos.Y = nowy;
            //     cc.renderCenterPos.Z = nowz;
            // }
        }


        Matrixf mat = new Matrixf();

        public void CustomRender(float dt)
        {
            if (LineDebug && capi != null)
            {
                if (debugMeshRef == null) genDebugMesh();

                BlockPos originPos = CenterPosition.AsBlockPos;

                for (int i = 0; i < bufferSize - 1; i++)
                {
                    int idx1 = (bufferHead + i) % MaxBufferSize;
                    int idx2 = (bufferHead + i + 1) % MaxBufferSize;

                    ref FastVec3d p1 = ref PointsData[idx1].Pos;
                    ref FastVec3d p2 = ref PointsData[idx2].Pos;

                    debugUpdateMesh.xyz[i * 6 + 0] = (float)(p1.X - originPos.X);
                    debugUpdateMesh.xyz[i * 6 + 1] = (float)(p1.Y - originPos.Y) + 0.005f;
                    debugUpdateMesh.xyz[i * 6 + 2] = (float)(p1.Z - originPos.Z);

                    debugUpdateMesh.xyz[i * 6 + 3] = (float)(p2.X - originPos.X);
                    debugUpdateMesh.xyz[i * 6 + 4] = (float)(p2.Y - originPos.Y) + 0.005f;
                    debugUpdateMesh.xyz[i * 6 + 5] = (float)(p2.Z - originPos.Z);
                }


                capi.Render.UpdateMesh(debugMeshRef, debugUpdateMesh);

                IShaderProgram prog = capi.Shader.GetProgram((int)EnumShaderProgram.Autocamera);
                prog.Use();

                capi.Render.LineWidth = 6;
                capi.Render.BindTexture2d(0);

                capi.Render.GLDisableDepthTest();

                Vec3d cameraPos = capi.World.Player.Entity.CameraPos;

                mat.Set(capi.Render.CameraMatrixOrigin);
                mat.Translate(
                    (float)(originPos.X - cameraPos.X),
                    (float)(originPos.Y - cameraPos.Y),
                    (float)(originPos.Z - cameraPos.Z)
                );

                prog.UniformMatrix("projectionMatrix", capi.Render.CurrentProjectionMatrix);
                prog.UniformMatrix("modelViewMatrix", mat.Values);

                capi.Render.RenderMesh(debugMeshRef);

                prog.Stop();


                capi.Render.GLEnableDepthTest();
            }

        }

        double accum_substep = 0f;
        int accum_step = 0;

        public void updateFixedStep(float dt)
        {
            accum_substep += dt;
            if (accum_substep > 1) accum_substep = 0.25f;

            while (accum_substep >= timeStepData.SubstepTime)
            {
                accum_substep -= timeStepData.SubstepTime;
                accum_step += 1;

                if (accum_step >= SubstepCount)
                {
                    substepNow(true, 1.0);
                    accum_step = 0;
                }
                else
                {
                    substepNow(false, (double)accum_step / SubstepCount);
                }
            }
        }

        void gatherCollisionConstraints()
        {
            CollisionConstraints.Clear();

            BlockPos minPos = new BlockPos();
            BlockPos maxPos = new BlockPos();

            for (int i = 0; i < bufferSize - 1; i++)
            {
                int idx1 = (bufferHead + i) % MaxBufferSize;
                int idx2 = (bufferHead + i + 1) % MaxBufferSize;

                ref FastVec3d p1 = ref PointsData[idx1].Pos;
                ref FastVec3d p2 = ref PointsData[idx2].Pos;

                double minX = Math.Min(p1.X, p2.X);
                double maxX = Math.Max(p1.X, p2.X);

                double minY = Math.Min(p1.Y, p2.Y);
                double maxY = Math.Max(p1.Y, p2.Y);

                double minZ = Math.Min(p1.Z, p2.Z);
                double maxZ = Math.Max(p1.Z, p2.Z);

                const double safeguardDist = 1.0;

                minPos.SetAndCorrectDimension(
                    (int)(minX - RopeWidth / 2 - safeguardDist),
                    (int)(minY - RopeWidth / 2 - safeguardDist),
                    (int)(minZ - RopeWidth / 2 - safeguardDist)
                );

                maxPos.SetAndCorrectDimension(
                    (int)(maxX + RopeWidth / 2 + safeguardDist),
                    (int)(maxY + RopeWidth / 2 + safeguardDist),
                    (int)(maxZ + RopeWidth / 2 + safeguardDist)
                );

                BlockAccess.WalkBlocks(minPos, maxPos, (cblock, x, y, z) => {
                    Cuboidf[] collisionBoxes = cblock.GetCollisionBoxes(BlockAccess, null);

                    if (collisionBoxes != null)
                    {
                        foreach (var cuboid in collisionBoxes)
                        {
                            if (cuboid != null)
                            {
                                CollisionConstraints.Add(new LineSegemntCollisionConstraint{
                                    minBoxBounds = new FastVec3f(cuboid.MinX, cuboid.MinY, cuboid.MinZ),
                                    maxBoxBounds = new FastVec3f(cuboid.MaxX, cuboid.MaxY, cuboid.MaxZ),
                                    Friction = 0, // TODO vary friction depending on the block
                                    PointIndex1 = (ushort)idx1,
                                    PointIndex2 = (ushort)idx2,
                                });
                            }
                        }
                    }
                });
            }

        }

        void substepNow(bool isFullStep, double stepRatio)
        {
            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                ref var point = ref PointsData[idx];

                point.substepUpdate(this, Points[idx], timeStepData, stepRatio, Forces);
            }

            var lengthConstraint = new LengthConstraint {
                TargetLength = RopeLength,
                Compliance = LengthCompliance,
                Damping = LengthDamping,
                Comparison = ConstraintComparison.Equal,
            };

            lengthConstraint.Update(PointsData, bufferHead, bufferSize, MaxBufferSize, timeStepData, Forces);

            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                ref var constraint = ref DistanceConstraints[idx];

                constraint.Update(PointsData, timeStepData, SegmentDistances[idx], LengthCompliance, LengthDamping, Forces);
            }

            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                ref var constraint = ref AngleConstraints[idx];

                constraint.Update(PointsData, timeStepData, Forces);
            }

            var collisionSpan = CollectionsMarshal.AsSpan<LineSegemntCollisionConstraint>(CollisionConstraints);

            for (int i = 0; i < collisionSpan.Length; i++)
            {
                ref var constraint = ref collisionSpan[i];

                constraint.Update(PointsData, timeStepData, Forces);
            }

            if (isFullStep)
            {
                for (int i = 0; i < bufferSize; i++)
                {
                    int idx = (bufferHead + i) % MaxBufferSize;
                    ref var point = ref Points[idx];

                    point.stepUpdate(timeStepData, api.World);
                }

                gatherCollisionConstraints();
            }
        }

        public void slowTick3s()
        {
            if (double.IsNaN(CenterPosition.X)) return;

            windSpeed = api.World.BlockAccessor.GetWindSpeedAt(CenterPosition) * (0.2 + noiseGen.Noise(0, (api.World.Calendar.TotalHours * 50) % 2000) * 0.8);
        }

        public void restoreReferences()
        {
            if (!Active) return;

            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                ref var point = ref Points[idx];

                point.restoreReferences(this, api.World);
            }
        }

        public void updateActiveState(EnumActiveStateChange stateChange)
        {
            if (Active && stateChange == EnumActiveStateChange.RegionNowLoaded) return;
            if (!Active && stateChange == EnumActiveStateChange.RegionNowUnloaded) return;

            bool wasActive = Active;

            Active = true;

            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                ref var point = ref PointsData[idx];

                Active &= api.World.BlockAccessor.GetChunkAtBlockPos((int)point.Pos.X, (int)point.Pos.Y, (int)point.Pos.Z) != null;
            }

            if (!wasActive && Active) restoreReferences();
        }

        public void CollectDirtyPoints(List<ClothPointPacket> packets)
        {
            for (int i = 0; i < bufferSize; i++)
            {
                int idx = (bufferHead + i) % MaxBufferSize;
                var point = Points[idx];

                if (point.Dirty)
                {
                    packets.Add(new ClothPointPacket() {
                        ClothId = ClothId,
                        PointId = i,
                        Point = point,
                        PointData = PointsData[i]
                    });

                    point.Dirty = false;
                }
            }
        }

        public void updatePoint(ClothPointPacket msg)
        {
            // TODO FIXME rewrite network sync
            Points[msg.PointId].updateFromPoint(msg.Point, api.World);

            PointsData[msg.PointId] = msg.PointData;
        }

        public void OnPinnnedEntityLoaded(Entity entity)
        {
            PinStorage.restoreReferences(entity);
        }
    }

    public enum EnumActiveStateChange
    {
        Default,
        RegionNowLoaded,
        RegionNowUnloaded
    }
}

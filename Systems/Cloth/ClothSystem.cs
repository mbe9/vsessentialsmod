using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Datastructures;

namespace Vintagestory.GameContent
{
    [ProtoContract]
    public enum EnumClothType
    {
        Rope,
        Cloth
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
        const int IterationCount = 1;
        public const int SubstepCount = 1000;

        [ProtoMember(1)]
        public int ClothId;
        [ProtoMember(2)]
        EnumClothType clothType;
        [ProtoMember(3)]
        List<ClothPoint> Points = new List<ClothPoint>();
        [ProtoMember(4)]
        public List<ClothPointData> PointsData = new List<ClothPointData>();
        [ProtoMember(4)]
        List<DistanceConstraint> DistanceConstraints = new List<DistanceConstraint>();
        [ProtoMember(4)]
        List<AngleConstraint> AngleConstraints = new List<AngleConstraint>();
        [ProtoMember(4)]
        List<ClothConstraint> Constraints = new List<ClothConstraint>();
        [ProtoMember(4)]
        public ClothPinStorage PinStorage = new();
        [ProtoMember(5)]
        public bool Active { get; set; }

        List<Cuboidf[]> cachedCollisionCuboids = new();

        /// <summary>
        /// 10 joints per meter
        /// </summary>
        public static float Resolution = 2;

        public float StretchWarn = 0.6f;
        public float StretchRip = 0.75f;

        public bool LineDebug=false;
        public bool boyant = false;
        protected ICoreClientAPI capi;
        public ICoreAPI api;        
        public Vec3d windSpeed = new Vec3d();
        public ParticlePhysics pp;
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

        public Vec3d CenterPosition
        {
            get
            {
                // Loop twice to not loose decimal precision

                Vec3d pos = new Vec3d();
                int cnt = 0;
                foreach (var pointlist in Points2d)
                {
                    foreach (var point in pointlist.Points)
                    {
                        cnt++;
                    }
                }

                foreach (var pointlist in Points2d)
                {
                    foreach (var point in pointlist.Points)
                    {
                        pos.Add(point.Pos.X / cnt, point.Pos.Y / cnt, point.Pos.Z / cnt);
                    }
                }

                return pos;
            }
        }

        public ClothPoint FirstPoint => Points[0];
        public ClothPoint LastPoint => Points[Points.Count - 1];

        public ClothPoint[] Ends => new ClothPoint[] { FirstPoint, LastPoint };

        public int Width => Points2d.Count;
        public int Length => Points2d[0].Points.Count;


        public static ClothSystem CreateCloth(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end)
        {
            return new ClothSystem(api, cm, start, end, EnumClothType.Cloth);
        }

        public static ClothSystem CreateRope(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end, AssetLocation clothSectionModel)
        {
            return new ClothSystem(api, cm, start, end, EnumClothType.Rope, clothSectionModel);
        }


        private ClothSystem() { }

        double minLen = 1;
        double maxLen = 10;

        public bool ChangeRopeLength(double len)
        {
            var plist = Points2d[0];

            double currentLength = plist.Points.Count / Resolution;

            bool isAdd = len > 0;

            if (isAdd && len + currentLength > maxLen) return false;
            if (!isAdd && len + currentLength < minLen) return false;

            int pointIndex = plist.Points.Max(p => p.PointIndex)+1;
            var fp = FirstPoint;
            var pine = fp.PinnedToEntity;
            var pinb = fp.PinnedToBlockPos;
            var pino = fp.pinnedToOffset;
            fp.UnPin();
            float step = 1 / Resolution;
            int totalPoints = Math.Abs((int)(len * Resolution));

            if (isAdd)
            {
                for (int i = 0; i <= totalPoints; i++)
                {
                    plist.Points.Insert(0, new ClothPoint(this, pointIndex++, fp.Pos.X + step*(i+1), fp.Pos.Y, fp.Pos.Z));

                    ClothPoint p1 = plist.Points[0];
                    ClothPoint p2 = plist.Points[1];

                    var points = new ClothPoint[2] {p1, p2};

                    var constraint = new ClothConstraint(points, ConstraintType.Distance, 1000.0);
                    constraint.TargetDistance = Resolution;
                    Constraints.Add(constraint);
                }
            }
            else
            {
                for (int i = 0; i <= totalPoints; i++)
                {
                    var point = plist.Points[0];
                    plist.Points.RemoveAt(0);
                        
                    for (int k = 0; k < Constraints.Count; k++)
                    {
                        var c = Constraints[k];
                        if (c.Point1 == point || c.Point2 == point)
                        {
                            Constraints.RemoveAt(k);
                            k--;
                        }
                    }
                }
            }

            if (pine != null) FirstPoint.PinTo(pine, pino);
            if (pinb != null) FirstPoint.PinTo(pinb, pino);
            genDebugMesh();

            return true;
        }

        private ClothSystem(ICoreAPI api, ClothManager cm, Vec3d start, Vec3d end, EnumClothType clothType, AssetLocation ropeSectionModel = null)
        {
            this.clothType = clothType;
            this.ropeSectionModel = ropeSectionModel;

            Init(api, cm);            
            float step = 1 / Resolution;

            var dir = end - start;

            switch (clothType)
            {
            case EnumClothType.Rope:
            {
                double len = dir.Length();

                var plist = new PointList();
                Points2d.Add(plist);

                int totalPoints = (int)(len * Resolution);

                for (int i = 0; i <= totalPoints; i++)
                {
                    var t = (float)i / totalPoints;

                    plist.Points.Add(new ClothPoint(this, i, start.X + dir.X * t, start.Y + dir.Y * t, start.Z + dir.Z * t));

                    if (i > 0)
                    {
                        ClothPoint p1 = plist.Points[i - 1];
                        ClothPoint p2 = plist.Points[i];

                        var points = new ClothPoint[2] {p1, p2};

                        var constraint = new ClothConstraint(points, ConstraintType.Distance, 1000.0);
                        constraint.TargetDistance = (p1.Pos - p2.Pos).Length();
                        Constraints.Add(constraint);
                    }
                }
                break;
            }

            case EnumClothType.Cloth:
            {
                double hlen = (end - start).HorLength();
                double vlen = Math.Abs(end.Y - start.Y);

                int hleni = (int)(hlen * Resolution);
                int vleni = (int)(vlen * Resolution);
                int totalPoints = hleni * vleni;

                int index = 0;

                for (int a = 0; a < hleni; a++)
                {
                    Points2d.Add(new PointList());

                    for (int y = 0; y < vleni; y++)
                    {
                        var th = a / hlen;
                        var tv = y / vlen;

                        Points2d[a].Points.Add(new ClothPoint(this, index++, start.X + dir.X * th, start.Y + dir.Y * tv, start.Z + dir.Z * th));

                        // add a vertical constraint
                        if (a > 0)
                        {
                            ClothPoint p1 = Points2d[a - 1].Points[y];
                            ClothPoint p2 = Points2d[a].Points[y];

                            var points = new ClothPoint[2] {p1, p2};

                            var constraint = new ClothConstraint(points, ConstraintType.Distance, 1000.0);
                            constraint.TargetDistance = Resolution;
                            Constraints.Add(constraint);
                        }

                        // add a new horizontal constraints
                        if (y > 0)
                        {
                            ClothPoint p1 = Points2d[a].Points[y - 1];
                            ClothPoint p2 = Points2d[a].Points[y];

                            var points = new ClothPoint[2] {p1, p2};

                            var constraint = new ClothConstraint(points, ConstraintType.Distance, 1000.0);
                            constraint.TargetDistance = Resolution;
                            Constraints.Add(constraint);
                        }
                    }
                }
                break;
            }
            }
        }

        public void genDebugMesh()
        {
            if (capi == null) return;

            debugMeshRef?.Dispose();
            debugUpdateMesh = new MeshData(20, 15, false, false, true, true);

            int vertexIndex = 0;

            for (int i = 0; i < Constraints.Count; i++)
            {
                var c = Constraints[i];
                int color = (i % 2) > 0 ? ColorUtil.WhiteArgb : ColorUtil.BlackArgb;

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
            pp = cm.partPhysics;
            this.BlockAccess = pp.BlockAccess;
            this.timeStepData = new TimeStepData(pp.PhysicsTickTime, SubstepCount);

            noiseGen = NormalizedSimplexNoise.FromDefaultOctaves(4, 100, 0.9, api.World.Seed + CenterPosition.GetHashCode());
        }


        public void WalkPoints(Action<ClothPoint> onPoint)
        {
            foreach (var point in Points)
            {
                onPoint(point);
            }
        }


        public int UpdateMesh(MeshData updateMesh, float dt)
        {
            var cfloats = updateMesh.CustomFloats;
            Vec3d campos = capi.World.Player.Entity.CameraPos;
            int basep = cfloats.Count;


            Vec4f lightRgba = api.World.BlockAccessor.GetLightRGBs(Constraints[Constraints.Count / 2].Point1.Pos.AsBlockPos);

            for (int i = 0; i < Constraints.Count; i++)
            {
                ClothConstraint cc = Constraints[i];
                Vec3d p1 = cc.Point1.Pos;
                Vec3d p2 = cc.Point2.Pos;

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

            return Constraints.Count;
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

                for (int i = 0; i < Constraints.Count; i++)
                {
                    ClothConstraint cc = Constraints[i];
                    Vec3d p1 = cc.Point1.Pos;
                    Vec3d p2 = cc.Point2.Pos;

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

        CachedCuboidList CollisionBoxList = new CachedCuboidList();
        Cuboidd particleCollBox = new Cuboidd();
        Cuboidd blockCollBox = new Cuboidd();
        BlockPos minPos = new BlockPos();
        BlockPos maxPos = new BlockPos();
        BlockPos tmp = new BlockPos();

        void gatherCollisionCuboids()
        {
            const double size = 0.1;

            var pointSpan = CollectionsMarshal.AsSpan<ClothPointData>(PointsData);
            for (int i = 0; i < pointSpan.Length; i++)
            {
                ref var point = ref pointSpan[i];

                minPos.SetAndCorrectDimension(
                    (int)(point.Pos.X - size / 2) - 1,
                    (int)(point.Pos.Y - size / 2) - 1,
                    (int)(point.Pos.Z - size / 2) - 1
                );

                maxPos.SetAndCorrectDimension(
                    (int)(point.Pos.X + size / 2) + 1,
                    (int)(point.Pos.Y + size / 2) + 1,
                    (int)(point.Pos.Z + size / 2) + 1
                );

                BlockAccess.WalkBlocks(minPos, maxPos, (cblock, x, y, z) => {
                    Cuboidf[] collisionBoxes = cblock.GetCollisionBoxes(BlockAccess, null);

                    if (cachedCollisionCuboids.Count > i)
                    {
                        cachedCollisionCuboids[i] = collisionBoxes;
                    }
                    else
                    {
                        cachedCollisionCuboids.Add(collisionBoxes);
                    }
                });
            }

        }

        void substepNow(bool isFullStep, double stepRatio)
        {
            var pointSpan = CollectionsMarshal.AsSpan<ClothPointData>(PointsData);
            var constraintsSpan = CollectionsMarshal.AsSpan<ClothConstraint>(Constraints);

            for (int i = 0; i < pointSpan.Length; i++)
            {
                ref var point = ref pointSpan[i];

                point.substepUpdate(this, Points[i], timeStepData, stepRatio);
            }

            for (int i = 0; i < constraintsSpan.Length; i++)
            {
                ref var constraint = ref constraintsSpan[i];

                constraint.Update(pointSpan, timeStepData);
            }

            // Handle collision constraints
            for (int i = 0; i < pointSpan.Length && i < cachedCollisionCuboids.Count; i++)
            {
                ref var point = ref pointSpan[i];

                Cuboidf[] collisionBoxes = cachedCollisionCuboids[i];

                if (cachedCollisionCuboids[i] == null) continue;

                for (var c = 0; c < collisionBoxes.Count(); c++)
                {
                    var collBox = collisionBoxes[c];

                    if (collBox == null) continue;

                    blockCollBox.SetAndTranslate(collBox, x, y, z);
                    blockCollBox.GrowBy(size / 2, size / 2, size / 2);

                    double constraintValue = 0;
                    Vec3d gradient;

                    CalcCollisionConstraint(p.Pos, blockCollBox, out constraintValue, out gradient);

                    UpdatePointOnCollision(p, constraintValue, gradient);
                }
            }

                double size = 0.1;
                {
                    int pointIdx = 0;

                    // Handle collision constraints
                    for (int i = 0; i < Points2d.Count; i++)
                    {
                        for (int j = 0; j < Points2d[i].Points.Count; j++)
                        {
                            var p = Points2d[i].Points[j];

                            minPos.SetAndCorrectDimension(
                                (int)(p.Pos.X - size / 2),
                                (int)(p.Pos.Y - size / 2), // -1 for the extra high collision box of fences
                                (int)(p.Pos.Z - size / 2)
                            );

                            maxPos.SetAndCorrectDimension(
                                (int)(p.Pos.X + size / 2),
                                (int)(p.Pos.Y + size / 2),
                                (int)(p.Pos.Z + size / 2)
                            );

                            BlockAccess.WalkBlocks(minPos, maxPos, (cblock, x, y, z) => {
                                Cuboidf[] collisionBoxes = cblock.GetCollisionBoxes(BlockAccess, new BlockPos());

                                if (collisionBoxes != null)
                                {
                                    for (var c = 0; c < collisionBoxes.Count(); c++) {
                                        var collBox = collisionBoxes[c];

                                        if (collBox == null) continue;

                                        blockCollBox.SetAndTranslate(collBox, x, y, z);
                                        blockCollBox.GrowBy(size / 2, size / 2, size / 2);

                                        double constraintValue = 0;
                                        Vec3d gradient;

                                        CalcCollisionConstraint(p.Pos, blockCollBox, out constraintValue, out gradient);

                                        UpdatePointOnCollision(p, constraintValue, gradient);
                                    }
                                }

                            }, false);

                            pointIdx++;
                        }
                    }
            }

            if (isFullStep)
            {
                foreach (var p in Points)
                {
                    p.stepUpdate(timeStepData, api.World);
                }

                gatherCollisionCuboids();
            }
        }

        public void CalcCollisionConstraint(Vec3d pos, Cuboidd coll, out double value, out Vec3d gradient)
        {
            const double epsilon = 0.000001;

            var collCenter = new Vec3d(
                (coll.MaxX + coll.MinX) / 2.0,
                (coll.MaxY + coll.MinY) / 2.0,
                (coll.MaxZ + coll.MinZ) / 2.0
            );

            double dx = (pos.X - collCenter.X);
            double dy = (pos.Y - collCenter.Y);
            double dz = (pos.Z - collCenter.Z);

            double abs_dx = Math.Abs(dx);
            double abs_dy = Math.Abs(dy);
            double abs_dz = Math.Abs(dz);

            double vx = abs_dx - coll.Width / 2.0;
            double vy = abs_dy - coll.Height / 2.0;
            double vz = abs_dz - coll.Length / 2.0;

            if (vx >= vy && vx >= vz)
            {
                value = vx;
                if (abs_dx > epsilon) gradient = new Vec3d(Math.Sign(dx), 0, 0);
                else gradient = new Vec3d(1, 0, 0);
            }
            else if (vy >= vx && vy >= vz)
            {
                value = vy;
                if (abs_dy > epsilon) gradient = new Vec3d(0.0, Math.Sign(dy), 0.0);
                else gradient = new Vec3d(0, 1, 0);
            }
            else
            {
                value = vz;
                if (abs_dz > epsilon) gradient = new Vec3d(0, 0, Math.Sign(dz));
                else gradient = new Vec3d(0, 0, 1);
            }
        }

        public void UpdatePointOnCollision(ClothPoint point, double constraintValue, Vec3d constraintGradient)
        {
            const double epsilon = 0.000001;

            if (constraintValue > 0.0 || point.InvMass < epsilon) return;

            double alpha = 0.0 * timeStepData.InvSubstepTimeSqr;
            double lambda_delta =
                ( -constraintValue ) /
                ( constraintGradient.LengthSq() * point.InvMass +
                  alpha );

            point.Pos += point.InvMass * constraintGradient * lambda_delta;
        }

        public void slowTick3s()
        {
            if (double.IsNaN(CenterPosition.X)) return;

            windSpeed = api.World.BlockAccessor.GetWindSpeedAt(CenterPosition) * (0.2 + noiseGen.Noise(0, (api.World.Calendar.TotalHours * 50) % 2000) * 0.8);
        }

        public void restoreReferences()
        {
            if (!Active) return;

            Dictionary<int, ClothPoint> pointsByIndex = new Dictionary<int, ClothPoint>();
            WalkPoints((p) => {
                pointsByIndex[p.PointIndex] = p;
                p.restoreReferences(this, api.World);
            });

            foreach (var c in Constraints)
            {
                c.RestorePoints(pointsByIndex);
            }            
        }

        public void updateActiveState(EnumActiveStateChange stateChange)
        {
            if (Active && stateChange == EnumActiveStateChange.RegionNowLoaded) return;
            if (!Active && stateChange == EnumActiveStateChange.RegionNowUnloaded) return;

            bool wasActive = Active;

            Active = true;
            WalkPoints((p) => {
                Active &= api.World.BlockAccessor.GetChunkAtBlockPos((int)p.Pos.X, (int)p.Pos.Y, (int)p.Pos.Z) != null;
            });

            if (!wasActive && Active) restoreReferences();
        }




        public void CollectDirtyPoints(List<ClothPointPacket> packets)
        {
            for (int i = 0; i < Points2d.Count; i++)
            {
                for (int j = 0; j < Points2d[i].Points.Count; j++)
                {
                    var point = Points2d[i].Points[j];
                    if (point.Dirty)
                    {
                        packets.Add(new ClothPointPacket() { ClothId = ClothId, PointX = i, PointY = j, Point = point });
                        point.Dirty = false;
                    }
                }
            }
        }

        public void updatePoint(ClothPointPacket msg)
        {
            ClothPoint point = Points2d[msg.PointX].Points[msg.PointY];
            point.updateFromPoint(msg.Point, api.World);
        }

        public void OnPinnnedEntityLoaded(Entity entity)
        {
            pinStorage.restoreReferences(entity);
        }
    }

    public enum EnumActiveStateChange
    {
        Default,
        RegionNowLoaded,
        RegionNowUnloaded
    }
}

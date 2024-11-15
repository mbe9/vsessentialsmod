using ProtoBuf;
using System;
using System.Collections.Generic;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Util;
using Vintagestory.API.Datastructures;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Vintagestory.GameContent
{
    public enum ClothPointFlags : byte
    {
        PositionFixed = 1 << 0,
        Pinned = 1 << 1,
    }

    [ProtoContract]
    public struct ClothPointData
    {
        [ProtoMember(1)]
        public FastVec3d Pos;
        [ProtoMember(2)]
        public FastVec3d PrevPos;
        [ProtoMember(3)]
        public float InvMass;
        [ProtoMember(4)]
        public float GravityStrength = 1;
        [ProtoMember(5)]
        public ushort FrictionXY = 0;
        [ProtoMember(6)]
        public ushort FrictionXZ = 0;
        [ProtoMember(7)]
        public ushort FrictionYZ = 0;
        [ProtoMember(8)]
        public byte Movable = 1;
        [ProtoMember(9)]
        public byte Pinned = 0;

        public ClothPointData() {}

        public ClothPointData(Vec3d Pos, Vec3d PrevPos, double mass)
        {
            this.Pos.X = Pos.X;
            this.Pos.Y = Pos.Y;
            this.Pos.Z = Pos.Z;

            this.PrevPos.X = PrevPos.X;
            this.PrevPos.Y = PrevPos.Y;
            this.PrevPos.Z = PrevPos.Z;

            if (!double.IsInfinity(mass))
            {
                this.InvMass = (float)(1.0 / mass);
            }
            else
            {
                this.InvMass = 0;
                this.Movable = 0;
            }
        }

        public ClothPointData(in FastVec3d Pos, in FastVec3d PrevPos, double mass)
        {
            this.Pos = Pos;
            this.PrevPos = PrevPos;

            if (!double.IsInfinity(mass))
            {
                this.InvMass = (float)(1.0 / mass);
            }
            else
            {
                this.InvMass = 0;
                this.Movable = 0;
            }
        }

        public void substepUpdate(ClothSystem cs, ClothPoint p, TimeStepData time, double stepRatio, FastVec3f[] forces)
        {
            if (Movable == 0) return;

            if (Pinned == 0)
            {
                FastVec3d accel;
                accel.X = 0.0;//InvMass * cs.windSpeed.X * isNotCollidingMult;
                accel.Y = -GravityStrength * 10.0;
                accel.Z = 0.0;

                // Verlet integration scheme
                // x_(n+1) = 2 * x_(n) - x_(n-1) + accel * dt^2

                double dx = Pos.X - PrevPos.X + accel.X * (time.SubstepTime * time.SubstepTime);
                double dy = Pos.Y - PrevPos.Y + accel.Y * (time.SubstepTime * time.SubstepTime);
                double dz = Pos.Z - PrevPos.Z + accel.Z * (time.SubstepTime * time.SubstepTime);

                // Adjust deltas with accordance to friction, if there were any collisions
                {
                    double dxdy_length = Math.Sqrt((dx * dx) + (dy * dy));
                    double dxdz_length = Math.Sqrt((dx * dx) + (dz * dz));
                    double dydz_length = Math.Sqrt((dy * dy) + (dz * dz));

                    // These can divide by zero, but Clamps below should guard against that case
                    double coeffXY = (double)FrictionXY * 0.001 / dxdy_length;
                    double coeffXZ = (double)FrictionXZ * 0.001 / dxdz_length;
                    double coeffYZ = (double)FrictionYZ * 0.001 / dydz_length;

                    dx *= 1.0 - Math.Clamp(coeffXY + coeffXZ, 0.0, 1.0);
                    dy *= 1.0 - Math.Clamp(coeffXY + coeffYZ, 0.0, 1.0);
                    dz *= 1.0 - Math.Clamp(coeffXZ + coeffYZ, 0.0, 1.0);
                }

                (PrevPos.X, Pos.X) = (Pos.X, Pos.X + dx);
                (PrevPos.Y, Pos.Y) = (Pos.Y, Pos.Y + dy);
                (PrevPos.Z, Pos.Z) = (Pos.Z, Pos.Z + dz);

                FrictionXY = 0;
                FrictionXZ = 0;
                FrictionYZ = 0;

                if (forces != null)
                {
                    double mass = 1.0 / InvMass;
                    forces[p.InternalIndex].X += (float)(accel.X * mass);
                    forces[p.InternalIndex].Y += (float)(accel.Y * mass);
                    forces[p.InternalIndex].Z += (float)(accel.Z * mass);
                }
            }
            else
            {
                if (p.pinInfo != null)
                {
                    (PrevPos.X, Pos.X) = (Pos.X, p.pinInfo.PrevPinnedPos.X * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.X * stepRatio);
                    (PrevPos.Y, Pos.Y) = (Pos.Y, p.pinInfo.PrevPinnedPos.Y * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.Y * stepRatio);
                    (PrevPos.Z, Pos.Z) = (Pos.Z, p.pinInfo.PrevPinnedPos.Z * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.Z * stepRatio);
                }
            }
        }
    }

    [ProtoContract]
    public class ClothPoint
    {
        [ProtoMember(1)]
        int PinId = -1;
        [ProtoMember(2)]
        ushort PointIndex;

        public ClothPinInfo pinInfo;

        ClothSystem cs;

        double accum1s;

        public bool Dirty { get; internal set; }

        public ClothPoint(ClothSystem cs, ushort pointIndex)
        {
            this.cs = cs;

            this.PointIndex = pointIndex;
        }

        protected ClothPoint() { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ref ClothPointData getPointData()
        {
            return ref cs.PointsData[PointIndex];
        }

        public ushort InternalIndex { get => PointIndex; set => PointIndex = value; }
        public int InternalPinId { get => PinId; set => PinId = value; }

        public Entity PinnedToEntity => pinInfo?.pinnedTo;
        public BlockPos PinnedToBlockPos => pinInfo?.pinnedToBlockPos;
        public bool Pinned => (pinInfo != null);

        public Vec3d Pos {
            get {
                ref var p = ref getPointData();
                return new Vec3d(p.Pos.X, p.Pos.Y, p.Pos.Z);
            }
        }

        public void ExchangePins(ClothPoint other)
        {
            var tmpPinId = PinId;
            PinId = other.PinId;
            other.PinId = tmpPinId;

            // TODO: exchange Movable independently of Pinned

            var tmpPinned = getPointData().Pinned;
            getPointData().Pinned = other.getPointData().Pinned;
            other.getPointData().Pinned = tmpPinned;

            var tmpMovable = getPointData().Movable;
            getPointData().Movable = other.getPointData().Movable;
            other.getPointData().Movable = tmpMovable;

            MarkDirty();
            other.MarkDirty();
        }

        public void PinTo(Entity toEntity, Vec3f offset)
        {
            if (PinId >= 0) UnPin();

            PinId = cs.PinStorage.AddPin(toEntity, offset);
            pinInfo = cs.PinStorage.GetPin(PinId);

            getPointData().Pinned = 1;
            getPointData().Movable = 1;

            MarkDirty();
        }

        public void PinTo(BlockPos blockPos, Vec3f offset)
        {
            if (PinId >= 0) UnPin();

            PinId = cs.PinStorage.AddPin(blockPos, offset);
            pinInfo = cs.PinStorage.GetPin(PinId);

            getPointData().Pinned = 1;
            getPointData().Movable = 0;

            MarkDirty();
        }

        public void UnPin()
        {
            cs.PinStorage.RemovePin(PinId);
            PinId = -1;
            pinInfo = null;

            getPointData().Pinned = 0;
            getPointData().Movable = 1;

            MarkDirty();
        }

        public void MarkDirty()
        {
            Dirty = true;
        }


        public void stepUpdate(TimeStepData time, IWorldAccessor world)
        {
            if (pinInfo != null && pinInfo.pinnedTo == null && pinInfo.pinnedToPlayerUid != null)
            {
                var eplr = world.PlayerByUid(pinInfo.pinnedToPlayerUid)?.Entity;
                if (eplr?.World != null) pinInfo.pinnedTo = eplr;
            }

            if (pinInfo != null)
            {
                if (pinInfo.pinnedTo != null)
                {
                    if (pinInfo.pinnedTo.ShouldDespawn && pinInfo.pinnedTo.DespawnReason?.Reason != EnumDespawnReason.Unload)
                    {
                        UnPin();
                        return;
                    }

                    // New ideas:
                    // don't apply force onto the player/entity on compression
                    // apply huge forces onto the player on strong extension (to prevent massive stretching) (just set player motion to 0 or so. or we add a new countermotion field thats used in EntityControlledPhysics?) 

                    var weight = pinInfo.pinnedTo.Properties.Weight;
                    
                    float counterTensionStrength = GameMath.Clamp(50f / weight, 0.1f, 2f);

                    bool extraResist =
                        (pinInfo.pinnedTo as EntityAgent)?.Controls.Sneak == true
                        || (pinInfo.pinnedTo is EntityPlayer
                            && (pinInfo.pinnedTo.AnimManager?.IsAnimationActive("sit") == true
                                || pinInfo.pinnedTo.AnimManager?.IsAnimationActive("sleep") == true));

                    // TODO: apply the extra inertia to the player, not the point
                    // if (extraResist) {
                    //     getPointData().InvMass = 1.0f / (weight * 200.0f);
                    // } else {
                    //     getPointData().InvMass = 1.0f / weight;
                    // }

                    pinInfo.PrevPinnedPos = pinInfo.CurrPinnedPos;
                    pinInfo.CurrPinnedPos = pinInfo.getPinnedPos();


                    bool pushable = true;// PushingPhysics && (eplr == null || eplr.Player.WorldData.CurrentGameMode != EnumGameMode.Creative);
                    
                    if (pushable)
                    {
                        // pos.Motion += (Pos - PrevPos) * time.InvSubstepTime;
                    }
                }
                else
                {
                    if (pinInfo.pinnedToBlockPos != null)
                    {
                        accum1s += time.StepTime;

                        if (accum1s >= 1)
                        {
                            accum1s = 0;
                            Block block = cs.api.World.BlockAccessor.GetBlock(PinnedToBlockPos);
                            if (!block.HasBehavior<BlockBehaviorRopeTieable>())
                            {
                                UnPin();
                            }
                        }
                    }
                }
            }
        }

        public void restoreReferences(ClothSystem cs, IWorldAccessor world)
        {
            this.cs = cs;

            cs.PinStorage.restoreReferences(world);
            pinInfo = cs.PinStorage.GetPin(PinId);

            // if (pinnedToEntityId != 0)
            // {
            //     pinnedTo = world.GetEntityById(pinnedToEntityId);
            //     if (pinnedTo == null)
            //     {
            //        // UnPin();
            //     }
            //     else
            //     {
            //         PinTo(pinnedTo, pinnedToOffset);
            //     }
            // }

            // if (pinnedToBlockPos != null)
            // {
            //     PinTo(pinnedToBlockPos, pinnedToOffset);
            // }
        }

        public void updateFromPoint(ClothPoint point, IWorldAccessor world)
        {
            this.PinId = point.PinId;
            this.PointIndex = point.PointIndex;

            cs.PinStorage.restoreReferences(world);
            pinInfo = cs.PinStorage.GetPin(PinId);

            // PointIndex = point.PointIndex;
            // InvMass = point.InvMass;
            // Pos.Set(point.Pos);
            // GravityStrength = point.GravityStrength;
            // pinned = point.pinned;
            // pinnedToEntityId = point.pinnedToEntityId;
            // pinnedToPlayerUid = point.pinnedToPlayerUid;
            // if (pinnedToEntityId != 0)
            // {
            //     pinnedTo = world.GetEntityById(pinnedToEntityId);
            //     if (pinnedTo != null)
            //     {
            //         PinTo(pinnedTo, pinnedToOffset);
            //     }
            //     else UnPin();

            // }

            // pinnedToBlockPos = pinnedToBlockPos.SetOrCreate(point.pinnedToBlockPos);
            // pinnedToOffset = pinnedToOffset.SetOrCreate(point.pinnedToOffset);

            // pinnedToOffsetStartYaw = point.pinnedToOffsetStartYaw;
        }
    }

}

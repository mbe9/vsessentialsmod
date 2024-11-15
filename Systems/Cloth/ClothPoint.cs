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
        public byte Movable = 1;
        [ProtoMember(6)]
        public byte Pinned = 0;

        public ClothPointData() {}

        // public void SetFlag(ClothPointFlags flag, bool value)
        // {
        //     if (value)
        //     {
        //         flags |= flag;
        //     }
        //     else
        //     {
        //         flags &= ~flag;
        //     }
        // }

        public void substepUpdate(ClothSystem cs, ClothPoint p, TimeStepData time, double stepRatio)
        {
            if (Movable == 0) return;

            if (Pinned == 0)
            {
                // 1.0 if the point is NOT colliding, 0.0 otherwise
                // Done this way to remove branching
                // double isNotCollidingMult = 1.0 - (double)(flags & (byte)ClothPointFlags.IsColliding);

                FastVec3d accel;
                accel.X = 0.0;//InvMass * cs.windSpeed.X * isNotCollidingMult;
                accel.Y = - InvMass * GravityStrength * 10.0;
                accel.Z = 0.0;

                // Verlet integration scheme
                // x_(n+1) = 2 * x_(n) - x_(n-1) + accel * dt^2
                (PrevPos.X, Pos.X) = (Pos.X, 2.0 * Pos.X - PrevPos.X + accel.X * (time.SubstepTime * time.SubstepTime));
                (PrevPos.Y, Pos.Y) = (Pos.Y, 2.0 * Pos.Y - PrevPos.Y + accel.Y * (time.SubstepTime * time.SubstepTime));
                (PrevPos.Z, Pos.Z) = (Pos.Z, 2.0 * Pos.Z - PrevPos.Z + accel.Z * (time.SubstepTime * time.SubstepTime));
            }
            else
            {
                if (p.pinInfo != null)
                {
                    Pos.X = p.pinInfo.PrevPinnedPos.X * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.X * stepRatio;
                    Pos.Y = p.pinInfo.PrevPinnedPos.Y * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.Y * stepRatio;
                    Pos.Z = p.pinInfo.PrevPinnedPos.Z * (1.0 - stepRatio) + p.pinInfo.CurrPinnedPos.Z * stepRatio;
                }
            }
        }
    }

    [ProtoContract]
    public class ClothPoint
    {
        [ProtoMember(1)]
        float OriginalMass = 1.0f;
        [ProtoMember(2)]
        int pinId = -1;
        [ProtoMember(3)]
        ushort PointIndex;

        public ClothPinInfo pinInfo;
        public ClothPinStorage pinStorage;
        public List<ClothPointData> pointsData;

        ClothSystem cs;

        double accum1s;

        public bool Dirty { get; internal set; }

        public ClothPoint(ClothSystem cs, ushort pointIndex)
        {
            this.cs = cs;
            this.pinStorage = cs.PinStorage;
            this.pointsData = cs.PointsData;

            this.PointIndex = pointIndex;
        }

        protected ClothPoint() { }

        ref ClothPointData getPointData()
        {
            var pointSpan = CollectionsMarshal.AsSpan<ClothPointData>(pointsData);

            return ref pointSpan[PointIndex];
        }

        public Entity PinnedToEntity => pinInfo?.pinnedTo;
        public BlockPos PinnedToBlockPos => pinInfo?.pinnedToBlockPos;
        public bool Pinned => (pinInfo != null);


        public void PinTo(Entity toEntity, Vec3f offset)
        {
            if (pinId >= 0) return;

            pinId = pinStorage.AddPin(toEntity, offset);
            pinInfo = pinStorage.GetPin(pinId);

            getPointData().Pinned = 1;
            getPointData().Movable = 1;
            // getPointData().InvMass = 1.0f / toEntity.Properties.Weight;

            MarkDirty();
        }

        public void PinTo(BlockPos blockPos, Vec3f offset)
        {
            if (pinId >= 0) return;

            pinId = pinStorage.AddPin(blockPos, offset);
            pinInfo = pinStorage.GetPin(pinId);

            getPointData().Pinned = 1;
            getPointData().Movable = 0;
            // Blocks are unmovable, as if they have infinite mass
            // getPointData().InvMass = 0.0f;

            MarkDirty();
        }

        public void UnPin()
        {
            pinStorage.RemovePin(pinId);
            pinId = -1;
            pinInfo = null;

            getPointData().Pinned = 0;
            getPointData().Movable = 1;
            // getPointData().InvMass = 1.0f / OriginalMass;

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

            if (pinnedToEntityId != 0)
            {
                pinnedTo = world.GetEntityById(pinnedToEntityId);
                if (pinnedTo == null)
                {
                   // UnPin();
                }
                else
                {
                    PinTo(pinnedTo, pinnedToOffset);
                }
            }

            if (pinnedToBlockPos != null)
            {
                PinTo(pinnedToBlockPos, pinnedToOffset);
            }
        }

        public void updateFromPoint(ClothPoint point, IWorldAccessor world)
        {
            PointIndex = point.PointIndex;
            InvMass = point.InvMass;
            Pos.Set(point.Pos);
            GravityStrength = point.GravityStrength;
            pinned = point.pinned;
            pinnedToEntityId = point.pinnedToEntityId;
            pinnedToPlayerUid = point.pinnedToPlayerUid;
            if (pinnedToEntityId != 0)
            {
                pinnedTo = world.GetEntityById(pinnedToEntityId);
                if (pinnedTo != null)
                {
                    PinTo(pinnedTo, pinnedToOffset);
                }
                else UnPin();
                
            }

            pinnedToBlockPos = pinnedToBlockPos.SetOrCreate(point.pinnedToBlockPos);
            pinnedToOffset = pinnedToOffset.SetOrCreate(point.pinnedToOffset);

            pinnedToOffsetStartYaw = point.pinnedToOffsetStartYaw;
        }
    }

}

using ProtoBuf;
using System;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Util;
using Vintagestory.API.Datastructures;

namespace Vintagestory.GameContent
{
    [ProtoContract]
    public class ClothPoint
    {
        public static bool PushingPhysics = false;

        [ProtoMember(1)]
        public int PointIndex;
        [ProtoMember(2)]
        public double InvMass;
        [ProtoMember(3)]
        public Vec3d Pos;
        [ProtoMember(4)]
        public Vec3d PrevPos;
        // [ProtoMember(6)]
        // public Vec3f Tension = new Vec3f();
        [ProtoMember(7)]
        float GravityStrength = 1;
        [ProtoMember(8)]
        bool pinned;
        [ProtoMember(9)]
        public long pinnedToEntityId;
        [ProtoMember(10)]
        BlockPos pinnedToBlockPos;
        [ProtoMember(11)]
        public Vec3f pinnedToOffset;
        [ProtoMember(12)]
        float pinnedToOffsetStartYaw;
        [ProtoMember(13)]
        string pinnedToPlayerUid; // player entity ids change over time >.<

        Vec3d CurrPinnedPos;
        Vec3d PrevPinnedPos;

        // public float InvMass;

        public bool Dirty { get; internal set; }


        public EnumCollideFlags CollideFlags;
        public float YCollideRestMul;
        Vec4f tmpvec = new Vec4f();
        ClothSystem cs;
        Entity pinnedTo;
        Matrixf pinOffsetTransform;

        // These values are set be the constraints, they should actually get summed up though. 
        // For rope, a single set works though, because we only need the ends, connected by 1 constraint
        // In otherwords: Cloth pulling motion thing is not supported
        // public Vec3d TensionDirection = new Vec3d();
        // public double extension;


        // Damping factor. Velocities are multiplied by this
        // private float dampFactor = 0.9f;

        double accum1s;
        bool extraResist = false;

        public ClothPoint(ClothSystem cs)
        {
            this.cs = cs;
            Pos = new Vec3d();
            PrevPos = new Vec3d();
            init();
        }

        protected ClothPoint() { }

        public ClothPoint(ClothSystem cs, int pointIndex, double x, double y, double z)
        {
            this.cs = cs;
            this.PointIndex = pointIndex;
            Pos = new Vec3d(x, y, z);
            PrevPos = new Vec3d(x, y, z);
            init();
        }

        public void setMass(double mass)
        {
            InvMass = 1.0 / mass;
        }

        void init()
        {
            setMass(1);
        }

        public Entity PinnedToEntity => pinnedTo;
        public BlockPos PinnedToBlockPos => pinnedToBlockPos;
        public bool Pinned => pinned;


        public void PinTo(Entity toEntity, Vec3f pinOffset)
        {
            pinned = true;
            pinnedTo = toEntity;
            pinnedToEntityId = toEntity.EntityId;
            pinnedToOffset = pinOffset;
            pinnedToOffsetStartYaw = toEntity.SidedPos.Yaw;
            pinOffsetTransform = Matrixf.Create();
            pinnedToBlockPos = null;
            if (toEntity is EntityPlayer eplr) pinnedToPlayerUid = eplr.PlayerUID;
            setMass(toEntity.Properties.Weight);

            PrevPinnedPos = getPinnedPos();
            CurrPinnedPos = PrevPinnedPos;

            MarkDirty();
        }

        public void PinTo(BlockPos blockPos, Vec3f offset)
        {
            this.pinnedToBlockPos = blockPos;
            pinnedToOffset = offset;
            pinnedToPlayerUid = null;
            pinned = true;
            pinnedTo = null;
            pinnedToEntityId = 0;
            // Blocks are unmovable, as if they have infinite mass
            InvMass = 0.0;

            Pos.Set(pinnedToBlockPos).Add(pinnedToOffset);
            PrevPinnedPos = Pos;
            CurrPinnedPos = Pos;

            MarkDirty();
        }

        public void UnPin()
        {
            pinned = false;
            pinnedTo = null;
            pinnedToPlayerUid = null;
            pinnedToEntityId = 0;
            // TODO: get original point properties from clothsystem
            setMass(1.0);

            MarkDirty();
        }

        public void MarkDirty()
        {
            Dirty = true;
        }

        public void substepUpdate(TimeStepData time, double stepRatio)
        {
            const double epsilon = 0.00001;

            if (InvMass < epsilon) return;

            Vec3d force = Vec3d.Zero; //Tension.Clone();
            force.Y -= GravityStrength * 10;

            // Calculate the acceleration
            Vec3d acceleration = force * InvMass;

            if (CollideFlags == 0)
            {
                acceleration.X += (float)cs.windSpeed.X * InvMass;
            }

            // Verlet integration scheme
            Vec3d newPos = 2 * Pos - PrevPos + (acceleration * time.SubstepTime * time.SubstepTime);

            PrevPos = Pos;
            Pos = newPos;

            if (pinned && CurrPinnedPos != null && PrevPinnedPos != null)
            {
                Pos = PrevPinnedPos * (1.0 - stepRatio) + CurrPinnedPos * stepRatio;
            }
        }

        public void stepUpdate(TimeStepData time, IWorldAccessor world)
        {
            if (pinnedTo == null && pinnedToPlayerUid != null)
            {
                var eplr = world.PlayerByUid(pinnedToPlayerUid)?.Entity;
                if (eplr?.World != null) PinTo(eplr, pinnedToOffset);
            }

            if (pinned) 
            {
                if (pinnedTo != null)
                {
                    if (pinnedTo.ShouldDespawn && pinnedTo.DespawnReason?.Reason != EnumDespawnReason.Unload)
                    {
                        UnPin();
                        return;
                    }

                    // New ideas:
                    // don't apply force onto the player/entity on compression
                    // apply huge forces onto the player on strong extension (to prevent massive stretching) (just set player motion to 0 or so. or we add a new countermotion field thats used in EntityControlledPhysics?) 

                    float weight = pinnedTo.Properties.Weight;
                    
                    float counterTensionStrength = GameMath.Clamp(50f / weight, 0.1f, 2f);

                    bool extraResistNew =
                        (pinnedTo as EntityAgent)?.Controls.Sneak == true
                        || (pinnedTo is EntityPlayer
                            && (pinnedTo.AnimManager?.IsAnimationActive("sit") == true
                                || pinnedTo.AnimManager?.IsAnimationActive("sleep") == true));

                    if (extraResistNew != extraResist){
                        if (extraResistNew) {
                            setMass(weight * 200);
                        } else {
                            InvMass *= 200;
                        }
                        extraResist = extraResistNew;
                    }

                    // float tensionResistStrength = weight / 10f * (extraResist ? 200 : 1);

                    var eplr = pinnedTo as EntityPlayer;
                    var eagent = pinnedTo as EntityAgent;
                    Vec4f outvec;

                    AttachmentPointAndPose apap = eplr?.AnimManager?.Animator?.GetAttachmentPointPose("RightHand");
                    if (apap == null) apap = pinnedTo?.AnimManager?.Animator?.GetAttachmentPointPose("rope");

                    if (apap != null)
                    {
                        Matrixf modelmat = new Matrixf();
                        if (eplr != null) modelmat.RotateY(eagent.BodyYaw + GameMath.PIHALF);
                        else modelmat.RotateY(pinnedTo.SidedPos.Yaw + GameMath.PIHALF);

                        modelmat.Translate(-0.5, 0, -0.5);

                        apap.MulUncentered(modelmat);
                        outvec = modelmat.TransformVector(new Vec4f(0,0,0,1));
                    }
                    else
                    {
                        pinOffsetTransform.Identity();
                        pinOffsetTransform.RotateY(pinnedTo.SidedPos.Yaw - pinnedToOffsetStartYaw);
                        tmpvec.Set(pinnedToOffset.X, pinnedToOffset.Y, pinnedToOffset.Z, 1);
                        outvec = pinOffsetTransform.TransformVector(tmpvec);
                    }

                    EntityPos pos = pinnedTo.SidedPos;

                    bool pushable = true;// PushingPhysics && (eplr == null || eplr.Player.WorldData.CurrentGameMode != EnumGameMode.Creative);
                    
                    if (pushable)
                    {
                        // pos.Motion += (Pos - PrevPos) * time.InvSubstepTime;
                    }

                    PrevPinnedPos = CurrPinnedPos;
                    CurrPinnedPos = new Vec3d(pos.X + outvec.X, pos.Y + outvec.Y, pos.Z + outvec.Z);
                }
                else
                {
                    if (pinnedToBlockPos != null)
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
            else
            {
                // Update velocity

                // TODO: because HandleBoyancy and UpdateMotion work with velocity directly,
                // we are losing some (most?) of the precision of Verlet integration here.
                //
                // More precise way to do this would be to maybe work with position only in collision and return updated one?

                // Vec3f velocity = Vec3f.Zero;
                // velocity.Set((Pos - PrevPos) * time.InvSubstepTime);

                // Damp the velocity
                // nextVelocity *= dampFactor;

                // Collision detection
                // float size = 0.1f;
                // cs.pp.HandleBoyancy(Pos, velocity, cs.boyant, GravityStrength, (float)time.SubstepTime, size);
                // CollideFlags = UpdateMotion(out Pos, Pos, size);

                // dt *= 0.99f;
                // Pos = PrevPos;
                // Pos.Add(velocity.X * time.SubstepTime,
                //         velocity.Y * time.SubstepTime,
                //         velocity.Z * time.SubstepTime);


                // Velocity.Set(nextVelocity);
                // Tension.Set(0, 0, 0);
            }
        }

        Vec3d getPinnedPos()
        {
            if (pinned)
            {
                if (pinnedTo != null)
                {
                    var eplr = pinnedTo as EntityPlayer;
                    var eagent = pinnedTo as EntityAgent;
                    Vec4f outvec;

                    AttachmentPointAndPose apap = eplr?.AnimManager?.Animator?.GetAttachmentPointPose("RightHand");
                    if (apap == null) apap = pinnedTo?.AnimManager?.Animator?.GetAttachmentPointPose("rope");

                    if (apap != null)
                    {
                        Matrixf modelmat = new Matrixf();
                        if (eplr != null) modelmat.RotateY(eagent.BodyYaw + GameMath.PIHALF);
                        else modelmat.RotateY(pinnedTo.SidedPos.Yaw + GameMath.PIHALF);

                        modelmat.Translate(-0.5, 0, -0.5);

                        apap.MulUncentered(modelmat);
                        outvec = modelmat.TransformVector(new Vec4f(0,0,0,1));
                    }
                    else
                    {
                        pinOffsetTransform.Identity();
                        pinOffsetTransform.RotateY(pinnedTo.SidedPos.Yaw - pinnedToOffsetStartYaw);
                        tmpvec.Set(pinnedToOffset.X, pinnedToOffset.Y, pinnedToOffset.Z, 1);
                        outvec = pinOffsetTransform.TransformVector(tmpvec);
                    }

                    EntityPos pos = pinnedTo.SidedPos;

                    return new Vec3d(pos.X + outvec.X, pos.Y + outvec.Y, pos.Z + outvec.Z);
                }
                else if (pinnedToBlockPos != null)
                {
                    return new Vec3d().Set(pinnedToBlockPos).Add(pinnedToOffset);
                }
                else
                {
                    return null;
                }
            }
            else
            {
                return null;
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
        public void restoreReferences(Entity entity)
        {
            if (pinnedToEntityId == entity.EntityId)
            {
                PinTo(entity, pinnedToOffset);
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

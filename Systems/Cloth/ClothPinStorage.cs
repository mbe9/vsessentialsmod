using ProtoBuf;
using System;
using System.Collections.Generic;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Util;
using Vintagestory.API.Datastructures;

namespace Vintagestory.GameContent
{
    [ProtoContract]
    public class ClothPinInfo
    {
        [ProtoMember(1)]
        public long pinnedToEntityId;
        [ProtoMember(2)]
        public BlockPos pinnedToBlockPos;
        [ProtoMember(3)]
        public Vec3f pinnedToOffset;
        [ProtoMember(4)]
        public float pinnedToOffsetStartYaw;
        [ProtoMember(5)]
        public string pinnedToPlayerUid; // player entity ids change over time >.<
        [ProtoMember(6)]
        public FastVec3d CurrPinnedPos;
        [ProtoMember(7)]
        public FastVec3d PrevPinnedPos;

        public Entity pinnedTo;

        public ClothPinInfo(Entity toEntity, Vec3f pinOffset)
        {
            pinnedTo = toEntity;
            pinnedToEntityId = toEntity.EntityId;
            pinnedToOffset = pinOffset;
            pinnedToOffsetStartYaw = toEntity.SidedPos.Yaw;
            pinnedToBlockPos = null;
            if (toEntity is EntityPlayer eplr) pinnedToPlayerUid = eplr.PlayerUID;

            PrevPinnedPos = getPinnedPos();
            CurrPinnedPos = PrevPinnedPos;


        }

        public ClothPinInfo(BlockPos blockPos, Vec3f offset)
        {
            pinnedToBlockPos = blockPos;
            pinnedToOffset = offset;
            pinnedToPlayerUid = null;
            pinnedTo = null;
            pinnedToEntityId = 0;

            PrevPinnedPos = getPinnedPos();
            CurrPinnedPos = PrevPinnedPos;
        }

        public FastVec3d getPinnedPos()
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
                    Vec4f tmpvec = new();
                    Matrixf pinOffsetTransform = new();
                    pinOffsetTransform.Identity();
                    pinOffsetTransform.RotateY(pinnedTo.SidedPos.Yaw - pinnedToOffsetStartYaw);
                    tmpvec.Set(pinnedToOffset.X, pinnedToOffset.Y, pinnedToOffset.Z, 1);
                    outvec = pinOffsetTransform.TransformVector(tmpvec);
                }

                EntityPos pos = pinnedTo.SidedPos;

                return new FastVec3d(pos.X + outvec.X, pos.Y + outvec.Y, pos.Z + outvec.Z);
            }
            else if (pinnedToBlockPos != null)
            {
                return new FastVec3d(pinnedToBlockPos.X + pinnedToOffset.X,
                                     pinnedToBlockPos.Y + pinnedToOffset.Y,
                                     pinnedToBlockPos.Z + pinnedToOffset.Z);
            }
            else
            {
                return new FastVec3d(0, 0, 0);
            }
        }
    }

    [ProtoContract]
    public class ClothPinStorage
    {
        [ProtoMember(1)]
        List<ClothPinInfo> data = new();
        [ProtoMember(2)]
        public int Count = 0;

        public int GetFreeStorage()
        {
            int i = 0;
            for (; i < data.Count; i++)
            {
                if (data[i] == null)
                {
                    if ((i == Count) && (data.Count > (i + 1)))
                    {
                        // rest of the list is full of nulls,
                        // can delete them safely
                        data.RemoveRange(i + 1, data.Count - (i + 1));
                    }

                    return i;
                }
            }

            data.Add(null);
            return i;
        }

        public int AddPin(Entity toEntity, Vec3f offset)
        {
            int id = GetFreeStorage();
            data[id] = new ClothPinInfo(toEntity, offset);

            Count += 1;

            return id;
        }

        public int AddPin(BlockPos blockPos, Vec3f offset)
        {
            int id = GetFreeStorage();
            data[id] = new ClothPinInfo(blockPos, offset);

            Count += 1;

            return id;
        }

        public ClothPinInfo GetPin(int id)
        {
            if (id >= 0 && id < data.Count)
            {
                return data[id];
            }
            else
            {
                return null;
            }
        }

        public void RemovePin(int id)
        {
            if (id >= 0 && id < data.Count)
            {
                data[id] = null;
                Count -= 1;
            }
        }

        public void restoreReferences(Entity entity)
        {
            foreach (var info in data)
            {
                if (info != null)
                {
                    if (entity.EntityId == info.pinnedToEntityId)
                    {
                        info.pinnedTo = entity;
                    }
                }
            }
        }

        public void restoreReferences(IWorldAccessor world)
        {
            foreach (var info in data)
            {
                if (info != null)
                {
                    if (info.pinnedToEntityId != 0)
                    {
                        info.pinnedTo = world.GetEntityById(info.pinnedToEntityId);
                    }
                }
            }
        }
    }

}

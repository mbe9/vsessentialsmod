﻿using System;
using System.Collections.Generic;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Config;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;

namespace Vintagestory.GameContent
{
    public class EntityPartitionChunk
    {
        public List<Entity>[] Entities;
        public List<Entity>[] NonInteractableEntities;

        public EntityPartitionChunk()
        {
            Entities = new List<Entity>[EntityPartitioning.partitionsLength * EntityPartitioning.partitionsLength];
            NonInteractableEntities = new List<Entity>[EntityPartitioning.partitionsLength * EntityPartitioning.partitionsLength];
        }

        public List<Entity> Add(Entity e, int gridIndex)
        {
            List<Entity> list = e.IsInteractable ? FetchOrCreateList(ref Entities[gridIndex]) : FetchOrCreateList(ref NonInteractableEntities[gridIndex]);
            list.Add(e);
            return list;
        }

        private List<Entity> FetchOrCreateList(ref List<Entity> list)
        {
            if (list == null) list = new List<Entity>(4);
            return list;
        }
    }

    public struct GridAndChunkIndex
    {
        public int GridIndex;
        public long ChunkIndex;

        public GridAndChunkIndex(int gridIndex, long chunkIndex)
        {
            this.GridIndex = gridIndex;
            this.ChunkIndex = chunkIndex;
        }
    }


    public class EntityPartitioning : ModSystem, IEntityPartitioning
    {
        public const int partitionsLength = 8;
        int gridSizeInBlocks;

        ICoreAPI api;
        ICoreClientAPI capi;
        ICoreServerAPI sapi;

        public Dictionary<long, EntityPartitionChunk> Partitions = new Dictionary<long, EntityPartitionChunk>();

        const int chunkSize = GlobalConstants.ChunkSize;
        int chunkMapSizeX;
        int chunkMapSizeZ;


        /// <summary>
        /// Updated every frame. The largest hitbox length of all loaded entities.
        /// </summary>
        public double LargestTouchDistance;

        public override double ExecuteOrder()
        {
            return 0;
        }

        public override bool ShouldLoad(EnumAppSide side)
        {
            return true;
        }
        

        public override void Start(ICoreAPI api)
        {
            this.api = api;

            gridSizeInBlocks = chunkSize / partitionsLength;
        }



        public override void StartClientSide(ICoreClientAPI api)
        {
            this.capi = api;
            api.Event.RegisterGameTickListener(OnClientTick, 32);
        }


        public override void StartServerSide(ICoreServerAPI api)
        {
            this.sapi = api;
            api.Event.RegisterGameTickListener(OnServerTick, 32);
            api.Event.PlayerSwitchGameMode += OnSwitchedGameMode;
        }

        private void OnClientTick(float dt)
        {
            partitionEntities(capi.World.LoadedEntities.Values);
        }

        private void OnServerTick(float dt)
        {
            partitionEntities(((CachingConcurrentDictionary<long, Entity>)sapi.World.LoadedEntities).Values);
        }

        void partitionEntities(ICollection<Entity> entities)
        {
            chunkMapSizeX = api.World.BlockAccessor.MapSizeX / chunkSize;
            chunkMapSizeZ = api.World.BlockAccessor.MapSizeZ / chunkSize;
            double largestTouchDistance = 0;

            Partitions.Clear();

            foreach (var val in entities)
            {
                if (val.IsInteractable) largestTouchDistance = Math.Max(largestTouchDistance, val.SelectionBox.XSize / 2);

                PartitionEntity(val);
            }
            this.LargestTouchDistance = largestTouchDistance;   // Only write to the field when we finished the operation, there could be 10k entities
        }


        private void PartitionEntity(Entity entity)
        {
            EntityPos pos = entity.SidedPos;

            int lgx = ((int)pos.X / gridSizeInBlocks) % partitionsLength;
            int lgz = ((int)pos.Z / gridSizeInBlocks) % partitionsLength;
            int gridIndex = lgz * partitionsLength + lgx;
            if (gridIndex < 0) return;    // entities could be outside the map edge

            long nowInChunkIndex3d = MapUtil.Index3dL((int)pos.X / chunkSize, (int)pos.Y / chunkSize, (int)pos.Z / chunkSize, chunkMapSizeX, chunkMapSizeZ);

            EntityPartitionChunk partition;
            if (!Partitions.TryGetValue(nowInChunkIndex3d, out partition))
            {
                Partitions[nowInChunkIndex3d] = partition = new EntityPartitionChunk();
            }

            var list = partition.Add(entity, gridIndex);
            if (entity is EntityPlayer ep) ep.entityListForPartitioning = list;
        }


        public void RePartitionPlayer(EntityPlayer entity)
        {
            entity.entityListForPartitioning?.Remove(entity);
            PartitionEntity(entity);
        }

        private void OnSwitchedGameMode(IServerPlayer player)
        {
            RePartitionPlayer(player.Entity);
        }

        [Obsolete("In version 1.18.2 and later, this returns Interactable entities only, so recommended to call GetNearestInteractableEntity() or GetNearestNonInteractableEntity() explicitly for clarity in the calling code")]
        public Entity GetNearestEntity(Vec3d position, double radius, ActionConsumable<Entity> matches = null)
        {
            return GetNearestEntity(position, radius, matches, true);
        }

        /// <summary>
        /// Search all nearby interactable entities (for which IsInteractable is true) to find the nearest one meeting the "matches" condition
        /// </summary>
        public Entity GetNearestInteractableEntity(Vec3d position, double radius, ActionConsumable<Entity> matches = null)
        {
            return GetNearestEntity(position, radius, matches, true);
        }

        /// <summary>
        /// Search all nearby interactable entities (for which IsInteractable is true) to find the nearest one meeting the "matches" condition
        /// </summary>
        public Entity GetNearestNonInteractableEntity(Vec3d position, double radius, ActionConsumable<Entity> matches = null)
        {
            return GetNearestEntity(position, radius, matches, false);
        }

        public Entity GetNearestEntity(Vec3d position, double radius, ActionConsumable<Entity> matches, bool interactable)
        {
            Entity nearestEntity = null;
            double radiusSq = radius * radius;
            double nearestDistanceSq = radiusSq;

            if (api.Side == EnumAppSide.Client)
            {
                WalkEntities(position, radius, (e) =>
                {
                    double distSq = e.Pos.SquareDistanceTo(position);

                    if (distSq < nearestDistanceSq && matches(e))
                    {
                        nearestDistanceSq = distSq;
                        nearestEntity = e;
                    }

                    return true;
                }, onIsInRangePartition, interactable);
            } else
            {
                WalkEntities(position, radius, (e) =>
                {
                    double distSq = e.ServerPos.SquareDistanceTo(position);

                    if (distSq < nearestDistanceSq && matches(e))
                    {
                        nearestDistanceSq = distSq;
                        nearestEntity = e;
                    }

                    return true;
                }, onIsInRangePartition, interactable);
            }

            return nearestEntity;
        }



        public delegate bool RangeTestDelegate(Entity e, Vec3d pos, double radiuSq);

        private bool onIsInRangeServer(Entity e, Vec3d pos, double radiusSq)
        {
            double dx = e.ServerPos.X - pos.X;
            double dy = e.ServerPos.Y - pos.Y;
            double dz = e.ServerPos.Z - pos.Z;

            return (dx * dx + dy * dy + dz * dz) < radiusSq;
        }

        private bool onIsInRangeClient(Entity e, Vec3d pos, double radiusSq)
        {
            double dx = e.Pos.X - pos.X;
            double dy = e.Pos.Y - pos.Y;
            double dz = e.Pos.Z - pos.Z;

            return (dx * dx + dy * dy + dz * dz) < radiusSq;
        }

        private bool onIsInRangePartition(Entity e, Vec3d pos, double radiusSq)
        {
            return true;
        }


        [Obsolete("In version 1.18.2 and later, this walks through Interactable entities only, so recommended to call WalkInteractableEntities() or WalkNonInteractableEntities() explicitly for clarity in the calling code")]
        public void WalkEntities(Vec3d centerPos, double radius, ActionConsumable<Entity> callback)
        {
            WalkEntities(centerPos, radius, callback, true);
        }

        /// <summary>
        /// This performs a entity search inside a spacially partioned search grid thats refreshed every 16ms, limited to Interactable entities only for performance reasons.
        /// This can be a lot faster for when there are thousands of entities on a small space. It is used by EntityBehaviorRepulseAgents to improve performance, because otherwise when spawning 1000 creatures nearby, it has to do 1000x1000 = 1mil search operations every frame
        /// A small search grid allows us to ignore most of those during the search.  Return false to stop the walk.
        /// </summary>
        /// <param name="centerPos"></param>
        /// <param name="radius"></param>
        /// <param name="callback">Return false to stop the walk</param>
        public void WalkInteractableEntities(Vec3d centerPos, double radius, ActionConsumable<Entity> callback)
        {
            WalkEntities(centerPos, radius, callback, true);
        }

        /// <summary>
        /// This performs a entity search inside a spacially partioned search grid thats refreshed every 16ms, here limited to Non-Interactable entities only
        /// This can be a lot faster for when there are thousands of entities on a small space. It is used by AITaskSeekAndFindFood to improve performance, it ignores creatures and looks only for non-interactable entities such as dropped items
        /// A small search grid allows us to ignore most entities during the search.  Return false to stop the walk.
        /// </summary>
        /// <param name="centerPos"></param>
        /// <param name="radius"></param>
        /// <param name="callback">Return false to stop the walk</param>
        public void WalkNonInteractableEntities(Vec3d centerPos, double radius, ActionConsumable<Entity> callback)
        {
            WalkEntities(centerPos, radius, callback, false);
        }

        private void WalkEntities(Vec3d centerPos, double radius, ActionConsumable<Entity> callback, bool interactable)
        {
            if (api.Side == EnumAppSide.Client)
            {
                WalkEntities(centerPos, radius, callback, onIsInRangeClient, interactable);
            } else
            {
                WalkEntities(centerPos, radius, callback, onIsInRangeServer, interactable);
            }
        }

        /// <summary>
        /// Same as <see cref="WalkInteractableEntities(Vec3d,double,Vintagestory.API.Common.ActionConsumable{Vintagestory.API.Common.Entities.Entity}(Vintagestory.API.Common.Entities.Entity))"/> but does no exact radius distance check, walks all entities that it finds in the grid
        /// </summary>
        /// <param name="centerPos"></param>
        /// <param name="radius"></param>
        /// <param name="callback"></param>
        public void WalkEntityPartitions(Vec3d centerPos, double radius, ActionConsumable<Entity> callback)
        {
            WalkEntities(centerPos, radius, callback, onIsInRangePartition, true);
        }


        private void WalkEntities(Vec3d centerPos, double radius, ActionConsumable<Entity> callback, RangeTestDelegate onRangeTest, bool interactable)
        {
            int gridXMax = api.World.BlockAccessor.MapSizeX / gridSizeInBlocks - 1;
            int cyTop = api.World.BlockAccessor.MapSizeY / chunkSize - 1;
            int gridZMax = api.World.BlockAccessor.MapSizeZ / gridSizeInBlocks - 1;

            int mingx = (int)GameMath.Clamp((centerPos.X - radius) / gridSizeInBlocks, 0, gridXMax);
            int maxgx = (int)GameMath.Clamp((centerPos.X + radius) / gridSizeInBlocks, 0, gridXMax);

            int mincy = (int)GameMath.Clamp((centerPos.Y - radius) / chunkSize, 0, cyTop);
            int maxcy = (int)GameMath.Clamp((centerPos.Y + radius) / chunkSize, 0, cyTop);

            int mingz = (int)GameMath.Clamp((centerPos.Z - radius) / gridSizeInBlocks, 0, gridZMax);
            int maxgz = (int)GameMath.Clamp((centerPos.Z + radius) / gridSizeInBlocks, 0, gridZMax);

            double radiusSq = radius * radius;

            EntityPartitionChunk partitionChunk = null;
            long index3d;
            long lastIndex3d = -1;

            for (int cy = mincy; cy <= maxcy; cy++)
            {
                for (int gridX = mingx; gridX <= maxgx; gridX++)
                {
                    int cx = gridX * gridSizeInBlocks / chunkSize;
                    int lgx = gridX % partitionsLength;

                    for (int gridZ = mingz; gridZ <= maxgz; gridZ++)
                    {
                        int cz = gridZ * gridSizeInBlocks / chunkSize;

                        index3d = MapUtil.Index3dL(cx, cy, cz, chunkMapSizeX, chunkMapSizeZ);
                        if (index3d != lastIndex3d)
                        {
                            lastIndex3d = index3d;
                            Partitions.TryGetValue(index3d, out partitionChunk);
                        }
                        if (partitionChunk == null) continue;

                        int index = (gridZ % partitionsLength) * partitionsLength + lgx;
                        List<Entity> entities = interactable ? partitionChunk.Entities[index] : partitionChunk.NonInteractableEntities[index];
                        if (entities == null) continue;

                        foreach (Entity entity in entities)
                        {
                            if (onRangeTest(entity, centerPos, radiusSq) && !callback(entity))   // continues looping entities and calling the callback, but stops if the callback returns false
                            {
                                return;
                            }
                        }
                    }
                }
            }
        }

    }
}

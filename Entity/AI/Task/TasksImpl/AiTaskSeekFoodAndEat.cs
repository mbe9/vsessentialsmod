﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Vintagestory.API;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace Vintagestory.GameContent
{
    class FailedAttempt
    {
        public long LastTryMs;
        public int Count;
    }

    public class LooseItemFoodSource : IAnimalFoodSource
    {
        EntityItem entity;

        public LooseItemFoodSource(EntityItem entity)
        {
            this.entity = entity;
        }

        public Vec3d Position => entity.ServerPos.XYZ;

        public string Type => "food";

        public float ConsumeOnePortion()
        {
            entity.World.SpawnCubeParticles(entity.ServerPos.XYZ, entity.Itemstack, 0.25f, 20);

            entity.Itemstack.StackSize--;
            if (entity.Itemstack.StackSize <= 0) entity.Die();
            return 0.5f;
        }

        public bool IsSuitableFor(Entity entity)
        {
            return true;
        }
    }

    public class AiTaskSeekFoodAndEat : AiTaskBase
    {
        AssetLocation eatSound;

        POIRegistry porregistry;
        IAnimalFoodSource targetPoi;

        float moveSpeed = 0.02f;
        long stuckatMs = 0;
        bool nowStuck = false;

        float eatTime = 1f;

        float eatTimeNow = 0;
        bool soundPlayed = false;
        bool doConsumePortion = true;

        bool eatLooseItems;
        HashSet<EnumFoodCategory> eatItemCategories = new HashSet<EnumFoodCategory>();

        float quantityEaten;

        AnimationMetaData eatAnimMeta;

        Dictionary<IAnimalFoodSource, FailedAttempt> failedSeekTargets = new Dictionary<IAnimalFoodSource, FailedAttempt>();

        public AiTaskSeekFoodAndEat(EntityAgent entity) : base(entity)
        {
            porregistry = entity.Api.ModLoader.GetModSystem<POIRegistry>();
        }

        public override void LoadConfig(JsonObject taskConfig, JsonObject aiConfig)
        {
            base.LoadConfig(taskConfig, aiConfig);

            if (taskConfig["eatSound"] != null)
            {
                string eatsoundstring = taskConfig["eatSound"].AsString(null);
                if (eatsoundstring != null) eatSound = new AssetLocation(eatsoundstring).WithPathPrefix("sounds/");
            }
            
            if (taskConfig["movespeed"] != null)
            {
                moveSpeed = taskConfig["movespeed"].AsFloat(0.02f);
            }
            
            if (taskConfig["eatTime"] != null)
            {
                eatTime = taskConfig["eatTime"].AsFloat(1.5f);
            }

            if (taskConfig["doConsumePortion"] != null)
            {
                doConsumePortion = taskConfig["doConsumePortion"].AsBool(true);
            }

            if (taskConfig["eatLooseItems"] != null)
            {
                eatLooseItems = taskConfig["eatLooseItems"].AsBool(true);
            }

            if (taskConfig["eatItemCategories"] != null)
            {
                foreach (var val in taskConfig["eatItemCategories"].AsArray<EnumFoodCategory>(new EnumFoodCategory[0]))
                {
                    eatItemCategories.Add(val);
                }
            }

            if (taskConfig["eatAnimation"].Exists)
            {
                eatAnimMeta = new AnimationMetaData()
                {
                    Code = taskConfig["eatAnimation"].AsString()?.ToLowerInvariant(),
                    Animation = taskConfig["eatAnimation"].AsString()?.ToLowerInvariant(),
                    AnimationSpeed = taskConfig["eatAnimationSpeed"].AsFloat(1f)
                }.Init();
            }
        }

        public override bool ShouldExecute()
        {
            if (entity.World.Rand.NextDouble() < 0.005) return false;
            if (cooldownUntilMs > entity.World.ElapsedMilliseconds) return false;
            if (cooldownUntilTotalHours > entity.World.Calendar.TotalHours) return false;
            if (whenInEmotionState != null && !entity.HasEmotionState(whenInEmotionState)) return false;
            if (whenNotInEmotionState != null && entity.HasEmotionState(whenNotInEmotionState)) return false;

            EntityBehaviorMultiply bh = entity.GetBehavior<EntityBehaviorMultiply>();
            if (bh != null && !bh.ShouldEat && entity.World.Rand.NextDouble() < 0.996) return false; // 0.4% chance go to the food source anyway just because (without eating anything).

            targetPoi = null;
            
            entity.World.Api.ModLoader.GetModSystem<EntityPartitioning>().WalkEntities(entity.ServerPos.XYZ, 3, (e) =>
            {
                if (e is EntityItem)
                {
                    EntityItem ei = (EntityItem)e;
                    EnumFoodCategory? cat = ei.Itemstack?.Collectible?.NutritionProps?.FoodCategory;
                    if (cat != null && eatItemCategories.Contains((EnumFoodCategory)cat))
                    {
                        targetPoi = new LooseItemFoodSource(ei);
                        return false;
                    }
                }

                return true;
            });

            if (targetPoi == null)
            {
                targetPoi = porregistry.GetNearestPoi(entity.ServerPos.XYZ, 48, (poi) =>
                {
                    if (poi.Type != "food") return false;
                    IAnimalFoodSource foodPoi;

                    if ((foodPoi = poi as IAnimalFoodSource)?.IsSuitableFor(entity) == true)
                    {
                        FailedAttempt attempt;
                        failedSeekTargets.TryGetValue(foodPoi, out attempt);
                        if (attempt == null || (attempt.Count < 4 || attempt.LastTryMs < world.ElapsedMilliseconds - 60000)) return true;
                    }

                    return false;
                }) as IAnimalFoodSource;
            }
            

            return targetPoi != null;
        }



        public float MinDistanceToTarget()
        {
            return System.Math.Max(0.5f, (entity.CollisionBox.X2 - entity.CollisionBox.X1) / 2 + 0.05f);
        }

        public override void StartExecute()
        {
            base.StartExecute();
            stuckatMs = -9999;
            nowStuck = false;
            soundPlayed = false;
            eatTimeNow = 0;
            pathTraverser.GoTo(targetPoi.Position, moveSpeed, MinDistanceToTarget(), OnGoalReached, OnStuck);
        }

        public override bool ContinueExecute(float dt)
        {
            Vec3d pos = targetPoi.Position;

            pathTraverser.CurrentTarget.X = pos.X;
            pathTraverser.CurrentTarget.Y = pos.Y;
            pathTraverser.CurrentTarget.Z = pos.Z;

            Cuboidd targetBox = entity.CollisionBox.ToDouble().Translate(entity.ServerPos.X, entity.ServerPos.Y, entity.ServerPos.Z);
            double distance = targetBox.ShortestDistanceFrom(pos);          

            float minDist = MinDistanceToTarget();

            if (distance < minDist)
            {
                EntityBehaviorMultiply bh = entity.GetBehavior<EntityBehaviorMultiply>();
                if (bh != null && !bh.ShouldEat) return false;

                if (targetPoi.IsSuitableFor(entity) != true) return false;
                
                if (eatAnimMeta != null)
                {
                    if (animMeta != null)
                    {
                        entity.AnimManager.StopAnimation(animMeta.Code);
                    }

                    entity.AnimManager.StartAnimation(eatAnimMeta.Code);
                }

                eatTimeNow += dt;

                if (eatTimeNow > eatTime * 0.75f && !soundPlayed)
                {
                    soundPlayed = true;
                    if (eatSound != null) entity.World.PlaySoundAt(eatSound, entity, null, true, 16, 1);
                }


                if (eatTimeNow >= eatTime)
                {
                    ITreeAttribute tree = entity.WatchedAttributes.GetTreeAttribute("hunger");
                    if (tree == null) entity.WatchedAttributes["hunger"] = tree = new TreeAttribute();

                    if (doConsumePortion)
                    {
                        float sat = targetPoi.ConsumeOnePortion();
                        quantityEaten += sat;
                        tree.SetFloat("saturation", sat + tree.GetFloat("saturation", 0));
                        entity.WatchedAttributes.MarkPathDirty("hunger");
                    }
                    else quantityEaten = 1;

                    failedSeekTargets.Remove(targetPoi);

                    return false;
                }
            }


            if (nowStuck && entity.World.ElapsedMilliseconds > stuckatMs + eatTime * 1000)
            {
                return false;
            }


            return true;
        }


        float GetSaturation()
        {
            ITreeAttribute tree = entity.WatchedAttributes.GetTreeAttribute("hunger");
            if (tree == null) entity.WatchedAttributes["hunger"] = tree = new TreeAttribute();

            return tree.GetFloat("saturation");
        }


        public override void FinishExecute(bool cancelled)
        {
            base.FinishExecute(cancelled);
            pathTraverser.Stop();

            if (eatAnimMeta != null)
            {
                entity.AnimManager.StopAnimation(eatAnimMeta.Code);
            }

            if (cancelled)
            {
                cooldownUntilTotalHours = 0;
            }

            if (quantityEaten < 1)
            {
                cooldownUntilTotalHours = 0;
            } else
            {
                quantityEaten = 0;
            }
        }



        private void OnStuck()
        {
            stuckatMs = entity.World.ElapsedMilliseconds;
            nowStuck = true;

            FailedAttempt attempt = null;
            failedSeekTargets.TryGetValue(targetPoi, out attempt);
            if (attempt == null)
            {
                failedSeekTargets[targetPoi] = attempt = new FailedAttempt();
            }

            attempt.Count++;
            attempt.LastTryMs = world.ElapsedMilliseconds;
            
        }

        private void OnGoalReached()
        {
            pathTraverser.Active = true;

            failedSeekTargets.Remove(targetPoi);
        }


    }
}

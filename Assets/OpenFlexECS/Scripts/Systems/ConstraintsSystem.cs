using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace OpenFlex.ECS
{
    [UpdateAfter(typeof(PredictPositionsSystem))]
    public class ConstraintsSystem : JobComponentSystem
    {
        NativeMultiHashMap<int, int> hashMap;
        public struct Data
        {
            public int Length;
            public ComponentDataArray<PredictedPositions> predPositions;
            public ComponentDataArray<DeltaPositions> deltaPositions;
            public ComponentDataArray<MassInv> massesInv;
           // public FixedArrayArray<DistanceConstraintUni> distCnstrs;
        }

        public struct DistanceConstraints
        {
            public int Length;
            public ComponentDataArray<DistanceConstraint> distConstraints;
        }

        [Inject]
        private Data m_Data;

        [Inject]
        private DistanceConstraints m_Constraints;

        //protected override void OnCreateManager(int capacity)
        //{
        //    m_BoidGroup = GetComponentGroup(
        //        ComponentType.ReadOnly(typeof(Boid)),
        //        ComponentType.ReadOnly(typeof(Position)),
        //        typeof(Heading));
        //    m_TargetGroup = GetComponentGroup(
        //        ComponentType.ReadOnly(typeof(BoidTarget)),
        //        ComponentType.ReadOnly(typeof(Position)));
        //    m_ObstacleGroup = GetComponentGroup(
        //        ComponentType.ReadOnly(typeof(BoidObstacle)),
        //        ComponentType.ReadOnly(typeof(Position)));
        //}

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            hashMap = new NativeMultiHashMap<int, int>(m_Data.Length * 27, Allocator.TempJob);




            for (int cnstrIter = 0; cnstrIter < 3; cnstrIter++)
            {
                hashMap.Clear();
                var hashPositionsJob = new PositionBasedDynamicsJobsECS.HashPositionsNeighbours
                {
                    cellRadius = 1f, //TODO verify this
                    positions = m_Data.predPositions,
                    hashMap = hashMap
                };
                inputDeps = hashPositionsJob.Schedule(m_Data.Length, 64, inputDeps);


                //var clearDeltasJob = new PositionBasedConstraintsJobs.ClearDeltaPositions()
                //{
                //    deltaPositions = m_Data.deltaPositions
                //};
                //inputDeps = clearDeltasJob.Schedule(m_Data.Length, 64, inputDeps);

                var projectCollisions = new PositionBasedDynamicsJobsECS.ProjectInterParticlesCollisionsParallelJob
                {
                    positions = m_Data.predPositions,
                    deltas = m_Data.deltaPositions,
                    hashMap = hashMap,
                    radius = 0.5f,
                    cellRadius = 1f, //TODO verify this
                    stiffness = 1.0f
                };
                inputDeps = projectCollisions.Schedule(m_Data.Length, 64, inputDeps);

                var addDeltasToPredictedJob = new PositionBasedDynamicsJobsECS.AddDeltasToPredictedPositions()
                {
                    predPositions = m_Data.predPositions,
                    deltaPositions = m_Data.deltaPositions
                };
                inputDeps = addDeltasToPredictedJob.Schedule(m_Data.Length, 64, inputDeps);

                //var projectCollisions = new PositionBasedConstraintsJobs.ProjectParticlesVsParticlesCollisionsBruteForceJob
                //{
                //    positions = m_Data.predPositions,
                //    length = m_Data.Length,
                //    radius = 0.5f,
                //    stiffness = 1.0f
                //};
                //inputDeps = projectCollisions.Schedule(inputDeps);


                //var distanceConstrsintsJob = new PositionBasedConstraintsJobs.ProjectDistanceConstraints()
                //{
                //    length = m_Constraints.Length,
                //    distanceConstraints = m_Constraints.distConstraints,
                //    predPositions = m_Data.predPositions,
                //    massesInv = m_Data.massesInv,
                //    stiffness = 1f
                //};
                //inputDeps = distanceConstrsintsJob.Schedule(inputDeps);

                //var distanceConstrsintsParallelJob = new PositionBasedConstraintsJobs.ProjectDistanceConstraintsParallelJob()
                //{
                //    distanceConstraints = m_Data.distCnstrs,
                //    deltaPositions = m_Data.deltaPositions,
                //    predPositions = m_Data.predPositions,
                //    massesInv = m_Data.massesInv,
                //    stiffness = 1f
                //};
                //inputDeps = distanceConstrsintsParallelJob.Schedule(m_Data.Length, 64, inputDeps);

                //var addDeltasToPredictedJob = new PositionBasedConstraintsJobs.AddDeltasToPredictedPositions()
                //{
                //    predPositions = m_Data.predPositions,
                //    deltaPositions = m_Data.deltaPositions
                //};
                //inputDeps = addDeltasToPredictedJob.Schedule(m_Data.Length, 64, inputDeps);

                var projectToBoundsJob = new PositionBasedDynamicsJobsECS.ProjectToBounds()
                {
                    maxBounds = new float3(20, 20, 20),
                    minBounds = new float3(-20, 0, -20),
                    radius = 0.5f,
                    predPositions = m_Data.predPositions
                };
                inputDeps =  projectToBoundsJob.Schedule(m_Data.Length, 64, inputDeps);

                inputDeps.Complete();
            }
      
            hashMap.Dispose();
            return inputDeps;
        }

    }
}
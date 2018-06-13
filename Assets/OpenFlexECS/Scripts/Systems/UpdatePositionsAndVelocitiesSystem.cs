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
    [UpdateAfter(typeof(ConstraintsSystem))]
   // [UpdateBefore(typeof(FixedUpdate))]
    public class UpdatePositionsAndVelocitiesSystem : JobComponentSystem
    {
        public struct Data
        {
            public int Length;
            public ComponentDataArray<Position> positions;
            public ComponentDataArray<PredictedPositions> predPositions;
            public ComponentDataArray<Velocity> velocities;
            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;
        }

        [Inject]
        private Data m_Data;

        [BurstCompile]
        struct PredictPositionsJob : IJobParallelFor
        {
            public float dtInv;


            [ReadOnly]
            public ComponentDataArray<PredictedPositions> predPositions;

            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;

            public ComponentDataArray<Position> positions;

            public ComponentDataArray<Velocity> velocities;



            public void Execute(int i)
            {
                if (massesInv[i].Value != 0.0)
                {
                    float3 vel = (predPositions[i].Value - positions[i].Value) * dtInv;

                    velocities[i] = new Velocity { Value = vel };
                    positions[i] = new Position { Value = predPositions[i].Value };
                }
            }
        }



        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var predictPositionsJob = new PredictPositionsJob()
            {
                dtInv = 1.0f / Time.fixedDeltaTime,
      
                positions = m_Data.positions,
                predPositions = m_Data.predPositions,
                velocities = m_Data.velocities,
                massesInv = m_Data.massesInv
            };
            return predictPositionsJob.Schedule(m_Data.Length, 64, inputDeps);
         
        }
    }
}
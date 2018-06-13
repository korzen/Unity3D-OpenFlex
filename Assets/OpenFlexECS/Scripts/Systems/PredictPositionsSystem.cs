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
    
    public class PredictPositionsSystem : JobComponentSystem
    {
        public struct Data
        {
            public int Length;
            [ReadOnly]
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
            public float dt;
            public float damping;

            public float3 acceleration;

            [ReadOnly]
            public ComponentDataArray<Position> positions;
            public ComponentDataArray<PredictedPositions> predPositions;
            public ComponentDataArray<Velocity> velocities;
            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;


            public void Execute(int i)
            {
                if (massesInv[i].Value != 0.0)
                {
                    float3 vel = velocities[i].Value + acceleration * dt;
                    vel *= damping;
                    float3 predPos = positions[i].Value + vel * dt;

                    velocities[i] = new Velocity { Value = vel };
                    predPositions[i] = new PredictedPositions { Value = predPos };
                }
            }
        }



        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var predictPositionsJob = new PredictPositionsJob()
            {
                dt = Time.fixedDeltaTime,
                damping = 1f - 0.005f,
                acceleration = Vector3.down * 9.81f, //gravity
                positions = m_Data.positions,
                predPositions = m_Data.predPositions,
                velocities = m_Data.velocities,
                massesInv = m_Data.massesInv
            };
            return predictPositionsJob.Schedule(m_Data.Length, 64, inputDeps);
         
        }
    }
}
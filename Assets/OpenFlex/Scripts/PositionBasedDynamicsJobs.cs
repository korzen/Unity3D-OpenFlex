using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Mathematics.Experimental;
using Unity.Burst;

namespace OpenFlex
{
    public class PositionBasedDynamicsJobs
    {


        [BurstCompile]
        public struct PredictPositionsJob : IJobParallelFor
        {
            public float dt;

            public Vector4 acceleration;
            [ReadOnly]
            public NativeArray<Vector4> positions;
            public NativeArray<Vector4> predPositions;
            public NativeArray<Vector4> velocities;
            

            public void Execute(int i)
            {
                if (positions[i].w != 0.0)
                {
                    velocities[i] += acceleration * dt;
                    predPositions[i] = positions[i] + velocities[i] * dt;
                }
            }
        }

        [BurstCompile]
        public struct UpdateVelocityiesJob : IJobParallelFor
        {
            public float dtInv;

            [ReadOnly]
            public NativeArray<Vector4> predPositions;
            public NativeArray<Vector4> positions;
            public NativeArray<Vector4> velocities;


            public void Execute(int i)
            {
                if (positions[i].w != 0.0)
                {
                    velocities[i] = (predPositions[i] - positions[i]) * dtInv;
                    positions[i] = predPositions[i];
                }
            }
        }

        [BurstCompile]
        public struct AddToArrayParallelForJob : IJobParallelFor
        {

          //  [WriteOnly]
            public NativeArray<Vector4> arrA;

            [ReadOnly]
            public NativeArray<Vector4> arrB;


            public void Execute(int i)
            {
                arrA[i] += arrB[i];
            }
        }

        [BurstCompile]
        public struct ProjectParticlesFloorBoundsJob : IJobParallelFor
        {
            public float floorLevel;
            public float radius;

            public NativeArray<Vector4> predPositions;
            public NativeArray<Vector4> positions;


            public void Execute(int i)
            {
                float y = floorLevel + radius;
                Vector4 predPos = predPositions[i];
                if (predPos.y < y)
                {
                    predPos.y = y;

                    predPositions[i] = predPos;
                    positions[i] = predPos;
                }
                
            }
        }

        [BurstCompile]
        public struct ProjectParticlesToBoundsParallelForJob : IJobParallelFor
        {
            public Vector3 minBounds;
            public Vector3 maxBounds;
            public float radius;

            public NativeArray<Vector4> predPositions;
            //public NativeArray<Vector4> positions;


            public void Execute(int i)
            {
                Vector4 pos = predPositions[i];
                if (pos.x < minBounds.x + radius)
                {
                    pos.x = minBounds.x + radius;
                }
                else if (pos.x > maxBounds.x - radius)
                {
                    pos.x = maxBounds.x - radius;
                }

                if (pos.y < minBounds.y + radius)
                {
                    pos.y = minBounds.y + radius;
                }
                else if (pos.y > maxBounds.y - radius)
                {
                    pos.y = maxBounds.y - radius;
                }

                if (pos.z < minBounds.z + radius)
                {
                    pos.z = minBounds.z + radius;
                }
                else if (pos.z > maxBounds.z - radius)
                {
                    pos.z = maxBounds.z - radius;
                }

                predPositions[i] = pos;
                // positions[i] = pos;

            }
        }

       

        [BurstCompile]
        public struct ProjectParticlesVsParticlesCollisionsJob : IJob
        {
            public int particlesCount;
            public float radius;
            public float kS;

            public int maxNeighboursPerParticle;

            public NativeArray<Vector4> positions;

            [ReadOnly]
            public NativeArray<int> particlesNeighbours;

            [ReadOnly]
            public NativeArray<int> particlesNeighboursCount;

            public void Execute()
            {
                float radiusSum = radius + radius;
                float radiusSumSq = radiusSum * radiusSum;

                for (int idA = 0; idA < particlesCount; idA++)
                {

                    for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                    {
                        int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                        float4 dir = positions[idA] - positions[idB];
                        dir.w = 0;
                        float distanceSq = math.lengthSquared(dir);

                        if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                            continue;

                        float distance = math.sqrt(distanceSq);

                        Vector4 dP = (distance - radiusSum) * (dir / distance) * kS;

                        positions[idA] -= dP;
                        positions[idB] += dP;
                    }


                }
            }
        }

        [BurstCompile]
        public struct ProjectParticlesVsParticlesCollisionsParallelForJob : IJobParallelFor
        {

            public float radius;
            public float kS;

            public int maxNeighboursPerParticle;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            [ReadOnly]
            public NativeArray<int> particlesNeighbours;

            [ReadOnly]
            public NativeArray<int> particlesNeighboursCount;


            [WriteOnly]
            public NativeArray<Vector4> deltaPositions;

            public void Execute(int idA)
            {
                float radiusSum = radius + radius;
                float radiusSumSq = radiusSum * radiusSum;
                float4 dP = new float4();
 
                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                    float4 dir = positions[idA] - positions[idB];
                    dir.w = 0;
                    float distanceSq = math.lengthSquared(dir);

                    if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                        continue;

                    float distance = math.sqrt(distanceSq);

                    dP -= (distance - radiusSum) * (dir / distance) * kS;
                }

                deltaPositions[idA] = dP;


            }
        }

    }
}
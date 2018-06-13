using System;

using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace OpenFlex.ECS
{
    public class PositionBasedDynamicsJobsECS
    {

        [BurstCompile]
        public struct HashPositions : IJobParallelFor
        {
            [ReadOnly]
            public ComponentDataArray<PredictedPositions> positions;
            public NativeMultiHashMap<int, int>.Concurrent hashMap;
            public float cellRadius;

            public void Execute(int index)
            {
                var hash = GridHash.Hash(positions[index].Value, cellRadius);
                hashMap.Add(hash, index);
            }
        }

        /// <summary>
        /// Hashes the position to all 27 cell (3x3x3, exact cell + 26 neighbouring cells)
        /// </summary>
        [BurstCompile]
        public struct HashPositionsNeighbours : IJobParallelFor
        {

            [ReadOnly]
            public ComponentDataArray<PredictedPositions> positions;
            public NativeMultiHashMap<int, int>.Concurrent hashMap;
            public float cellRadius;

            public void Execute(int index)
            {
                int3 pos = GridHash.Quantize(positions[index].Value, cellRadius);

                //TODO compare perf
                //for (int i = 0; i < 27; i++)
                //{
                //    int3 offset = GridHash.cellOffsets27[i];
                //    int hash = GridHash.Hash(pos + offset);
                //    hashMap.Add(hash, index);
                //}

                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            int3 offset = new int3(x, y, z);
                            int hash = GridHash.Hash(pos + offset);
                            hashMap.Add(hash, index);
                        }
                    }
                }
            }
        }

        [BurstCompile]
        public struct ProjectInterParticlesCollisionsJob : IJob 
        {
            public int length;
            public float radius;
            public float cellRadius;
            public float stiffness;

            public ComponentDataArray<PredictedPositions> positions;

            [WriteOnly]
            public NativeMultiHashMap<int, int> hashMap;
            

            public void Execute()
            {
                float radiusSum = radius + radius;
                float radiusSumSq = radiusSum * radiusSum;

                for (int idA = 0; idA < length; idA++)
                {
                    float3 posA = positions[idA].Value;
                    int3 pos = GridHash.Quantize(posA, cellRadius);
                    int hash = GridHash.Hash(pos);

                    NativeMultiHashMapIterator<int> iterator;
                    int idB; // neighbourId
                    var found = hashMap.TryGetFirstValue(hash, out idB, out iterator);

                    while (found)
                    {
                        
                        float3 posB = positions[idB].Value;

                        float3 dir = posA - posB;
     
                        float distanceSq = math.lengthSquared(dir);

                        if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                        {
                            found = hashMap.TryGetNextValue(out idB, ref iterator);
                            continue;
                        }

                        float distance = math.sqrt(distanceSq);
                        float massCorr = 0.5f;
                        float3 dP = (distance - radiusSum) * (dir / distance) * stiffness * massCorr;

                        positions[idA] = new PredictedPositions { Value = posA - dP };
                        positions[idB] = new PredictedPositions { Value = posB + dP };

                        found = hashMap.TryGetNextValue(out idB, ref iterator);
                    }
                }
            }
        }

        [BurstCompile]
        public struct ProjectInterParticlesCollisionsParallelJob : IJobParallelFor
        {
            
            public float radius;
            public float cellRadius;
            public float stiffness;

            [ReadOnly]
            public ComponentDataArray<PredictedPositions> positions;

            [WriteOnly]
            public ComponentDataArray<DeltaPositions> deltas;

            [ReadOnly]
            public NativeMultiHashMap<int, int> hashMap;

            public void Execute(int idA)
            {
                float radiusSum = radius + radius;
                float radiusSumSq = radiusSum * radiusSum;

                float3 posA = positions[idA].Value;
                int3 pos = GridHash.Quantize(posA, cellRadius);
                int hash = GridHash.Hash(pos);

                NativeMultiHashMapIterator<int> iterator;
                int idB; // neighbourId
                var found = hashMap.TryGetFirstValue(hash, out idB, out iterator);

                float3 delta = new float3(0, 0, 0);
                int cnstrsCount = 0;
                while (found)
                {

                    float3 posB = positions[idB].Value;

                    float3 dir = posA - posB;

                    float distanceSq = math.lengthSquared(dir);

                    if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                    {
                        found = hashMap.TryGetNextValue(out idB, ref iterator);
                        continue;
                    }

                    float distance = math.sqrt(distanceSq);
                    float massCorr = 0.5f;
                    float3 dP = (distance - radiusSum) * (dir / distance) * stiffness * massCorr;
                    delta -= dP;
                    cnstrsCount++;
                    
                    found = hashMap.TryGetNextValue(out idB, ref iterator);
                }


                deltas[idA] = new DeltaPositions
                {
                    //Delta = math.select(default(float3), delta / cnstrsCount,  cnstrsCount > 0),
                    Delta = delta,
                    ConstraintsCount = cnstrsCount
                };
            }
        }

        [BurstCompile]
        public struct ProjectInterParticlesCollisionsBruteForceJob : IJob
        {
            public int length;
            public float radius;
            public float stiffness;

            public ComponentDataArray<PredictedPositions> positions;



            public void Execute()
            {
                float radiusSum = radius + radius;
                float radiusSumSq = radiusSum * radiusSum;

                for (int idA = 0; idA < length; idA++)
                {
                    float3 posA = positions[idA].Value;

                    for (int idB = idA+1; idB < length; idB++)
                    {

                        float3 posB = positions[idB].Value;

                        float3 dir = posA - posB;

                        float distanceSq = math.lengthSquared(dir);

                        if (distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                        {
                            continue;
                        }

                        float distance = math.sqrt(distanceSq);
                        float massCorr = 0.5f;
                        float3 dP = (distance - radiusSum) * (dir / distance) * stiffness * massCorr;

                        positions[idA] = new PredictedPositions { Value = posA - dP };
                        positions[idB] = new PredictedPositions { Value = posB + dP };
                    }
         
                }
            }
        }

        [BurstCompile]
        public struct ClearDeltaPositions : IJobParallelFor
        {
            [WriteOnly]
            public ComponentDataArray<DeltaPositions> deltaPositions;

            public void Execute(int i)
            {
                deltaPositions[i] = new DeltaPositions { Delta = new float3(0), ConstraintsCount = 0 };
            }
        }

        [BurstCompile]
        public struct AddDeltasToPredictedPositions : IJobParallelFor
        {

            public ComponentDataArray<PredictedPositions> predPositions;

            [ReadOnly]
            public ComponentDataArray<DeltaPositions> deltaPositions;

            public void Execute(int i)
            {
                float3 pos = predPositions[i].Value + deltaPositions[i].Delta ;
                predPositions[i] = new PredictedPositions { Value = pos };
            }
        }

        [BurstCompile]
        public struct AddDeltasToPredictedPositionsDivByCount : IJobParallelFor
        {

            public ComponentDataArray<PredictedPositions> predPositions;

            [ReadOnly]
            public ComponentDataArray<DeltaPositions> deltaPositions;

            public void Execute(int i)
            {
                if (deltaPositions[i].ConstraintsCount > 0)
                {
                    float3 pos = predPositions[i].Value + (deltaPositions[i].Delta / deltaPositions[i].ConstraintsCount);
                    predPositions[i] = new PredictedPositions { Value = pos };
                }
            }
        }

        [BurstCompile]
        public struct ProjectDistanceConstraints : IJob
        {
            public int length;
            public float stiffness;

            [ReadOnly]
            public ComponentDataArray<DistanceConstraint> distanceConstraints;

            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;
            public ComponentDataArray<PredictedPositions> predPositions;
            

            public void Execute()
            {
                for (int i = 0; i < length; i++)
                {


                    DistanceConstraint cnstr = distanceConstraints[i];

                    float3 predPosA = predPositions[cnstr.idA].Value;
                    float3 predPosB = predPositions[cnstr.idB].Value;

                    float massInvA = massesInv[cnstr.idA].Value;
                    float massInvB = massesInv[cnstr.idB].Value;

                    float3 normal = predPosA - predPosB;
                    float length = math.length(normal.xyz);

                    float invMass = massInvA + massInvB;
                    if (invMass <= math.epsilon_normal || length <= math.epsilon_normal)
                        return;

                    float3 dP = (1.0f / invMass) * (length - cnstr.restLength) * (normal / length) * stiffness;

                    predPositions[cnstr.idA] = new PredictedPositions { Value = predPosA - dP * massInvA };
                    predPositions[cnstr.idB] = new PredictedPositions { Value = predPosB + dP * massInvB };
                }
            }
        }


        [BurstCompile]
        public struct ProjectDistanceConstraintsParallelJob : IJobParallelFor
        {

            public float stiffness;

            [ReadOnly]
            public FixedArrayArray<DistanceConstraintUni> distanceConstraints;

            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;

            [ReadOnly]
            public ComponentDataArray<PredictedPositions> predPositions;

            [WriteOnly]
            public ComponentDataArray<DeltaPositions> deltaPositions;


            public void Execute(int idA)
            {
                float3 predPosA = predPositions[idA].Value;
                float massInvA = massesInv[idA].Value;
         
                NativeArray<DistanceConstraintUni> cnstrsA = distanceConstraints[idA];

                int cnstrCount = 0;
                float3 delta = new float3(0, 0, 0);
                for (int i = 0; i < 34; i++)
                {
                    DistanceConstraintUni cnstr = cnstrsA[i];
                    if (cnstr.otherId == -1)
                        break;

                    float3 predPosB = predPositions[cnstr.otherId].Value;
                    float massInvB = massesInv[cnstr.otherId].Value;

                    float3 normal = predPosA - predPosB;
                    float length = math.length(normal.xyz);

                    float invMass = massInvA + massInvB;
                    if (invMass <= math.epsilon_normal || length <= math.epsilon_normal)
                        return;

                    float3 dP = (1.0f / invMass) * (length - cnstr.restLength) * (normal / length) * stiffness;
                    delta -= dP * massInvA;
                    cnstrCount++;
                }

                deltaPositions[idA] = new DeltaPositions { Delta = delta / cnstrCount, ConstraintsCount = cnstrCount };
            }
        }

        [BurstCompile]
        public struct ProjectDistanceConstraintsParallelJob2 : IJob
        {
            public int length;
            public float stiffness;

            [ReadOnly]
            public FixedArrayArray<DistanceConstraintUni> distanceConstraints;

            [ReadOnly]
            public ComponentDataArray<MassInv> massesInv;

            public ComponentDataArray<PredictedPositions> predPositions;

            [WriteOnly]
            public ComponentDataArray<DeltaPositions> deltaPositions;


            public void Execute()
            {
                for (int idA = 0; idA < length; idA++)
                {
                    float3 predPosA = predPositions[idA].Value;
                    float massInvA = massesInv[idA].Value;

                    NativeArray<DistanceConstraintUni> cnstrsA = distanceConstraints[idA];
                    float3 delta = new float3(0, 0, 0);
                    int cnstrCount = 0;
                    for (int i = 0; i < 34; i++)
                    {
                        DistanceConstraintUni cnstr = cnstrsA[i];
                        if (cnstr.otherId == -1)
                            break;

                        float3 predPosB = predPositions[cnstr.otherId].Value;
                        float massInvB = massesInv[cnstr.otherId].Value;

                        float3 normal = predPosA - predPosB;
                        float length = math.length(normal.xyz);

                        float invMass = massInvA + massInvB;
                        if (invMass <= math.epsilon_normal || length <= math.epsilon_normal)
                            continue;

                        float3 dP = (1.0f / invMass) * (length - cnstr.restLength) * (normal / length) * stiffness;
                        delta -= dP * massInvA;
                        cnstrCount++;


                    }
                    predPositions[idA] = new PredictedPositions { Value = predPosA + delta / cnstrCount };
                }
            }
        }

        [BurstCompile]
        public struct ProjectToBounds : IJobParallelFor
        {
            public float3 minBounds;
            public float3 maxBounds;
            public float radius;

            public ComponentDataArray<PredictedPositions> predPositions;

            public void Execute(int i)
            {

                float3 pos = predPositions[i].Value;
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

                predPositions[i] = new PredictedPositions { Value = pos };

            }
        }
    }
}
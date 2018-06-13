using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Burst;
using Unity.Jobs.LowLevel;

using Unity.Collections.LowLevel;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace OpenFlex
{

    public class UniformGrid_MT
    {

        public int m_gridSizeX = 16;
        public int m_gridSizeY = 16;
        public int m_gridSizeZ = 16;

        [HideInInspector]
        public NativeArray<int> particlesInCellCount;

        [HideInInspector]
        public NativeArray<int> particlesInCellIds;

        [HideInInspector]
        public NativeArray<int> cellNeighbours;

        [HideInInspector]
        public NativeArray<int> particleNeighboursCount;

        [HideInInspector]
        public NativeArray<int> particleNeighbours;

        public int m_maxParticlesCount = 16;
        public int m_maxParticlesInCell = 16;
        public int m_maxNeighboursPerParticle = 64;


        public int GetCellId(int x, int y, int z)
        {
            return x + y * m_gridSizeX + z * m_gridSizeX * m_gridSizeY;
        }


        // Use this for initialization
        public UniformGrid_MT(int sizeX, int sizeY, int sizeZ, int maxParticlesCount, int maxParticlesInCell, int maxNeighboursPerParticle)
        {
            m_gridSizeX = sizeX;
            m_gridSizeY = sizeY;
            m_gridSizeZ = sizeZ;

            m_maxParticlesCount = maxParticlesCount;
            m_maxParticlesInCell = maxParticlesInCell;
            m_maxNeighboursPerParticle = maxNeighboursPerParticle;


            int N = m_gridSizeX * m_gridSizeY * m_gridSizeZ;


            this.particlesInCellCount = new NativeArray<int>(N, Allocator.Persistent);
            this.particlesInCellIds = new NativeArray<int>(N * m_maxParticlesInCell, Allocator.Persistent);
            this.cellNeighbours =  new NativeArray<int>(N * 27, Allocator.Persistent); 

            this.particleNeighbours = new NativeArray<int>(m_maxParticlesCount * m_maxNeighboursPerParticle, Allocator.Persistent);
            this.particleNeighboursCount = new NativeArray<int>(m_maxParticlesCount, Allocator.Persistent);


            for (int i = 0; i < m_gridSizeX; i++)
            {
                for (int j = 0; j < m_gridSizeY; j++)
                {
                    for (int k = 0; k < m_gridSizeZ; k++)
                    {
                        for (int n = 0; n < 27; n++)
                        {
                            //    m_cells2[GetCellId(i, j, k)].neighbours2[n] = -1;
                            cellNeighbours[GetCellId(i, j, k) * 27 + n] = -1;
                        }

                        int neighbourCount = 0;
                        for (int x = -1; x < 2; x++)
                        {
                            for (int y = -1; y < 2; y++)
                            {
                                for (int z = -1; z < 2; z++)
                                {
                                    if (i + x >= 0 && i + x < m_gridSizeX && j + y >= 0 && j + y < m_gridSizeY && k + z >= 0 && k + z < m_gridSizeZ)
                                    {
                                        int idn = (i + x) + (j + y) * m_gridSizeX + (k + z) * m_gridSizeX * m_gridSizeY;

                                        //   m_cells2[GetCellId(i, j, k)].neighbours2[neighbourCount++] = idn;
                                        cellNeighbours[GetCellId(i, j, k) * 27 + neighbourCount++] = idn;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }


        public void UpdateGrid(NativeArray<Vector4> positions, int pointsCount, Matrix4x4 mat)
        {


            ClearGrid();
            int max = 0;

            for (int i = 0; i < pointsCount; i++)
            {
                Vector4 pos = mat.MultiplyPoint(positions[i]);

                //assuming indices are always valid because the box keeps the particles contained
                if ((int)pos.x >= 0 && (int)pos.y >= 0 && (int)pos.z >= 0 && (int)pos.x < m_gridSizeX && (int)pos.y < m_gridSizeY && (int)pos.z < m_gridSizeZ)
                {
                    //  Cell2 cell2 = m_cells2[GetCellId((int)pos.x, (int)pos.y, (int)pos.z)];
                    //   cell2.particles[cell2.particlesCount++] = i;
                    int cellId = GetCellId((int)pos.x, (int)pos.y, (int)pos.z);

                    if (particlesInCellCount[cellId] < m_maxParticlesInCell)
                    {
                        particlesInCellIds[cellId * m_maxParticlesInCell + particlesInCellCount[cellId]] = i;
                        particlesInCellCount[cellId]++;

                    }

                    max = Mathf.Max(max, particlesInCellCount[cellId]);
                    //           m_pointsCells[i] = cell;
                }
                else
                {
                    //           m_pointsCells[i] = null;
                }
            }

            if (max >= m_maxParticlesInCell)
                Debug.Log("Max: " + max);
            
        }


        [ComputeJobOptimization]
        public struct UpdateGridJob2 : IJob
        {
            public int gridSizeX;
            public int gridSizeY;
            public int gridSizeZ;

            public int particlesCount;
            public int maxParticlesInCell;
            public Matrix4x4 mat;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            public NativeArray<int> particlesInCellCount;
            public NativeArray<int> particlesInCellIds;

            public void Execute()
            {
                for (int i = 0; i < particlesCount; i++)
                {
                    Vector4 pos = mat.MultiplyPoint(positions[i]);

                    int x = (int)pos.x;
                    int y = (int)pos.y;
                    int z = (int)pos.z;

                    //assuming indices are always valid because the box keeps the particles contained
                    if (x >= 0 && y >= 0 && z >= 0 && x < gridSizeX && y < gridSizeY && z < gridSizeZ)
                    {
                        int cellId = x + y * gridSizeX + z * gridSizeX * gridSizeY;

                        if (particlesInCellCount[cellId] < maxParticlesInCell)
                        {
                            particlesInCellIds[cellId * maxParticlesInCell + particlesInCellCount[cellId]] = i;
                            particlesInCellCount[cellId]++;
                        }
                    }
                }

            }
        }

        [ComputeJobOptimization]
        public struct UpdateNeighboursJob2 : IJob
        {

            public int gridSizeX;
            public int gridSizeY;
            public int gridSizeZ;
            public int particlesCount;
            public float collisionRadius;
            public int maxParticlesInCell;
            public int maxNeighboursPerParticle;
            public Matrix4x4 mat;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            public NativeArray<int> particlesInCellCount;
            public NativeArray<int> particlesInCellIds;

            public NativeArray<int> cellNeighbours;
            public NativeArray<int> particleNeighbours;
            public NativeArray<int> particleNeighboursCount;

            public void Execute()
            {
                for (int idA = 0; idA < particlesCount; idA++)
                {


                    int cellAId = -1;

                    Vector4 pos = mat.MultiplyPoint(positions[idA]);

                    int x = (int)pos.x;
                    int y = (int)pos.y;
                    int z = (int)pos.z;

                    float collisionRadiusSq = collisionRadius * collisionRadius + 0.0001f;
                    //assuming indices are always valid because the box keeps the particles contained
                    if (x >= 0 && y >= 0 && z >= 0 && x < gridSizeX && y < gridSizeY && z < gridSizeZ)
                    {
                        cellAId = x + y * gridSizeX + z * gridSizeX * gridSizeY;
                    }
                    else
                    {
                        particleNeighboursCount[idA] = 0;
                        return;
                    }

                    int totalNeighboursCount = 0;
                    for (int n = 0; n < 27; n++)
                    {
                        int neighbourCellId = cellNeighbours[cellAId * 27 + n];
                        if (neighbourCellId == -1)
                            break;

                        for (int g = 0; g < particlesInCellCount[neighbourCellId]; g++)
                        {
                            int idB = particlesInCellIds[neighbourCellId * maxParticlesInCell + g];

                            if (idA != idB)
                            {
                                if (totalNeighboursCount < maxNeighboursPerParticle)
                                {
                                    particleNeighbours[idA * maxNeighboursPerParticle + totalNeighboursCount] = idB;
                                    totalNeighboursCount++;
                                }

                            }
                        }

                    }

                    particleNeighboursCount[idA] = totalNeighboursCount;
                }
            }
        }

        [ComputeJobOptimization]
        public struct UpdateGridJob : IJobParallelFor
        { 
            public int gridSizeX;
            public int gridSizeY;
            public int gridSizeZ;

            public int maxParticlesInCell;
            public Matrix4x4 mat;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            [NativeDisableParallelForRestriction]
            public NativeArray<int> particlesInCellCount;

            [NativeDisableParallelForRestriction]
            public NativeArray<int> particlesInCellIds;

            public void Execute(int i)
            {
                Vector4 pos = mat.MultiplyPoint(positions[i]);

                int x = (int)pos.x;
                int y = (int)pos.y;
                int z = (int)pos.z;

                //assuming indices are always valid because the box keeps the particles contained
                if (x >= 0 && y >= 0 && z >= 0 && x < gridSizeX && y < gridSizeY && z < gridSizeZ)
                {
                    int cellId = x + y * gridSizeX + z * gridSizeX * gridSizeY;

                    if (particlesInCellCount[cellId] < maxParticlesInCell)
                    {
                        particlesInCellIds[cellId * maxParticlesInCell + particlesInCellCount[cellId]] = i;
                        //particlesInCellCount[cellId]++;
                        unsafe
                        {
                            particlesInCellCount[cellId] = Interlocked.Increment(ref ((int*)particlesInCellCount.GetUnsafePtr())[cellId]);
                        }
                    }
                }
            }
        }

        [ComputeJobOptimization]
        public struct UpdateNeighboursJob : IJobParallelFor
        {

            public int gridSizeX;
            public int gridSizeY;
            public int gridSizeZ;

            public float collisionRadius;
            public int maxParticlesInCell;
            public int maxNeighboursPerParticle;
            public Matrix4x4 mat;

            [ReadOnly] public NativeArray<Vector4> positions;

            [ReadOnly] public NativeArray<int> particlesInCellCount;
            [ReadOnly] public NativeArray<int> particlesInCellIds;

            [ReadOnly] public NativeArray<int> cellNeighbours;



           // [NativeDisableContainerSafetyRestriction]
            [NativeDisableParallelForRestriction] public NativeArray<int> particleNeighbours;

            [WriteOnly] public NativeArray<int> particleNeighboursCount;

            public void Execute(int idA)
            {
                int cellAId = -1;

                Vector4 pos = mat.MultiplyPoint(positions[idA]);

                int x = (int)pos.x;
                int y = (int)pos.y;
                int z = (int)pos.z;

                float collisionRadiusSq = collisionRadius * collisionRadius + 0.0001f;
                //assuming indices are always valid because the box keeps the particles contained
                if (x >= 0 && y >= 0 && z >= 0 && x < gridSizeX && y < gridSizeY && z < gridSizeZ)
                {
                    cellAId = x + y * gridSizeX + z * gridSizeX * gridSizeY;
                }
                else
                {
                    particleNeighboursCount[idA] = 0;
                    return;
                }

                int totalNeighboursCount = 0;
                for (int n = 0; n < 27; n++)
                {
                    int neighbourCellId = cellNeighbours[cellAId * 27 + n];
                    if (neighbourCellId == -1)
                        break;

                    for (int g = 0; g < particlesInCellCount[neighbourCellId]; g++)
                    {
                        int idB = particlesInCellIds[neighbourCellId * maxParticlesInCell + g];

                        if (idA != idB)
                        {
                            if (totalNeighboursCount < maxNeighboursPerParticle)
                            {
                                particleNeighbours[idA * maxNeighboursPerParticle + totalNeighboursCount] = idB;
                                totalNeighboursCount++;
                            }

                        }
                    }

                }

                particleNeighboursCount[idA] = totalNeighboursCount;
            }
        }

        public void UpdateNeighbours(NativeArray<Vector4> positions, int pointsCount, float collisionRadius, Matrix4x4 mat)
        {
            //   float radiusSum = radius + radius;
            //   float radiusSumSq = radiusSum * radiusSum;
            float collisionRadiusSq = collisionRadius * collisionRadius + 0.0001f;

            for (int idA = 0; idA < pointsCount; idA++)
            {
                //  Cell2 cellA = bodyUGrid.m_pointsCells[idA];

                int cellAId = -1;
                //    m_neighbours[idA].Clear();


                Vector4 pos = mat.MultiplyPoint(positions[idA]);

                if ((int)pos.x >= 0 && (int)pos.y >= 0 && (int)pos.z >= 0 && (int)pos.x < m_gridSizeX && (int)pos.y < m_gridSizeY && (int)pos.z < m_gridSizeZ)
                {
                    cellAId = GetCellId((int)pos.x, (int)pos.y, (int)pos.z);
                }
                else
                {
                    particleNeighboursCount[idA] = 0;
                    continue;
                }

                int totalNeighboursCount = 0;
                for (int n = 0; n < 27; n++)
                {
                    // int neighbourId = cellA.neighbours2[n];
                    int neighbourCellId = cellNeighbours[cellAId * 27 + n];
                    if (neighbourCellId == -1)
                        break;

                    for (int g = 0; g < particlesInCellCount[neighbourCellId]; g++)
                    {
                        // int idB = cellB.particles[g];
                        int idB = particlesInCellIds[neighbourCellId * m_maxParticlesInCell + g];

                        if (idA != idB)
                        {
                            //Vector3 posB = positions[idB] - tr;
                            //Vector4 relPos = pos - posB;
                            //float distanceSq = Vector3.SqrMagnitude(relPos);
                            //if (distanceSq <= collisionRadiusSq && totalNeighboursCount < m_maxNeighboursPerPoint)
                            if (totalNeighboursCount < m_maxNeighboursPerParticle)
                            {
                                particleNeighbours[idA * m_maxNeighboursPerParticle + totalNeighboursCount] = idB;
                                totalNeighboursCount++;
                            }

                        }
                    }

                }

                particleNeighboursCount[idA] = totalNeighboursCount;
            }
            

        }

        public void GetNeighbours(Vector3 p, List<int> results, Matrix4x4 mat)
        {
            Vector4 pos = mat.MultiplyPoint(p);


            int cellAId = -1;

            if ((int)pos.x >= 0 && (int)pos.y >= 0 && (int)pos.z >= 0 && (int)pos.x < m_gridSizeX && (int)pos.y < m_gridSizeY && (int)pos.z < m_gridSizeZ)
            {
                cellAId = GetCellId((int)pos.x, (int)pos.y, (int)pos.z);
            }
            else
            {
                return;
            }

            for (int n = 0; n < 27; n++)
            {
                int neighbourCellId = cellNeighbours[cellAId * 27 + n];
                if (neighbourCellId == -1)
                    break;

                for (int g = 0; g < particlesInCellCount[neighbourCellId]; g++)
                {

                    int idB = particlesInCellIds[neighbourCellId * m_maxParticlesInCell + g];
                    results.Add(idB);
                }

            }

            return;
        }

        public void UpdateNeighboursBruteForce(NativeArray<Vector4> positions, int pointsCount, float collisionRadius)
        {

            float collisionRadiusSq = collisionRadius * collisionRadius + 0.0001f;

            for (int idA = 0; idA < pointsCount; idA++)
            {
                for (int nId = 0; nId < m_maxNeighboursPerParticle; nId++)
                {
                    particleNeighbours[idA * m_maxNeighboursPerParticle + nId] = -1;
                }
            }


            for (int idA = 0; idA < pointsCount; idA++)
            {

                int totalNeighboursCount = 0;
                for (int idB = 0; idB < pointsCount; idB++)
                {

                    if (idA != idB)
                    {
                        Vector3 relPos = positions[idA] - positions[idB];
                        float distanceSq = Vector3.SqrMagnitude(relPos);

                        if (distanceSq <= collisionRadiusSq && totalNeighboursCount < m_maxNeighboursPerParticle)
                        {
                            particleNeighbours[idA * m_maxNeighboursPerParticle + totalNeighboursCount] = idB;
                            totalNeighboursCount++;
                        }

                        //if (totalNeighboursCount >= m_maxNeighboursPerPoint)
                        //    Debug.Log("asdasd");
                    }

                }

                particleNeighboursCount[idA] = totalNeighboursCount;

            }



        }

        public void ClearGrid()
        {
            for (int i = 0; i < m_gridSizeX * m_gridSizeY * m_gridSizeZ; i++)
            {
                particlesInCellCount[i] = 0;
            }
        }

        public void Dispose()
        {
            this.particlesInCellCount.Dispose();
            this.particlesInCellIds.Dispose();
            this.cellNeighbours.Dispose();

            this.particleNeighbours.Dispose();
            this.particleNeighboursCount.Dispose();
        }


        //public virtual void OnDrawGizmosSelected()
        //{
        //    Vector3 t = transform.localPosition;
        //    Vector3 s = transform.localScale;
        //    Gizmos.color = Color.red;
        //    //  Gizmos.DrawWireCube(new Vector3(m_width/2, m_height/2, m_depth/2)*cellSpace, new Vector3(m_width, m_height, m_depth)*cellSpace);
        //    Vector3 centre = new Vector3(m_gridSizeX / 2 * s.x, m_gridSizeY / 2 * s.y, m_gridSizeZ / 2 * s.z) + t;
        //    Vector3 size = new Vector3(m_gridSizeX, m_gridSizeY, m_gridSizeZ);

        //    //centre.Scale(s);
        //    size.Scale(s);
        //    Gizmos.DrawWireCube(centre, size);


        //    //Gizmos.DrawCube()
        //    //DebugExtension.DrawLocalCube(transform, new Vector3(m_gridSizeX, m_gridSizeY, m_gridSizeZ), Color.blue, new Vector3(m_gridSizeX / 2, m_gridSizeY / 2, m_gridSizeZ / 2));
        //    if (Application.isPlaying)
        //    {
        //        Gizmos.color = Color.green;
        //        //if (particlesInCellCount != null)
        //        {
        //            for (int i = 0; i < m_gridSizeX; i++)
        //            {
        //                for (int j = 0; j < m_gridSizeY; j++)
        //                {
        //                    for (int k = 0; k < m_gridSizeZ; k++)
        //                    {

        //                        if (particlesInCellCount[GetCellId(i, j, k)] > 0)
        //                            Gizmos.DrawWireCube(new Vector3(((float)i + 0.5f) * s.x, ((float)j + 0.5f) * s.y, ((float)k + 0.5f) * s.z) + t, s);


        //                    }
        //                }
        //            }
        //        }
        //    }
        //}

    }


}
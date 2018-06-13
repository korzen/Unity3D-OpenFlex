using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Burst;

namespace OpenFlex
{
    public class OpenFlexCPU_MT : OpenFlexAPI
    {

        protected int maxParticlesCount;
        protected SolverParams solverParams;
        protected FluidsParams fluidParams;


        #region ARRAYS
        

        protected NativeArray<Vector4> positions;
        protected NativeArray<Vector4> predPositions;
        protected NativeArray<Vector4> deltaPositions;
        protected NativeArray<int> actives;
        protected NativeArray<int> phases;
        protected NativeArray<float> massesInv;
        protected NativeArray<Vector4> velocities;
        protected NativeArray<Vector3> normals;

        protected NativeArray<float> densities;
        protected NativeArray<float> lambdas;
        #endregion

        protected UniformGrid_MT grid;

        public override void CreateSolver(int maxParticles, byte maxNeighborsPerParticle)
        {
            maxParticlesCount = maxParticles;

            positions = new NativeArray<Vector4>(maxParticles, Allocator.Persistent);
            predPositions = new NativeArray<Vector4>(maxParticles, Allocator.Persistent);
            deltaPositions = new NativeArray<Vector4>(maxParticles, Allocator.Persistent);
            velocities = new NativeArray<Vector4>(maxParticles, Allocator.Persistent);
            normals = new NativeArray<Vector3>(maxParticles, Allocator.Persistent);
            massesInv = new NativeArray<float>(maxParticles, Allocator.Persistent);
            densities = new NativeArray<float>(maxParticles, Allocator.Persistent);
            lambdas = new NativeArray<float>(maxParticles, Allocator.Persistent);

            actives = new NativeArray<int>(maxParticles, Allocator.Persistent);
            phases = new NativeArray<int>(maxParticles, Allocator.Persistent);

            for (int i = 0; i < maxParticles; i++)
            {
                actives[i] = i;
                phases[i] = MakePhase(0, (int)Phase.eFlexPhaseSelfCollide);
            }

            grid = new UniformGrid_MT(32, 32, 32, maxParticles, 16, maxNeighborsPerParticle);

        }

        public override void Shutdown()
        {
            DestroySolver();
            grid.Dispose();
        }

        public override void UpdateSolver(float dt, int substeps)
        {
            float subStepDt = dt / substeps;
            float r = solverParams.particleRadius;
            for (int step = 0; step < substeps; step++)
            {

                //Profiler.BeginSample("__UNIFORM_GRID_CONSTRUCTION__");
                //grid.UpdateGrid(predPositions, maxParticlesCount, Matrix4x4.identity);
                grid.ClearGrid();
    
                var updateGridJob = new UniformGrid_MT.UpdateGridJob()
                {
                    gridSizeX = grid.m_gridSizeX,
                    gridSizeY = grid.m_gridSizeY,
                    gridSizeZ = grid.m_gridSizeZ,
                    mat = Matrix4x4.identity,
                    positions = this.positions,
                    //  particlesCount = maxParticlesCount,

                    maxParticlesInCell = grid.m_maxParticlesInCell,
                    particlesInCellCount = grid.particlesInCellCount,
                    particlesInCellIds = grid.particlesInCellIds

                };

                JobHandle updateGridJobHndl = updateGridJob.Schedule(maxParticlesCount, 32);
                //updateGridJobHndl.Complete();
                //Profiler.EndSample();



                Profiler.BeginSample("__UNIFORM_GRID_NEIGHBOUR_SEARCH__");
                //grid.UpdateNeighbours(predPositions, maxParticlesCount, r, Matrix4x4.identity);
                var updateNeighboursJob = new UniformGrid_MT.UpdateNeighboursJob()
                {
                    gridSizeX = grid.m_gridSizeX,
                    gridSizeY = grid.m_gridSizeY,
                    gridSizeZ = grid.m_gridSizeZ,
                    //    particlesCount = maxParticlesCount,
                    mat = Matrix4x4.identity,
                    positions = this.positions,
                    cellNeighbours = grid.cellNeighbours,
                    particleNeighbours = grid.particleNeighbours,
                    collisionRadius = r,
                    maxNeighboursPerParticle = grid.m_maxNeighboursPerParticle,
                    particleNeighboursCount = grid.particleNeighboursCount,
                    maxParticlesInCell = grid.m_maxParticlesInCell,
                    particlesInCellCount = grid.particlesInCellCount,
                    particlesInCellIds = grid.particlesInCellIds

                };

                JobHandle updateNeighboursJobHndl = updateNeighboursJob.Schedule(maxParticlesCount, 32, updateGridJobHndl);
                updateNeighboursJobHndl.Complete();

                Profiler.EndSample();



                Profiler.BeginSample("__PBD__PREDICT_POSITIONS__");
                //PositionBasedDynamics.PredictPositions(positions, predPositions, velocities, maxParticlesCount, Vector3.down * 9.81f, subStepDt);

                var predictPositionsJob = new PositionBasedDynamicsJobs.PredictPositionsJob()
                {
                    dt = subStepDt,
                    acceleration = Vector3.down * 9.81f,
                    positions = this.positions,
                    predPositions = this.predPositions,
                    velocities = this.velocities
                };

                JobHandle predictPositionsJobHndl = predictPositionsJob.Schedule(maxParticlesCount, 32);
                predictPositionsJobHndl.Complete();
                Profiler.EndSample();



                for (int cnstrIterations = 0; cnstrIterations < solverParams.constraintsIterationsCount; cnstrIterations++)
                {


                    Profiler.BeginSample("__PBD__PARTICLE_VS_PARTICLE_COLLISIONS__");
                    //PositionBasedDynamics.ProjectParticlesVsParticlesCollisions(predPositions, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, 0.4f, 0.1f);
                    //var particleVsParticleCollisionsJob = new PositionBasedDynamics.ProjectParticlesVsParticlesCollisionsJob()
                    //{
                    //    particlesCount = maxParticlesCount,
                    //    radius = 0.4f,
                    //    kS = 0.1f,

                    //    positions = this.predPositions,
                    //    particlesNeighbours = grid.particleNeighbours,
                    //    particlesNeighboursCount = grid.particleNeighboursCount,
                    //    maxNeighboursPerParticle = grid.m_maxNeighboursPerParticle,
                    //};

                    //JobHandle particleVsParticleCollisionsJobHndl = particleVsParticleCollisionsJob.Schedule();
                    //particleVsParticleCollisionsJobHndl.Complete();


                    var particleVsParticleCollisionsJob = new PositionBasedDynamicsJobs.ProjectParticlesVsParticlesCollisionsParallelForJob()
                    {
                        radius = 0.4f,
                        kS = 0.1f,

                        positions = this.predPositions,
                        deltaPositions = this.deltaPositions,
                        particlesNeighbours = grid.particleNeighbours,
                        particlesNeighboursCount = grid.particleNeighboursCount,
                        maxNeighboursPerParticle = grid.m_maxNeighboursPerParticle,
                    };

                    JobHandle particleVsParticleCollisionsJobHndl = particleVsParticleCollisionsJob.Schedule(maxParticlesCount, 32);
                    //particleVsParticleCollisionsJobHndl.Complete();
                    Profiler.EndSample();

                    //   Profiler.BeginSample("__PBF__CALCULATE_LAMBDAS__");
                    //   //PositionBasedFluids.CalculateLambdas(predPositions, densities, lambdas, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, fluidParams.restDensity, fluidParams.epsilonLambda, fluidParams.kernel);
                    //   var calculateLambdasJob = new PositionBasedFluids.CalculateLambdasJob()
                    //   {
                    //       positions = this.predPositions,
                    //       densities = this.densities,
                    //       lambdas = this.lambdas,
                    //       restDensity = fluidParams.restDensity,
                    //       epsilonLambda = fluidParams.epsilonLambda,
                    //       kernels = fluidParams.kernel,
                    //       particlesNeighbours = grid.particleNeighbours,
                    //       particlesNeighboursCount = grid.particleNeighboursCount,
                    //       maxNeighboursPerParticle = grid.m_maxNeighboursPerParticle,
                    //   };
                    //   JobHandle calculateLambdasJobHndl = calculateLambdasJob.Schedule(maxParticlesCount, 32);
                    ////   calculateLambdasJobHndl.Complete();

                    //   Profiler.EndSample();

                    //   Profiler.BeginSample("__PBF__PROJECT_FLUID_CONSTRAINTS__");
                    //   //PositionBasedFluids.ProjectFluidConstraints(predPositions, densities, lambdas, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, fluidParams.restDensity, fluidParams.kernel, deltaPositions);
                    //   var projectFluidConstraintsJob = new PositionBasedFluids.ProjectFluidConstraintsJob()
                    //   {
                    //       restDensity = fluidParams.restDensity,
                    //       epsilonLambda = fluidParams.epsilonLambda,
                    //       kernels = fluidParams.kernel,

                    //       positions = this.predPositions,
                    //       deltaPositions = this.deltaPositions,
                    //       densities = this.densities,
                    //       lambdas = this.lambdas,

                    //       particlesNeighbours = grid.particleNeighbours,
                    //       particlesNeighboursCount = grid.particleNeighboursCount,
                    //       maxNeighboursPerParticle = grid.m_maxNeighboursPerParticle,

                    //   };
                    //   JobHandle projectFluidConstraintsJobHndl = projectFluidConstraintsJob.Schedule(maxParticlesCount, 32, calculateLambdasJobHndl);

                    //   projectFluidConstraintsJobHndl.Complete();


                    var addDeltasToPredictedJob = new PositionBasedDynamicsJobs.AddToArrayParallelForJob()
                    {
                        arrA = this.predPositions,
                        arrB = this.deltaPositions
                    };

                    JobHandle addDeltasToPredictedJobHndl = addDeltasToPredictedJob.Schedule(maxParticlesCount, 32, particleVsParticleCollisionsJobHndl);
                    //   addDeltasToPredictedJobHndl.Complete();

                    //for (int i = 0; i < maxParticlesCount; i++)
                    //    predPositions[i] += deltaPositions[i];



                    //   Profiler.EndSample();


                    //PositionBasedDynamics.ProjectParticlesVsPhysXCollisions(positions, predPositions, maxParticlesCount, 0.4f);
                    //PositionBasedDynamics.ProjectParticlesFloorBounds(positions, predPositions, maxParticlesCount, 0.0f, r);
                    //PositionBasedDynamics.ProjectParticlesToBounds(positions, predPositions, maxParticlesCount, r, Vector3.zero, new Vector3(32,32,32));
                    Profiler.BeginSample("__PBD_PROJECT_TO_BOUNDS__");

                    var projectToBoundsJob = new PositionBasedDynamicsJobs.ProjectParticlesToBoundsParallelForJob()
                    {
                        maxBounds = new Vector3(grid.m_gridSizeX, grid.m_gridSizeY, grid.m_gridSizeZ),
                        minBounds = Vector3.zero,
                        radius = 0.5f,
                        predPositions = this.predPositions

                    };
                    
                    JobHandle projectToBoundsJobHndl = projectToBoundsJob.Schedule(maxParticlesCount, 32, addDeltasToPredictedJobHndl);

                    projectToBoundsJobHndl.Complete();
                    Profiler.EndSample();
  
                    //var projectParticlesFloorBoundsJob = new PositionBasedDynamics.ProjectParticlesFloorBoundsJob()
                    //{
                    //    floorLevel = 0.0f,
                    //    radius = r,
                    //    positions = this.positions,
                    //    predPositions = this.predPositions
                    //};
                    //JobHandle projectParticlesFloorBoundsJobHndl = projectParticlesFloorBoundsJob.Schedule(maxParticlesCount, 32);
                    //projectParticlesFloorBoundsJobHndl.Complete();
                }

                Profiler.BeginSample("__PBD__UPDATE_VELOCITIES__");
                //PositionBasedDynamics.UpdateVelocities(positions, predPositions, velocities, maxParticlesCount, subStepDt);
                var updateVelocitiesJob = new PositionBasedDynamicsJobs.UpdateVelocityiesJob()
                {
                    dtInv = 1.0f / subStepDt,
                    positions = this.positions,
                    predPositions = this.predPositions,
                    velocities = this.velocities
                };

                JobHandle updateVelocitiesJobHndl = updateVelocitiesJob.Schedule(maxParticlesCount, 32);

                updateVelocitiesJobHndl.Complete();
                Profiler.EndSample();
               }

        }

        public override void DestroySolver()
        {
            positions.Dispose();
            predPositions.Dispose();
            deltaPositions.Dispose();
            velocities.Dispose();
            normals.Dispose();
            massesInv.Dispose();
            densities.Dispose();
            lambdas.Dispose();

            actives.Dispose();
            phases.Dispose();
        }

        public override void UpdateSDF(IntPtr sdf, int dimx, int dimy, int dimz, IntPtr field, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void GetActive(IntPtr s, int[] indices, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetActive(IntPtr s, IntPtr indices, Memory target)
        {
            throw new NotImplementedException();
        }

        public override int GetActiveCount(IntPtr s)
        {
            throw new NotImplementedException();
        }



        public override void UpdateTriangleMesh(IntPtr mesh, Vector3[] vertices, int[] indices, int numVertices, int numTriangles, ref Vector3 lower, ref Vector3 upper, Memory source)
        {
            throw new NotImplementedException();
        }

        public override IntPtr CreateTriangleMesh()
        {
            throw new NotImplementedException();
        }

        public override void AcquireContext()
        {
            throw new NotImplementedException();
        }

        public override IntPtr Alloc(int size)
        {
            throw new NotImplementedException();
        }

        public override IntPtr CreateSDF()
        {
            throw new NotImplementedException();
        }

        public override void DestroySDF(IntPtr sdf)
        {
            throw new NotImplementedException();
        }



        public override void DestroyTriangleMesh(IntPtr mesh)
        {
            throw new NotImplementedException();
        }

        public override void Free(IntPtr ptr)
        {
            throw new NotImplementedException();
        }



        public override void GetAnisotropy(IntPtr solverPtr, ref float q1, ref float q2, ref float q3, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetBounds(IntPtr s, ref Vector3 lower, ref Vector3 upper, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetContacts(IntPtr solverPtr, ref float planes, ref float velocities, ref int indices, IntPtr counts, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetDensities(float[] densities, int n)
        {
            this.densities.CopyFrom(densities);
        }



        public override int GetDiffuseParticles(IntPtr solverPtr, IntPtr pos, IntPtr vel, IntPtr indices, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetDynamicTriangles(IntPtr solverPtr, int[] indices, Vector3[] normals, int numTris, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetNormals(IntPtr s, Vector4[] normals, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetNormals(IntPtr s, IntPtr normals, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetParams(IntPtr s, ref Params parameterss)
        {
            throw new NotImplementedException();
        }

        public override void GetPositions(Vector4[] p, int n)
        {
            positions.CopyTo(p);
        }

        public override void GetMassesInv(float[] m, int n)
        {
            massesInv.CopyTo(m);
        }

        public override void GetPhases(IntPtr s, int[] phases, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetPhases(IntPtr s, IntPtr phases, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetRigidTransforms(IntPtr s, IntPtr rotations, IntPtr translations, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetRigidTransforms(IntPtr s, Quaternion[] rotations, Vector3[] translations, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetSmoothParticles(IntPtr s, Vector4[] p, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetSmoothParticles(IntPtr s, IntPtr p, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetSprings(IntPtr s, IntPtr indices, IntPtr restLengths, IntPtr stiffness, int numSprings, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetSprings(IntPtr s, int[] indices, float[] restLengths, float[] stiffness, int numSprings, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetTriangleMeshBounds(IntPtr mesh, ref Vector3 lower, ref Vector3 upper)
        {
            throw new NotImplementedException();
        }

        public override void GetVelocities(IntPtr s, Vector3[] velocities, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override void GetVelocities(IntPtr s, IntPtr v, int n, Memory target)
        {
            throw new NotImplementedException();
        }

        public override int GetVersion()
        {
            throw new NotImplementedException();
        }

        public override Error Init(int version, ErrorCallback errorFunc, int deviceIndex)
        {
            throw new NotImplementedException();
        }

        public override SolverCallback RegisterSolverCallback(IntPtr solverPtr, SolverCallback function, SolverCallbackStage stage)
        {
            throw new NotImplementedException();
        }

        public override void RestoreContext()
        {
            throw new NotImplementedException();
        }

        public override void SetActive(IntPtr s, int[] indices, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetActive(IntPtr s, IntPtr indices, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetDiffuseParticles(IntPtr solverPtr, IntPtr pos, IntPtr vel, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetDynamicTriangles(IntPtr solverPtr, int[] indices, Vector3[] normals, int numTris, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetFence()
        {
            throw new NotImplementedException();
        }

        public override void SetInflatables(IntPtr solverPtr, int[] startTris, int[] numTris, float[] restVolumes, float[] overPressures, float[] constraintScales, int numInflatables, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetNormals(IntPtr s, Vector4[] normals, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetNormals(IntPtr s, IntPtr normals, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetParams(SolverParams solverParams, FluidsParams fluidParams)
        {
            this.solverParams = solverParams;
            this.fluidParams = fluidParams;
        }

        public override void SetParticles(Vector4[] p, int n)
        {
            positions.CopyFrom(p);
        }

        public override void SetMassesInv(float[] m, int n)
        {
            massesInv.CopyFrom(m);
        }

        public override void SetPhases(IntPtr s, int[] phases, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetPhases(IntPtr s, IntPtr phases, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetRestParticles(IntPtr s, IntPtr p, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetRestParticles(IntPtr s, Vector4[] p, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetRigids(IntPtr s, int[] offsets, int[] indices, Vector3[] restPositions, Vector4[] restNormals, float[] stiffness, Quaternion[] rotations, Vector3[] translations, int numRigids, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetShapes(IntPtr s, CollisionTriangleMesh[] triMesh, int numGeometryEntries, Vector4[] shapeAabbMins, Vector4[] shapeAabbMaxs, int[] shapeOffsets, Vector4[] shapePositions, Quaternion[] shapeRotations, Vector4[] shapePrevPositions, Quaternion[] shapePrevRotations, int[] shapeFlags, int numShapes, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetSprings(IntPtr s, IntPtr indices, IntPtr restLengths, IntPtr stiffness, int numSprings, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetSprings(IntPtr s, int[] indices, float[] restLengths, float[] stiffness, int numSprings, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetVelocities(IntPtr s, Vector3[] velocities, int n, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void SetVelocities(IntPtr s, IntPtr v, int n, Memory source)
        {
            throw new NotImplementedException();
        }



        public override void WaitFence()
        {
            throw new NotImplementedException();
        }


    }
}
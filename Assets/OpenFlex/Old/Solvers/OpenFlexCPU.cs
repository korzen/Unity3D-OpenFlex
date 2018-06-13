using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

namespace OpenFlex
{
    public class OpenFlexCPU : OpenFlexAPI
    {

        protected int maxParticlesCount;
        protected SolverParams solverParams;
        protected FluidsParams fluidParams;

        #region ARRAYS
        protected Vector4[] positions;
        protected Vector4[] predPositions;
        protected Vector4[] deltaPositions;
        protected int[] actives;
        protected int[] phases;
        protected float[] massesInv;
        protected Vector4[] velocities;
        protected Vector3[] normals;

        protected int[] springIndices;
        protected float[] springRestLengths;
        protected float[] springStiffness;

        protected float[] densities;
        protected float[] lambdas;
        #endregion


        public override void CreateSolver(int maxParticles, byte maxNeighborsPerParticle)
        {
            maxParticlesCount = maxParticles;

            positions = new Vector4[maxParticles];
            predPositions = new Vector4[maxParticles];
            deltaPositions = new Vector4[maxParticles];
            velocities = new Vector4[maxParticles];
            normals = new Vector3[maxParticles];
            densities = new float[maxParticles];
            lambdas = new float[maxParticles];

            actives = new int[maxParticles];
            phases = new int[maxParticles];
            for (int i = 0; i < maxParticles; i++)
            {
                actives[i] = i;
                phases[i] = MakePhase(0, (int)Phase.eFlexPhaseSelfCollide);
            }
        }

        public override void Shutdown()
        {

        }

        public override void UpdateSolver(float dt, int substeps)
        {
            float subStepDt = dt / substeps;
            float radius = solverParams.particleRadius;
            for (int step = 0; step < substeps; step++)
            {
                Profiler.BeginSample("__PBD__PREDICT_POSITIONS__");
                PositionBasedDynamics.PredictPositions(positions, predPositions, velocities, massesInv, maxParticlesCount, Vector3.down * 9.81f, subStepDt);
                Profiler.EndSample();

                Profiler.BeginSample("__UNIFORM_GRID_CONSTRUCTION__");
                //grid.UpdateGrid(predPositions, maxParticlesCount);
                //grid.UpdateNeighbours(predPositions, maxParticlesCount, radius);
                Profiler.EndSample();

                for (int cnstrIterations = 0; cnstrIterations < solverParams.constraintsIterationsCount; cnstrIterations++)
                {
                    //PositionBasedDynamics.ProjectParticlesVsParticlesCollisions(predPositions, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, 0.4f, 0.1f);

                    //Profiler.BeginSample("__PBF__CALCULATE_LAMBDAS__");
                    //PositionBasedFluids.CalculateLambdas(predPositions, densities, lambdas, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, fluidParams.restDensity, fluidParams.epsilonLambda, fluidParams.kernel);
                    //Profiler.EndSample();

                    //Profiler.BeginSample("__PBF__PROJECT_FLUID_CONSTRAINTS__");
                    //PositionBasedFluids.ProjectFluidConstraints(predPositions, densities, lambdas, maxParticlesCount, grid.particleNeighbours, grid.particleNeighboursCount, grid.m_maxNeighboursPerParticle, fluidParams.restDensity, fluidParams.kernel, deltaPositions);
                    //for (int i = 0; i < maxParticlesCount; i++)
                    //    predPositions[i] += deltaPositions[i];
                    //Profiler.EndSample();

                    //PositionBasedDynamics.ProjectParticlesVsPhysXCollisions(positions, predPositions, maxParticlesCount, 0.4f);
                    PositionBasedDynamics.ProjectParticlesFloorBounds(positions, predPositions, maxParticlesCount, 0.0f, radius);
                }

                Profiler.BeginSample("__PBD__UPDATE_VELOCITIES__");
                PositionBasedDynamics.UpdateVelocities(positions, predPositions, velocities, massesInv, maxParticlesCount, subStepDt);
                Profiler.EndSample();
            }

        }



        public override void DestroySolver()
        {

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
            Array.Copy(this.densities, densities, n);
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
            Array.Copy(positions, p, n);
        }

        public override void GetMassesInv(float[] p, int n)
        {
            Array.Copy(massesInv, p, n);
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
            Array.Copy(p, positions, n);
        }

        public override void SetMassesInv(float[] p, int n)
        {
            Array.Copy(p, massesInv, n);
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
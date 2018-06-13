using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace OpenFlex
{

    public class OpenFlexGPU : OpenFlexAPI
    {
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

        public override void CreateSolver(int maxParticles, byte maxNeighborsPerParticle)
        {
            throw new NotImplementedException();
        }

        public override IntPtr CreateTriangleMesh()
        {
            throw new NotImplementedException();
        }

        public override void DestroySDF(IntPtr sdf)
        {
            throw new NotImplementedException();
        }

        public override void DestroySolver()
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
            throw new NotImplementedException();
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
            throw new NotImplementedException();
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
            throw new NotImplementedException();
        }


        public override void SetParticles(Vector4[] p, int n)
        {
            throw new NotImplementedException();
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

        public override void Shutdown()
        {
            throw new NotImplementedException();
        }

        public override void UpdateSDF(IntPtr sdf, int dimx, int dimy, int dimz, IntPtr field, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void UpdateSolver(float dt, int substeps)
        {
            throw new NotImplementedException();
        }

        public override void UpdateTriangleMesh(IntPtr mesh, Vector3[] vertices, int[] indices, int numVertices, int numTriangles, ref Vector3 lower, ref Vector3 upper, Memory source)
        {
            throw new NotImplementedException();
        }

        public override void WaitFence()
        {
            throw new NotImplementedException();
        }

        public override void SetMassesInv(float[] m, int n)
        {
            throw new NotImplementedException();
        }

        public override void GetMassesInv(float[] m, int n)
        {
            throw new NotImplementedException();
        }
    }
}
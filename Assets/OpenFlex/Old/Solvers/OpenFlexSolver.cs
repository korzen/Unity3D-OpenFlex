//using System;
//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;


//namespace OpenFlex
//{


//    public class OpenFlexSolver : MonoBehaviour
//    {
//        public FlexContainer flexContainer;
//        public UniformGrid uniformGrid;

//        public SolverParams solverParams;
//        public FluidsParams fluidsParams;
        

//        private OpenFlexAPI openFlex;
        
  

//        void Start()
//        {
//            openFlex = new OpenFlexCPU_MT();

//            if(flexContainer == null)
//                flexContainer = GetComponent<FlexContainer>();

//            if (uniformGrid == null)
//                uniformGrid = GetComponent<UniformGrid>();

//            openFlex.CreateSolver(flexContainer.m_maxParticlesCount, 64);
//            openFlex.SetUniformGrid(uniformGrid);

//            flexContainer.UpdateContainer();

//            SetParticles(flexContainer);

//        }



//        void FixedUpdate()
//        {
//            //SetParticles(flexContainer);

//            //StepPhysics();

//            //GetParticles(flexContainer);

//            //UpdateGameObjects();
//        }

//        void Update()
//        {
//            fluidsParams.kernel.RecalculateSecondaryParameters();

//            openFlex.SetParams(solverParams, fluidsParams);

//            SetParticles(flexContainer);

//            StepPhysics();

//            GetParticles(flexContainer);

//            UpdateGameObjects();
//        }

//        void LateUpdate()
//        {

//        }

//        private void StepPhysics()
//        {
//            openFlex.UpdateSolver(solverParams.timeStep, solverParams.solverIterationsCount);
//        }

//        private void SetParticles(FlexContainer cnt)
//        {
//            openFlex.SetParticles(flexContainer.m_particles, flexContainer.m_particlesCount);
//            //Flex.SetActive(solverPtr, cnt.m_activeSetHndl.AddrOfPinnedObject(), cnt.m_activeParticlesCount, memory);

//            //Flex.SetParticles(solverPtr, cnt.m_particlesHndl.AddrOfPinnedObject(), cnt.m_particlesCount, memory);
//            //Flex.SetRestParticles(solverPtr, cnt.m_restParticlesHndl.AddrOfPinnedObject(), cnt.m_particlesCount, memory);
//            //Flex.SetVelocities(solverPtr, cnt.m_velocitiesHndl.AddrOfPinnedObject(), cnt.m_particlesCount, memory);
//            //Flex.SetNormals(solverPtr, cnt.m_normalsHndl.AddrOfPinnedObject(), cnt.m_particlesCount, memory);
//            //Flex.SetPhases(solverPtr, cnt.m_phases, cnt.m_particlesCount, memory);

//        }

//        private void GetParticles(FlexContainer cnt)
//        {

//            openFlex.GetParticles(flexContainer.m_particles, flexContainer.m_particlesCount);
//            //OpenFlex.GetVelocities(m_solverPtr, m_cntr.m_velocitiesHndl.AddrOfPinnedObject(), m_cntr.m_particlesCount, memory);
//            //Flex.GetPhases(m_solverPtr, m_cntr.m_phasesHndl.AddrOfPinnedObject(), m_cntr.m_particlesCount, memory);

//            //Flex.GetSmoothParticles(m_solverPtr, m_cntr.m_smoothedParticlesHndl.AddrOfPinnedObject(), m_cntr.m_particlesCount, memory);
//            //Flex.GetNormals(m_solverPtr, m_cntr.m_normalsHndl.AddrOfPinnedObject(), m_cntr.m_particlesCount, memory);
//            openFlex.GetDensities(flexContainer.m_densities, flexContainer.m_particlesCount);
//            //Flex.GetBounds(m_solverPtr, ref m_minBounds, ref m_maxBounds, memory);

//            //if (m_cntr.m_shapeCoefficients.Length > 0)
//            //    Flex.GetRigidTransforms(m_solverPtr, m_cntr.m_shapeRotationHndl.AddrOfPinnedObject(), m_cntr.m_shapeTranslationsHndl.AddrOfPinnedObject(), memory);

//        }

//        private void PushConstraintsToGPU(IntPtr solverPtr, FlexContainer cnt, Flex.Memory memory)
//        {

//            if (cnt.m_springsCount > 0)
//                Flex.SetSprings(solverPtr, cnt.m_springIndices, cnt.m_springRestLengths, cnt.m_springCoefficients, cnt.m_springsCount, memory);
//            else
//                Flex.SetSprings(solverPtr, null, null, null, 0, memory);

//            if (cnt.m_shapesCount > 0)
//                Flex.SetRigids(solverPtr, cnt.m_shapeOffsets, cnt.m_shapeIndices, cnt.m_shapeRestPositions, null, cnt.m_shapeCoefficients, cnt.m_shapeRotations, cnt.m_shapeTranslations, cnt.m_shapeOffsets.Length - 1, memory);
//            else
//                Flex.SetRigids(solverPtr, null, null, null, null, null, null, null, 0, memory);

//            if (cnt.m_trianglesCount > 0)
//                Flex.SetDynamicTriangles(solverPtr, cnt.m_triangleIndices, cnt.m_triangleNormals, cnt.m_trianglesCount, memory);
//            else
//                Flex.SetDynamicTriangles(solverPtr, null, null, 0, memory);

//            if (cnt.m_inflatablesCount > 0)
//                Flex.SetInflatables(solverPtr, cnt.m_inflatableStarts, cnt.m_inflatableCounts, cnt.m_inflatableVolumes, cnt.m_inflatablePressures, cnt.m_inflatableStiffness, cnt.m_inflatablesCount, memory);
//            else
//                Flex.SetInflatables(solverPtr, null, null, null, null, null, 0, memory);

//        }

//        private void UpdateGameObjects()
//        {

//            for (int iId = 0; iId < flexContainer.m_flexGameObjects.Count && iId < flexContainer.m_activeInstacesCount; iId++)
//            {
//                FlexParticles flexInstance = flexContainer.m_flexGameObjects[iId];
//                if (flexInstance != null)
//                {
//                    Array.Copy(flexContainer.m_particles, flexInstance.m_particlesIndex, flexInstance.m_particles, 0, flexInstance.m_particlesCount);
//                    Array.Copy(flexContainer.m_velocities, flexInstance.m_particlesIndex, flexInstance.m_velocities, 0, flexInstance.m_particlesCount);
//                    Array.Copy(flexContainer.m_densities, flexInstance.m_particlesIndex, flexInstance.m_densities, 0, flexInstance.m_particlesCount);

//                    //FlexShapeMatching shapes = flexContainer.m_flexGameObjects[iId].GetComponent<FlexShapeMatching>();
//                    //if (shapes)
//                    //{
//                    //    if (shapes.m_shapesIndex == -1)
//                    //        continue;

//                    //    Array.Copy(flexContainer.m_shapeTranslations, shapes.m_shapesIndex, shapes.m_shapeTranslations, 0, shapes.m_shapesCount);
//                    //    Array.Copy(flexContainer.m_shapeRotations, shapes.m_shapesIndex, shapes.m_shapeRotations, 0, shapes.m_shapesCount);
//                    //}
//                }
//            }

//        }


//        private void OnApplicationQuit()
//        {
//            if (openFlex != null)
//                openFlex.Shutdown();
//        }

//    }
//}
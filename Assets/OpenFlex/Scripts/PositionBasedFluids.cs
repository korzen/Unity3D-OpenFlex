using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

namespace OpenFlex
{
    [Serializable]
    public struct PBFKernels
    {
        public float H; //0.1f

        public float C; //0.01f
        public float K; //0.001f

        private float deltaQMag;
        private float KPOLY;
        private float SPIKY;
        private float wQH;

        public void RecalculateSecondaryParameters()
        {
            this.KPOLY = 315.0f / (64.0f * Mathf.PI * Mathf.Pow(H, 9));
            this.SPIKY = 45.0f / (Mathf.PI * Mathf.Pow(H, 6));
            this.deltaQMag = 0.3f * H; //0.1f - 0.3f
            this.wQH = KPOLY * Mathf.Pow((H * H - deltaQMag * deltaQMag), 3);
        }

        public float WPoly6(Vector3 pi, Vector3 pj)
        {
            Vector3 r = pi - pj;
            float rLen = Vector3.Magnitude(r);
            if (rLen > H || rLen == 0)
            {
                return 0;
            }

            return KPOLY * Mathf.Pow((H * H - Vector3.Dot(r, r)), 3);
        }

        public float WPoly6(float4 pi, float4 pj)
        {
            float4 r = pi - pj;
            float rLen = math.length(r);
            if (rLen > H || rLen == 0)
            {
                return 0;
            }

            return KPOLY * math.pow((H * H - math.dot(r, r)), 3);
        }

        public Vector3 gradWPoly6(Vector3 pi, Vector3 pj)
        {
            Vector3 r = pi - pj;
            float rLen = Vector3.Magnitude(r);
            if (rLen > H || rLen == 0)
            {
                return new Vector3(0.0f, 0.0f, 0.0f);
            }

            float coeff = Mathf.Pow((H * H) - (rLen * rLen), 2);
            coeff *= -6 * KPOLY;
            return r * coeff;
        }


        public Vector3 WSpiky(Vector3 pi, Vector3 pj)
        {
            Vector3 r = pi - pj;
            float rLen = Vector3.Magnitude(r);
            if (rLen > H || rLen == 0)
            {
                return new Vector3(0.0f, 0.0f, 0.0f);
            }

            float coeff = (H - rLen) * (H - rLen);
            coeff *= SPIKY;
            coeff /= rLen;
            return r * -coeff;
        }

        public float4 WSpiky(float4 pi, float4 pj)
        {
            float4 r = pi - pj;
            float rLen = math.length(r);
            if (rLen > H || rLen == 0)
            {
                return new float4(0.0f, 0.0f, 0.0f, 0.0f);
            }

            float coeff = (H - rLen) * (H - rLen);
            coeff *= SPIKY;
            coeff /= rLen;
            return r * -coeff;
        }

        public float WAirPotential(Vector3 pi, Vector3 pj)
        {
            Vector3 r = pi - pj;
            float rLen = Vector3.Magnitude(r);
            if (rLen > H || rLen == 0)
            {
                return 0.0f;
            }

            return 1 - (rLen / H);
        }


        public float sCorrCalc(Vector3 pi, Vector3 pj)
        {
            float corr = WPoly6(pi, pj) / wQH;
            corr *= corr * corr * corr;
            return -K * corr;
        }

        public float sCorrCalc(float4 pi, float4 pj)
        {
            float corr = WPoly6(pi, pj) / wQH;
            corr *= corr * corr * corr;
            return -K * corr;
        }
    }

    [Serializable]
    public struct FluidsParams
    {
        public float restDensity;
        public float epsilonLambda;
        public PBFKernels kernel;

    }

    public class PositionBasedFluids
    {

        [ComputeJobOptimization]
        public static void CalculateLambdas(Vector4[] positions, float[] densities, float[] lambdas, int particlesCount, int[] particlesNeighbours, int[] particlesNeighboursCount, int maxNeighboursPerParticle, float restDensity, float epsilonLambda, PBFKernels kernels)
        {

            for (int idA = 0; idA < particlesCount; idA++)
            {

                Vector3 predPosA = positions[idA];

                float density = 0.0f;
                Vector3 gradientI = new Vector3();
                float sumGradients = 0.0f;



                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];
                    Vector3 predPosB = positions[idB];

                    //Calculate the lambda value for pressure correction
                    density += kernels.WPoly6(predPosA, predPosB);

                    //Calculate gradient with respect to j
                    Vector3 gradientJ = kernels.WSpiky(predPosA, predPosB) / restDensity;
                    //float3 gradientJ = gradWPoly6(predPosA, predPosB) / REST_DENSITY;


                    //Add magnitude squared to sum
                    //sumGradients += glm::length2(gradientJ);
                    sumGradients += Vector3.Dot(gradientJ, gradientJ);
                    gradientI += gradientJ;
                }

                //Add the particle i gradient magnitude squared to sum
                sumGradients += Vector3.Dot(gradientI, gradientI);

                float densityConstraint = (density / restDensity) - 1.0f;

                densities[idA] = density;
                lambdas[idA] = (-1.0f) * densityConstraint / (sumGradients + epsilonLambda);

            }

        }

   
        public static void ProjectFluidConstraints(Vector4[] particles, float[] densities, float[] lambdas, int particlesCount, int[] particlesNeighbours, int[] particlesNeighboursCount, int maxNeighboursPerParticle, float restDensity, PBFKernels kernels, Vector4[] deltaPositions)
        {

            for (int idA = 0; idA < particlesCount; idA++)
            {
                Vector3 predPosA = particles[idA];
                float lambda = lambdas[idA];
                Vector3 deltaP = new Vector3();

                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                    Vector3 predPosB = particles[idB];

                    float lambdaSum = lambda + lambdas[idB];
                    float sCorr = kernels.sCorrCalc(predPosA, predPosB);

                    deltaP += kernels.WSpiky(predPosA, predPosB) * (lambdaSum + sCorr);
                }

                deltaPositions[idA] = deltaP / restDensity;
            }

        }



        [ComputeJobOptimization]
        public struct CalculateLambdasJob : IJobParallelFor
        {
            public int maxNeighboursPerParticle;

            public float restDensity;
            public float epsilonLambda;

            public PBFKernels kernels;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            public NativeArray<float> densities;
            public NativeArray<float> lambdas;

            [ReadOnly]
            public NativeArray<int> particlesNeighbours;

            [ReadOnly]
            public NativeArray<int> particlesNeighboursCount;

            public void Execute(int idA)
            {

                Vector3 predPosA = positions[idA];

                float density = 0.0f;
                Vector3 gradientI = new Vector3();
                float sumGradients = 0.0f;

                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];
                    Vector3 predPosB = positions[idB];

                    //Calculate the lambda value for pressure correction
                    density += kernels.WPoly6(predPosA, predPosB);

                    //Calculate gradient with respect to j
                    Vector3 gradientJ = kernels.WSpiky(predPosA, predPosB) / restDensity;
                    //float3 gradientJ = gradWPoly6(predPosA, predPosB) / REST_DENSITY;


                    //Add magnitude squared to sum
                    //sumGradients += glm::length2(gradientJ);
                    sumGradients += Vector3.Dot(gradientJ, gradientJ);
                    gradientI += gradientJ;
                }

                //Add the particle i gradient magnitude squared to sum
                sumGradients += Vector3.Dot(gradientI, gradientI);

                float densityConstraint = (density / restDensity) - 1.0f;

                densities[idA] = density;
                lambdas[idA] = (-1.0f) * densityConstraint / (sumGradients + epsilonLambda);
            }
        }

        [ComputeJobOptimization]
        public struct ProjectFluidConstraintsJob : IJobParallelFor
        {
            public int maxNeighboursPerParticle;

            public float restDensity;
            public float epsilonLambda;

            public PBFKernels kernels;

            public NativeArray<Vector4> deltaPositions;

            [ReadOnly]
            public NativeArray<Vector4> positions;

            [ReadOnly]
            public NativeArray<float> densities;

            [ReadOnly]
            public NativeArray<float> lambdas;

            [ReadOnly]
            public NativeArray<int> particlesNeighbours;

            [ReadOnly]
            public NativeArray<int> particlesNeighboursCount;

            public void Execute(int idA)
            {
                //Vector3 predPosA = positions[idA];
                //float lambda = lambdas[idA];
                //Vector3 deltaP = new Vector3();

                //for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                //{
                //    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                //    Vector3 predPosB = positions[idB];

                //    float lambdaSum = lambda + lambdas[idB];
                //    float sCorr = kernels.sCorrCalc(predPosA, predPosB);

                //    deltaP += kernels.WSpiky(predPosA, predPosB) * (lambdaSum + sCorr);
                //}

                //deltaPositions[idA] = deltaP / restDensity;

                float4 predPosA = positions[idA];
                float lambda = lambdas[idA];
                float4 deltaP = new float4();

                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                    float4 predPosB = positions[idB];

                    float lambdaSum = lambda + lambdas[idB];
                    float sCorr = kernels.sCorrCalc(predPosA, predPosB);

                    deltaP += kernels.WSpiky(predPosA, predPosB) * (lambdaSum + sCorr);
                }

                deltaPositions[idA] = deltaP / restDensity;
            }
        }
    }
}
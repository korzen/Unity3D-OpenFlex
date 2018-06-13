using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;


namespace OpenFlex
{
    public class PositionBasedDynamics
    {


        public static void PredictPositions(Vector4[] positions, Vector4[] predPositions, Vector4[] velocities, float[] massesInv, int particlesCount, Vector4 acceleration, float dt)
        {
            for (int i = 0; i < particlesCount; i++)
            {
                if (massesInv[i] != 0.0)
                {
                    velocities[i] += acceleration * dt;
                    predPositions[i] = positions[i] + velocities[i] * dt;
                }
            }
        }

        public static void UpdateVelocities(Vector4[] positions, Vector4[] predPositions, Vector4[] velocities, float[] massesInv, int particlesCount, float dt)
        {
            float dtInv = 1.0f / dt;
            for (int i = 0; i < particlesCount; i++)
            {
                if (massesInv[i] != 0.0)
                {
                    velocities[i] = (predPositions[i] - positions[i]) * dtInv;
                    positions[i]  = predPositions[i];
                }
            }
        }

        public static void ProjectParticlesVsPhysXCollisions(Vector4[] positions, Vector4[] predPositions, float[] massesInv, int particlesCount, float radius)
        {

            for (int i = 0; i < particlesCount; i++)
            {
                RaycastHit hit;
                Vector3 dir = predPositions[i] - positions[i];
                float dirMag = dir.magnitude;

                if (dirMag > 0.001f && Physics.SphereCast(positions[i], radius, dir.normalized, out hit, dirMag))
             //   if (dirMag > 0.001f && Physics.Raycast(positions[i], dir.normalized, out hit, dirMag))
                {
                    predPositions[i] = hit.point + hit.normal * radius * 1.01f;
  

                 //   positions[i] = hit.point + hit.normal * radius * 1.01f;
                 //   positions[i].w = 1; //TODO
                    //  Debug.DrawRay(positions[i], dir, Color.red);
                }

            }
        }

        public static void ProjectParticlesFloorBounds(Vector4[] positions, Vector4[] predPositions, int particlesCount, float floorLevel, float radius)
        {
            float y = floorLevel + radius;
            for (int i = 0; i < particlesCount; i++)
            {
                if (predPositions[i].y < y)
                {
                    predPositions[i].y = y;
                 //   positions[i].y = y;
                }
            }
        }

        public static void ProjectParticlesFloorBounds(NativeArray<Vector4> positions, NativeArray<Vector4> predPositions, int particlesCount, float floorLevel, float radius)
        {
            float y = floorLevel + radius;
            for (int i = 0; i < particlesCount; i++)
            {
                Vector4 v = predPositions[i];
                if (v.y < y)
                {
                    v.y = y;
                    predPositions[i] = v;
                  //  positions[i] = v;
                }
            }
        }

        public static void ProjectParticlesToBounds(NativeArray<Vector4> positions, NativeArray<Vector4> predPositions, int particlesCount, float radius, Vector3 minBounds, Vector3 maxBounds)
        {
            for (int i = 0; i < particlesCount; i++)
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

        public static void ProjectParticlesVsParticlesCollisions(Vector4[] positions, int particlesCount, float radius, float kS)
        {
            float radiusSum = radius + radius;
            float radiusSumSq = radiusSum * radiusSum;
            for (int i = 0; i < particlesCount; i++)
            {
                for (int j = i+1; j < particlesCount; j++)
                {
                    Vector3 dir = positions[j] - positions[i];
                    float dist = dir.magnitude;
                    if (dist < radiusSum)
                    {
                        Vector4 resp =  (radiusSum - dist + 0.01f) * dir.normalized * kS;
                        positions[i] -= resp;
                        positions[j] += resp;
                    }
                }
            }
        }



        public static void ProjectParticlesVsParticlesCollisions(Vector4[] positions, float[] massesInv, int particlesCount, int[] particlesNeighbours, int[] particlesNeighboursCount, int maxNeighboursPerParticle, float radius, float kS)
        {
            float radiusSum = radius + radius;
            float radiusSumSq = radiusSum * radiusSum;

            for (int idA = 0; idA < particlesCount; idA++)
            {

                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                    Vector4 dir = positions[idA] - positions[idB];
                    dir.w = 0;
                    float distanceSq = Vector4.SqrMagnitude(dir);

                    if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                        continue;

                    float distance = Mathf.Sqrt(distanceSq);

                    float wA = massesInv[idA];
                    float wB = massesInv[idB];

                    Vector4 dP = (1.0f / (wA + wB)) * (distance - radiusSum) * (dir / distance) * kS;
        
                    positions[idA] -= dP * wA;
                    positions[idB] += dP * wB;
                }


            }
        }

        public static void ProjectParticlesVsParticlesCollisions(NativeArray<Vector4> positions, int particlesCount, NativeArray<int> particlesNeighbours, NativeArray<int> particlesNeighboursCount, int maxNeighboursPerParticle, float radius, float kS)
        {
            float radiusSum = radius + radius;
            float radiusSumSq = radiusSum * radiusSum;

            for (int idA = 0; idA < particlesCount; idA++)
            {

                for (int nId = 0; nId < particlesNeighboursCount[idA]; nId++)
                {
                    int idB = particlesNeighbours[idA * maxNeighboursPerParticle + nId];

                    Vector4 dir = positions[idA] - positions[idB];
                    dir.w = 0;
                    float distanceSq = Vector4.SqrMagnitude(dir);

                    if (idA == idB || distanceSq > radiusSumSq || distanceSq <= float.Epsilon)
                        continue;

                    float distance = Mathf.Sqrt(distanceSq);

                    //  float w1 = body.posLocks[idA] ? 0.0f : body.massesInv[idA];
                    //  float w2 = body.posLocks[idB] ? 0.0f : body.massesInv[idB];

                    // float invMass = w1 + w2;

                    //Vector4 dP = (1.0f / invMass) * (distance - radiusSum) * (dir / distance) * kS;
                    Vector4 dP = (distance - radiusSum) * (dir / distance) * kS;

                    positions[idA] -= dP;
                    positions[idB] += dP;
                }


            }
        }



    }
}
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;



namespace OpenFlex.ECS
{
    public class OpenFlexECS : MonoBehaviour
    {
        
        public DefKit.TetMesh tetMesh;
        public int gridSize = 16;

        public static EntityManager EntityManager;
        public static EntityArchetype ParticleArchetype;
        public static MeshInstanceRenderer ParticleLook;
        

        void Start()
        {
            EntityManager = World.Active.GetOrCreateManager<EntityManager>();

            ParticleArchetype = EntityManager.CreateArchetype(
                typeof(Position),
                typeof(PredictedPositions),
                typeof(MassInv),
                typeof(Velocity),
                typeof(DeltaPositions),
                typeof(TransformMatrix));

            ParticleLook = GetLookFromPrototype("ParticleRenderPrototype");

            AddParticleGrid(gridSize, gridSize, gridSize);
           // AddSoftbody(tetMesh);
        }

        private void AddParticleGrid(int xSize, int ySize, int zSize)
        {
            int particlesCount = xSize * ySize * zSize;
            NativeArray<Entity> particles = new NativeArray<Entity>(particlesCount, Allocator.Temp);
            EntityManager.CreateEntity(ParticleArchetype, particles);
            for (int i = 0; i < particlesCount; i++)
            {
                EntityManager.SetComponentData(particles[i], new MassInv { Value = 1 });
                EntityManager.AddSharedComponentData(particles[i], ParticleLook);
            }

            int pId = 0;
            for (int x = 0; x < xSize; x++)
            {
                for (int y = 0; y < ySize; y++)
                {
                    for (int z = 0; z < zSize; z++)
                    {
                        float rand = UnityEngine.Random.Range(-0.1f, 0.1f);
                        float3 rand3 = new float3(rand, rand, rand);
                        float3 pos = transform.TransformPoint(new float3(x, y, z));
                        
                        EntityManager.SetComponentData(particles[pId], new Position { Value = pos + rand3});
                        pId++;

                    }
                }
            }
            particles.Dispose();
        }

        private void AddSoftbody(DefKit.TetMesh tetMesh)
        {


            int[] constraintsCounter = new int[tetMesh.pointsCount];
            for (int i = 0; i < tetMesh.edgesCount; i++)
            {
                //bilateral constraints
                Entity distanceConstraint = EntityManager.CreateEntity(typeof(DistanceConstraint));
                EntityManager.SetComponentData(distanceConstraint, new DistanceConstraint
                {
                    idA = tetMesh.edges[i].idA,
                    idB = tetMesh.edges[i].idB,
                    restLength = tetMesh.edges[i].restLength
                });

                constraintsCounter[tetMesh.edges[i].idA]++;
                constraintsCounter[tetMesh.edges[i].idB]++;
            }

            int maxCount = 0;
            for (int i = 0; i < tetMesh.pointsCount; i++)
            {
                maxCount = math.max(maxCount, constraintsCounter[i]);
                constraintsCounter[i] = 0;
            }

            NativeArray<Entity> particles = new NativeArray<Entity>(tetMesh.pointsCount, Allocator.Temp);
            EntityManager.CreateEntity(ParticleArchetype, particles);
            for (int i = 0; i < tetMesh.pointsCount; i++)
            {
                EntityManager.SetComponentData(particles[i], new Position { Value = tetMesh.nodesPositions[i] });
                EntityManager.SetComponentData(particles[i], new MassInv { Value = 1 });
                EntityManager.AddComponent(particles[i], ComponentType.FixedArray(typeof(DistanceConstraintUni), maxCount));
                EntityManager.AddSharedComponentData(particles[i], ParticleLook);

                //fill the arrays with -1s (end of entry marker)
                NativeArray<DistanceConstraintUni> cnstrsArr = EntityManager.GetFixedArray<DistanceConstraintUni>(particles[i]);
                for (int c = 0; c < maxCount; c++)
                    cnstrsArr[c] = new DistanceConstraintUni { otherId = -1, restLength = 0};

            }


            //unilateral constraints
            for (int i = 0; i < tetMesh.edgesCount; i++)
            {
                int idA = tetMesh.edges[i].idA;
                int idB = tetMesh.edges[i].idB;
                float restLength = tetMesh.edges[i].restLength;

                NativeArray<DistanceConstraintUni> cnstrsArrA = EntityManager.GetFixedArray<DistanceConstraintUni>(particles[idA]);
                NativeArray<DistanceConstraintUni> cnstrsArrB = EntityManager.GetFixedArray<DistanceConstraintUni>(particles[idB]);

                cnstrsArrA[constraintsCounter[idA]] = new DistanceConstraintUni { otherId = idB, restLength = restLength };
                cnstrsArrB[constraintsCounter[idB]] = new DistanceConstraintUni { otherId = idA, restLength = restLength };

                constraintsCounter[idA]++;
                constraintsCounter[idB]++;
            }

            particles.Dispose();

        }

        private MeshInstanceRenderer GetLookFromPrototype(string protoName)
        {
            var proto = GameObject.Find(protoName);
            var result = proto.GetComponent<MeshInstanceRendererComponent>().Value;
            Object.Destroy(proto);
            return result;
        }


        // Update is called once per frame
        void Update()
        {

        }
    }
}
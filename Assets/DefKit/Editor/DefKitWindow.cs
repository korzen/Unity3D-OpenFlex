using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Runtime.InteropServices;

namespace DefKit
{

    public class DefKitWindow : EditorWindow
    {

        [DllImport("UTet", CallingConvention = CallingConvention.Cdecl)]
        public static extern int Tetrahedralize2(Vector3[] vertices, int verticesCount, int[] triangles, int trianglesCount, float[] nodes, out int nodesCount, Link[] links, out int linksCount, Triangle[] tris, out int trisCount, Tetrahedron[] tetras, out int tetrasCount);

        [DllImport("UTet", CallingConvention = CallingConvention.Cdecl)]
        public static extern int TetrahedralizeGetSizes2(Vector3[] vertices, int verticesCount, int[] triangles, int trianglesCount, out int nodesCount, out int linksCount, out int trisCount, out int tetrasCount);

        string newName = "DefKitSoftbody";

        bool groupEnabled;

        bool addController = true;
        bool addMeshCollider = true;
        int colliderLayer = 11;
        bool generateNodes = true;
        
        bool generateSpringJoints = true;
        float springKs = 1000.0f;
        float springKd = 10.0f;
        

        Mesh inputMesh;
        MeshFilter inputMeshFilter;
        GameObject nodePrefab;

        private GameObject previousGameObject;

        void OnGUI()
        {
      

            GUILayout.Label("DefKit", EditorStyles.boldLabel);

            newName = EditorGUILayout.TextField("Name", newName);

            inputMesh =  EditorGUILayout.ObjectField("Input Mesh", inputMesh, typeof(Mesh), true) as Mesh;


            GUILayout.Label("Softbody", EditorStyles.boldLabel);

            generateNodes = EditorGUILayout.Toggle("Generate Physics", generateNodes);

            if (generateNodes)
            {
                nodePrefab = EditorGUILayout.ObjectField("Nodes Prefab", nodePrefab, typeof(GameObject), false) as GameObject;
             //   nodePrefab = EditorGUILayout.ObjectField("Nodes Prefab", nodePrefab, typeof(Rigidbody), false) as Rigidbody;
                generateSpringJoints = EditorGUILayout.Toggle("Springs Joints", generateSpringJoints);
                if (generateSpringJoints)
                {
                    springKs = EditorGUILayout.FloatField("Springs Stiffness", springKs);
                    springKd = EditorGUILayout.FloatField("Springs Damper", springKd);
                }
            }

            addMeshCollider = EditorGUILayout.Toggle("Mesh Collider", addMeshCollider);

            if (addMeshCollider)
                colliderLayer = EditorGUILayout.LayerField("Collider Layer", colliderLayer);

       //     addController = EditorGUILayout.Toggle("Softbody Controller", addController);

         //   deletePrevious = EditorGUILayout.Toggle("Replace Previous", deletePrevious);


            if (GUILayout.Button("Generate"))
            {
                Generate();

            }
        }

        private void Generate()
        {

            Vector3[] vertices = inputMesh.vertices;
            int vertexCount = inputMesh.vertexCount;

            int[] triangles = inputMesh.triangles;
            int trianglesCount = triangles.Length / 3;

            
		    int nodesCount = 0;
		    int edgesCount = 0;
		    int facesCount = 0;
            int tetrasCount = 0;


            int err = TetrahedralizeGetSizes2(vertices, vertexCount, triangles, trianglesCount, out nodesCount, out edgesCount, out facesCount, out tetrasCount);

            if (err != 0)
            {
                PrintError(err);
                return;
            }

            float[] nodesOut = new float[nodesCount * 3];
            float[] attribsOut = new float[nodesCount * 2];
            Link[] edgesOut = new Link[edgesCount];
            int[] edgesNeighboursOut = new int[edgesCount];
            Triangle[] facesOut = new Triangle[facesCount];
            int[] facesNeighboursOut = new int[facesCount * 2];
            int[] facesMarkersOut = new int[facesCount];


            Tetrahedron[] tetrasOut = new Tetrahedron[tetrasCount];


            int err2 = Tetrahedralize2(vertices, vertexCount, triangles, trianglesCount, nodesOut, out nodesCount, edgesOut, out edgesCount, facesOut, out facesCount, tetrasOut, out tetrasCount);
            if (err2 != 0)
            {
                PrintError(err2);
                return;
            }

            GameObject go = new GameObject();
            go.name = newName;

            go.layer = colliderLayer;

            TetMesh tetMesh = go.AddComponent<TetMesh>();
            tetMesh.InitArrays(nodesCount);

            for (int i = 0; i < nodesCount; i++)
            {
                tetMesh.nodesPositions[i] = new Vector3(nodesOut[i * 3 + 0], nodesOut[i * 3 + 1], nodesOut[i * 3 + 2]);
            }

            tetMesh.edges = new Link[edgesCount];
            tetMesh.edgesCount = edgesCount;
            for (int i = 0; i < edgesCount; i++)
            {
                tetMesh.edges[i] = edgesOut[i];
            }

            tetMesh.trianglesCount = facesCount;
            tetMesh.triangles = new Triangle[facesCount];
            for (int i = 0; i < facesCount; i++)
            {
                tetMesh.triangles[i] = facesOut[i];
 
            }

            tetMesh.tetrasCount = tetrasCount;
            tetMesh.tetras = new Tetrahedron[tetrasCount];
            for (int i = 0; i < tetrasCount; i++)
            {
                tetMesh.tetras[i] = tetrasOut[i];
            }


            //NODES GENERATING
            if (generateNodes && nodePrefab != null)
            {
                Rigidbody[] rigidbodies = new Rigidbody[tetMesh.pointsCount];


                PhysXMassSpringModel physXmsm = go.AddComponent<PhysXMassSpringModel>();
                physXmsm.m_rigidBodiesCount = nodesCount;

                physXmsm.m_rigidBodies = rigidbodies;
                physXmsm.m_stiffness = springKs;
                physXmsm.m_damping = springKd;

                PhysXMeshUpdater softMesh = go.AddComponent<PhysXMeshUpdater>();

                for (int i = 0; i < tetMesh.pointsCount; i++)
                {
                    GameObject nodeGO = PrefabUtility.InstantiatePrefab(nodePrefab) as GameObject;
                    nodeGO.transform.position = tetMesh.nodesPositions[i];

                    nodeGO.name = newName+"Node_" + i;
                    nodeGO.transform.parent = tetMesh.transform;

                    rigidbodies[i] = nodeGO.GetComponent<Rigidbody>();


                }

                if (generateSpringJoints)
                {
                    SpringJoint[] springJoints = new SpringJoint[tetMesh.edgesCount];
        
                    for (int i = 0; i < tetMesh.edgesCount; i++)
                    {
                        Link link = tetMesh.edges[i];

                        int idA = link.idA;
                        int idB = link.idB;

                        Rigidbody rbA = rigidbodies[idA].GetComponent<Rigidbody>();
                        Rigidbody rbB = rigidbodies[idB].GetComponent<Rigidbody>();

                        SpringJoint sj = rbA.gameObject.AddComponent<SpringJoint>();
                        sj.connectedBody = rbB;
                        sj.spring = springKs;
                        sj.damper = springKd;

                        springJoints[i] = sj;
                    }
                    physXmsm.m_springsCount = edgesCount;
                    physXmsm.m_springJoints = springJoints;
                }
            }

            //MESH GENERATING
            MeshFilter meshFilter = go.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = go.AddComponent<MeshRenderer>();

            Material mat = new Material(Shader.Find("Diffuse"));
            mat.name = this.newName + "Mat";
            meshRenderer.material = mat;

            meshFilter.sharedMesh = this.inputMesh;
            meshFilter.sharedMesh.MarkDynamic();

            if (addMeshCollider)
            {
                MeshCollider mc = go.AddComponent<MeshCollider>();
                mc.sharedMesh = this.inputMesh;
            }

            //   if (deletePrevious && previousGameObject != null)
            //       DestroyImmediate(previousGameObject, false);


            previousGameObject = go;
            Selection.objects = new Object[]{go};
            Debug.Log("TetGen: "+nodesCount +" nodes, "+edgesCount +" links, "+facesCount +" faces, " +tetrasCount +" tetras.");

            this.Close();
        }

     

        private void PrintError(int errorCode)
        {
            switch (errorCode)
            {
                case 1:
                    Debug.LogError("Out of memory.");
                    break;
                case 2:
                    Debug.LogError("Encounter an internal error.");
                    break;
                case 3:
                    Debug.LogError("A self-intersection was detected.");
                    break;
                case 4:
                    Debug.LogError("A very small input feature size was detected.");
                    break;
                case 5:
                    Debug.LogError("Two very close input facets were detected.");
                    break;
                case 10:
                    Debug.LogError("An input mesh error was detected.\n"
                        +"Hint: make sure that the input mesh has no open edges");
                    break;
            }
        }


        [MenuItem("Tools/DefKit")]
        public static void ShowWindows()
        {
            DefKitWindow win = EditorWindow.GetWindow<DefKitWindow>();

            win.nodePrefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/DefKit/Prefabs/DefaultNode.prefab");
            win.inputMesh = AssetDatabase.LoadAssetAtPath<Mesh>("Assets/DefKit/Meshes/StanfordBunny.asset");

        }
    }
}
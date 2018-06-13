using UnityEngine;

namespace DefKit
{
    /// <summary>
    /// Update the visual Mesh according to its underlying physics model.
    /// (Note that due to texture seams, some visual mesh vertices may be duplicated)
    /// </summary>
    public class PhysXMeshUpdater : MonoBehaviour
    {

        [HideInInspector]
        public int[] mappings;

        private PhysXMassSpringModel m_msm;

        private Mesh m_mesh;

        private MeshCollider m_collider;

        private Vector3[] m_vertices;
        private Vector3[] m_normals;

        private float m_maxSearchDistance = 0.00001f;



        void Start()
        {
            m_msm = GetComponent<PhysXMassSpringModel>();
            m_mesh = GetComponent<MeshFilter>().mesh;
            m_collider = GetComponent<MeshCollider>();
            m_vertices = m_mesh.vertices;
            m_normals = m_mesh.normals;
            this.mappings = new int[m_mesh.vertexCount];
            for (int i = 0; i < m_mesh.vertexCount; i++)
            {
                Vector3 v = m_vertices[i];
                bool mappingFound = false;

                float minDistance = 100000.0f;
                int minId = 0;

                //     for (int j = 0; j < m_rigidBodies.Length; j++)
                for (int j = 0; j < m_msm.m_rigidBodiesCount; j++)
                {
                    float dist = Vector3.Distance(v, transform.InverseTransformPoint(m_msm.m_rigidBodies[j].position));
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        minId = j;
                    }
                }

                if (minDistance < this.m_maxSearchDistance)
                {
                    this.mappings[i] = minId;
                    mappingFound = true;
                }

                if (!mappingFound)
                    Debug.Log("MappingMissing: " + i);

            }
        }
        void Update()
        {
            for (int i = 0; i < m_mesh.vertexCount; i++)
            {
                m_vertices[i] = transform.InverseTransformPoint(m_msm.m_rigidBodies[mappings[i]].position);
                //  m_vertices[i] = transform.InverseTransformPoint(m_msm.positions[mappings[i]]);
                //  m_normals[i] = transform.InverseTransformDirection(m_msm.normals[mappings[i]]);

                //m_vertices[i] = m_rigidBodies[mappings[i]].position;
                //m_normals[i] = m_body.normals[mappings[i]];
            }
            

            m_mesh.vertices = m_vertices;


            m_mesh.RecalculateNormals();
            m_mesh.RecalculateBounds();



            if (m_collider != null)
            {
                m_collider.sharedMesh = null;
                m_collider.sharedMesh = m_mesh;
            }

        }

    }
}
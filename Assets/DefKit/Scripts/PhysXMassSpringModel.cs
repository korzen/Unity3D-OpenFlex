using UnityEngine;
using System.Collections;

namespace DefKit
{
    /// <summary>
    /// Updates the PhysX rigidbodies and spring parameters
    /// </summary>
    public class PhysXMassSpringModel : MonoBehaviour
    {

        public float m_stiffness = 1000.0f;

        public float m_damping = 10.0f;

        public int m_rigidBodiesCount = 0;

        public int m_springsCount = 0;

        [HideInInspector]
        public Rigidbody[] m_rigidBodies;

        [HideInInspector]
        public SpringJoint[] m_springJoints;


        // Use this for initialization
        void Start()
        {
            SetSpringProperties(m_stiffness, m_damping);
        }


        public void SetSpringProperties(float stiffness, float damping)
        {
            for (int i = 0; i < m_springsCount; i++)
            {
                m_springJoints[i].spring = stiffness;
                m_springJoints[i].damper = damping;
            }
        }
    }
}
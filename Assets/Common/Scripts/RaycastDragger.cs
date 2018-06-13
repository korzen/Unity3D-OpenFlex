using UnityEngine;
using System.Collections;

public class RaycastDragger : MonoBehaviour
{

    public LayerMask m_layer;
    private RaycastHit m_hit;
    private bool m_drag;

    // Use this for initialization
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        if (m_drag)
        {
            //Vector3 mousePos = Input.mousePosition;
            // Vector3 move = Camera.main.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, camTransform.position.y - myTransform.position.y)) - myTransform.position;

            Vector3 t = new Vector3(1 * Input.GetAxis("Mouse X"), 1 * Input.GetAxis("Mouse Y"), 0);
            t = Camera.main.transform.TransformDirection(t);
            m_hit.rigidbody.position += t;
        }
        else
        {
            if (Physics.Raycast(ray, out m_hit, float.MaxValue, m_layer))
            {
                GetComponent<MeshRenderer>().enabled = true;
                transform.position = m_hit.point;

                if (Input.GetMouseButtonDown(0))
                {
                    m_drag = true;
                    m_hit.rigidbody.isKinematic = true;
                }
            }
            else
            {
                GetComponent<MeshRenderer>().enabled = false;
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            if (m_drag)
            {
                m_drag = false;
                m_hit.rigidbody.isKinematic = false;
            }
        }

    }
}

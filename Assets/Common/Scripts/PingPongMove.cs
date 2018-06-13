using UnityEngine;
using System.Collections;
public class PingPongMove : MonoBehaviour
{
    public Transform farEnd;
    public float secondsPerCycle = 1f;

    private Vector3 frometh;
    private Vector3 untoeth;


    void Start()
    {
        frometh = transform.position;
        untoeth = farEnd.position;
    }

    void Update()
    {
        transform.position = Vector3.Lerp(frometh, untoeth, Mathf.SmoothStep(0f, 1f, Mathf.PingPong(Time.time / secondsPerCycle, 1f)));
    }
}
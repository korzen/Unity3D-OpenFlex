using UnityEngine;
using System.Collections;
using System;


namespace DefKit
{
    /// <summary>
    /// Holds the information about the topology (points, edges, triangles, tetrahedrons) of a tetrahedalized mesh
    /// </summary>
    public class TetMesh : MonoBehaviour
    {
        public int pointsCount = -1;
        public int edgesCount = -1;
        public int trianglesCount = -1;
        public int tetrasCount = -1;

        public float debugRadius = 0.1f;

     //   [HideInInspector]
        public int[] pointTypes;

      //  [HideInInspector]
        public Vector3[] nodesPositions;

   //     [HideInInspector]
        public Link[] edges;

   //     [HideInInspector]
        public Triangle[] triangles;

  //      [HideInInspector]
        public Tetrahedron[] tetras;


   //     [HideInInspector]
        public bool[] onSurface;


        public bool drawDebug = false;
        
        //public bool colorDebug = false;

        public virtual void Awake()
        {
            //if (this.applyTransform)
            //{
            //    Transform tr = transform;
            //    for (int i = 0; i < this.pointsCount; i++)
            //    {
            //        this.restPositions[i] = tr.TransformPoint(this.restPositions[i]);
            //        //this.predictedPositions[i] = this.initialPositions[i];
            //        //this.positions[i] = this.restPositions[i];
            //    //    this.prevPositions[i] = this.restPositions[i];
            //    }
            //}
        }

        public virtual void Start()
        {
            //Transform tr = transform;
            //for (int i = 0; i < this.pointsCount; i++)
            //{
            //    this.initialPositions[i] = tr.TransformPoint(this.initialPositions[i]);
            //    //this.predictedPositions[i] = this.initialPositions[i];
            //    this.positions[i] = this.initialPositions[i];
            //    this.prevPositions[i] = this.initialPositions[i];
            //}


        }



        public virtual void InitArrays(int pointsCount)
        {
            this.pointsCount = pointsCount;
            this.nodesPositions = new Vector3[pointsCount];
            this.onSurface = new bool[pointsCount];
        }

      

        public virtual void OnDrawGizmos()
        {
            Transform transform = this.transform;


            if (this.drawDebug && this.edges != null)
            {

                for (int i = 0; i < this.edgesCount; i++)
                {
                    Link link = this.edges[i];

                    if (link.type == -1)
                        continue;
                    else if(link.type == 0)
                        Gizmos.color = Color.red;
                    else if(link.type == 1)
                        Gizmos.color = Color.green;
                    else if (link.type == 2)
                        Gizmos.color = Color.blue;
                    else
                        Gizmos.color = Color.white;

                    Gizmos.DrawLine(transform.TransformPoint(this.nodesPositions[link.idA]), transform.TransformPoint(this.nodesPositions[link.idB]));

                }
            }


            if (this.drawDebug && this.nodesPositions != null)
            {
                Gizmos.color = Color.green;
                for (int i = 0; i < this.pointsCount; i++)
                {
                    //   
                    //if(colorDebug)
                    //{
                    // //   Gizmos.color = this.colors[i];
                    //    Gizmos.DrawSphere(this.positions[i], this.debugRadius);
                    //}
                    //else
                    {
                        Gizmos.DrawSphere(transform.TransformPoint(this.nodesPositions[i]), this.debugRadius);
                    }
                    
                }
            }



        }
    }


}
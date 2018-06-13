using UnityEngine;
using System;

namespace DefKit
{
    [Serializable]
    public struct Link 
    {
        public int idA;
        public int idB;
        public float restLength;
        public int type;


        /// <summary>
        /// Link (spring) structure
        /// </summary>
        public Link(int id1, int id2, float restLength)
        {
            
            this.idA = id1;
            this.idB = id2;
            this.restLength = restLength;
            this.type = 0;
        }

        public Link(int id1, int id2, Vector3 pos1, Vector3 pos2)
        {
            this.idA = id1;
            this.idB = id2;
            this.restLength = Vector3.Distance(pos1, pos2);
            this.type = 0;
        }

        public void Init(int id1, int id2, float restLength)
        {
            this.idA = id1;
            this.idB = id2;

            this.restLength = restLength;
        }

        public void Init(int id1, int id2, Vector3 pos1, Vector3 pos2)
        {
            this.idA = id1;
            this.idB = id2;

            this.restLength = Vector3.Distance(pos1, pos2);
        }

        public bool contains(int id1, int id2)
        {
            if ((this.idA == id1 && this.idB == id2) || (this.idA == id2 && this.idB == id1))
                return true;

            return false;
        }

        public bool contains(int id)
        {
            if (this.idA == id || this.idB == id)
                return true;

            return false;
        }
    }

    
}
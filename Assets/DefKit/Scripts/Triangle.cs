using UnityEngine;
using System;

namespace DefKit
{
    /// <summary>
    /// Stores three points indices consisting ona triangle and it's id
    /// </summary>
    [Serializable]
    public struct Triangle
    {

        public int id;
        public int pointAid;
        public int pointBid;
        public int pointCid;

        public Triangle(int id, int idA, int idB, int idC)
        {
            this.id = id;
            this.pointAid = idA;
            this.pointBid = idB;
            this.pointCid = idC;
        }

        public int GetById(int id)
        {
            if (id == 0)
                return pointAid;
            else if (id == 1)
                return pointBid;
            else if (id == 2)
                return pointCid;

            return -1;
        }
    }
}
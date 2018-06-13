using UnityEngine;
using System.Collections;
using System;

namespace DefKit
{
    /// <summary>
    /// Stores the four points indices consisting on a tetrahedron as well as it's current and rest volume
    /// </summary>
    [Serializable]
    public struct Tetrahedron 
    {

            public int pA;
            public int pB;
            public int pC;
            public int pD;

            public float restVolume;
            public float volume;

            public Tetrahedron(int mp0, int mp1, int mp2, int mp3, float volume)
            {
                pA = mp0;
                pB = mp1;
                pC = mp2;
                pD = mp3;
                this.volume = volume;
                restVolume = volume;
            }
    }
}
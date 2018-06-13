using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Transforms2D;
using System;

namespace OpenFlex.ECS
{
    [Serializable]
    public struct MassInv : IComponentData
    {
        public float Value;
    }

    public struct PredictedPositions : IComponentData
    {
        public float3 Value;
    }

    public struct DeltaPositions : IComponentData
    {
        public float3 Delta;
        public int ConstraintsCount;
    }

    public struct Velocity : IComponentData
    {
        public float3 Value;
    }

    public struct Phase : IComponentData
    {
        public int Value;
    }

    //bilateral constraint
    public struct DistanceConstraint : IComponentData
    {
        public int idA;
        public int idB;
        public float restLength;
    }

    //unilateral constraint
    public struct DistanceConstraintUni
    {
        public int otherId;
        public float restLength;
    }
}
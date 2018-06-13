using System;

[Serializable]
public struct SolverParams
{

    public float timeStep;
    public int solverIterationsCount;
    public int constraintsIterationsCount;

    public float particleRadius;
    public float damping;

}
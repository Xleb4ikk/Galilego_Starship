using System;
using UnityEngine;

namespace Galilego.Physics
{
    [Serializable]
    public sealed class MoonRail
    {
        public string Name = "Moon";
        public Transform VisualTransform;
        public double Mass = 1d;
        public double SemiMajorAxis = 1d;
        public double Eccentricity;
        public double InclinationDegrees;
        public double LongitudeOfAscendingNodeDegrees;
        public double ArgumentOfPeriapsisDegrees;
        public double MeanAnomalyAtEpochDegrees;
        public double EpochTimeSeconds;
    }
}

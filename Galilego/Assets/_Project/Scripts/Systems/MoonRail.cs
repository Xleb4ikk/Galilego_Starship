using System;
using UnityEngine;

namespace Galilego.Physics
{
    [Serializable]
    public sealed class MoonRail
    {
        public string Name = "Moon";
        public Transform VisualTransform;
        public double StandardGravitationalParameter = 1d;
        public double Mass = 1d;
        public double Radius = 1d;
        public double PeriapsisDistance = 1d;
        public double ApoapsisDistance = 1d;
        public double SemiMajorAxis = 1d;
        public double Eccentricity;
        public double InclinationDegrees;
        public double LongitudeOfAscendingNodeDegrees;
        public double ArgumentOfPeriapsisDegrees;
        public double MeanAnomalyAtEpochDegrees;
        public double EpochTimeSeconds;

        public void ApplyPeriapsisAndApoapsis()
        {
            if (PeriapsisDistance <= 0d || ApoapsisDistance <= 0d)
            {
                return;
            }

            SemiMajorAxis = 0.5d * (PeriapsisDistance + ApoapsisDistance);
            Eccentricity = (ApoapsisDistance - PeriapsisDistance) / (ApoapsisDistance + PeriapsisDistance);
        }

        public void ApplySemiMajorAxisAndEccentricity()
        {
            if (SemiMajorAxis <= 0d)
            {
                return;
            }

            PeriapsisDistance = SemiMajorAxis * (1d - Eccentricity);
            ApoapsisDistance = SemiMajorAxis * (1d + Eccentricity);
        }

        public void SyncMassFromGravitationalParameter()
        {
            if (StandardGravitationalParameter <= 0d || Mass > 0d)
            {
                return;
            }

            Mass = PhysicsSolver.StandardGravitationalParameterToMass(StandardGravitationalParameter);
        }

        public double ResolveSemiMajorAxis()
        {
            if (PeriapsisDistance > 0d && ApoapsisDistance > 0d)
            {
                return 0.5d * (PeriapsisDistance + ApoapsisDistance);
            }

            return SemiMajorAxis;
        }

        public double ResolveEccentricity()
        {
            if (PeriapsisDistance > 0d && ApoapsisDistance > 0d)
            {
                return (ApoapsisDistance - PeriapsisDistance) / (ApoapsisDistance + PeriapsisDistance);
            }

            return Eccentricity;
        }

        public double ResolveStandardGravitationalParameter()
        {
            if (StandardGravitationalParameter > 0d)
            {
                return StandardGravitationalParameter;
            }

            return PhysicsSolver.MassToStandardGravitationalParameter(Mass);
        }
    }
}

// ============================================================================
// ORBITAL ELEMENTS DATA (BURST-COMPATIBLE)
// ============================================================================
// Burst-compatible structure for orbital elements calculation
// Used for parallel batch processing with IJobParallelFor

using System;
using Unity.Mathematics;
using Galilego.Core;

namespace Galilego.Core
{
    /// <summary>
    /// Burst-compatible orbital elements data structure.
    /// Uses byte instead of bool and double3 instead of Vector3d for Burst compatibility.
    /// </summary>
    public struct OrbitalElementsData
    {
        public byte IsValid;
        public byte IsBound;
        public double SemiMajorAxis;
        public double Eccentricity;
        public double3 EccentricityVector;
        public double InclinationDegrees;
        public double LongitudeOfAscendingNodeDegrees;
        public double ArgumentOfPeriapsisDegrees;
        public double TrueAnomalyDegrees;
        public double MeanAnomalyDegrees;
        public double PeriapsisDistance;
        public double ApoapsisDistance;
        public double OrbitalPeriodSeconds;
        public double SpecificOrbitalEnergy;
        public double SpecificAngularMomentum;

        public static OrbitalElementsData Invalid => new OrbitalElementsData
        {
            IsValid = 0,
            IsBound = 0,
            SemiMajorAxis = double.NaN,
            Eccentricity = double.NaN,
            EccentricityVector = new double3(double.NaN),
            InclinationDegrees = double.NaN,
            LongitudeOfAscendingNodeDegrees = double.NaN,
            ArgumentOfPeriapsisDegrees = double.NaN,
            TrueAnomalyDegrees = double.NaN,
            MeanAnomalyDegrees = double.NaN,
            PeriapsisDistance = double.NaN,
            ApoapsisDistance = double.NaN,
            OrbitalPeriodSeconds = double.NaN,
            SpecificOrbitalEnergy = double.NaN,
            SpecificAngularMomentum = double.NaN
        };

        /// <summary>
        /// Convert from managed OrbitalElements to Burst-compatible OrbitalElementsData.
        /// </summary>
        public static OrbitalElementsData FromOrbitalElements(OrbitalElements elements)
        {
            return new OrbitalElementsData
            {
                IsValid = (byte)(elements.IsValid ? 1 : 0),
                IsBound = (byte)(elements.IsBound ? 1 : 0),
                SemiMajorAxis = elements.SemiMajorAxis,
                Eccentricity = elements.Eccentricity,
                EccentricityVector = new double3(
                    elements.EccentricityVector.X,
                    elements.EccentricityVector.Y,
                    elements.EccentricityVector.Z),
                InclinationDegrees = elements.InclinationDegrees,
                LongitudeOfAscendingNodeDegrees = elements.LongitudeOfAscendingNodeDegrees,
                ArgumentOfPeriapsisDegrees = elements.ArgumentOfPeriapsisDegrees,
                TrueAnomalyDegrees = elements.TrueAnomalyDegrees,
                MeanAnomalyDegrees = elements.MeanAnomalyDegrees,
                PeriapsisDistance = elements.PeriapsisDistance,
                ApoapsisDistance = elements.ApoapsisDistance,
                OrbitalPeriodSeconds = elements.OrbitalPeriodSeconds,
                SpecificOrbitalEnergy = elements.SpecificOrbitalEnergy,
                SpecificAngularMomentum = elements.SpecificAngularMomentum
            };
        }

        /// <summary>
        /// Convert from Burst-compatible OrbitalElementsData to managed OrbitalElements.
        /// Uses reflection to access private constructor.
        /// </summary>
        public OrbitalElements ToOrbitalElements()
        {
            // Since OrbitalElements has a private constructor, we use the FromState method
            // as a workaround by reconstructing state vectors from elements.
            // However, for direct conversion, we'll use a helper method in OrbitalElements class.
            
            // This is a placeholder - actual implementation will be in OrbitalElements class
            throw new NotImplementedException("Use OrbitalElements.FromData() instead");
        }
    }
}

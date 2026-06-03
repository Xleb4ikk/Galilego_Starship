// ============================================================================
// APSIS CALCULATION JOB (BURST-COMPILED)
// ============================================================================
// Parallel calculation of periapsis and apoapsis for trajectory segments
// Replaces sequential processing in ApsisCalculator

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Core;

namespace Galilego.Simulation
{
    /// <summary>
    /// Result of apsis calculation for a single segment.
    /// Contains both periapsis and apoapsis data.
    /// </summary>
    public struct ApsisResultPair
    {
        // Periapsis data
        public double3 PePosition;
        public double PeAltitude;
        public double PeTime;
        public byte PeValid;
        
        // Apoapsis data
        public double3 ApPosition;
        public double ApAltitude;
        public double ApTime;
        public byte ApValid;
        
        public static ApsisResultPair Invalid => new ApsisResultPair 
        { 
            PeValid = 0, 
            ApValid = 0,
            PePosition = double3.zero,
            ApPosition = double3.zero,
            PeAltitude = double.NaN,
            ApAltitude = double.NaN,
            PeTime = double.NaN,
            ApTime = double.NaN
        };
    }

    /// <summary>
    /// Burst-compiled job for parallel apsis calculation.
    /// Processes multiple orbital segments simultaneously.
    /// </summary>
    [BurstCompile]
    public struct ApsisCalculationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<OrbitalElementsData> Elements;
        [ReadOnly] public NativeArray<double> SegmentStartTimes;
        [ReadOnly] public double Mu;
        [ReadOnly] public double CentralBodyRadius;
        [ReadOnly] public double CircularOrbitThreshold;
        
        [WriteOnly] public NativeArray<ApsisResultPair> Results;
        
        public void Execute(int index)
        {
            var elem = Elements[index];
            
            // Check validity
            if (elem.IsValid == 0)
            {
                Results[index] = ApsisResultPair.Invalid;
                return;
            }
            
            // Check for circular orbit (no distinct apsides)
            if (elem.Eccentricity < CircularOrbitThreshold)
            {
                Results[index] = ApsisResultPair.Invalid;
                return;
            }
            
            const double epsilon = 1e-10;
            
            // Check if eccentricity vector is valid
            if (math.lengthsq(elem.EccentricityVector) < epsilon)
            {
                Results[index] = ApsisResultPair.Invalid;
                return;
            }
            
            // Calculate periapsis distance: r_pe = a(1-e)
            double periapsisDistance = elem.SemiMajorAxis * (1.0 - elem.Eccentricity);
            
            // Direction to periapsis - normalized eccentricity vector
            double3 periapsisDirection = math.normalize(elem.EccentricityVector);
            
            // Periapsis position in astrodynamic frame
            double3 periapsisPosition = periapsisDirection * periapsisDistance;
            
            // Calculate periapsis altitude
            double periapsisAltitude = math.length(periapsisPosition) - CentralBodyRadius;
            
            // Calculate time to periapsis
            double timeToPeriapsis = double.NaN;
            if (elem.IsBound != 0 && !double.IsNaN(elem.MeanAnomalyDegrees))
            {
                // Convert mean anomaly from degrees to radians
                double M = elem.MeanAnomalyDegrees * (math.PI / 180.0);
                M = NormalizeAngle(M);
                
                // Calculate mean motion: n = sqrt(μ / a³)
                double n = math.sqrt(Mu / (elem.SemiMajorAxis * elem.SemiMajorAxis * elem.SemiMajorAxis));
                
                // Time to periapsis: Δt_pe = (2π - M) / n
                timeToPeriapsis = (2.0 * math.PI - M) / n;
                
                // Wrap if needed
                if (timeToPeriapsis > elem.OrbitalPeriodSeconds)
                {
                    timeToPeriapsis -= elem.OrbitalPeriodSeconds;
                }
            }
            
            double absolutePeTime = SegmentStartTimes[index] + timeToPeriapsis;
            
            // Initialize result
            var result = new ApsisResultPair
            {
                PePosition = periapsisPosition,
                PeAltitude = periapsisAltitude,
                PeTime = absolutePeTime,
                PeValid = 1,
                ApValid = 0
            };
            
            // Calculate apoapsis (only for elliptical orbits)
            if (elem.Eccentricity < 1.0)
            {
                // Calculate apoapsis distance: r_ap = a(1+e)
                double apoapsisDistance = elem.SemiMajorAxis * (1.0 + elem.Eccentricity);
                
                // Direction to apoapsis - opposite to periapsis
                double3 apoapsisDirection = -periapsisDirection;
                
                // Apoapsis position in astrodynamic frame
                double3 apoapsisPosition = apoapsisDirection * apoapsisDistance;
                
                // Calculate apoapsis altitude
                double apoapsisAltitude = math.length(apoapsisPosition) - CentralBodyRadius;
                
                // Calculate time to apoapsis
                double timeToApoapsis = double.NaN;
                if (elem.IsBound != 0 && !double.IsNaN(elem.MeanAnomalyDegrees))
                {
                    double M = elem.MeanAnomalyDegrees * (math.PI / 180.0);
                    M = NormalizeAngle(M);
                    
                    double n = math.sqrt(Mu / (elem.SemiMajorAxis * elem.SemiMajorAxis * elem.SemiMajorAxis));
                    
                    // Apoapsis is at M = π
                    if (M < math.PI)
                    {
                        timeToApoapsis = (math.PI - M) / n;
                    }
                    else
                    {
                        timeToApoapsis = (3.0 * math.PI - M) / n;
                    }
                }
                
                double absoluteApTime = SegmentStartTimes[index] + timeToApoapsis;
                
                result.ApPosition = apoapsisPosition;
                result.ApAltitude = apoapsisAltitude;
                result.ApTime = absoluteApTime;
                result.ApValid = 1;
            }
            
            Results[index] = result;
        }
        
        private static double NormalizeAngle(double angle)
        {
            const double twoPi = math.PI * 2.0;
            angle = angle % twoPi;
            if (angle < 0.0)
            {
                angle += twoPi;
            }
            return angle;
        }
    }
}

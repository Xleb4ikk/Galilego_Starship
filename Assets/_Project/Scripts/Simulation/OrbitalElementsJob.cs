// ============================================================================
// ORBITAL ELEMENTS JOB (BURST-COMPILED)
// ============================================================================
// Parallel batch calculation of orbital elements using Burst compilation
// Direct port of OrbitalElements.FromState() using Unity.Mathematics

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    /// <summary>
    /// Burst-compiled job for parallel calculation of orbital elements.
    /// Processes multiple state vectors simultaneously for maximum CPU utilization.
    /// </summary>
    [BurstCompile]
    public struct OrbitalElementsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double3> Positions;
        [ReadOnly] public NativeArray<double3> Velocities;
        [ReadOnly] public NativeArray<double> Mus;
        
        [WriteOnly] public NativeArray<Core.OrbitalElementsData> Results;

        public void Execute(int index)
        {
            double3 relativePosition = Positions[index];
            double3 relativeVelocity = Velocities[index];
            double mu = Mus[index];

            const double epsilon = 1e-10;

            // Validate inputs
            double radius = math.length(relativePosition);
            double speedSquared = math.lengthsq(relativeVelocity);
            
            if (radius <= epsilon || mu <= 0.0)
            {
                Results[index] = Core.OrbitalElementsData.Invalid;
                return;
            }

            // Calculate angular momentum: h = r × v
            double3 angularMomentum = math.cross(relativePosition, relativeVelocity);
            double angularMomentumMagnitude = math.length(angularMomentum);
            
            if (angularMomentumMagnitude <= epsilon)
            {
                Results[index] = Core.OrbitalElementsData.Invalid;
                return;
            }

            // Calculate node vector: n = k × h (where k = (0, 0, 1) in astrodynamic frame)
            double3 kVector = new double3(0.0, 0.0, 1.0);
            double3 node = math.cross(kVector, angularMomentum);
            double nodeMagnitude = math.length(node);

            // Calculate eccentricity vector: e = (v × h)/μ - r/|r|
            double3 eccentricityVector = 
                (math.cross(relativeVelocity, angularMomentum) / mu) - 
                (relativePosition / radius);
            
            double eccentricity = math.length(eccentricityVector);

            // Calculate specific orbital energy: ε = v²/2 - μ/r
            double energy = (0.5 * speedSquared) - (mu / radius);
            
            // Determine if orbit is parabolic
            bool isParabolic = math.abs(energy) <= epsilon;
            
            // Calculate semi-major axis: a = -μ/(2ε)
            double semiMajorAxis = isParabolic 
                ? double.PositiveInfinity 
                : -mu / (2.0 * energy);

            // Determine if orbit is bound (elliptical)
            bool isBound = !isParabolic && semiMajorAxis > 0.0 && eccentricity < 1.0;

            // Calculate periapsis distance: r_p = h²/(μ(1+e))
            double periapsisDistance = (angularMomentumMagnitude * angularMomentumMagnitude) / 
                (mu * (1.0 + eccentricity));

            // Calculate apoapsis distance: r_a = a(1+e)
            double apoapsisDistance = isBound 
                ? semiMajorAxis * (1.0 + eccentricity) 
                : double.PositiveInfinity;

            // Calculate orbital period: T = 2π√(a³/μ)
            double orbitalPeriodSeconds = isBound 
                ? 2.0 * math.PI * math.sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / mu) 
                : double.PositiveInfinity;

            // Calculate inclination: i = arccos(h_z / |h|)
            double inclination = SafeAcos(angularMomentum.z / angularMomentumMagnitude);

            // Calculate longitude of ascending node: Ω = atan2(n_y, n_x)
            double longitudeOfAscendingNode = nodeMagnitude > epsilon 
                ? NormalizeAngle(math.atan2(node.y, node.x)) 
                : 0.0;

            // Calculate argument of periapsis: ω = arccos(n·e / (|n||e|))
            double argumentOfPeriapsis = 0.0;
            if (nodeMagnitude > epsilon && eccentricity > epsilon)
            {
                argumentOfPeriapsis = SafeAcos(math.dot(node, eccentricityVector) / (nodeMagnitude * eccentricity));
                if (eccentricityVector.z < 0.0)
                {
                    argumentOfPeriapsis = (2.0 * math.PI) - argumentOfPeriapsis;
                }
            }

            // Calculate true anomaly: ν
            double trueAnomaly = 0.0;
            if (eccentricity > epsilon)
            {
                trueAnomaly = SafeAcos(math.dot(eccentricityVector, relativePosition) / (eccentricity * radius));
                if (math.dot(relativePosition, relativeVelocity) < 0.0)
                {
                    trueAnomaly = (2.0 * math.PI) - trueAnomaly;
                }
            }
            else if (nodeMagnitude > epsilon)
            {
                trueAnomaly = SafeAcos(math.dot(node, relativePosition) / (nodeMagnitude * radius));
                if (relativePosition.z < 0.0)
                {
                    trueAnomaly = (2.0 * math.PI) - trueAnomaly;
                }
            }
            else
            {
                trueAnomaly = NormalizeAngle(math.atan2(relativePosition.y, relativePosition.x));
            }

            // Calculate mean anomaly: M = E - e·sin(E)
            double meanAnomaly = double.NaN;
            if (isBound)
            {
                // Convert true anomaly to eccentric anomaly: E = 2·atan2(√(1-e)·sin(ν/2), √(1+e)·cos(ν/2))
                double eccentricAnomaly = 2.0 * math.atan2(
                    math.sqrt(1.0 - eccentricity) * math.sin(trueAnomaly * 0.5),
                    math.sqrt(1.0 + eccentricity) * math.cos(trueAnomaly * 0.5));
                
                meanAnomaly = NormalizeAngle(eccentricAnomaly - (eccentricity * math.sin(eccentricAnomaly)));
            }

            // Store results
            Results[index] = new Core.OrbitalElementsData
            {
                IsValid = 1,
                IsBound = (byte)(isBound ? 1 : 0),
                SemiMajorAxis = semiMajorAxis,
                Eccentricity = eccentricity,
                EccentricityVector = eccentricityVector,
                InclinationDegrees = RadiansToDegrees(inclination),
                LongitudeOfAscendingNodeDegrees = RadiansToDegrees(longitudeOfAscendingNode),
                ArgumentOfPeriapsisDegrees = RadiansToDegrees(argumentOfPeriapsis),
                TrueAnomalyDegrees = RadiansToDegrees(NormalizeAngle(trueAnomaly)),
                MeanAnomalyDegrees = double.IsNaN(meanAnomaly) ? double.NaN : RadiansToDegrees(meanAnomaly),
                PeriapsisDistance = periapsisDistance,
                ApoapsisDistance = apoapsisDistance,
                OrbitalPeriodSeconds = orbitalPeriodSeconds,
                SpecificOrbitalEnergy = energy,
                SpecificAngularMomentum = angularMomentumMagnitude
            };
        }

        private static double SafeAcos(double value)
        {
            return math.acos(math.clamp(value, -1.0, 1.0));
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

        private static double RadiansToDegrees(double radians)
        {
            return radians * (180.0 / math.PI);
        }
    }
}

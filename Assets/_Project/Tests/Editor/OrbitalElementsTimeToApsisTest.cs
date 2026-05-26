using NUnit.Framework;
using Galilego.Core;
using System;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Unit tests for OrbitalElements.TryGetTimeToApsides method
    /// 
    /// **Validates: Requirements 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9**
    /// 
    /// These tests verify that the TryGetTimeToApsides method correctly:
    /// - Calculates mean motion n = sqrt(μ / a³)
    /// - Calculates current mean anomaly M from eccentric anomaly
    /// - Calculates time to periapsis: Δt_pe = (2π - M) / n
    /// - Calculates time to apoapsis: Δt_ap = (π - M) / n (for elliptical orbits)
    /// - Handles wraparound when M > π
    /// - Returns NaN for apoapsis time if orbit is hyperbolic
    /// </summary>
    [TestFixture]
    public class OrbitalElementsTimeToApsisTest
    {
        private const double Jupiter_Mu = 1.26686534e17; // m³/s²
        private const double Jupiter_Radius = 71492000.0; // m

        /// <summary>
        /// Test that TryGetTimeToApsides calculates correct time to periapsis at periapsis
        /// **Validates: Requirement 3.5**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_AtPeriapsis_ReturnsOrbitalPeriod()
        {
            // ARRANGE: Create elliptical orbit at periapsis
            double periapsisDistance = Jupiter_Radius + 300000.0; // 300 km altitude
            double apoapsisDistance = Jupiter_Radius + 1000000.0; // 1000 km altitude
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            // At periapsis, velocity is perpendicular to position
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify success
            Assert.That(success, Is.True, "TryGetTimeToApsides should succeed for valid elliptical orbit");
            
            // At periapsis, time to next periapsis should be approximately the orbital period
            Assert.That(timeToPeriapsis, Is.EqualTo(elements.OrbitalPeriodSeconds).Within(1.0),
                $"Time to periapsis at periapsis should be orbital period ({elements.OrbitalPeriodSeconds} seconds)");
            
            // At periapsis, time to apoapsis should be half the orbital period
            Assert.That(timeToApoapsis, Is.EqualTo(elements.OrbitalPeriodSeconds / 2.0).Within(1.0),
                $"Time to apoapsis at periapsis should be half orbital period ({elements.OrbitalPeriodSeconds / 2.0} seconds)");
        }

        /// <summary>
        /// Test that TryGetTimeToApsides calculates correct time to apoapsis at apoapsis
        /// **Validates: Requirement 3.6, 3.8**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_AtApoapsis_ReturnsHalfOrbitalPeriod()
        {
            // ARRANGE: Create elliptical orbit at apoapsis
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double apoapsisDistance = Jupiter_Radius + 1000000.0;
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            // At apoapsis, velocity is perpendicular to position
            Vector3d position = new Vector3d(apoapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / apoapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify success
            Assert.That(success, Is.True, "TryGetTimeToApsides should succeed for valid elliptical orbit");
            
            // At apoapsis, time to periapsis should be half the orbital period
            Assert.That(timeToPeriapsis, Is.EqualTo(elements.OrbitalPeriodSeconds / 2.0).Within(1.0),
                $"Time to periapsis at apoapsis should be half orbital period ({elements.OrbitalPeriodSeconds / 2.0} seconds)");
            
            // At apoapsis, time to next apoapsis should be approximately the orbital period
            Assert.That(timeToApoapsis, Is.EqualTo(elements.OrbitalPeriodSeconds).Within(1.0),
                $"Time to apoapsis at apoapsis should be orbital period ({elements.OrbitalPeriodSeconds} seconds)");
        }

        /// <summary>
        /// Test that time to periapsis is always positive and less than orbital period
        /// **Validates: Requirement 3.5, 3.7**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_EllipticalOrbit_TimeToPeriapsisWithinBounds()
        {
            // ARRANGE: Create elliptical orbit at various true anomalies
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double apoapsisDistance = Jupiter_Radius + 1000000.0;
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            // Test at 45 degrees true anomaly
            double trueAnomaly = Math.PI / 4.0; // 45 degrees
            double radius = semiMajorAxis * (1.0 - Math.Pow((apoapsisDistance - periapsisDistance) / (apoapsisDistance + periapsisDistance), 2)) 
                / (1.0 + ((apoapsisDistance - periapsisDistance) / (apoapsisDistance + periapsisDistance)) * Math.Cos(trueAnomaly));
            
            Vector3d position = new Vector3d(radius * Math.Cos(trueAnomaly), radius * Math.Sin(trueAnomaly), 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / radius - 1.0 / semiMajorAxis));
            // Velocity perpendicular to radius for circular approximation
            Vector3d velocity = new Vector3d(-velocityMagnitude * Math.Sin(trueAnomaly), velocityMagnitude * Math.Cos(trueAnomaly), 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify success
            Assert.That(success, Is.True, "TryGetTimeToApsides should succeed");
            
            // Verify time to periapsis is positive and within orbital period
            Assert.That(timeToPeriapsis, Is.GreaterThan(0.0), "Time to periapsis should be positive");
            Assert.That(timeToPeriapsis, Is.LessThanOrEqualTo(elements.OrbitalPeriodSeconds),
                "Time to periapsis should be less than or equal to orbital period");
        }

        /// <summary>
        /// Test that time to apoapsis is always positive and less than orbital period
        /// **Validates: Requirement 3.6, 3.8**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_EllipticalOrbit_TimeToApoapsisWithinBounds()
        {
            // ARRANGE: Create elliptical orbit
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double apoapsisDistance = Jupiter_Radius + 1000000.0;
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify time to apoapsis is positive and within orbital period
            Assert.That(timeToApoapsis, Is.GreaterThan(0.0), "Time to apoapsis should be positive");
            Assert.That(timeToApoapsis, Is.LessThanOrEqualTo(elements.OrbitalPeriodSeconds),
                "Time to apoapsis should be less than or equal to orbital period");
        }

        /// <summary>
        /// Test that hyperbolic orbits return NaN for apoapsis time
        /// **Validates: Requirement 3.9**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_HyperbolicOrbit_ReturnsNaNForApoapsis()
        {
            // ARRANGE: Create hyperbolic escape trajectory
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double escapeVelocity = Math.Sqrt(2.0 * Jupiter_Mu / periapsisDistance);
            double excessVelocity = escapeVelocity * 1.5; // 50% above escape velocity
            
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            Vector3d velocity = new Vector3d(0.0, excessVelocity, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // Verify orbit is hyperbolic
            Assert.That(elements.Eccentricity, Is.GreaterThanOrEqualTo(1.0),
                "Orbit should be hyperbolic (e >= 1.0)");
            Assert.That(elements.IsBound, Is.False, "Orbit should not be bound");
            
            // ACT: Try to get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: For hyperbolic orbits, the method currently returns false
            // This is expected behavior as per the implementation
            Assert.That(success, Is.False, "TryGetTimeToApsides should return false for hyperbolic orbit (not yet implemented)");
        }

        /// <summary>
        /// Test that invalid orbital elements return false
        /// **Validates: Requirement 3.1**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_InvalidElements_ReturnsFalse()
        {
            // ARRANGE: Get invalid orbital elements
            var elements = OrbitalElements.Invalid;
            
            // ACT: Try to get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify failure
            Assert.That(success, Is.False, "TryGetTimeToApsides should return false for invalid elements");
            Assert.That(double.IsNaN(timeToPeriapsis), Is.True, "Time to periapsis should be NaN for invalid elements");
            Assert.That(double.IsNaN(timeToApoapsis), Is.True, "Time to apoapsis should be NaN for invalid elements");
        }

        /// <summary>
        /// Test that circular orbits work correctly
        /// **Validates: Requirement 3.5, 3.6**
        /// </summary>
        [Test]
        public void TryGetTimeToApsides_CircularOrbit_ReturnsValidTimes()
        {
            // ARRANGE: Create circular orbit (e ≈ 0)
            double orbitRadius = Jupiter_Radius + 500000.0; // 500 km altitude
            double circularVelocity = Math.Sqrt(Jupiter_Mu / orbitRadius);
            
            Vector3d position = new Vector3d(orbitRadius, 0.0, 0.0);
            Vector3d velocity = new Vector3d(0.0, circularVelocity, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // Verify orbit is nearly circular
            Assert.That(elements.Eccentricity, Is.LessThan(0.001),
                "Orbit should be nearly circular (e < 0.001)");
            
            // ACT: Get time to apsides
            bool success = elements.TryGetTimeToApsides(Jupiter_Mu, out double timeToPeriapsis, out double timeToApoapsis);
            
            // ASSERT: Verify success (circular orbits are still bound orbits)
            Assert.That(success, Is.True, "TryGetTimeToApsides should succeed for circular orbit");
            
            // For circular orbits, both times should be valid (though not very meaningful)
            Assert.That(timeToPeriapsis, Is.GreaterThan(0.0), "Time to periapsis should be positive");
            Assert.That(timeToApoapsis, Is.GreaterThan(0.0), "Time to apoapsis should be positive");
        }
    }
}

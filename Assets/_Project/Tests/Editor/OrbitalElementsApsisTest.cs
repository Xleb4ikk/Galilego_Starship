using NUnit.Framework;
using Galilego.Core;
using System;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Unit tests for OrbitalElements.TryGetApsisPositions method
    /// 
    /// **Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5, 2.7, 2.8, 2.9**
    /// 
    /// These tests verify that the TryGetApsisPositions method correctly:
    /// - Calculates periapsis and apoapsis positions in astrodynamic frame
    /// - Uses eccentricity vector to determine direction to periapsis
    /// - Calculates distances using r_pe = a(1-e) and r_ap = a(1+e)
    /// - Handles hyperbolic orbits (e >= 1.0) by returning only periapsis
    /// - Returns false if orbital elements are invalid
    /// </summary>
    [TestFixture]
    public class OrbitalElementsApsisTest
    {
        private const double Jupiter_Mu = 1.26686534e17; // m³/s²
        private const double Jupiter_Radius = 71492000.0; // m

        /// <summary>
        /// Test that TryGetApsisPositions calculates correct periapsis distance for elliptical orbit
        /// **Validates: Requirement 2.1**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_EllipticalOrbit_CalculatesCorrectPeriapsisDistance()
        {
            // ARRANGE: Create elliptical orbit around Jupiter
            // Periapsis: 300 km above surface, Apoapsis: 1000 km above surface
            double periapsisAltitude = 300000.0; // 300 km
            double apoapsisAltitude = 1000000.0; // 1000 km
            double periapsisDistance = Jupiter_Radius + periapsisAltitude;
            double apoapsisDistance = Jupiter_Radius + apoapsisAltitude;
            
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            double eccentricity = (apoapsisDistance - periapsisDistance) / (apoapsisDistance + periapsisDistance);
            
            // Create position and velocity vectors for this orbit
            // At periapsis, velocity is perpendicular to position
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            // Calculate orbital elements
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get apsis positions
            bool success = elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify success
            Assert.That(success, Is.True, "TryGetApsisPositions should succeed for valid elliptical orbit");
            
            // Verify periapsis distance matches formula r_pe = a(1-e)
            double expectedPeriapsisDistance = semiMajorAxis * (1.0 - eccentricity);
            Assert.That(periapsisPos.Magnitude, Is.EqualTo(expectedPeriapsisDistance).Within(1.0),
                $"Periapsis distance should be {expectedPeriapsisDistance} meters (within 1m tolerance)");
        }

        /// <summary>
        /// Test that TryGetApsisPositions calculates correct apoapsis distance for elliptical orbit
        /// **Validates: Requirement 2.2**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_EllipticalOrbit_CalculatesCorrectApoapsisDistance()
        {
            // ARRANGE: Create elliptical orbit around Jupiter
            double periapsisAltitude = 300000.0; // 300 km
            double apoapsisAltitude = 1000000.0; // 1000 km
            double periapsisDistance = Jupiter_Radius + periapsisAltitude;
            double apoapsisDistance = Jupiter_Radius + apoapsisAltitude;
            
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            double eccentricity = (apoapsisDistance - periapsisDistance) / (apoapsisDistance + periapsisDistance);
            
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get apsis positions
            bool success = elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify apoapsis distance matches formula r_ap = a(1+e)
            double expectedApoapsisDistance = semiMajorAxis * (1.0 + eccentricity);
            Assert.That(apoapsisPos.Magnitude, Is.EqualTo(expectedApoapsisDistance).Within(1.0),
                $"Apoapsis distance should be {expectedApoapsisDistance} meters (within 1m tolerance)");
        }

        /// <summary>
        /// Test that periapsis and apoapsis are in opposite directions
        /// **Validates: Requirement 2.4, 2.5**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_EllipticalOrbit_ApsisDirectionsAreOpposite()
        {
            // ARRANGE: Create elliptical orbit
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double apoapsisDistance = Jupiter_Radius + 1000000.0;
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get apsis positions
            elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify directions are opposite (dot product = -1)
            double dotProduct = Vector3d.Dot(periapsisPos.Normalized, apoapsisPos.Normalized);
            Assert.That(dotProduct, Is.EqualTo(-1.0).Within(0.01),
                "Periapsis and apoapsis should be in opposite directions");
        }

        /// <summary>
        /// Test that hyperbolic orbits return only periapsis (apoapsis is zero)
        /// **Validates: Requirement 2.3, 2.9**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_HyperbolicOrbit_ReturnsOnlyPeriapsis()
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
            
            // ACT: Get apsis positions
            bool success = elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify success
            Assert.That(success, Is.True, "TryGetApsisPositions should succeed for hyperbolic orbit");
            
            // Verify periapsis exists
            Assert.That(periapsisPos.Magnitude, Is.GreaterThan(0.0),
                "Periapsis should exist for hyperbolic orbit");
            
            // Verify apoapsis is zero (no apoapsis for hyperbolic orbit)
            Assert.That(apoapsisPos.Magnitude, Is.EqualTo(0.0).Within(0.01),
                "Apoapsis should be zero for hyperbolic orbit");
        }

        /// <summary>
        /// Test that invalid orbital elements return false
        /// **Validates: Requirement 2.9**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_InvalidElements_ReturnsFalse()
        {
            // ARRANGE: Get invalid orbital elements
            var elements = OrbitalElements.Invalid;
            
            // ACT: Try to get apsis positions
            bool success = elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify failure
            Assert.That(success, Is.False, "TryGetApsisPositions should return false for invalid elements");
            Assert.That(periapsisPos, Is.EqualTo(Vector3d.Zero), "Periapsis should be zero for invalid elements");
            Assert.That(apoapsisPos, Is.EqualTo(Vector3d.Zero), "Apoapsis should be zero for invalid elements");
        }

        /// <summary>
        /// Test that circular orbits return false (no distinct apsides)
        /// **Validates: Requirement 2.4**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_CircularOrbit_ReturnsFalse()
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
            
            // ACT: Try to get apsis positions
            bool success = elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify failure (circular orbits have no distinct apsides)
            Assert.That(success, Is.False, "TryGetApsisPositions should return false for circular orbit");
        }

        /// <summary>
        /// Test that eccentricity vector points to periapsis
        /// **Validates: Requirement 2.4**
        /// </summary>
        [Test]
        public void TryGetApsisPositions_UsesEccentricityVectorDirection()
        {
            // ARRANGE: Create elliptical orbit
            double periapsisDistance = Jupiter_Radius + 300000.0;
            double apoapsisDistance = Jupiter_Radius + 1000000.0;
            double semiMajorAxis = (periapsisDistance + apoapsisDistance) / 2.0;
            
            Vector3d position = new Vector3d(periapsisDistance, 0.0, 0.0);
            double velocityMagnitude = Math.Sqrt(Jupiter_Mu * (2.0 / periapsisDistance - 1.0 / semiMajorAxis));
            Vector3d velocity = new Vector3d(0.0, velocityMagnitude, 0.0);
            
            var elements = OrbitalElements.FromState(position, velocity, Jupiter_Mu);
            
            // ACT: Get apsis positions
            elements.TryGetApsisPositions(out Vector3d periapsisPos, out Vector3d apoapsisPos);
            
            // ASSERT: Verify periapsis direction matches eccentricity vector direction
            Vector3d eDir = elements.EccentricityVector.Normalized;
            Vector3d peDir = periapsisPos.Normalized;
            
            double dotProduct = Vector3d.Dot(eDir, peDir);
            Assert.That(dotProduct, Is.EqualTo(1.0).Within(0.01),
                "Periapsis direction should match eccentricity vector direction");
        }
    }
}

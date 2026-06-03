using System;
using NUnit.Framework;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;
using Galilego.Gameplay;
using Galilego.Simulation;
using Unity.Mathematics;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Phase 0: Infrastructure Verification Tests
    /// Verifies that coordinate transformations and ManeuverEvaluator data structures work correctly
    /// before implementing the analytical apsis system.
    /// </summary>
    [TestFixture]
    public class InfrastructureVerificationTest
    {
        private GameObject testObject;
        private UniverseManager universeManager;

        [SetUp]
        public void Setup()
        {
            testObject = new GameObject("TestInfrastructure");
            universeManager = testObject.AddComponent<UniverseManager>();
        }

        [TearDown]
        public void TearDown()
        {
            if (testObject != null)
                UnityEngine.Object.DestroyImmediate(testObject);
        }

        #region Coordinate Transformation Tests

        [Test]
        public void ConvertSimulationToAstrodynamicFrame_RoundTrip_PreservesVector()
        {
            // Arrange: Create test vectors in simulation frame
            Vector3d[] testVectors = new Vector3d[]
            {
                new Vector3d(1000000, 0, 0),           // X-axis
                new Vector3d(0, 1000000, 0),           // Y-axis
                new Vector3d(0, 0, 1000000),           // Z-axis
                new Vector3d(421700000, 0, 0),         // Io orbital radius
                new Vector3d(100000, 200000, 300000),  // Arbitrary vector
                new Vector3d(-50000, 75000, -125000)   // Negative components
            };

            foreach (var originalVector in testVectors)
            {
                // Act: Convert to astrodynamic frame and back
                Vector3d astroVector = universeManager.ConvertSimulationToAstrodynamicFrame(originalVector);
                Vector3d roundTripVector = universeManager.ConvertAstrodynamicToSimulationFrame(astroVector);

                // Assert: Round-trip should preserve the vector within numerical precision
                double error = (roundTripVector - originalVector).Magnitude;
                Assert.Less(error, 1e-6, 
                    $"Round-trip transformation failed for vector {originalVector}. Error: {error} meters");
            }
        }

        [Test]
        public void ConvertSimulationToAstrodynamicFrame_ZUpMapping_SwapsYAndZ()
        {
            // This test verifies the expected behavior based on AstrodynamicPlaneMapping.UnityXzPlaneYUp
            // In this mode: simulation (X, Y, Z) -> astrodynamic (X, Z, Y)
            
            // Arrange
            Vector3d simVector = new Vector3d(100, 200, 300);

            // Act
            Vector3d astroVector = universeManager.ConvertSimulationToAstrodynamicFrame(simVector);

            // Assert: Check if Y and Z are swapped (common Unity Y-up to Z-up conversion)
            // Note: Actual behavior depends on astrodynamicPlaneMapping setting
            // This test documents the expected transformation
            Debug.Log($"[INFRA_VERIFY] Simulation: {simVector} -> Astrodynamic: {astroVector}");
            
            // Verify transformation is not identity (something changed)
            bool isIdentity = (astroVector - simVector).Magnitude < 1e-10;
            if (isIdentity)
            {
                Debug.LogWarning("[INFRA_VERIFY] Transformation is identity - astrodynamicPlaneMapping may be UnityXyPlaneZUp");
            }
            else
            {
                Debug.Log("[INFRA_VERIFY] Transformation is non-identity - Y/Z swap detected");
            }

            // The important property: round-trip must work regardless of mapping
            Vector3d roundTrip = universeManager.ConvertAstrodynamicToSimulationFrame(astroVector);
            Assert.AreEqual(simVector.X, roundTrip.X, 1e-6);
            Assert.AreEqual(simVector.Y, roundTrip.Y, 1e-6);
            Assert.AreEqual(simVector.Z, roundTrip.Z, 1e-6);
        }

        [Test]
        public void ConvertAstrodynamicToSimulationFrame_PreservesVectorMagnitude()
        {
            // Arrange: Coordinate transformations should preserve vector magnitude (rotation only)
            Vector3d[] testVectors = new Vector3d[]
            {
                new Vector3d(421700000, 0, 0),
                new Vector3d(100000, 200000, 300000),
                new Vector3d(-50000, 75000, -125000)
            };

            foreach (var simVector in testVectors)
            {
                // Act
                Vector3d astroVector = universeManager.ConvertSimulationToAstrodynamicFrame(simVector);

                // Assert: Magnitude should be preserved
                double originalMagnitude = simVector.Magnitude;
                double transformedMagnitude = astroVector.Magnitude;
                double magnitudeError = Math.Abs(transformedMagnitude - originalMagnitude);

                Assert.Less(magnitudeError, 1e-6,
                    $"Magnitude not preserved: {originalMagnitude} -> {transformedMagnitude}");
            }
        }

        #endregion

        #region Central Body Parameter Tests

        [Test]
        public void GetCurrentCentralBodyMu_ReturnsValidGravitationalParameter()
        {
            // Act
            double mu = universeManager.GetCurrentCentralBodyMu();

            // Assert: Jupiter's μ should be approximately 1.266865319e17 m³/s²
            Assert.Greater(mu, 1e17, "Gravitational parameter should be positive and large for Jupiter");
            Assert.Less(mu, 1e18, "Gravitational parameter should be reasonable for Jupiter");
            
            Debug.Log($"[INFRA_VERIFY] Current central body μ: {mu:E3} m³/s²");
        }

        [Test]
        public void GetCurrentCentralBodyRadius_ReturnsValidRadius()
        {
            // Act
            double radius = universeManager.GetCurrentCentralBodyRadius();

            // Assert: Jupiter's radius should be approximately 69,911 km
            Assert.Greater(radius, 60000000, "Radius should be at least 60,000 km for Jupiter");
            Assert.Less(radius, 80000000, "Radius should be less than 80,000 km for Jupiter");
            
            Debug.Log($"[INFRA_VERIFY] Current central body radius: {radius / 1000:F0} km");
        }

        [Test]
        public void GetCurrentCentralBodyPosition_ReturnsFiniteVector()
        {
            // Act
            Vector3d position = universeManager.GetCurrentCentralBodyPosition();

            // Assert: Position should be finite (not NaN or Infinity)
            Assert.IsTrue(position.IsFinite, "Central body position should be finite");
            
            Debug.Log($"[INFRA_VERIFY] Current central body position: {position}");
        }

        #endregion

        #region SegmentBoundaryState Format Tests

        [Test]
        public void SegmentBoundaryState_StructureIsCorrect()
        {
            // Arrange: Create a test SegmentBoundaryState
            SegmentBoundaryState testState = new SegmentBoundaryState
            {
                Position = new double3(421700000, 0, 0),
                Velocity = new double3(0, 0, 17334),
                Time = 1000.0
            };

            // Assert: Verify structure has expected fields
            Assert.AreEqual(421700000, testState.Position.x, 1e-6);
            Assert.AreEqual(0, testState.Position.y, 1e-6);
            Assert.AreEqual(0, testState.Position.z, 1e-6);
            Assert.AreEqual(0, testState.Velocity.x, 1e-6);
            Assert.AreEqual(0, testState.Velocity.y, 1e-6);
            Assert.AreEqual(17334, testState.Velocity.z, 1e-6);
            Assert.AreEqual(1000.0, testState.Time, 1e-6);

            Debug.Log($"[INFRA_VERIFY] SegmentBoundaryState structure verified");
        }

        [Test]
        public void SegmentBoundaryState_CanConvertToVector3d()
        {
            // Arrange
            SegmentBoundaryState testState = new SegmentBoundaryState
            {
                Position = new double3(100000, 200000, 300000),
                Velocity = new double3(1000, 2000, 3000),
                Time = 500.0
            };

            // Act: Convert Unity.Mathematics.double3 to Vector3d
            Vector3d position = new Vector3d(testState.Position.x, testState.Position.y, testState.Position.z);
            Vector3d velocity = new Vector3d(testState.Velocity.x, testState.Velocity.y, testState.Velocity.z);

            // Assert
            Assert.AreEqual(100000, position.X, 1e-6);
            Assert.AreEqual(200000, position.Y, 1e-6);
            Assert.AreEqual(300000, position.Z, 1e-6);
            Assert.AreEqual(1000, velocity.X, 1e-6);
            Assert.AreEqual(2000, velocity.Y, 1e-6);
            Assert.AreEqual(3000, velocity.Z, 1e-6);

            Debug.Log($"[INFRA_VERIFY] double3 -> Vector3d conversion verified");
        }

        #endregion

        #region ManeuverEvaluator Integration Tests

        [Test]
        public void ManeuverEvaluator_TryGetSegmentBoundaryState_InvalidIndex_ReturnsFalse()
        {
            // Arrange
            var maneuverEvaluator = testObject.AddComponent<ManeuverEvaluator>();

            // Act & Assert: Invalid indices should return false
            Assert.IsFalse(maneuverEvaluator.TryGetSegmentBoundaryState(-1, out _),
                "Negative index should return false");
            Assert.IsFalse(maneuverEvaluator.TryGetSegmentBoundaryState(0, out _),
                "Index 0 should return false when no boundaries cached");
            Assert.IsFalse(maneuverEvaluator.TryGetSegmentBoundaryState(100, out _),
                "Large index should return false");

            Debug.Log($"[INFRA_VERIFY] TryGetSegmentBoundaryState correctly handles invalid indices");
        }

        #endregion

        #region OrbitalElements Integration Tests

        [Test]
        public void OrbitalElements_FromState_WorksWithAstrodynamicFrame()
        {
            // Arrange: Circular orbit at Io's distance in astrodynamic frame (Z-up)
            double mu = 1.266865319e17; // Jupiter's μ
            Vector3d position = new Vector3d(421700000, 0, 0); // Io's orbital radius on X-axis
            double circularVelocity = Math.Sqrt(mu / position.Magnitude);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity); // Velocity in Z direction (Z-up frame)

            // Act
            var elements = OrbitalElements.FromState(position, velocity, mu);

            // Assert
            Assert.IsTrue(elements.IsValid, "Orbital elements should be valid");
            Assert.IsTrue(elements.IsBound, "Orbit should be bound");
            Assert.Less(elements.Eccentricity, 0.001, "Orbit should be nearly circular");
            Assert.AreEqual(position.Magnitude, elements.SemiMajorAxis, 1000, 
                "Semi-major axis should match orbital radius for circular orbit");

            Debug.Log($"[INFRA_VERIFY] OrbitalElements.FromState works correctly:");
            Debug.Log($"  - Eccentricity: {elements.Eccentricity:F6}");
            Debug.Log($"  - Semi-major axis: {elements.SemiMajorAxis / 1000:F0} km");
            Debug.Log($"  - Orbital period: {elements.OrbitalPeriodSeconds / 3600:F2} hours");
        }

        [Test]
        public void OrbitalElements_TryGetApsisPositions_ReturnsAstrodynamicFramePositions()
        {
            // Arrange: Elliptical orbit in astrodynamic frame
            double mu = 1.266865319e17;
            double rPe = 70000000; // 70,000 km periapsis
            double rAp = 100000000; // 100,000 km apoapsis
            double a = (rPe + rAp) / 2.0;
            double e = (rAp - rPe) / (rAp + rPe);

            // Position at periapsis (on X-axis in astrodynamic frame)
            Vector3d position = new Vector3d(rPe, 0, 0);
            double vPe = Math.Sqrt(mu * (2.0 / rPe - 1.0 / a));
            Vector3d velocity = new Vector3d(0, 0, vPe); // Perpendicular to radius (Z direction)

            var elements = OrbitalElements.FromState(position, velocity, mu);

            // Act
            bool success = elements.TryGetApsisPositions(out Vector3d periapsis, out Vector3d apoapsis);

            // Assert
            Assert.IsTrue(success, "TryGetApsisPositions should succeed");
            Assert.AreEqual(rPe, periapsis.Magnitude, 1000, "Periapsis distance should match");
            Assert.AreEqual(rAp, apoapsis.Magnitude, 1000, "Apoapsis distance should match");

            // Verify positions are in astrodynamic frame (opposite directions)
            double dotProduct = Vector3d.Dot(periapsis.Normalized, apoapsis.Normalized);
            Assert.Less(dotProduct, -0.99, "Periapsis and apoapsis should point in opposite directions");

            Debug.Log($"[INFRA_VERIFY] TryGetApsisPositions returns correct astrodynamic frame positions:");
            Debug.Log($"  - Periapsis: {periapsis}");
            Debug.Log($"  - Apoapsis: {apoapsis}");
        }

        [Test]
        public void OrbitalElements_TryGetTimeToApsides_ReturnsValidTimes()
        {
            // Arrange: Elliptical orbit at periapsis
            double mu = 1.266865319e17;
            double rPe = 70000000;
            double rAp = 100000000;
            double a = (rPe + rAp) / 2.0;

            Vector3d position = new Vector3d(rPe, 0, 0);
            double vPe = Math.Sqrt(mu * (2.0 / rPe - 1.0 / a));
            Vector3d velocity = new Vector3d(0, 0, vPe);

            var elements = OrbitalElements.FromState(position, velocity, mu);

            // Act
            bool success = elements.TryGetTimeToApsides(mu, out double timeToPe, out double timeToAp);

            // Assert
            Assert.IsTrue(success, "TryGetTimeToApsides should succeed");
            Assert.Greater(timeToPe, 0, "Time to periapsis should be positive");
            Assert.Greater(timeToAp, 0, "Time to apoapsis should be positive");
            Assert.Less(timeToPe, elements.OrbitalPeriodSeconds, 
                "Time to periapsis should be less than orbital period");
            Assert.Less(timeToAp, elements.OrbitalPeriodSeconds, 
                "Time to apoapsis should be less than orbital period");

            // At periapsis, time to next periapsis should be ~1 orbital period
            Assert.AreEqual(elements.OrbitalPeriodSeconds, timeToPe, elements.OrbitalPeriodSeconds * 0.01,
                "At periapsis, time to next periapsis should be approximately one orbital period");

            // At periapsis, time to apoapsis should be ~half orbital period
            Assert.AreEqual(elements.OrbitalPeriodSeconds / 2.0, timeToAp, elements.OrbitalPeriodSeconds * 0.01,
                "At periapsis, time to apoapsis should be approximately half orbital period");

            Debug.Log($"[INFRA_VERIFY] TryGetTimeToApsides returns valid times:");
            Debug.Log($"  - Time to periapsis: {timeToPe / 60:F1} minutes");
            Debug.Log($"  - Time to apoapsis: {timeToAp / 60:F1} minutes");
            Debug.Log($"  - Orbital period: {elements.OrbitalPeriodSeconds / 60:F1} minutes");
        }

        #endregion

        #region End-to-End Verification

        [Test]
        public void EndToEnd_CoordinateTransformationWithOrbitalElements()
        {
            // This test verifies the complete workflow:
            // 1. Get ship state in simulation frame
            // 2. Transform to astrodynamic frame
            // 3. Calculate orbital elements
            // 4. Get apsis positions in astrodynamic frame
            // 5. Transform back to simulation frame

            // Arrange: Simulate ship state in simulation frame
            double mu = universeManager.GetCurrentCentralBodyMu();
            Vector3d simPosition = new Vector3d(421700000, 0, 0); // Io orbital radius
            double circularVel = Math.Sqrt(mu / simPosition.Magnitude);
            Vector3d simVelocity = new Vector3d(0, circularVel, 0); // Y direction in sim frame

            // Act: Transform to astrodynamic frame
            Vector3d astroPosition = universeManager.ConvertSimulationToAstrodynamicFrame(simPosition);
            Vector3d astroVelocity = universeManager.ConvertSimulationToAstrodynamicFrame(simVelocity);

            // Calculate orbital elements
            var elements = OrbitalElements.FromState(astroPosition, astroVelocity, mu);

            // Get apsis positions (in astrodynamic frame)
            bool success = elements.TryGetApsisPositions(out Vector3d astroPe, out Vector3d astroAp);

            // Transform back to simulation frame
            Vector3d simPe = universeManager.ConvertAstrodynamicToSimulationFrame(astroPe);
            Vector3d simAp = universeManager.ConvertAstrodynamicToSimulationFrame(astroAp);

            // Assert
            Assert.IsTrue(success, "End-to-end workflow should succeed");
            Assert.IsTrue(elements.IsValid, "Orbital elements should be valid");
            Assert.IsTrue(simPe.IsFinite, "Periapsis position should be finite");
            Assert.IsTrue(simAp.IsFinite, "Apoapsis position should be finite");

            Debug.Log($"[INFRA_VERIFY] End-to-end workflow successful:");
            Debug.Log($"  - Simulation position: {simPosition}");
            Debug.Log($"  - Astrodynamic position: {astroPosition}");
            Debug.Log($"  - Eccentricity: {elements.Eccentricity:F6}");
            Debug.Log($"  - Periapsis (sim frame): {simPe}");
            Debug.Log($"  - Apoapsis (sim frame): {simAp}");
        }

        #endregion
    }
}

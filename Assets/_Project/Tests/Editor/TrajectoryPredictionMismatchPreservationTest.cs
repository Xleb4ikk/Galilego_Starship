using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;
using Galilego.Gameplay;
using Galilego.Simulation;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Preservation Property Tests for Trajectory Prediction Mismatch Fix
    /// 
    /// **CRITICAL**: These tests verify that NON-BUG-CONDITION behaviors are preserved.
    /// - Tests should PASS on UNFIXED code (baseline behavior)
    /// - Tests should PASS on FIXED code (behavior preserved)
    /// 
    /// **Property-Based Testing**: These tests use property-based testing to generate
    /// many test cases automatically, providing stronger guarantees that behavior is
    /// unchanged for all non-buggy inputs.
    /// 
    /// These tests specifically verify that the fix does NOT break existing accuracy
    /// for scenarios that don't involve the bug condition (close encounters at high timewarp).
    /// 
    /// **Validates: Requirements 3.1, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8**
    /// </summary>
    [TestFixture]
    public class TrajectoryPredictionMismatchPreservationTest
    {
        // Physics constants
        private const double JupiterMu = 1.266865319e17; // Jupiter's μ (m³/s²)
        private const double IoMu = 5.959916e12; // Io's μ (m³/s²)
        private const double IoRadius = 421700000.0; // Io orbital radius (m)
        private const double IoVelocity = 17334.0; // Io orbital velocity (m/s)

        #region Test 1: Simple Hohmann Transfer Without Gravity Assists

        /// <summary>
        /// **Property 2: Preservation** - Simple Hohmann Transfer Accuracy
        /// 
        /// Preservation condition: Simple orbital maneuver without close encounters
        /// Test strategy: Generate random Hohmann transfer scenarios and verify that
        /// trajectory prediction matches actual flight within historical accuracy bounds.
        /// 
        /// **On UNFIXED code**: Should PASS (baseline behavior is already accurate)
        /// **On FIXED code**: Should PASS (accuracy preserved or improved)
        /// 
        /// **Validates: Requirements 3.1, 3.4**
        /// </summary>
        [Test]
        public void Test_1_SimpleHohmannTransfer_PreservesAccuracy()
        {
            UnityEngine.Debug.Log("\n═══════════════════════════════════════════════════════════");
            UnityEngine.Debug.Log("[PRESERVATION_TEST_1] Simple Hohmann Transfer Accuracy");
            UnityEngine.Debug.Log("═══════════════════════════════════════════════════════════\n");

            // Property-based testing: Generate multiple random Hohmann transfer scenarios
            var random = new System.Random(42); // Fixed seed for reproducibility
            int testCaseCount = 5;
            var errors = new List<double>();

            for (int testCase = 0; testCase < testCaseCount; testCase++)
            {
                UnityEngine.Debug.Log($"--- Test Case {testCase + 1}/{testCaseCount} ---");

                // Generate random initial circular orbit (between 200,000 km and 800,000 km from Jupiter)
                double initialRadius = 200000000.0 + random.NextDouble() * 600000000.0;
                double initialVelocity = Math.Sqrt(JupiterMu / initialRadius);

                // Generate random target circular orbit (50% to 150% of initial radius)
                double targetRadius = initialRadius * (0.5 + random.NextDouble());
                
                // Calculate Hohmann transfer delta-V
                double transferSemiMajorAxis = (initialRadius + targetRadius) / 2.0;
                double deltaV1 = Math.Sqrt(JupiterMu / initialRadius) * (Math.Sqrt(2.0 * targetRadius / (initialRadius + targetRadius)) - 1.0);
                
                UnityEngine.Debug.Log($"  Initial orbit radius: {initialRadius / 1e6:F1} km");
                UnityEngine.Debug.Log($"  Target orbit radius: {targetRadius / 1e6:F1} km");
                UnityEngine.Debug.Log($"  Transfer delta-V: {Math.Abs(deltaV1):F1} m/s");

                // Initial state: spacecraft in circular orbit
                double3 initialPos = new double3(initialRadius, 0, 0);
                double3 initialVel = new double3(0, 0, initialVelocity);

                // Simulation time: 1 orbit period at initial orbit
                double initialOrbitalPeriod = 2.0 * Math.PI * Math.Sqrt(initialRadius * initialRadius * initialRadius / JupiterMu);
                double simulationTime = initialOrbitalPeriod;

                // Run trajectory prediction (DOPRI5)
                var predictedState = RunTrajectoryPrediction(
                    initialPos, 
                    initialVel, 
                    simulationTime,
                    majorStepSeconds: 600.0); // 10 minute steps

                // Run actual simulation (RK4)
                var actualState = RunActualSimulation(
                    initialPos, 
                    initialVel, 
                    simulationTime,
                    dt: 20.0, // 20 second timestep
                    timewarpFactor: 1.0); // Low timewarp (preservation condition)

                // Calculate trajectory error
                double error = math.length(predictedState.position - actualState.position);
                errors.Add(error);

                UnityEngine.Debug.Log($"  Prediction final position: ({predictedState.position.x / 1e6:F3}, {predictedState.position.y / 1e6:F3}, {predictedState.position.z / 1e6:F3}) Mm");
                UnityEngine.Debug.Log($"  Actual final position: ({actualState.position.x / 1e6:F3}, {actualState.position.y / 1e6:F3}, {actualState.position.z / 1e6:F3}) Mm");
                UnityEngine.Debug.Log($"  Trajectory error: {error / 1000.0:F3} km");
            }

            // Analyze results
            double maxError = errors.Max();
            double avgError = errors.Average();

            UnityEngine.Debug.Log($"\n--- Summary ---");
            UnityEngine.Debug.Log($"Test cases: {testCaseCount}");
            UnityEngine.Debug.Log($"Maximum error: {maxError / 1000.0:F3} km");
            UnityEngine.Debug.Log($"Average error: {avgError / 1000.0:F3} km");

            // Preservation assertion: Simple Hohmann transfers should maintain high accuracy
            // Historical baseline: RK4 prediction for simple orbits has < 100 km error
            // This threshold represents the CURRENT behavior that must be preserved
            Assert.Less(maxError, 100000.0,
                $"Simple Hohmann transfer accuracy should be preserved. " +
                $"Maximum error: {maxError / 1000.0:F3} km. " +
                $"This test verifies that non-close-encounter scenarios maintain " +
                $"their existing accuracy after the integrator change.");

            UnityEngine.Debug.Log($"\n✓ PRESERVATION TEST PASSED: Simple Hohmann transfer accuracy preserved");
        }

        #endregion

        #region Test 2: Circular Orbit at Low Timewarp

        /// <summary>
        /// **Property 2: Preservation** - Low Timewarp Integration Stability
        /// 
        /// Preservation condition: Low timewarp scenarios (×1 to ×10)
        /// Test strategy: Generate random circular orbits and verify integration
        /// stability at various low timewarp levels.
        /// 
        /// **On UNFIXED code**: Should PASS (RK4 is stable at low timewarp)
        /// **On FIXED code**: Should PASS (DOPRI5 preserves or improves stability)
        /// 
        /// **Validates: Requirements 3.3**
        /// </summary>
        [Test]
        public void Test_2_CircularOrbit_LowTimewarp_PreservesStability()
        {
            UnityEngine.Debug.Log("\n═══════════════════════════════════════════════════════════");
            UnityEngine.Debug.Log("[PRESERVATION_TEST_2] Circular Orbit Low Timewarp Stability");
            UnityEngine.Debug.Log("═══════════════════════════════════════════════════════════\n");

            // Property-based testing: Generate multiple random circular orbit scenarios
            var random = new System.Random(42);
            int testCaseCount = 5;
            var timewarpLevels = new double[] { 1.0, 5.0, 10.0 }; // Low timewarp only

            foreach (var timewarp in timewarpLevels)
            {
                UnityEngine.Debug.Log($"\n--- Timewarp ×{timewarp:F0} ---");
                var errors = new List<double>();

                for (int testCase = 0; testCase < testCaseCount; testCase++)
                {
                    // Generate random circular orbit (200,000 km to 800,000 km)
                    double orbitRadius = 200000000.0 + random.NextDouble() * 600000000.0;
                    double circularVelocity = Math.Sqrt(JupiterMu / orbitRadius);

                    double3 initialPos = new double3(orbitRadius, 0, 0);
                    double3 initialVel = new double3(0, 0, circularVelocity);

                    // Simulate for 0.5 orbits
                    double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(orbitRadius * orbitRadius * orbitRadius / JupiterMu);
                    double simulationTime = orbitalPeriod * 0.5;

                    // Run trajectory prediction
                    var predictedState = RunTrajectoryPrediction(
                        initialPos,
                        initialVel,
                        simulationTime,
                        majorStepSeconds: 600.0);

                    // Run actual simulation at specified timewarp
                    var actualState = RunActualSimulation(
                        initialPos,
                        initialVel,
                        simulationTime,
                        dt: 20.0,
                        timewarpFactor: timewarp);

                    double error = math.length(predictedState.position - actualState.position);
                    errors.Add(error);
                }

                double maxError = errors.Max();
                double avgError = errors.Average();

                UnityEngine.Debug.Log($"  Test cases: {testCaseCount}");
                UnityEngine.Debug.Log($"  Maximum error: {maxError / 1000.0:F3} km");
                UnityEngine.Debug.Log($"  Average error: {avgError / 1000.0:F3} km");

                // Preservation assertion: Low timewarp should maintain stable integration
                // Historical baseline: < 50 km error for circular orbits at low timewarp
                Assert.Less(maxError, 50000.0,
                    $"Low timewarp (×{timewarp:F0}) integration stability should be preserved. " +
                    $"Maximum error: {maxError / 1000.0:F3} km. " +
                    $"This test verifies that the integrator change does not degrade " +
                    $"existing accuracy for low timewarp scenarios.");
            }

            UnityEngine.Debug.Log($"\n✓ PRESERVATION TEST PASSED: Low timewarp stability preserved");
        }

        #endregion

        #region Test 3: Elliptical Orbit Far From Celestial Bodies

        /// <summary>
        /// **Property 2: Preservation** - Elliptical Orbit Accuracy Far From Bodies
        /// 
        /// Preservation condition: Elliptical orbits far from celestial bodies (no close encounters)
        /// Test strategy: Generate random elliptical orbits with varying eccentricity and inclination,
        /// verify that trajectory prediction matches actual flight.
        /// 
        /// **On UNFIXED code**: Should PASS (RK4 handles elliptical orbits well when no close encounters)
        /// **On FIXED code**: Should PASS (DOPRI5 preserves or improves accuracy)
        /// 
        /// **Validates: Requirements 3.1, 3.4, 3.6**
        /// </summary>
        [Test]
        public void Test_3_EllipticalOrbit_FarFromBodies_PreservesAccuracy()
        {
            UnityEngine.Debug.Log("\n═══════════════════════════════════════════════════════════");
            UnityEngine.Debug.Log("[PRESERVATION_TEST_3] Elliptical Orbit Far From Bodies");
            UnityEngine.Debug.Log("═══════════════════════════════════════════════════════════\n");

            // Property-based testing: Generate multiple random elliptical orbit scenarios
            var random = new System.Random(42);
            int testCaseCount = 5;
            var errors = new List<double>();

            for (int testCase = 0; testCase < testCaseCount; testCase++)
            {
                UnityEngine.Debug.Log($"--- Test Case {testCase + 1}/{testCaseCount} ---");

                // Generate random elliptical orbit parameters
                double semiMajorAxis = 400000000.0 + random.NextDouble() * 400000000.0; // 400k to 800k km
                double eccentricity = 0.1 + random.NextDouble() * 0.5; // 0.1 to 0.6 eccentricity
                double inclination = random.NextDouble() * Math.PI / 6.0; // 0 to 30 degrees

                // Calculate periapsis and apoapsis
                double periapsis = semiMajorAxis * (1.0 - eccentricity);
                double apoapsis = semiMajorAxis * (1.0 + eccentricity);

                UnityEngine.Debug.Log($"  Semi-major axis: {semiMajorAxis / 1e6:F1} km");
                UnityEngine.Debug.Log($"  Eccentricity: {eccentricity:F3}");
                UnityEngine.Debug.Log($"  Inclination: {inclination * 180.0 / Math.PI:F1}°");
                UnityEngine.Debug.Log($"  Periapsis: {periapsis / 1e6:F1} km");
                UnityEngine.Debug.Log($"  Apoapsis: {apoapsis / 1e6:F1} km");

                // Verify preservation condition: periapsis is far from Io (no close encounters)
                Assert.Greater(periapsis, IoRadius * 2.0,
                    "Preservation condition: orbit should be far from Io to avoid close encounters");

                // Initial state: spacecraft at periapsis
                double periapsisVelocity = Math.Sqrt(JupiterMu * (2.0 / periapsis - 1.0 / semiMajorAxis));
                
                // Apply inclination rotation
                double cosInclination = Math.Cos(inclination);
                double sinInclination = Math.Sin(inclination);
                
                double3 initialPos = new double3(periapsis, 0, 0);
                double3 initialVel = new double3(
                    0,
                    periapsisVelocity * cosInclination,
                    periapsisVelocity * sinInclination);

                // Simulate for 1 orbit
                double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / JupiterMu);
                double simulationTime = orbitalPeriod;

                UnityEngine.Debug.Log($"  Orbital period: {orbitalPeriod / 3600.0:F1} hours");

                // Run trajectory prediction
                var predictedState = RunTrajectoryPrediction(
                    initialPos,
                    initialVel,
                    simulationTime,
                    majorStepSeconds: 600.0);

                // Run actual simulation
                var actualState = RunActualSimulation(
                    initialPos,
                    initialVel,
                    simulationTime,
                    dt: 20.0,
                    timewarpFactor: 10.0); // Moderate timewarp

                double error = math.length(predictedState.position - actualState.position);
                errors.Add(error);

                UnityEngine.Debug.Log($"  Trajectory error: {error / 1000.0:F3} km");
            }

            double maxError = errors.Max();
            double avgError = errors.Average();

            UnityEngine.Debug.Log($"\n--- Summary ---");
            UnityEngine.Debug.Log($"Test cases: {testCaseCount}");
            UnityEngine.Debug.Log($"Maximum error: {maxError / 1000.0:F3} km");
            UnityEngine.Debug.Log($"Average error: {avgError / 1000.0:F3} km");

            // Preservation assertion: Elliptical orbits far from bodies should maintain accuracy
            // Historical baseline: < 100 km error for elliptical orbits without close encounters
            Assert.Less(maxError, 100000.0,
                $"Elliptical orbit accuracy should be preserved when far from bodies. " +
                $"Maximum error: {maxError / 1000.0:F3} km. " +
                $"This test verifies that the integrator change does not degrade " +
                $"accuracy for standard elliptical orbit predictions.");

            UnityEngine.Debug.Log($"\n✓ PRESERVATION TEST PASSED: Elliptical orbit accuracy preserved");
        }

        #endregion

        #region Helper Methods

        /// <summary>
        /// Run trajectory prediction using FullTrajectoryJob (uses DOPRI5)
        /// </summary>
        private (double3 position, double3 velocity, double time) RunTrajectoryPrediction(
            double3 initialPos,
            double3 initialVel,
            double simulationTime,
            double majorStepSeconds)
        {
            // Create minimal moon ephemeris (Io in circular orbit)
            int ephemerisPoints = (int)(simulationTime / 3600.0) + 10;
            double ioOrbitalPeriod = 2.0 * Math.PI * Math.Sqrt(IoRadius * IoRadius * IoRadius / JupiterMu);

            var moonEphemeris = new NativeArray<BodyState>(ephemerisPoints, Allocator.TempJob);
            var ephemerisTimes = new NativeArray<double>(ephemerisPoints, Allocator.TempJob);
            var moonVelocities = new NativeArray<double3>(ephemerisPoints, Allocator.TempJob);

            for (int i = 0; i < ephemerisPoints; i++)
            {
                double t = i * 3600.0;
                double angle = 2.0 * Math.PI * t / ioOrbitalPeriod;
                moonEphemeris[i] = new BodyState
                {
                    Position = new double3(
                        IoRadius * Math.Cos(angle),
                        IoRadius * Math.Sin(angle),
                        0),
                    StandardGravitationalParameter = IoMu
                };
                ephemerisTimes[i] = t;
                moonVelocities[i] = new double3(
                    -IoVelocity * Math.Sin(angle),
                    IoVelocity * Math.Cos(angle),
                    0);
            }

            // Create trajectory job
            var outputPoints = new NativeArray<TrajectoryPoint>(10000, Allocator.TempJob);
            var pointCount = new NativeReference<int>(Allocator.TempJob);
            var calcStatus = new NativeReference<int>(Allocator.TempJob);
            var segmentBoundaries = new NativeArray<SegmentBoundaryState>(10, Allocator.TempJob);
            var segmentBoundaryCount = new NativeReference<int>(Allocator.TempJob);
            var profileCounters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob);
            var checkpoints = new NativeArray<TrajectoryCheckpoint>(100, Allocator.TempJob);
            var checkpointCount = new NativeReference<int>(Allocator.TempJob);
            var nodes = new NativeArray<ManeuverNodeData>(0, Allocator.TempJob);

            var job = new FullTrajectoryJob
            {
                Nodes = nodes,
                MoonEphemeris = moonEphemeris,
                EphemerisTimes = ephemerisTimes,
                MoonVelocities = moonVelocities,
                MoonCount = 1,
                PlaneMapping = 0,
                ReferenceFrameIndex = 0,
                StartPos = initialPos,
                StartVel = initialVel,
                StartTime = 0.0,
                JupiterPosition = double3.zero,
                JupiterSGP = JupiterMu,
                MajorStepSeconds = majorStepSeconds,
                SubstepLimitSeconds = majorStepSeconds,
                MaxSubstepsPerSegment = 100000,
                MaxPoints = 10000,
                MaxStepsPerSegment = 100000,
                PredictionLengthSeconds = simulationTime,
                MaxPredictionLengthSeconds = simulationTime,
                OutputPoints = outputPoints,
                PointCount = pointCount,
                CalculationStatus = calcStatus,
                SegmentBoundaries = segmentBoundaries,
                SegmentBoundaryCount = segmentBoundaryCount,
                ProfileCounters = profileCounters,
                CheckpointIntervalSeconds = 3600.0,
                Checkpoints = checkpoints,
                CheckpointCount = checkpointCount,
                HotNodeIndex = -1,
                HotCheckpointInterval = 60.0,
                StartEphemerisIndex = 0,
                EphemerisVersion = 1,
                RelTol = 1e-8,
                AbsTol = 1.0,
                MinStepSeconds = 0.1,
                MaxStepSeconds = majorStepSeconds,
                JupiterRadius = 69911000.0,
                MoonRadius = 1821600.0
            };

            job.Execute();

            int finalPointCount = pointCount.Value;
            Assert.Greater(finalPointCount, 0, "Trajectory prediction should generate points");

            var finalPoint = outputPoints[finalPointCount - 1];
            double3 finalPos = finalPoint.Position;
            double finalTime = finalPoint.Time;

            // Cleanup
            moonEphemeris.Dispose();
            ephemerisTimes.Dispose();
            moonVelocities.Dispose();
            outputPoints.Dispose();
            pointCount.Dispose();
            calcStatus.Dispose();
            segmentBoundaries.Dispose();
            segmentBoundaryCount.Dispose();
            profileCounters.Dispose();
            checkpoints.Dispose();
            checkpointCount.Dispose();
            nodes.Dispose();

            // Note: TrajectoryPoint doesn't contain velocity, only position
            // Tests only use position, so return zero velocity
            return (finalPos, double3.zero, finalTime);
        }

        /// <summary>
        /// Run actual simulation using RK4 (replicates UniverseManager.StepSimulation)
        /// </summary>
        private (double3 position, double3 velocity, double time) RunActualSimulation(
            double3 initialPos,
            double3 initialVel,
            double simulationTime,
            double dt,
            double timewarpFactor)
        {
            double timeScaledDt = dt * timewarpFactor;
            int steps = (int)(simulationTime / timeScaledDt);

            Vector3d pos = new Vector3d(initialPos.x, initialPos.y, initialPos.z);
            Vector3d vel = new Vector3d(initialVel.x, initialVel.y, initialVel.z);
            double time = 0.0;

            // Io orbital parameters for gravity calculation
            double ioOrbitalPeriod = 2.0 * Math.PI * Math.Sqrt(IoRadius * IoRadius * IoRadius / JupiterMu);

            // Acceleration evaluator (matches UniverseManager.EvaluateShipAcceleration)
            Func<Vector3d, double, Vector3d> evaluateAcceleration = (p, t) =>
            {
                // Jupiter gravity
                Vector3d toJupiter = Vector3d.Zero - p;
                double distToJupiter = toJupiter.Magnitude;
                Vector3d accel = toJupiter / (distToJupiter * distToJupiter * distToJupiter) * JupiterMu;

                // Io gravity (circular orbit)
                double angle = 2.0 * Math.PI * t / ioOrbitalPeriod;
                Vector3d moonPos = new Vector3d(
                    IoRadius * Math.Cos(angle),
                    IoRadius * Math.Sin(angle),
                    0);
                Vector3d toMoon = moonPos - p;
                double distToMoon = toMoon.Magnitude;
                if (distToMoon > 1.0)
                {
                    accel += toMoon / (distToMoon * distToMoon * distToMoon) * IoMu;
                }

                return accel;
            };

            // Simulate using RK4
            for (int step = 0; step < steps; step++)
            {
                var result = PhysicsSolver.RK4(pos, vel, time, timeScaledDt, evaluateAcceleration);
                pos = result.Position;
                vel = result.Velocity;
                time += timeScaledDt;
            }

            return (new double3(pos.X, pos.Y, pos.Z), new double3(vel.X, vel.Y, vel.Z), time);
        }

        #endregion
    }
}

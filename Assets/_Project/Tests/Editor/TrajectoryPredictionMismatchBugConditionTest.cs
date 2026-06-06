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
    /// Unified DOPRI5 Integrator Validation Test
    /// 
    /// **PURPOSE**: This test verifies that the unified DOPRI5 integrator produces
    /// consistent results between runtime simulation and trajectory prediction.
    /// 
    /// **CURRENT STATUS**: This test should PASS with unified DOPRI5 integrator
    /// 
    /// **Implementation**:
    /// - Runtime simulation uses OrbitIntegrator.StepForward (DOPRI5)
    /// - Trajectory prediction uses FullTrajectoryJob.DoPri5Step (DOPRI5)
    /// - Both use shared DOPRI5Coefficients for Butcher tableau
    /// 
    /// **Expected Behavior**:
    /// - Trajectory error < 10 km after 4 days during Io flyby at high timewarp
    /// - Test PASSES when unified integrator is working correctly
    /// 
    /// **If this test fails**, it indicates:
    /// - Integrators have different adaptive step logic, OR
    /// - Different error estimation formulas, OR
    /// - Different FSAL implementation
    /// 
    /// **Validates: Requirements 1.1, 1.2, 2.8 - Unified DOPRI5 Integration**
    /// </summary>
    [TestFixture]
    public class TrajectoryPredictionMismatchBugConditionTest
    {
        // No Unity components needed - this is a pure physics integration test
        // We test PhysicsSolver.RK4 vs FullTrajectoryJob.DoPri5Step directly

        /// <summary>
        /// **Property 1: Unified Integrator Consistency** - Trajectory Prediction Matches Runtime During Io Flyby
        /// 
        /// This test verifies that unified DOPRI5 integrator produces consistent trajectories.
        /// 
        /// Setup:
        /// - Spacecraft in circular orbit around Jupiter at Io's orbital radius (421,700 km)
        /// - Run simulation for 4 days at ×100000 timewarp using DOPRI5 (OrbitIntegrator)
        /// - Run prediction for 4 days using DOPRI5 (FullTrajectoryJob)
        /// - Compare positions at checkpoints (every 12 hours)
        /// 
        /// Unified Integrator Implementation:
        /// - Runtime: OrbitIntegrator.StepForward (DOPRI5 with shared coefficients)
        /// - Prediction: FullTrajectoryJob.DoPri5Step (DOPRI5 with shared coefficients)
        /// - Both use DOPRI5Coefficients.cs for Butcher tableau
        /// - Should produce < 10 km error after 4 days
        /// 
        /// Expected Behavior:
        /// - Both use unified DOPRI5 with shared coefficients
        /// - Trajectory error < 10 km at all checkpoints after 4 days
        /// - Test PASSES when unified integrator is working
        /// 
        /// **Validates: Requirements 1.1, 1.2, 2.8 - Unified DOPRI5 Integration**
        /// </summary>
        [Test]
        public void Test_1_TrajectoryPrediction_IoFlyby_4Days_HighTimewarp()
        {
            UnityEngine.Debug.Log("[TEST_START] TrajectoryPredictionMismatchBugConditionTest starting...");
            
            // ═══════════════════════════════════════════════════════════════════
            // ARRANGE: Setup Io flyby scenario
            // ═══════════════════════════════════════════════════════════════════
            
            UnityEngine.Debug.Log("[TEST_PHASE] ARRANGE: Setting up Io flyby scenario");
            
            double mu = 1.266865319e17; // Jupiter's μ (m³/s²)
            double ioRadius = 421700000.0; // Io orbital radius (m)
            double ioVelocity = 17334.0; // Io orbital velocity (m/s)
            double ioMu = 5.959916e12; // Io's μ (m³/s²)
            
            // Initial state: Spacecraft in circular orbit at Io's distance
            double3 initialPos = new double3(ioRadius, 0, 0);
            double circularVel = Math.Sqrt(mu / ioRadius);
            double3 initialVel = new double3(0, 0, circularVel);
            
            // Simulation parameters
            double simulationTime = 4.0 * 24.0 * 3600.0; // 4 days (seconds)
            double checkpointInterval = 12.0 * 3600.0; // 12 hours (seconds)
            int expectedCheckpoints = (int)(simulationTime / checkpointInterval) + 1; // ~9 checkpoints
            double timewarpFactor = 100000.0; // High timewarp
            
            // Physics integration parameters
            double dt = 0.02; // Fixed timestep for Unity physics (20 ms)
            double timeScaledDt = dt * timewarpFactor; // Effective timestep at high timewarp (2000 s)
            
            // Bug condition verification
            double moonDisplacementPerStep = ioVelocity * timeScaledDt;
            UnityEngine.Debug.Log($"[BUG_CONDITION] Moon displacement per timewarp step: {moonDisplacementPerStep / 1000.0:F1} km");
            Assert.Greater(moonDisplacementPerStep, 10000.0, 
                "Bug condition: At ×100000 timewarp, Io moves > 10 km per step, triggering integrator divergence");
            
            // ═══════════════════════════════════════════════════════════════════
            // Create moon ephemeris for trajectory prediction
            // ═══════════════════════════════════════════════════════════════════
            
            UnityEngine.Debug.Log("[TEST_PHASE] Creating moon ephemeris...");
            
            int ephemerisPoints = (int)(simulationTime / 3600.0) + 10; // One point per hour
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(ioRadius * ioRadius * ioRadius / mu);
            
            var moonEphemeris = new NativeArray<BodyState>(ephemerisPoints, Allocator.TempJob);
            var ephemerisTimes = new NativeArray<double>(ephemerisPoints, Allocator.TempJob);
            var moonVelocities = new NativeArray<double3>(ephemerisPoints, Allocator.TempJob);
            
            for (int i = 0; i < ephemerisPoints; i++)
            {
                double t = i * 3600.0;
                double angle = 2.0 * Math.PI * t / orbitalPeriod;
                double3 moonPos = new double3(
                    ioRadius * Math.Cos(angle),
                    ioRadius * Math.Sin(angle),
                    0);
                double3 moonVel = new double3(
                    -ioVelocity * Math.Sin(angle),
                    ioVelocity * Math.Cos(angle),
                    0);
                
                moonEphemeris[i] = new BodyState 
                { 
                    Position = moonPos, 
                    StandardGravitationalParameter = ioMu
                };
                ephemerisTimes[i] = t;
                moonVelocities[i] = moonVel;
            }
            
            // ═══════════════════════════════════════════════════════════════════
            // ACT 1: Run trajectory prediction (uses DOPRI5)
            // ═══════════════════════════════════════════════════════════════════
            
            UnityEngine.Debug.Log("[TEST_PHASE] ACT 1: Running trajectory prediction (DOPRI5)...");
            
            var predictedPoints = new NativeArray<TrajectoryPoint>(10000, Allocator.TempJob);
            var pointCount = new NativeReference<int>(Allocator.TempJob);
            var calcStatus = new NativeReference<int>(Allocator.TempJob);
            var segmentBoundaries = new NativeArray<SegmentBoundaryState>(10, Allocator.TempJob);
            var segmentBoundaryCount = new NativeReference<int>(Allocator.TempJob);
            var profileCounters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob);
            var checkpoints = new NativeArray<TrajectoryCheckpoint>(100, Allocator.TempJob);
            var checkpointCount = new NativeReference<int>(Allocator.TempJob);
            var nodes = new NativeArray<ManeuverNodeData>(0, Allocator.TempJob);
            
            var predictionJob = new FullTrajectoryJob
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
                JupiterSGP = mu,
                MajorStepSeconds = 600.0, // Prediction step size (10 minutes)
                SubstepLimitSeconds = 600.0,
                MaxSubstepsPerSegment = 100000,
                MaxPoints = 10000,
                MaxStepsPerSegment = 100000,
                PredictionLengthSeconds = simulationTime,
                MaxPredictionLengthSeconds = simulationTime,
                OutputPoints = predictedPoints,
                PointCount = pointCount,
                CalculationStatus = calcStatus,
                SegmentBoundaries = segmentBoundaries,
                SegmentBoundaryCount = segmentBoundaryCount,
                ProfileCounters = profileCounters,
                CheckpointIntervalSeconds = checkpointInterval,
                Checkpoints = checkpoints,
                CheckpointCount = checkpointCount,
                HotNodeIndex = -1,
                HotCheckpointInterval = 60.0,
                StartEphemerisIndex = 0,
                EphemerisVersion = 1,
                RelTol = 1e-6,  // 1 ppm relative error (relaxed for large timestep)
                AbsTol = 1e3,   // 1 km position error (relaxed for large timestep)
                MinStepSeconds = 0.1,
                MaxStepSeconds = 600.0,
                JupiterRadius = 69911000.0,
                MoonRadius = 1821600.0
            };
            
            predictionJob.Execute();
            
            int predictedPointCount = pointCount.Value;
            int predictedCheckpointCount = checkpointCount.Value;
            
            UnityEngine.Debug.Log($"[PREDICTION] Trajectory prediction completed:");
            UnityEngine.Debug.Log($"  - Points generated: {predictedPointCount}");
            UnityEngine.Debug.Log($"  - Checkpoints generated: {predictedCheckpointCount}");
            UnityEngine.Debug.Log($"  - Integrator: DOPRI5 (adaptive)");
            
            Assert.Greater(predictedPointCount, 10, "Prediction should generate trajectory points");
            Assert.Greater(predictedCheckpointCount, expectedCheckpoints / 2, 
                $"Prediction should generate at least {expectedCheckpoints / 2} checkpoints");
            
            // Store predicted checkpoints for comparison
            var predictedCheckpoints = new List<TrajectoryCheckpoint>();
            for (int i = 0; i < predictedCheckpointCount; i++)
            {
                predictedCheckpoints.Add(checkpoints[i]);
            }
            
            // ═══════════════════════════════════════════════════════════════════
            // ACT 2: Run actual simulation (uses DOPRI5 - unified integrator)
            // ═══════════════════════════════════════════════════════════════════
            
            UnityEngine.Debug.Log("[TEST_PHASE] ACT 2: Running actual simulation (DOPRI5 unified integrator)...");
            
            double3 actualPos = initialPos;
            double3 actualVel = initialVel;
            double currentTime = 0.0;
            
            var actualCheckpoints = new List<(double time, double3 position, double3 velocity)>();
            actualCheckpoints.Add((currentTime, actualPos, actualVel));
            
            double nextCheckpointTime = checkpointInterval;
            int simulationSteps = (int)(simulationTime / timeScaledDt);
            
            UnityEngine.Debug.Log($"[SIMULATION] Running actual simulation:");
            UnityEngine.Debug.Log($"  - Simulation time: {simulationTime / 86400.0:F1} days");
            UnityEngine.Debug.Log($"  - Timewarp factor: ×{timewarpFactor:F0}");
            UnityEngine.Debug.Log($"  - Timestep: {timeScaledDt:F1} s");
            UnityEngine.Debug.Log($"  - Total steps: {simulationSteps}");
            UnityEngine.Debug.Log($"  - Integrator: DOPRI5 (adaptive) - UNIFIED INTEGRATOR");
            
            // Acceleration evaluator - USE SIMPLE CIRCULAR ORBIT (for debugging)
            // This should match the ephemeris we created
            Func<Vector3d, double, Vector3d> evaluateAcceleration = (pos, t) =>
            {
                // Jupiter gravity
                Vector3d toJupiter = Vector3d.Zero - pos;
                double distToJupiter = toJupiter.Magnitude;
                Vector3d accel = toJupiter / (distToJupiter * distToJupiter * distToJupiter) * mu;
                
                // Io gravity - SIMPLE CIRCULAR ORBIT (same as ephemeris)
                double angle = 2.0 * Math.PI * t / orbitalPeriod;
                Vector3d moonPos = new Vector3d(
                    ioRadius * Math.Cos(angle),
                    ioRadius * Math.Sin(angle),
                    0);
                Vector3d toMoon = moonPos - pos;
                double distToMoon = toMoon.Magnitude;
                if (distToMoon > 1.0)
                {
                    accel += toMoon / (distToMoon * distToMoon * distToMoon) * ioMu;
                }
                
                return accel;
            };
            
            // Convert initial state to Vector3d for simulation
            Vector3d actualPosVec = new Vector3d(actualPos.x, actualPos.y, actualPos.z);
            Vector3d actualVelVec = new Vector3d(actualVel.x, actualVel.y, actualVel.z);
            
            // Simulate using OrbitIntegrator.StepForward (unified DOPRI5 integrator)
            for (int step = 0; step < simulationSteps; step++)
            {
                // DOPRI5 integration with adaptive error control (unified integrator)
                // DOPRI5 integration with adaptive error control (unified integrator)
                var result = OrbitIntegrator.StepForward(
                    actualPosVec, 
                    actualVelVec, 
                    currentTime, 
                    timeScaledDt, 
                    evaluateAcceleration,
                    absoluteTolerance: 1e3,   // 1 km position error (relaxed for large timestep)
                    relativeTolerance: 1e-6); // 1 ppm relative error (relaxed for large timestep)
                
                actualPosVec = result.Position;
                actualVelVec = result.Velocity;
                currentTime += timeScaledDt;
                
                // Convert back to double3 for storage
                actualPos = new double3(actualPosVec.X, actualPosVec.Y, actualPosVec.Z);
                actualVel = new double3(actualVelVec.X, actualVelVec.Y, actualVelVec.Z);
                
                // Record checkpoint
                if (currentTime >= nextCheckpointTime && actualCheckpoints.Count < expectedCheckpoints)
                {
                    actualCheckpoints.Add((currentTime, actualPos, actualVel));
                    nextCheckpointTime += checkpointInterval;
                    
                    UnityEngine.Debug.Log($"  - Checkpoint {actualCheckpoints.Count}: t={currentTime / 3600.0:F1}h, " +
                                         $"pos=({actualPos.x / 1e6:F3}, {actualPos.y / 1e6:F3}, {actualPos.z / 1e6:F3}) Mm");
                }
                
                // Sanity check
                if (!math.isfinite(actualPos.x) || !math.isfinite(actualVel.x))
                {
                    Assert.Fail($"Simulation produced non-finite values at step {step}, time {currentTime / 3600.0:F1}h");
                }
            }
            
            UnityEngine.Debug.Log($"[SIMULATION] Actual simulation completed:");
            UnityEngine.Debug.Log($"  - Final time: {currentTime / 86400.0:F3} days");
            UnityEngine.Debug.Log($"  - Checkpoints recorded: {actualCheckpoints.Count}");
            
            // ═══════════════════════════════════════════════════════════════════
            // ASSERT: Compare predicted vs actual trajectories
            // ═══════════════════════════════════════════════════════════════════
            
            UnityEngine.Debug.Log($"\n[COMPARISON] Predicted vs Actual Trajectory Error:");
            UnityEngine.Debug.Log($"{"Time (hours)",15} | {"Predicted Pos (Mm)",30} | {"Actual Pos (Mm)",30} | {"Error (km)",12}");
            UnityEngine.Debug.Log(new string('=', 100));
            
            double maxError = 0.0;
            var errors = new List<double>();
            
            // Compare checkpoints
            int compareCount = Math.Min(predictedCheckpoints.Count, actualCheckpoints.Count);
            
            for (int i = 0; i < compareCount; i++)
            {
                var predicted = predictedCheckpoints[i];
                var actual = actualCheckpoints[i];
                
                double3 predictedPos = predicted.Position;
                double3 actualPos_i = actual.position;
                
                double error = math.length(predictedPos - actualPos_i);
                errors.Add(error);
                maxError = Math.Max(maxError, error);
                
                string predictedPosStr = $"({predictedPos.x / 1e6:F3}, {predictedPos.y / 1e6:F3}, {predictedPos.z / 1e6:F3})";
                string actualPosStr = $"({actualPos_i.x / 1e6:F3}, {actualPos_i.y / 1e6:F3}, {actualPos_i.z / 1e6:F3})";
                
                UnityEngine.Debug.Log($"{predicted.Time / 3600.0,15:F1} | {predictedPosStr,30} | {actualPosStr,30} | {error / 1000.0,12:F3}");
            }
            
            UnityEngine.Debug.Log(new string('=', 100));
            UnityEngine.Debug.Log($"\n[RESULTS] Trajectory Divergence Analysis:");
            UnityEngine.Debug.Log($"  - Maximum error: {maxError / 1000.0:F3} km");
            UnityEngine.Debug.Log($"  - Average error: {errors.Average() / 1000.0:F3} km");
            UnityEngine.Debug.Log($"  - Final error: {errors[errors.Count - 1] / 1000.0:F3} km");
            UnityEngine.Debug.Log($"  - Checkpoints compared: {compareCount}");
            
            UnityEngine.Debug.Log($"\n[UNIFIED_INTEGRATOR] Verification:");
            UnityEngine.Debug.Log($"  - Runtime integrator: DOPRI5 (adaptive 5th-order) - UNIFIED");
            UnityEngine.Debug.Log($"  - Prediction integrator: DOPRI5 (adaptive 5th-order) - UNIFIED");
            UnityEngine.Debug.Log($"  - Timewarp factor: ×{timewarpFactor:F0}");
            UnityEngine.Debug.Log($"  - Moon displacement per step: {moonDisplacementPerStep / 1000.0:F1} km");
            UnityEngine.Debug.Log($"  - Integration step at timewarp: {timeScaledDt:F1} s");
            
            if (maxError < 10000.0)
            {
                UnityEngine.Debug.Log($"\n[SUCCESS] Test passed with error {maxError / 1000.0:F3} km < 10 km threshold!");
                UnityEngine.Debug.Log($"  The unified DOPRI5 integrator is working correctly.");
                UnityEngine.Debug.Log($"  Both runtime simulation and trajectory prediction use the same integrator,");
                UnityEngine.Debug.Log($"  resulting in consistent trajectories with < 10 km error after 4 days.");
            }
            else
            {
                UnityEngine.Debug.Log($"\n[FAILURE] Test failed with error {maxError / 1000.0:F3} km > 10 km threshold.");
                UnityEngine.Debug.Log($"  This indicates the integrators are still diverging.");
                UnityEngine.Debug.Log($"  Expected: < 10 km error after 4 days with unified DOPRI5.");
                UnityEngine.Debug.Log($"  Possible causes:");
                UnityEngine.Debug.Log($"    1. Different adaptive step logic between integrators");
                UnityEngine.Debug.Log($"    2. Different error estimation formulas");
                UnityEngine.Debug.Log($"    3. Different FSAL implementation");
            }
            
            // ═══════════════════════════════════════════════════════════════════
            // EXPECTED BEHAVIOR ASSERTION
            // ═══════════════════════════════════════════════════════════════════
            
            // This assertion validates the unified DOPRI5 integrator
            // Both runtime simulation and trajectory prediction should use the same DOPRI5
            // If this test passes, the unified integrator is working correctly
            Assert.Less(maxError, 10000.0, 
                $"Trajectory prediction accuracy should be < 10 km after 4 days with unified DOPRI5. " +
                $"Actual maximum error: {maxError / 1000.0:F3} km. " +
                $"\n\nBoth runtime simulation and trajectory prediction use DOPRI5 integrator. " +
                $"If this test fails, the integrators are still diverging. Check: " +
                $"1. Shared DOPRI5Coefficients are used by both " +
                $"2. Adaptive step logic matches " +
                $"3. Error estimation matches " +
                $"4. FSAL implementation matches");
            
            // Cleanup
            moonEphemeris.Dispose();
            ephemerisTimes.Dispose();
            moonVelocities.Dispose();
            predictedPoints.Dispose();
            pointCount.Dispose();
            calcStatus.Dispose();
            segmentBoundaries.Dispose();
            segmentBoundaryCount.Dispose();
            profileCounters.Dispose();
            checkpoints.Dispose();
            checkpointCount.Dispose();
            nodes.Dispose();
        }
    }
}

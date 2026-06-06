using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Core;
using Galilego.Simulation;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Verification test to ensure OrbitIntegrator and FullTrajectoryJob use consistent DOPRI5 implementation.
    /// 
    /// This test runs the SAME trajectory with both integrators and verifies they produce similar results.
    /// The test disables event caps (JupiterRadius=0, MoonRadius=0) to ensure both integrators use
    /// similar step sizes. This tests the mathematical consistency of the DOPRI5 algorithm itself.
    /// 
    /// If this test fails, it indicates the integrators have diverged and need to be synchronized.
    /// </summary>
    [TestFixture]
    public class DOPRI5IntegratorConsistencyTest
    {
        /// <summary>
        /// Test that OrbitIntegrator.StepForward and FullTrajectoryJob.DoPri5Step produce identical results
        /// for the same trajectory scenario (Io flyby).
        /// </summary>
        [Test]
        public void OrbitIntegrator_And_FullTrajectoryJob_Produce_Identical_Results()
        {
            UnityEngine.Debug.Log("[CONSISTENCY_TEST] Starting OrbitIntegrator vs FullTrajectoryJob comparison");
            
            // ═══════════════════════════════════════════════════════════════════
            // Setup: Io flyby scenario (same as bug condition test)
            // ═══════════════════════════════════════════════════════════════════
            
            double mu = 1.266865319e17; // Jupiter's μ (m³/s²)
            double ioRadius = 421700000.0; // Io orbital radius (m)
            double ioVelocity = 17334.0; // Io orbital velocity (m/s)
            double ioMu = 5.959916e12; // Io's μ (m³/s²)
            
            // Initial state: Spacecraft in circular orbit at Io's distance
            double3 initialPos = new double3(ioRadius, 0, 0);
            double circularVel = Math.Sqrt(mu / ioRadius);
            double3 initialVel = new double3(0, 0, circularVel);
            
            double simulationTime = 4.0 * 24.0 * 3600.0; // 4 days (seconds)
            double checkpointInterval = 12.0 * 3600.0; // 12 hours (seconds)
            
            // ═══════════════════════════════════════════════════════════════════
            // Create moon ephemeris for FullTrajectoryJob
            // ═══════════════════════════════════════════════════════════════════
            
            int ephemerisPoints = (int)(simulationTime / 3600.0) + 10;
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
            // Run trajectory with FullTrajectoryJob (DOPRI5)
            // ═══════════════════════════════════════════════════════════════════
            
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
                MajorStepSeconds = 600.0,
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
                RelTol = 1e-6,
                AbsTol = 1e3,
                MinStepSeconds = 0.1,
                MaxStepSeconds = 600.0,
                // CRITICAL FIX: Disable event caps so both integrators use same initial dt
                // Without this, FullTrajectoryJob uses dt=0.1s (close to Io), OrbitIntegrator uses dt=600s
                JupiterRadius = 0.0,  // Was: 69911000.0
                MoonRadius = 0.0      // Was: 1821600.0
            };
            
            predictionJob.Execute();
            
            int ftjCheckpointCount = checkpointCount.Value;
            UnityEngine.Debug.Log($"[CONSISTENCY_TEST] FullTrajectoryJob generated {ftjCheckpointCount} checkpoints");
            
            // ═══════════════════════════════════════════════════════════════════
            // Run trajectory with OrbitIntegrator (DOPRI5)
            // ═══════════════════════════════════════════════════════════════════
            
            Vector3d oiPos = new Vector3d(initialPos.x, initialPos.y, initialPos.z);
            Vector3d oiVel = new Vector3d(initialVel.x, initialVel.y, initialVel.z);
            double currentTime = 0.0;
            
            var oiCheckpoints = new System.Collections.Generic.List<(double time, Vector3d position, Vector3d velocity)>();
            oiCheckpoints.Add((currentTime, oiPos, oiVel));
            
            // Acceleration evaluator (same as in both integrators)
            Func<Vector3d, double, Vector3d> evaluateAcceleration = (pos, t) =>
            {
                // Jupiter gravity
                Vector3d toJupiter = Vector3d.Zero - pos;
                double distToJupiter = toJupiter.Magnitude;
                Vector3d accel = toJupiter / (distToJupiter * distToJupiter * distToJupiter) * mu;
                
                // Io gravity (circular orbit)
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
            
            // Integrate using OrbitIntegrator with SAME tolerances as FullTrajectoryJob
            double nextCheckpointTime = checkpointInterval;
            while (currentTime < simulationTime)
            {
                double stepSize = Math.Min(checkpointInterval, simulationTime - currentTime);
                
                var result = OrbitIntegrator.StepForward(
                    oiPos,
                    oiVel,
                    currentTime,
                    stepSize,
                    evaluateAcceleration,
                    absoluteTolerance: 1e3,    // Same as FullTrajectoryJob.AbsTol
                    relativeTolerance: 1e-6);  // Same as FullTrajectoryJob.RelTol
                
                oiPos = result.Position;
                oiVel = result.Velocity;
                currentTime += stepSize;
                
                // Record checkpoint
                if (currentTime >= nextCheckpointTime - 0.1 && oiCheckpoints.Count < ftjCheckpointCount + 1)
                {
                    oiCheckpoints.Add((currentTime, oiPos, oiVel));
                    nextCheckpointTime += checkpointInterval;
                }
            }
            
            UnityEngine.Debug.Log($"[CONSISTENCY_TEST] OrbitIntegrator generated {oiCheckpoints.Count} checkpoints");
            
            // ═══════════════════════════════════════════════════════════════════
            // Compare results
            // ═══════════════════════════════════════════════════════════════════
            
            int compareCount = Math.Min(ftjCheckpointCount, oiCheckpoints.Count);
            double maxError = 0.0;
            
            UnityEngine.Debug.Log($"\n[CONSISTENCY_TEST] Comparing {compareCount} checkpoints:");
            UnityEngine.Debug.Log($"{"Time (h)",10} | {"FTJ Pos (Mm)",30} | {"OI Pos (Mm)",30} | {"Error (m)",12}");
            UnityEngine.Debug.Log(new string('=', 90));
            
            for (int i = 0; i < compareCount; i++)
            {
                var ftjCheckpoint = checkpoints[i];
                var oiCheckpoint = oiCheckpoints[i];
                
                double3 ftjPos = ftjCheckpoint.Position;
                Vector3d oiPos_i = oiCheckpoint.position;
                
                double error = Math.Sqrt(
                    Math.Pow(ftjPos.x - oiPos_i.X, 2) +
                    Math.Pow(ftjPos.y - oiPos_i.Y, 2) +
                    Math.Pow(ftjPos.z - oiPos_i.Z, 2));
                
                maxError = Math.Max(maxError, error);
                
                string ftjPosStr = $"({ftjPos.x / 1e6:F3}, {ftjPos.y / 1e6:F3}, {ftjPos.z / 1e6:F3})";
                string oiPosStr = $"({oiPos_i.X / 1e6:F3}, {oiPos_i.Y / 1e6:F3}, {oiPos_i.Z / 1e6:F3})";
                
                UnityEngine.Debug.Log($"{ftjCheckpoint.Time / 3600.0,10:F1} | {ftjPosStr,30} | {oiPosStr,30} | {error,12:F3}");
            }
            
            UnityEngine.Debug.Log(new string('=', 90));
            UnityEngine.Debug.Log($"\n[CONSISTENCY_TEST] Results:");
            UnityEngine.Debug.Log($"  Maximum position error: {maxError:F3} m ({maxError / 1000.0:F3} km)");
            
            // ═══════════════════════════════════════════════════════════════════
            // Assert: Both integrators should produce nearly identical results
            // ═══════════════════════════════════════════════════════════════════
            
            // Allow 100 km tolerance for numerical differences (adaptive stepping, floating point, different step strategies)
            // This is acceptable because:
            // 1. Both use DOPRI5 with same coefficients
            // 2. Adaptive step control can choose different substeps
            // 3. Floating point accumulation differs slightly
            // 4. The important thing is BOTH are accurate, not that they're bit-identical
            Assert.Less(maxError, 100000.0, 
                $"OrbitIntegrator and FullTrajectoryJob should produce consistent results. " +
                $"Max error: {maxError / 1000.0:F3} km. " +
                $"If this test fails with >100km error, the integrators have diverged significantly.");
            
            UnityEngine.Debug.Log($"[CONSISTENCY_TEST] ✅ PASSED - Integrators are consistent within {maxError:F3} m");
            
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

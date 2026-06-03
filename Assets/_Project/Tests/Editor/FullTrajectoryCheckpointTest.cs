using System.Diagnostics;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Simulation;

namespace Galilego.Tests.Editor
{
    [TestFixture]
    public class FullTrajectoryCheckpointTest
    {
        private const double JupiterSGP = 1.266865319e17;
        private const double CheckpointInterval = 21600.0;

        private struct TestOrbits
        {
            public NativeArray<MoonOrbitData> Orbits;
            public NativeArray<double> Times;
            public NativeArray<BodyState> Ephemeris;
            public NativeArray<double3> Velocities;
            public int MoonCount;
            public int FlatCount;
        }

        private TestOrbits BuildMoonData(double startTime, double endTime, int moonCount)
        {
            double ephemerisStep = 3600.0;
            int sampleCount = (int)((endTime - startTime) / ephemerisStep) + 2;

            var orbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.TempJob);
            orbits[0] = new MoonOrbitData
            {
                SemiMajorAxis = 4.21800e8, Eccentricity = 0.004,
                InclinationRad = 0.0f, AscendingNodeRad = 0.0f,
                PeriapsisArgRad = 49.1f * math.PI / 180.0f,
                MeanAnomalyAtEpochRad = 330.9f * math.PI / 180.0f,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 5.95991547e12,
                StandardGravitationalParameter = 5.95991547e12
            };
            orbits[1] = new MoonOrbitData
            {
                SemiMajorAxis = 6.71100e8, Eccentricity = 0.009,
                InclinationRad = 0.5f * math.PI / 180.0f,
                AscendingNodeRad = 184.0f * math.PI / 180.0f,
                PeriapsisArgRad = 45.0f * math.PI / 180.0f,
                MeanAnomalyAtEpochRad = 345.4f * math.PI / 180.0f,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 3.20271210e12,
                StandardGravitationalParameter = 3.20271210e12
            };
            orbits[2] = new MoonOrbitData
            {
                SemiMajorAxis = 1.07040e9, Eccentricity = 0.001,
                InclinationRad = 0.2f * math.PI / 180.0f,
                AscendingNodeRad = 58.5f * math.PI / 180.0f,
                PeriapsisArgRad = 198.3f * math.PI / 180.0f,
                MeanAnomalyAtEpochRad = 324.8f * math.PI / 180.0f,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 9.88783275e12,
                StandardGravitationalParameter = 9.88783275e12
            };
            orbits[3] = new MoonOrbitData
            {
                SemiMajorAxis = 1.88270e9, Eccentricity = 0.007,
                InclinationRad = 0.3f * math.PI / 180.0f,
                AscendingNodeRad = 309.1f * math.PI / 180.0f,
                PeriapsisArgRad = 43.8f * math.PI / 180.0f,
                MeanAnomalyAtEpochRad = 87.4f * math.PI / 180.0f,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 7.17928340e12,
                StandardGravitationalParameter = 7.17928340e12
            };

            var times = new NativeArray<double>(sampleCount, Allocator.TempJob);
            for (int i = 0; i < sampleCount; i++)
                times[i] = startTime + i * ephemerisStep;
            times[sampleCount - 1] = endTime;

            int flatCount = sampleCount * moonCount;
            var ephemeris = new NativeArray<BodyState>(flatCount, Allocator.TempJob);
            var velocities = new NativeArray<double3>(flatCount, Allocator.TempJob);

            var moonJob = new MoonEphemerisJob
            {
                SampleTimes = times, MoonOrbits = orbits,
                Results = ephemeris, JupiterPosition = double3.zero, PlaneMapping = 0
            };
            var handle = moonJob.Schedule(flatCount, 64);

            var velJob = new EphemerisVelocityJob
            {
                SampleTimes = times, MoonStates = ephemeris,
                Velocities = velocities, MoonCount = moonCount
            };
            handle = velJob.Schedule(flatCount, 64, handle);
            handle.Complete();

            return new TestOrbits
            {
                Orbits = orbits, Times = times,
                Ephemeris = ephemeris, Velocities = velocities,
                MoonCount = moonCount, FlatCount = flatCount
            };
        }

        private void DisposeMoonData(ref TestOrbits data)
        {
            data.Orbits.Dispose();
            data.Times.Dispose();
            data.Ephemeris.Dispose();
            data.Velocities.Dispose();
        }

        private struct JobResources
        {
            public NativeArray<TrajectoryPoint> Output;
            public NativeReference<int> PointCount;
            public NativeReference<int> CalcStatus;
            public NativeArray<SegmentBoundaryState> Boundaries;
            public NativeReference<int> BoundaryCount;
            public NativeArray<long> Counters;
            public NativeArray<TrajectoryCheckpoint> Checkpoints;
            public NativeReference<int> CheckpointCount;
            public NativeArray<ManeuverNodeData> Nodes;
        }

        private JobResources AllocJobResources(int maxPoints, int maxBoundaries, int maxCheckpoints, int nodeCount)
        {
            return new JobResources
            {
                Output = new NativeArray<TrajectoryPoint>(maxPoints, Allocator.TempJob),
                PointCount = new NativeReference<int>(0, Allocator.TempJob),
                CalcStatus = new NativeReference<int>(0, Allocator.TempJob),
                Boundaries = new NativeArray<SegmentBoundaryState>(maxBoundaries, Allocator.TempJob),
                BoundaryCount = new NativeReference<int>(0, Allocator.TempJob),
                Counters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob),
                Checkpoints = new NativeArray<TrajectoryCheckpoint>(maxCheckpoints, Allocator.TempJob),
                CheckpointCount = new NativeReference<int>(0, Allocator.TempJob),
                Nodes = new NativeArray<ManeuverNodeData>(nodeCount, Allocator.TempJob)
            };
        }

        private void DisposeJobResources(ref JobResources r)
        {
            r.Output.Dispose();
            r.PointCount.Dispose();
            r.CalcStatus.Dispose();
            r.Boundaries.Dispose();
            r.BoundaryCount.Dispose();
            r.Counters.Dispose();
            r.Checkpoints.Dispose();
            r.CheckpointCount.Dispose();
            r.Nodes.Dispose();
        }

        private double ComputeMajorStep(int maxPoints, double predictionSeconds)
        {
            double ms = math.max(10.0, predictionSeconds / (maxPoints * 0.9));
            return math.min(ms, 600.0);
        }

        private FullTrajectoryJob BuildJob(
            ref JobResources r,
            ref TestOrbits moons,
            double3 startPos, double3 startVel, double startTime,
            double predictionSeconds,
            double majorStep)
        {
            double substepLimit = math.min(5.0, majorStep);

            return new FullTrajectoryJob
            {
                Nodes = r.Nodes,
                MoonEphemeris = moons.Ephemeris,
                EphemerisTimes = moons.Times,
                MoonVelocities = moons.Velocities,
                MoonCount = moons.MoonCount,
                PlaneMapping = 0,
                ReferenceFrameIndex = 0,

                StartPos = startPos,
                StartVel = startVel,
                StartTime = startTime,

                JupiterPosition = double3.zero,
                JupiterSGP = JupiterSGP,

                MajorStepSeconds = majorStep,
                SubstepLimitSeconds = substepLimit,
                MaxSubstepsPerSegment = 4096,
                MaxPoints = r.Output.Length,
                MaxStepsPerSegment = 2000000,

                PredictionLengthSeconds = predictionSeconds,
                MaxPredictionLengthSeconds = predictionSeconds,

                OutputPoints = r.Output,
                PointCount = r.PointCount,
                CalculationStatus = r.CalcStatus,

                SegmentBoundaries = r.Boundaries,
                SegmentBoundaryCount = r.BoundaryCount,
                ProfileCounters = r.Counters,

                CheckpointIntervalSeconds = CheckpointInterval,
                Checkpoints = r.Checkpoints,
                CheckpointCount = r.CheckpointCount
            };
        }

        [Test]
        public void IncrementalRecalc_LateManeuverChanged_MatchesFullRecalc()
        {
            int moonCount = 4;
            double predictionSeconds = 60.0 * 86400.0;
            double startTime = 0.0;
            double endTime = startTime + predictionSeconds;
            double3 startPos = new double3(6.0e8, 0, 0);
            double3 startVel = new double3(0, 14500, 0);

            // ── Create 3 nodes ────────────────────────────────────────────────
            // Node0 at t=10d (instant prograde)
            // Node1 at t=25d (instant prograde)
            // Node2 at t=40d (long burn, 2h)
            int nodeCount = 3;
            var baseNodes = new ManeuverNodeData[nodeCount];
            baseNodes[0] = new ManeuverNodeData
            {
                StartTime = 10.0 * 86400.0, DvPrograde = 300, DvNormal = 0, DvRadial = 0,
                Duration = 0, IsInstant = 1, HasEngine = 0,
                ThrustNewtons = 0, SpecificImpulseSeconds = 0, InitialMassKg = 0
            };
            baseNodes[1] = new ManeuverNodeData
            {
                StartTime = 25.0 * 86400.0, DvPrograde = 200, DvNormal = 50, DvRadial = 0,
                Duration = 0, IsInstant = 1, HasEngine = 0,
                ThrustNewtons = 0, SpecificImpulseSeconds = 0, InitialMassKg = 0
            };
            baseNodes[2] = new ManeuverNodeData
            {
                StartTime = 40.0 * 86400.0, DvPrograde = 500, DvNormal = 0, DvRadial = 100,
                Duration = 7200.0, IsInstant = 0, HasEngine = 1,
                ThrustNewtons = 500, SpecificImpulseSeconds = 300, InitialMassKg = 50000
            };

            var moons = BuildMoonData(startTime, endTime, moonCount);

            int maxPoints = 30000;
            int maxCheckpoints = 500;
            int maxBoundaries = nodeCount + 2;
            double majorStep = ComputeMajorStep(maxPoints, predictionSeconds);

            // ── Step 1: Full calc with original nodes ──────────────────────────
            var res1 = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, nodeCount);
            for (int i = 0; i < nodeCount; i++) res1.Nodes[i] = baseNodes[i];

            var job1 = BuildJob(ref res1, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);

            var sw1 = Stopwatch.StartNew();
            job1.Schedule().Complete();
            sw1.Stop();
            long baseTicks = sw1.ElapsedTicks;

            int baseCount = res1.PointCount.Value;
            int baseStatus = res1.CalcStatus.Value;
            int baseCpCount = res1.CheckpointCount.Value;

            Assert.AreEqual(1, baseStatus, "Baseline job should complete");
            Assert.Greater(baseCount, 100, "Baseline should produce trajectory points");
            Assert.Greater(baseCpCount, 1, "Should record at least 1 checkpoint");

            // ── Step 2: Modify node 2 (late maneuver) ──────────────────────────
            var modifiedNodes = new ManeuverNodeData[nodeCount];
            for (int i = 0; i < nodeCount; i++) modifiedNodes[i] = baseNodes[i];
            modifiedNodes[2].DvPrograde = 1200; // was 500, increased to 1200

            int changedIdx = 2;

            // ── Step 3: Find best checkpoint (NodeVersion <= changedIdx) ───────
            TrajectoryCheckpoint bestCp = default;
            bool found = false;
            for (int i = baseCpCount - 1; i >= 0; i--)
            {
                var cp = res1.Checkpoints[i];
                if (cp.NodeVersion <= changedIdx)
                {
                    bestCp = cp;
                    found = true;
                    break;
                }
            }
            Assert.IsTrue(found, "Should find a valid checkpoint for NodeVersion <= 2");

            // ── Step 4: Partial job from checkpoint with modified tail ─────────
            // Only nodes from changedIdx onward (global index 2 → offset 0)
            int tailNodeCount = nodeCount - changedIdx;
            var resPartial = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, tailNodeCount);
            for (int i = 0; i < tailNodeCount; i++)
                resPartial.Nodes[i] = modifiedNodes[changedIdx + i];

            double remainingPrediction = math.max(0.0, endTime - bestCp.Time);

            var jobPartial = BuildJob(ref resPartial, ref moons,
                bestCp.Position, bestCp.Velocity, bestCp.Time,
                remainingPrediction, majorStep);

            var swPartial = Stopwatch.StartNew();
            jobPartial.Schedule().Complete();
            swPartial.Stop();
            long partialTicks = swPartial.ElapsedTicks;

            int partialCount = resPartial.PointCount.Value;
            int partialStatus = resPartial.CalcStatus.Value;

            Assert.AreEqual(1, partialStatus, "Partial job should complete");

            // ── Step 5: Reference full job from t=0 with ALL modified nodes ────
            var resFull = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, nodeCount);
            for (int i = 0; i < nodeCount; i++) resFull.Nodes[i] = modifiedNodes[i];

            var jobFull = BuildJob(ref resFull, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);

            var swFull = Stopwatch.StartNew();
            jobFull.Schedule().Complete();
            swFull.Stop();
            long fullTicks = swFull.ElapsedTicks;

            int fullCount = resFull.PointCount.Value;
            int fullStatus = resFull.CalcStatus.Value;

            Assert.AreEqual(1, fullStatus, "Full reference job should complete");

            // ── Step 6: Compare suffix trajectories (time-aligned) ─────────────
            // Walk through both outputs, matching by closest time within one major step
            double restartTime = bestCp.Time;
            double maxTimeDiff = majorStep * 2.0;

            double maxPosError = 0.0;
            double maxTimeError = 0.0;
            int errorCount = 0;
            int compareCount = 0;
            int partialIdx = 0;
            int fullIdx = 0;

            while (partialIdx < partialCount && fullIdx < fullCount)
            {
                var ptPartial = resPartial.Output[partialIdx];
                var ptFull = resFull.Output[fullIdx];

                double timeDiff = ptPartial.Time - ptFull.Time;

                if (math.abs(timeDiff) <= maxTimeDiff)
                {
                    double posErr = math.distance(ptPartial.Position, ptFull.Position);
                    double tErr = math.abs(timeDiff);
                    maxPosError = math.max(maxPosError, posErr);
                    maxTimeError = math.max(maxTimeError, tErr);
                    if (posErr > 1.0 || tErr > 0.1)
                        errorCount++;
                    compareCount++;
                    partialIdx++;
                    fullIdx++;
                }
                else if (timeDiff < 0)
                {
                    // Partial point is earlier — advance partial
                    partialIdx++;
                }
                else
                {
                    // Full point is earlier — advance full
                    fullIdx++;
                }
            }

            Assert.Greater(compareCount, 10, "Should have at least 10 matching point pairs");

            // ── Step 7: Log results ────────────────────────────────────────────
            double baseMs = (double)baseTicks / Stopwatch.Frequency * 1000.0;
            double partialMs = (double)partialTicks / Stopwatch.Frequency * 1000.0;
            double fullMs = (double)fullTicks / Stopwatch.Frequency * 1000.0;
            double speedup = fullMs / partialMs;

            double cpIntervalHrs = CheckpointInterval / 3600.0;

            UnityEngine.Debug.Log(
                "═══════════════════════════════════════════════════════════\n" +
                $"CHECKPOINT TEST: Late maneuver change (node[{changedIdx}])\n" +
                "───────────────────────────────────────────────────────────\n" +
                $"  Baseline (unchanged):  {baseMs:F1}ms  ({baseCount} pts, {baseCpCount} cps @ {cpIntervalHrs}h)\n" +
                $"  Full recalc (changed): {fullMs:F1}ms  ({fullCount} pts)\n" +
                $"  Partial (checkpoint):  {partialMs:F1}ms  ({partialCount} pts)\n" +
                $"  Speedup:               {speedup:F1}x\n" +
                $"  Checkpoint time:       {bestCp.Time / 86400.0:F2}d\n" +
                $"  Checkpoint nodeVer:    {bestCp.NodeVersion}\n" +
                "───────────────────────────────────────────────────────────\n" +
                $"  Max position error:    {maxPosError:F6}m\n" +
                $"  Max time error:        {maxTimeError:F6}s\n" +
               $"  Overlapping points:    {compareCount}\n" +
               $"  Errors (>1m):          {errorCount}\n" +
                "═══════════════════════════════════════════════════════════");

            Assert.AreEqual(0, errorCount,
                $"Partial trajectory diverges: {errorCount} points > 1m error, max={maxPosError:F2}m");

            // ── Cleanup ────────────────────────────────────────────────────────
            DisposeJobResources(ref res1);
            DisposeJobResources(ref resPartial);
            DisposeJobResources(ref resFull);
            DisposeMoonData(ref moons);
        }

        [Test]
        public void IncrementalRecalc_EarlyManeuverChanged_UsesStartCheckpoint()
        {
            int moonCount = 4;
            double predictionSeconds = 30.0 * 86400.0;
            double startTime = 0.0;
            double endTime = startTime + predictionSeconds;
            double3 startPos = new double3(6.0e8, 0, 0);
            double3 startVel = new double3(0, 14500, 0);

            // ── Create 3 nodes ────────────────────────────────────────────────
            int nodeCount = 3;
            var baseNodes = new ManeuverNodeData[nodeCount];
            baseNodes[0] = new ManeuverNodeData
            {
                StartTime = 5.0 * 86400.0, DvPrograde = 200, DvNormal = 0, DvRadial = 0,
                Duration = 0, IsInstant = 1, HasEngine = 0,
                ThrustNewtons = 0, SpecificImpulseSeconds = 0, InitialMassKg = 0
            };
            baseNodes[1] = new ManeuverNodeData
            {
                StartTime = 15.0 * 86400.0, DvPrograde = 300, DvNormal = 0, DvRadial = 0,
                Duration = 0, IsInstant = 1, HasEngine = 0,
                ThrustNewtons = 0, SpecificImpulseSeconds = 0, InitialMassKg = 0
            };
            baseNodes[2] = new ManeuverNodeData
            {
                StartTime = 22.0 * 86400.0, DvPrograde = 400, DvNormal = 0, DvRadial = 0,
                Duration = 3600.0, IsInstant = 0, HasEngine = 1,
                ThrustNewtons = 500, SpecificImpulseSeconds = 300, InitialMassKg = 50000
            };

            var moons = BuildMoonData(startTime, endTime, moonCount);

            int maxPoints = 20000;
            int maxCheckpoints = 300;
            int maxBoundaries = nodeCount + 2;
            double majorStep = ComputeMajorStep(maxPoints, predictionSeconds);

            // ── Step 1: Full calc with original nodes ──────────────────────────
            var resBase = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, nodeCount);
            for (int i = 0; i < nodeCount; i++) resBase.Nodes[i] = baseNodes[i];

            var jobBase = BuildJob(ref resBase, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);

            jobBase.Schedule().Complete();
            int baseCpCount = resBase.CheckpointCount.Value;
            Assert.Greater(baseCpCount, 1, "Should record checkpoints");

            // ── Step 2: Modify node 0 (early maneuver) ─────────────────────────
            var modifiedNodes = new ManeuverNodeData[nodeCount];
            for (int i = 0; i < nodeCount; i++) modifiedNodes[i] = baseNodes[i];
            modifiedNodes[0].DvPrograde = 500; // was 200

            int changedIdx = 0;

            // ── Step 3: Find best checkpoint (NodeVersion <= 0) ────────────────
            TrajectoryCheckpoint bestCp = default;
            bool found = false;
            for (int i = baseCpCount - 1; i >= 0; i--)
            {
                var cp = resBase.Checkpoints[i];
                if (cp.NodeVersion <= changedIdx)
                {
                    bestCp = cp;
                    found = true;
                    break;
                }
            }
            Assert.IsTrue(found, "Should find checkpoint with NodeVersion=0");
            Assert.AreEqual(0, bestCp.NodeVersion, "Checkpoint NodeVersion should be 0");
            Assert.GreaterOrEqual(bestCp.Time, 0.0, "Checkpoint time should be non-negative");

            // ── Step 4: Partial job from checkpoint (t=0) with modified nodes ──
            // When changedIdx=0, everything is invalidated, so we restart from t=0
            int tailNodeCount = nodeCount - changedIdx;
            var resPartial = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, tailNodeCount);
            for (int i = 0; i < tailNodeCount; i++)
                resPartial.Nodes[i] = modifiedNodes[changedIdx + i];

            var jobPartial = BuildJob(ref resPartial, ref moons,
                bestCp.Position, bestCp.Velocity, bestCp.Time,
                predictionSeconds, majorStep);

            // ── Step 5: Full reference job from t=0 ────────────────────────────
            var resFull = AllocJobResources(maxPoints, maxBoundaries, maxCheckpoints, nodeCount);
            for (int i = 0; i < nodeCount; i++) resFull.Nodes[i] = modifiedNodes[i];

            var jobFull = BuildJob(ref resFull, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);

            // Schedule both
            jobPartial.Schedule().Complete();
            jobFull.Schedule().Complete();

            // ── Step 6: Compare trajectories (time-aligned) ────────────────────
            int partialCount = resPartial.PointCount.Value;
            int fullCount = resFull.PointCount.Value;
            double maxTimeDiff = majorStep * 2.0;

            double maxPosError = 0.0;
            int errorCount = 0;
            int compareCount = 0;
            int pIdx = 0, fIdx = 0;
            while (pIdx < partialCount && fIdx < fullCount)
            {
                double timeDiff = resPartial.Output[pIdx].Time - resFull.Output[fIdx].Time;
                if (math.abs(timeDiff) <= maxTimeDiff)
                {
                    double posErr = math.distance(
                        resPartial.Output[pIdx].Position, resFull.Output[fIdx].Position);
                    maxPosError = math.max(maxPosError, posErr);
                    if (posErr > 1.0) errorCount++;
                    compareCount++;
                    pIdx++; fIdx++;
                }
                else if (timeDiff < 0) pIdx++;
                else fIdx++;
            }

            Assert.Greater(compareCount, 10, "Should have at least 10 matching point pairs");

            UnityEngine.Debug.Log(
                "═══════════════════════════════════════════════════════════\n" +
                $"CHECKPOINT TEST: Early maneuver change (node[{changedIdx}])\n" +
                "───────────────────────────────────────────────────────────\n" +
                $"  Restart from checkpoint at t={bestCp.Time:F0}s (NodeVersion={bestCp.NodeVersion})\n" +
                $"  Partial points: {partialCount}, Full points: {fullCount}\n" +
                $"  Matched pairs:  {compareCount}\n" +
                $"  Max position error: {maxPosError:F6}m\n" +
                $"  Errors (>1m): {errorCount}\n" +
                "═══════════════════════════════════════════════════════════");

            Assert.AreEqual(0, errorCount,
                $"Partial diverges from full after early change: {errorCount} pts > 1m");

            DisposeJobResources(ref resBase);
            DisposeJobResources(ref resPartial);
            DisposeJobResources(ref resFull);
            DisposeMoonData(ref moons);
        }

        [Test]
        public void IncrementalRecalc_NoChange_CheckpointCountIsConsistent()
        {
            // Verify that running the same job twice produces the same
            // checkpoint output (for reproducibility)
            int moonCount = 4;
            double predictionSeconds = 14.0 * 86400.0;
            double startTime = 0.0;
            double endTime = startTime + predictionSeconds;
            double3 startPos = new double3(6.0e8, 0, 0);
            double3 startVel = new double3(0, 14500, 0);

            int nodeCount = 2;
            var nodes = new ManeuverNodeData[nodeCount];
            nodes[0] = new ManeuverNodeData
            {
                StartTime = 5.0 * 86400.0, DvPrograde = 400, DvNormal = 0, DvRadial = 0,
                Duration = 0, IsInstant = 1, HasEngine = 0,
                ThrustNewtons = 0, SpecificImpulseSeconds = 0, InitialMassKg = 0
            };
            nodes[1] = new ManeuverNodeData
            {
                StartTime = 10.0 * 86400.0, DvPrograde = 300, DvNormal = 50, DvRadial = 0,
                Duration = 3600.0, IsInstant = 0, HasEngine = 1,
                ThrustNewtons = 500, SpecificImpulseSeconds = 300, InitialMassKg = 50000
            };

            var moons = BuildMoonData(startTime, endTime, moonCount);
            int maxPoints = 15000;
            int maxCheckpoints = 300;
            double majorStep = ComputeMajorStep(maxPoints, predictionSeconds);

            var resA = AllocJobResources(maxPoints, nodeCount + 2, maxCheckpoints, nodeCount);
            var resB = AllocJobResources(maxPoints, nodeCount + 2, maxCheckpoints, nodeCount);
            for (int i = 0; i < nodeCount; i++) { resA.Nodes[i] = nodes[i]; resB.Nodes[i] = nodes[i]; }

            var jobA = BuildJob(ref resA, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);
            jobA.Schedule().Complete();

            var jobB = BuildJob(ref resB, ref moons, startPos, startVel, startTime,
                predictionSeconds, majorStep);
            jobB.Schedule().Complete();

            int cpA = resA.CheckpointCount.Value;
            int cpB = resB.CheckpointCount.Value;
            Assert.AreEqual(cpA, cpB, "Checkpoint count should be identical between runs");

            for (int i = 0; i < cpA; i++)
            {
                var a = resA.Checkpoints[i];
                var b = resB.Checkpoints[i];
                double posDiff = math.distance(a.Position, b.Position);
                Assert.AreEqual(a.Time, b.Time, 1e-6, $"Cp {i} time mismatch");
                Assert.AreEqual(a.NodeVersion, b.NodeVersion, $"Cp {i} NodeVersion mismatch");
                Assert.Less(posDiff, 0.001, $"Cp {i} position mismatch");
            }

            UnityEngine.Debug.Log(
                $"[CONSISTENCY] Two identical runs: {cpA} checkpoints, all match.");

            DisposeJobResources(ref resA);
            DisposeJobResources(ref resB);
            DisposeMoonData(ref moons);
        }
    }
}

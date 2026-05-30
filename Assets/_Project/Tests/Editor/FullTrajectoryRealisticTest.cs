using System.Diagnostics;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Simulation;

namespace Galilego.Tests.Editor
{
    [TestFixture]
    public class FullTrajectoryRealisticTest
    {
        private const double JupiterSGP = 1.266865319e17;
        private const double ReferenceDistance = 1e8;

        [Test]
        public void RealisticScenario_30DayPrediction_WithMoons()
        {
            int moonCount = 4;
            double predictionSeconds = 30.0 * 86400.0;
            double ephemerisStep = 3600.0;
            int sampleCount = (int)(predictionSeconds / ephemerisStep) + 2;
            double startTime = 0.0;
            double endTime = startTime + predictionSeconds;

            var sw = new Stopwatch();

            // ── Moon orbital data ────────────────────────────────────────────
            var orbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.TempJob);
            orbits[0] = new MoonOrbitData // Io
            {
                SemiMajorAxis = 4.21800e8, Eccentricity = 0.004,
                InclinationRad = 0.0, AscendingNodeRad = 0.0,
                PeriapsisArgRad = 49.1 * math.PI / 180.0,
                MeanAnomalyAtEpochRad = 330.9 * math.PI / 180.0,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 5.95991547e12,
                StandardGravitationalParameter = 5.95991547e12
            };
            orbits[1] = new MoonOrbitData // Europa
            {
                SemiMajorAxis = 6.71100e8, Eccentricity = 0.009,
                InclinationRad = 0.5 * math.PI / 180.0,
                AscendingNodeRad = 184.0 * math.PI / 180.0,
                PeriapsisArgRad = 45.0 * math.PI / 180.0,
                MeanAnomalyAtEpochRad = 345.4 * math.PI / 180.0,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 3.20271210e12,
                StandardGravitationalParameter = 3.20271210e12
            };
            orbits[2] = new MoonOrbitData // Ganymede
            {
                SemiMajorAxis = 1.07040e9, Eccentricity = 0.001,
                InclinationRad = 0.2 * math.PI / 180.0,
                AscendingNodeRad = 58.5 * math.PI / 180.0,
                PeriapsisArgRad = 198.3 * math.PI / 180.0,
                MeanAnomalyAtEpochRad = 324.8 * math.PI / 180.0,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 9.88783275e12,
                StandardGravitationalParameter = 9.88783275e12
            };
            orbits[3] = new MoonOrbitData // Callisto
            {
                SemiMajorAxis = 1.88270e9, Eccentricity = 0.007,
                InclinationRad = 0.3 * math.PI / 180.0,
                AscendingNodeRad = 309.1 * math.PI / 180.0,
                PeriapsisArgRad = 43.8 * math.PI / 180.0,
                MeanAnomalyAtEpochRad = 87.4 * math.PI / 180.0,
                EpochTimeSeconds = 0,
                GravitationalParameter = JupiterSGP + 7.17928340e12,
                StandardGravitationalParameter = 7.17928340e12
            };

            // ── Ephemeris times ──────────────────────────────────────────────
            var sampleTimes = new NativeArray<double>(sampleCount, Allocator.TempJob);
            for (int i = 0; i < sampleCount; i++)
                sampleTimes[i] = startTime + i * ephemerisStep;
            sampleTimes[sampleCount - 1] = endTime;

            int flatCount = sampleCount * moonCount;
            var ephemeris = new NativeArray<BodyState>(flatCount, Allocator.TempJob);
            var velocities = new NativeArray<double3>(flatCount, Allocator.TempJob);
            var output = new NativeArray<TrajectoryPoint>(20000, Allocator.TempJob);
            var pointCount = new NativeReference<int>(0, Allocator.TempJob);
            var calcStatus = new NativeReference<int>(0, Allocator.TempJob);
            var boundaries = new NativeArray<SegmentBoundaryState>(1, Allocator.TempJob);
            var boundaryCount = new NativeReference<int>(0, Allocator.TempJob);
            var emptyNodes = new NativeArray<ManeuverNodeData>(0, Allocator.TempJob);
            var counters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob);

            // ── Compute ephemeris ────────────────────────────────────────────
            var moonJob = new MoonEphemerisJob
            {
                SampleTimes = sampleTimes,
                MoonOrbits = orbits,
                Results = ephemeris,
                JupiterPosition = double3.zero,
                PlaneMapping = 0
            };
            var handle = moonJob.Schedule(flatCount, 64);

            var velJob = new EphemerisVelocityJob
            {
                SampleTimes = sampleTimes,
                MoonStates = ephemeris,
                Velocities = velocities,
                MoonCount = moonCount
            };
            handle = velJob.Schedule(flatCount, 64, handle);

            // Start timing AFTER ephemeris is ready
            handle.Complete();
            sw.Start();

            // ── Trajectory job ───────────────────────────────────────────────
            double3 startPos = new double3(6.0e8, 0, 0);
            double3 startVel = new double3(0, 14500, 0);

            double majorStep = math.max(10.0, predictionSeconds / (20000 * 0.9));
            majorStep = math.min(majorStep, 600.0);
            double substepLimit = math.min(5.0, majorStep);

            var job = new FullTrajectoryJob
            {
                Nodes = emptyNodes,
                MoonEphemeris = ephemeris,
                EphemerisTimes = sampleTimes,
                MoonVelocities = velocities,
                MoonCount = moonCount,
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
                MaxPoints = 20000,
                MaxStepsPerSegment = 2000000,

                PredictionLengthSeconds = predictionSeconds,
                MaxPredictionLengthSeconds = predictionSeconds,

                OutputPoints = output,
                PointCount = pointCount,
                CalculationStatus = calcStatus,
                SegmentBoundaries = boundaries,
                SegmentBoundaryCount = boundaryCount,
                ProfileCounters = counters
            };

            job.Schedule().Complete();
            sw.Stop();

            int count = pointCount.Value;
            int status = calcStatus.Value;
            double elapsedMs = sw.Elapsed.TotalMilliseconds;

            double avgSubstepsPerMajor = counters[1] / (double)math.max(1, counters[0]);

            // ── Log results ───────────────────────────────────────────────────
            UnityEngine.Debug.Log(
                "═══════════════════════════════════════════════════════════\n" +
                $"REALISTIC TEST: 30 days, 4 Galilean moons\n" +
                "───────────────────────────────────────────────────────────\n" +
                $"Timer (job only): {elapsedMs:F1}ms\n" +
                $"  output points = {count}\n" +
                $"  major steps   = {counters[0]}\n" +
                $"  substeps      = {counters[1]}\n" +
                $"  avg substeps  = {avgSubstepsPerMajor:F1} per major step\n" +
                $"  evalAccel     = {counters[2]}\n" +
                $"  hermit        = {counters[3]}  ({counters[3] / (double)math.max(1, counters[1]):F1} per substep)\n" +
                $"  ephemSearch   = {counters[4]}\n" +
                "───────────────────────────────────────────────────────────\n" +
                $"Without moons (estimate): {elapsedMs * 0.4:F1}ms (if hermit=0)\n" +
                $"═══════════════════════════════════════════════════════════");

            Assert.AreEqual(1, status, "Job should complete successfully");
            Assert.Greater(count, 50, "Should produce at least 50 trajectory points");
            Assert.Greater(counters[0], 10, "Should perform multiple major steps");
            Assert.Greater(counters[1], counters[0], "Substeps >= majorSteps");

            // No NaN points
            for (int i = 0; i < count; i++)
            {
                var pt = output[i];
                Assert.IsFalse(double.IsNaN(pt.Position.x), $"Point {i}: NaN x");
                Assert.IsFalse(double.IsNaN(pt.Position.y), $"Point {i}: NaN y");
                Assert.IsFalse(double.IsNaN(pt.Position.z), $"Point {i}: NaN z");
            }

            // Cleanup
            orbits.Dispose();
            sampleTimes.Dispose();
            ephemeris.Dispose();
            velocities.Dispose();
            output.Dispose();
            pointCount.Dispose();
            calcStatus.Dispose();
            boundaries.Dispose();
            boundaryCount.Dispose();
            emptyNodes.Dispose();
            counters.Dispose();
        }

        [Test]
        public void RealisticScenario_1YearPrediction_Smoke()
        {
            int moonCount = 4;
            double predictionSeconds = 365.0 * 86400.0;
            double ephemerisStep = 3600.0 * 5;
            int sampleCount = (int)(predictionSeconds / ephemerisStep) + 2;
            double startTime = 0.0;

            var orbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.TempJob);
            orbits[0] = new MoonOrbitData { SemiMajorAxis = 4.21800e8, Eccentricity = 0.004, InclinationRad = 0, AscendingNodeRad = 0, PeriapsisArgRad = 49.1f * math.PI / 180.0f, MeanAnomalyAtEpochRad = 330.9f * math.PI / 180.0f, EpochTimeSeconds = 0, GravitationalParameter = JupiterSGP + 5.95991547e12, StandardGravitationalParameter = 5.95991547e12 };
            orbits[1] = new MoonOrbitData { SemiMajorAxis = 6.71100e8, Eccentricity = 0.009, InclinationRad = 0.5f * math.PI / 180.0f, AscendingNodeRad = 184.0f * math.PI / 180.0f, PeriapsisArgRad = 45.0f * math.PI / 180.0f, MeanAnomalyAtEpochRad = 345.4f * math.PI / 180.0f, EpochTimeSeconds = 0, GravitationalParameter = JupiterSGP + 3.20271210e12, StandardGravitationalParameter = 3.20271210e12 };
            orbits[2] = new MoonOrbitData { SemiMajorAxis = 1.07040e9, Eccentricity = 0.001, InclinationRad = 0.2f * math.PI / 180.0f, AscendingNodeRad = 58.5f * math.PI / 180.0f, PeriapsisArgRad = 198.3f * math.PI / 180.0f, MeanAnomalyAtEpochRad = 324.8f * math.PI / 180.0f, EpochTimeSeconds = 0, GravitationalParameter = JupiterSGP + 9.88783275e12, StandardGravitationalParameter = 9.88783275e12 };
            orbits[3] = new MoonOrbitData { SemiMajorAxis = 1.88270e9, Eccentricity = 0.007, InclinationRad = 0.3f * math.PI / 180.0f, AscendingNodeRad = 309.1f * math.PI / 180.0f, PeriapsisArgRad = 43.8f * math.PI / 180.0f, MeanAnomalyAtEpochRad = 87.4f * math.PI / 180.0f, EpochTimeSeconds = 0, GravitationalParameter = JupiterSGP + 7.17928340e12, StandardGravitationalParameter = 7.17928340e12 };

            var sampleTimes = new NativeArray<double>(sampleCount, Allocator.TempJob);
            for (int i = 0; i < sampleCount; i++)
                sampleTimes[i] = startTime + i * ephemerisStep;

            int flatCount = sampleCount * moonCount;
            var ephemeris = new NativeArray<BodyState>(flatCount, Allocator.TempJob);
            var velocities = new NativeArray<double3>(flatCount, Allocator.TempJob);
            var output = new NativeArray<TrajectoryPoint>(50000, Allocator.TempJob);
            var pointCount = new NativeReference<int>(0, Allocator.TempJob);
            var calcStatus = new NativeReference<int>(0, Allocator.TempJob);
            var boundaries = new NativeArray<SegmentBoundaryState>(1, Allocator.TempJob);
            var boundaryCount = new NativeReference<int>(0, Allocator.TempJob);
            var emptyNodes = new NativeArray<ManeuverNodeData>(0, Allocator.TempJob);
            var counters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob);

            var moonJob = new MoonEphemerisJob
            {
                SampleTimes = sampleTimes,
                MoonOrbits = orbits,
                Results = ephemeris,
                JupiterPosition = double3.zero,
                PlaneMapping = 0
            };
            var handle = moonJob.Schedule(flatCount, 64);

            var velJob = new EphemerisVelocityJob
            {
                SampleTimes = sampleTimes,
                MoonStates = ephemeris,
                Velocities = velocities,
                MoonCount = moonCount
            };
            handle = velJob.Schedule(flatCount, 64, handle);
            handle.Complete();

            double3 startPos = new double3(6.0e8, 0, 0);
            double3 startVel = new double3(0, 14500, 0);

            double majorStep = math.max(10.0, predictionSeconds / (50000 * 0.9));
            majorStep = math.min(majorStep, 600.0);
            double substepLimit = math.min(5.0, majorStep);

            var sw = Stopwatch.StartNew();

            var job = new FullTrajectoryJob
            {
                Nodes = emptyNodes,
                MoonEphemeris = ephemeris,
                EphemerisTimes = sampleTimes,
                MoonVelocities = velocities,
                MoonCount = moonCount,
                PlaneMapping = 0, ReferenceFrameIndex = 0,
                StartPos = startPos, StartVel = startVel, StartTime = startTime,
                JupiterPosition = double3.zero, JupiterSGP = JupiterSGP,
                MajorStepSeconds = majorStep, SubstepLimitSeconds = substepLimit,
                MaxSubstepsPerSegment = 4096, MaxPoints = 50000,
                MaxStepsPerSegment = 7000000,
                PredictionLengthSeconds = predictionSeconds,
                MaxPredictionLengthSeconds = predictionSeconds,
                OutputPoints = output, PointCount = pointCount,
                CalculationStatus = calcStatus,
                SegmentBoundaries = boundaries, SegmentBoundaryCount = boundaryCount,
                ProfileCounters = counters
            };

            job.Schedule().Complete();
            sw.Stop();

            int count = pointCount.Value;
            int status = calcStatus.Value;

            UnityEngine.Debug.Log(
                "═══════════════════════════════════════════════════════════\n" +
                $"REALISTIC TEST: 1 year, 4 Galilean moons\n" +
                $"Timer: {sw.Elapsed.TotalMilliseconds:F1}ms\n" +
                $"  points={count} major={counters[0]} substeps={counters[1]}\n" +
                $"  evalAccel={counters[2]} hermit={counters[3]} ephemSearch={counters[4]}\n" +
                "═══════════════════════════════════════════════════════════");

            Assert.AreEqual(1, status, "1-year job should complete");
            Assert.Greater(count, 100, "Should produce trajectory points");
            Assert.Less(count, 50000, "Should not exceed buffer");

            orbits.Dispose();
            sampleTimes.Dispose();
            ephemeris.Dispose();
            velocities.Dispose();
            output.Dispose();
            pointCount.Dispose();
            calcStatus.Dispose();
            boundaries.Dispose();
            boundaryCount.Dispose();
            emptyNodes.Dispose();
            counters.Dispose();
        }
    }
}

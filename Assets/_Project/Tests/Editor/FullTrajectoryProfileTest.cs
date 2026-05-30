using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Simulation;

namespace Galilego.Tests.Editor
{
    [TestFixture]
    public class FullTrajectoryProfileTest
    {
        [Test]
        public void FullTrajectoryJob_RunPrediction_LogsProfile()
        {
            double predictionSeconds = 7200.0;

            var counters = new NativeArray<long>(FullTrajectoryJob.PC_COUNT, Allocator.TempJob);
            var output = new NativeArray<TrajectoryPoint>(2000, Allocator.TempJob);
            var pointCount = new NativeReference<int>(0, Allocator.TempJob);
            var calcStatus = new NativeReference<int>(0, Allocator.TempJob);
            var boundaries = new NativeArray<SegmentBoundaryState>(1, Allocator.TempJob);
            var boundaryCount = new NativeReference<int>(0, Allocator.TempJob);
            var emptyNodes = new NativeArray<ManeuverNodeData>(0, Allocator.TempJob);
            var emptyEphemeris = new NativeArray<BodyState>(0, Allocator.TempJob);
            var emptyTimes = new NativeArray<double>(0, Allocator.TempJob);
            var emptyVelocities = new NativeArray<double3>(0, Allocator.TempJob);

            double jupiterSGP = 1.266865319e17;

            var job = new FullTrajectoryJob
            {
                Nodes = emptyNodes,
                MoonEphemeris = emptyEphemeris,
                EphemerisTimes = emptyTimes,
                MoonVelocities = emptyVelocities,
                MoonCount = 0,
                PlaneMapping = 0,
                ReferenceFrameIndex = 0,

                StartPos = new double3(4.22e8, 0, 0),
                StartVel = new double3(0, 17334, 0),
                StartTime = 0.0,

                JupiterPosition = double3.zero,
                JupiterSGP = jupiterSGP,

                MajorStepSeconds = 10.0,
                SubstepLimitSeconds = 5.0,
                MaxSubstepsPerSegment = 4096,
                MaxPoints = 2000,
                MaxStepsPerSegment = 1000000,

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

            int count = pointCount.Value;
            int status = calcStatus.Value;

            double elapsedDays = predictionSeconds / 86400.0;
            UnityEngine.Debug.Log(
                $"[PROFILE] FTJ prediction: {elapsedDays:F2} days, points={count}, status={status}\n" +
                $"[PROFILE] majorSteps={counters[0]} substeps={counters[1]} evalAccel={counters[2]} " +
                $"hermit={counters[3]} ephemSearch={counters[4]}");

            Assert.AreEqual(1, status, "Job should complete with status=1");
            Assert.Greater(count, 10, "Should produce at least 10 trajectory points");
            Assert.LessOrEqual(count, 2000, "Should not exceed MaxPoints");

            for (int i = 0; i < count; i++)
            {
                var pt = output[i];
                Assert.IsFalse(double.IsNaN(pt.Position.x), $"Point {i}: NaN position");
                Assert.IsFalse(double.IsNaN(pt.Time), $"Point {i}: NaN time");
            }

            emptyNodes.Dispose();
            emptyEphemeris.Dispose();
            emptyTimes.Dispose();
            emptyVelocities.Dispose();
            output.Dispose();
            pointCount.Dispose();
            calcStatus.Dispose();
            boundaries.Dispose();
            boundaryCount.Dispose();
            counters.Dispose();
        }
    }
}

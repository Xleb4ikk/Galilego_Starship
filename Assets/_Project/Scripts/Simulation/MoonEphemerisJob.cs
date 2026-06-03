using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    [BurstCompile]
    public struct MoonEphemerisJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double> SampleTimes;
        [ReadOnly] public NativeArray<MoonOrbitData> MoonOrbits;
        [NativeDisableParallelForRestriction] public NativeArray<BodyState> Results;
        public double3 JupiterPosition;
        public int PlaneMapping;

        public void Execute(int index)
        {
            int moonCount = MoonOrbits.Length;
            int timeIndex = index / moonCount;
            int moonIndex = index % moonCount;

            double t = SampleTimes[timeIndex];
            var orbit = MoonOrbits[moonIndex];
            double3 relPos = AccelerationEvaluator.EvaluateMoonPosition(
                ref orbit, t, PlaneMapping);

            Results[index] = new BodyState
            {
                Position = JupiterPosition + relPos,
                StandardGravitationalParameter = orbit.StandardGravitationalParameter
            };
        }
    }

    [BurstCompile]
    public struct EphemerisVelocityJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double> SampleTimes;
        [ReadOnly] public NativeArray<BodyState> MoonStates;
        [NativeDisableParallelForRestriction] public NativeArray<double3> Velocities;
        public int MoonCount;

        public void Execute(int index)
        {
            int moonCount = MoonCount;
            int timeIndex = index / moonCount;
            int moonIndex = index % moonCount;

            int prevIdx = math.max(0, timeIndex - 1) * moonCount + moonIndex;
            int nextIdx = math.min(SampleTimes.Length - 1, timeIndex + 1) * moonCount + moonIndex;

            if (timeIndex == 0 || timeIndex == SampleTimes.Length - 1)
            {
                Velocities[index] = double3.zero;
            }
            else
            {
                Velocities[index] = AccelerationEvaluator.ComputeCentralVelocity(
                    MoonStates[prevIdx].Position,
                    MoonStates[nextIdx].Position,
                    SampleTimes[timeIndex - 1],
                    SampleTimes[timeIndex + 1]);
            }
        }
    }
}

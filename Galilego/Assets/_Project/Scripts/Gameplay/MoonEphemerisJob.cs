using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Gameplay
{
    [BurstCompile]
    public struct MoonEphemerisJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double> SampleTimes;
        [ReadOnly] public NativeArray<MoonOrbitData> MoonOrbits;
        [NativeDisableParallelForRestriction] public NativeArray<BodyState> Results;
        public double3 JupiterPosition;
        public int PlaneMapping;

        public void Execute(int timeIndex)
        {
            double t = SampleTimes[timeIndex];
            int baseIdx = timeIndex * MoonOrbits.Length;

            for (int m = 0; m < MoonOrbits.Length; m++)
            {
                var orbit = MoonOrbits[m];
                double3 relPos = AccelerationEvaluator.EvaluateMoonPosition(
                    ref orbit, t, PlaneMapping);

                Results[baseIdx + m] = new BodyState
                {
                    Position = JupiterPosition + relPos,
                    StandardGravitationalParameter = orbit.StandardGravitationalParameter
                };
            }
        }
    }

    [BurstCompile]
    public struct EphemerisVelocityJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double> SampleTimes;
        [ReadOnly] public NativeArray<BodyState> MoonStates;
        [NativeDisableParallelForRestriction] public NativeArray<double3> Velocities;
        public int MoonCount;

        public void Execute(int timeIndex)
        {
            int baseIdx = timeIndex * MoonCount;
            int prevIdx = math.max(0, timeIndex - 1) * MoonCount;
            int nextIdx = math.min(SampleTimes.Length - 1, timeIndex + 1) * MoonCount;

            for (int m = 0; m < MoonCount; m++)
            {
                int prevMoon = prevIdx + m;
                int nextMoon = nextIdx + m;

                if (timeIndex == 0 || timeIndex == SampleTimes.Length - 1)
                {
                    Velocities[baseIdx + m] = double3.zero;
                }
                else
                {
                    Velocities[baseIdx + m] = AccelerationEvaluator.ComputeCentralVelocity(
                        MoonStates[prevMoon].Position,
                        MoonStates[nextMoon].Position,
                        SampleTimes[timeIndex - 1],
                        SampleTimes[timeIndex + 1]);
                }
            }
        }
    }
}

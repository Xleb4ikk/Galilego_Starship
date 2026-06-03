using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    /// <summary>
    /// Вычисляет позиции лун для prediction lines в параллельном Burst-джобе.
    /// Индекс плоскости: moonIdx * SamplesPerMoon + sampleIdx.
    /// Результаты — float3 (локальные позиции относительно frame).
    /// </summary>
    [BurstCompile]
    public struct MoonPredictionLinesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<MoonOrbitData> Orbits;
        [ReadOnly] public NativeArray<double>        SampleTimes;    // длина SamplesPerMoon
        [ReadOnly] public NativeArray<double3>       FramePositions; // длина SamplesPerMoon
        public double3 JupiterPosition;
        public int     SamplesPerMoon;
        public int     PlaneMapping;

        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> Results; // длина MoonCount * SamplesPerMoon

        public void Execute(int index)
        {
            int moonIdx   = index / SamplesPerMoon;
            int sampleIdx = index % SamplesPerMoon;

            double t     = SampleTimes[sampleIdx];
            var    orbit = Orbits[moonIdx];

            double3 relPos   = AccelerationEvaluator.EvaluateMoonPosition(ref orbit, t, PlaneMapping);
            double3 worldPos = JupiterPosition + relPos;
            double3 local    = worldPos - FramePositions[sampleIdx];

            Results[index] = (float3)local;
        }
    }
}
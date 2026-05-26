using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    [BurstCompile]
    public struct MoonOrbitLineJob : IJob
    {
        public MoonOrbitData Orbit;
        public double StartTime;
        public double HistorySeconds;
        public int EffectiveSamples;
        public double3 JupiterPosition;
        public double3 FramePosition;
        public int PlaneMapping;
        public float ScaleFactor;
        public NativeArray<float3> Results;

        public void Execute()
        {
            int lastIndex = EffectiveSamples - 1;
            for (int i = 0; i < EffectiveSamples; i++)
            {
                double t = EffectiveSamples <= 1 ? 0d : (double)i / lastIndex;
                double sampleTime = i == lastIndex
                    ? StartTime
                    : StartTime - HistorySeconds + t * HistorySeconds;
                double3 relPos = AccelerationEvaluator.EvaluateMoonPosition(ref Orbit, sampleTime, PlaneMapping);
                double3 offset = JupiterPosition + relPos - FramePosition;
                Results[i] = (float3)(offset * ScaleFactor);
            }
        }
    }

    [BurstCompile]
    public struct JupiterOrbitLineJob : IJob
    {
        public MoonOrbitData ActiveMoonOrbit;
        public double StartTime;
        public double HistorySeconds;
        public int EffectiveSamples;
        public double3 JupiterPosition;
        public int PlaneMapping;
        public float ScaleFactor;
        public NativeArray<float3> Results;

        public void Execute()
        {
            int lastIndex = EffectiveSamples - 1;
            for (int i = 0; i < EffectiveSamples; i++)
            {
                double t = EffectiveSamples <= 1 ? 0d : (double)i / lastIndex;
                double sampleTime = StartTime - HistorySeconds + t * HistorySeconds;
                double3 moonRelPos = AccelerationEvaluator.EvaluateMoonPosition(ref ActiveMoonOrbit, sampleTime, PlaneMapping);
                double3 moonWorldPos = JupiterPosition + moonRelPos;
                double3 offset = JupiterPosition - moonWorldPos;
                Results[i] = (float3)(offset * ScaleFactor);
            }
        }
    }
}

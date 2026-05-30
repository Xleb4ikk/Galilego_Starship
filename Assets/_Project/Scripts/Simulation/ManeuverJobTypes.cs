using System.Runtime.InteropServices;
using Unity.Mathematics;
using Galilego.Gameplay;

namespace Galilego.Simulation
{
    [StructLayout(LayoutKind.Sequential)]
    public struct MoonOrbitData
    {
        public double SemiMajorAxis;
        public double Eccentricity;
        public double InclinationRad;
        public double AscendingNodeRad;
        public double PeriapsisArgRad;
        public double MeanAnomalyAtEpochRad;
        public double EpochTimeSeconds;
        public double GravitationalParameter;
        public double StandardGravitationalParameter;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyState
    {
        public double3 Position;
        public double StandardGravitationalParameter;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ManeuverNodeData
    {
        public double StartTime;
        public double DvPrograde;
        public double DvNormal;
        public double DvRadial;
        public double Duration;
        public double ThrustNewtons;
        public double SpecificImpulseSeconds;
        public double InitialMassKg;
        public int IsInstant;
        public int HasEngine;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TrajectoryPoint
    {
        public double3 Position;
        public double Time;
        public int IsDashed;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct IntegrationState
    {
        public double3 Position;
        public double3 Velocity;
        public double Time;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TrajectoryCheckpoint
    {
        public double3 Position;
        public double3 Velocity;
        public double Time;
        public int NodeVersion;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ChunkConfig
    {
        public double StartTime;
        public double EndTime;
        public double OverlapTime;
        public int StartPointIndex;
        public int MaxPoints;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct EphemerisConfig
    {
        public int MoonCount;
        public double StepSeconds;
        public int SampleCount;
    }

    public static class JobTypeConversion
    {
        public static double3 ToDouble3(in Galilego.Core.Vector3d v)
        {
            return new double3(v.X, v.Y, v.Z);
        }

        public static Galilego.Core.Vector3d ToVector3d(double3 v)
        {
            return new Galilego.Core.Vector3d(v.x, v.y, v.z);
        }

        public static ManeuverNodeData ToNodeData(ManeuverNode node)
        {
            var data = new ManeuverNodeData
            {
                StartTime = node.StartTime,
                DvPrograde = node.DvPrograde,
                DvNormal = node.DvNormal,
                DvRadial = node.DvRadial,
                Duration = node.Duration,
                IsInstant = node.IsInstant ? 1 : 0,
                HasEngine = node.Engine.HasValue ? 1 : 0,
            };
            if (node.Engine.HasValue)
            {
                var e = node.Engine.Value;
                data.ThrustNewtons = e.ThrustNewtons;
                data.SpecificImpulseSeconds = e.SpecificImpulseSeconds;
                data.InitialMassKg = e.InitialMassKg;
            }
            return data;
        }
    }
}

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Gameplay
{
    [BurstCompile]
    public struct FullTrajectoryJob : IJob
    {
        [ReadOnly] public NativeArray<ManeuverNodeData> Nodes;
        [ReadOnly] public NativeArray<BodyState> MoonEphemeris;
        [ReadOnly] public NativeArray<double> EphemerisTimes;
        [ReadOnly] public NativeArray<double3> MoonVelocities;
        public int MoonCount;
        public int PlaneMapping;

        public double3 StartPos;
        public double3 StartVel;
        public double StartTime;

        public double3 JupiterPosition;
        public double JupiterSGP;

        public double MajorStepSeconds;
        public double SubstepLimitSeconds;
        public int MaxSubstepsPerSegment;
        public int MaxPoints;
        public int MaxStepsPerSegment;

        public double PredictionLengthSeconds;
        public double MaxPredictionLengthSeconds;

        public NativeArray<TrajectoryPoint> OutputPoints;
        public NativeReference<int> PointCount;
        public NativeReference<int> CalculationStatus;

        public void Execute()
        {
            int totalPoints = 0;
            int segmentIterLimit = 0;

            double requestedPrediction = PredictionLengthSeconds > 0.0
                ? PredictionLengthSeconds
                : 7200.0;
            double effectivePrediction = math.min(requestedPrediction, MaxPredictionLengthSeconds);
            double endTime = StartTime + effectivePrediction;

            double adaptiveStep = effectivePrediction / math.max(1, (int)(MaxPoints * 0.9));
            double majorStep = math.max(1e-6, math.max(MajorStepSeconds, adaptiveStep));
            majorStep = math.min(majorStep, 600.0);
            double substepLimit = math.min(SubstepLimitSeconds, majorStep);
            if (substepLimit <= 0.0) substepLimit = majorStep;

            int dynamicIterLimit = math.max(5000,
                (int)(effectivePrediction / math.max(1e-6, majorStep)) + 100);
            dynamicIterLimit = math.min(dynamicIterLimit, 2000000);

            double3 currentPos = StartPos;
            double3 currentVel = StartVel;
            double currentTime = StartTime;

            int nodeCount = Nodes.Length;

            for (int seg = 0; seg <= nodeCount; seg++)
            {
                double targetTime;
                ManeuverNodeData currentNode = default;
                bool hasCurrentNode = false;

                if (seg < nodeCount)
                {
                    currentNode = Nodes[seg];
                    targetTime = currentNode.StartTime;
                    hasCurrentNode = true;
                }
                else
                {
                    targetTime = endTime;
                }

                if (targetTime <= currentTime)
                {
                    if (hasCurrentNode)
                    {
                        double3 dv = CalculateWorldDeltaV(currentPos, currentVel, currentNode);
                        currentVel += dv;
                        totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints, currentPos, currentTime, 1);

                        if (currentNode.HasEngine != 0 && currentNode.IsInstant == 0)
                        {
                            currentTime = currentNode.StartTime + currentNode.Duration;
                        }
                    }
                    continue;
                }

                if (currentTime >= endTime) break;

                int isDashed = 0;
                if (seg > 0 && seg - 1 < nodeCount)
                {
                    var prevNode = Nodes[seg - 1];
                    isDashed = (math.abs(prevNode.DvPrograde) + math.abs(prevNode.DvNormal) +
                                math.abs(prevNode.DvRadial)) > 0.001 ? 1 : 0;
                }

                totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints,
                    currentPos, currentTime, isDashed);

                if (totalPoints >= MaxPoints) break;

                bool trajectoryLimit = false;
                int iterCount = 0;
                int safetyCounter = 0;

                while (currentTime < targetTime && !trajectoryLimit)
                {
                    iterCount++;
                    if (iterCount > dynamicIterLimit)
                    {
                        trajectoryLimit = true;
                        break;
                    }

                    double stepTime = math.min(majorStep, targetTime - currentTime);
                    if (stepTime <= 0.0) break;

                    int internalSteps = CalculateAdaptiveSubsteps(stepTime, substepLimit);
                    internalSteps = math.min(internalSteps, MaxSubstepsPerSegment);
                    double internalDt = stepTime / internalSteps;

                    bool aborted = false;
                    for (int k = 0; k < internalSteps; k++)
                    {
                        safetyCounter++;
                        if (safetyCounter > MaxStepsPerSegment)
                        {
                            trajectoryLimit = true;
                            aborted = true;
                            break;
                        }

                        var res = RK4Step(currentPos, currentVel, currentTime, internalDt);
                        currentPos = res.Position;
                        currentVel = res.Velocity;
                        currentTime += internalDt;

                        if (!IsFinite(currentPos) || !IsFinite(currentVel))
                        {
                            trajectoryLimit = true;
                            aborted = true;
                            break;
                        }
                    }

                    if (aborted) break;

                    totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints,
                        currentPos, currentTime, isDashed);

                    if (totalPoints >= MaxPoints)
                    {
                        trajectoryLimit = true;
                        break;
                    }
                }

                if (hasCurrentNode && !trajectoryLimit)
                {
                    double3 dv = CalculateWorldDeltaV(currentPos, currentVel, currentNode);
                    currentVel += dv;

                    totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints,
                        currentPos, currentTime, 1);

                    if (currentNode.HasEngine != 0 && currentNode.IsInstant == 0)
                    {
                        currentTime = currentNode.StartTime + currentNode.Duration;
                    }
                }

                if (totalPoints >= MaxPoints) break;
            }

            PointCount.Value = totalPoints;
            CalculationStatus.Value = 1;
        }

        private int AddPoint(NativeArray<TrajectoryPoint> points, int index, int max, double3 pos, double time, int isDashed)
        {
            if (index < max)
            {
                points[index] = new TrajectoryPoint
                {
                    Position = pos,
                    Time = time,
                    IsDashed = isDashed
                };
                return index + 1;
            }
            return index;
        }

        private IntegrationResult RK4Step(double3 pos, double3 vel, double time, double dt)
        {
            double halfDt = dt * 0.5;
            double sixthDt = dt / 6.0;

            double3 k1Pos = vel;
            double3 k1Vel = EvaluateAcceleration(pos, time);

            double3 k2Pos = vel + k1Vel * halfDt;
            double3 k2Vel = EvaluateAcceleration(pos + k1Pos * halfDt, time + halfDt);

            double3 k3Pos = vel + k2Vel * halfDt;
            double3 k3Vel = EvaluateAcceleration(pos + k2Pos * halfDt, time + halfDt);

            double3 k4Pos = vel + k3Vel * dt;
            double3 k4Vel = EvaluateAcceleration(pos + k3Pos * dt, time + dt);

            double3 newPos = pos + ((k1Pos + 2.0 * k2Pos + 2.0 * k3Pos + k4Pos) * sixthDt);
            double3 newVel = vel + ((k1Vel + 2.0 * k2Vel + 2.0 * k3Vel + k4Vel) * sixthDt);

            return new IntegrationResult { Position = newPos, Velocity = newVel };
        }

        private double3 EvaluateAcceleration(double3 pos, double time)
        {
            double3 total = double3.zero;

            if (JupiterSGP > 0.0)
            {
                total += BodyGravity(pos, JupiterPosition, JupiterSGP);
            }

            if (MoonEphemeris.Length > 0 && EphemerisTimes.Length > 0)
            {
                int idx = FindEphemerisIndex(time);
                if (idx >= 0 && idx < EphemerisTimes.Length - 1)
                {
                    double t0 = EphemerisTimes[idx];
                    double t1 = EphemerisTimes[idx + 1];
                    int b0 = idx * MoonCount;
                    int b1 = (idx + 1) * MoonCount;

                    for (int m = 0; m < MoonCount; m++)
                    {
                        double3 moonPos = AccelerationEvaluator.HermiteInterpolate(
                            MoonEphemeris[b0 + m].Position,
                            MoonVelocities[b0 + m],
                            MoonEphemeris[b1 + m].Position,
                            MoonVelocities[b1 + m],
                            t0, t1, time);

                        total += BodyGravity(pos, moonPos,
                            MoonEphemeris[b0 + m].StandardGravitationalParameter);
                    }
                }
                else if (idx >= 0 && idx < EphemerisTimes.Length)
                {
                    int b = idx * MoonCount;
                    for (int m = 0; m < MoonCount; m++)
                    {
                        total += BodyGravity(pos, MoonEphemeris[b + m].Position,
                            MoonEphemeris[b + m].StandardGravitationalParameter);
                    }
                }
            }

            return total;
        }

        private int FindEphemerisIndex(double time)
        {
            if (EphemerisTimes.Length == 0) return -1;
            if (time <= EphemerisTimes[0]) return 0;
            if (time >= EphemerisTimes[EphemerisTimes.Length - 1])
                return EphemerisTimes.Length - 1;

            int lo = 0;
            int hi = EphemerisTimes.Length - 1;
            while (lo < hi - 1)
            {
                int mid = (lo + hi) / 2;
                if (EphemerisTimes[mid] <= time)
                    lo = mid;
                else
                    hi = mid;
            }
            return lo;
        }

        private static double3 BodyGravity(double3 shipPos, double3 bodyPos, double sgp)
        {
            if (sgp == 0.0) return double3.zero;
            double3 offset = bodyPos - shipPos;
            double sqrDist = math.lengthsq(offset);
            if (sqrDist <= 0.0 || sqrDist < 100.0) return double3.zero;

            double invDist = 1.0 / math.sqrt(sqrDist);
            double invDistCubed = invDist / sqrDist;
            return offset * (sgp * invDistCubed);
        }

        private static double3 CalculateWorldDeltaV(double3 pos, double3 vel, ManeuverNodeData node)
        {
            if (math.lengthsq(vel) < 0.001) return double3.zero;

            OrbitalBasisJob.ComputeBasis(pos, vel, out double3 radial, out double3 normal, out double3 prograde);

            return prograde * node.DvPrograde + normal * node.DvNormal + radial * node.DvRadial;
        }

        private static int CalculateAdaptiveSubsteps(double majorStep, double baseSubstep)
        {
            double clamped = math.max(1e-9, math.min(baseSubstep, majorStep));
            return math.max(1, (int)math.ceil(majorStep / clamped));
        }

        private static bool IsFinite(double3 v)
        {
            return !double.IsNaN(v.x) && !double.IsNaN(v.y) && !double.IsNaN(v.z) &&
                   !double.IsInfinity(v.x) && !double.IsInfinity(v.y) && !double.IsInfinity(v.z);
        }

        private struct IntegrationResult
        {
            public double3 Position;
            public double3 Velocity;
        }
    }
}

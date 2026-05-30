using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    [BurstCompile]
    public struct FullTrajectoryJob : IJob
    {
        // ─── Profile counter indices ────────────────────────────────────────
        private const int PC_MAJOR_STEPS = 0;
        private const int PC_SUBSTEPS = 1;
        private const int PC_EVAL_ACCEL = 2;
        private const int PC_HERMITE = 3;
        private const int PC_EPHEM_SEARCH = 4;
        public const int PC_COUNT = 5;

        [ReadOnly] public NativeArray<ManeuverNodeData> Nodes;
        [ReadOnly] public NativeArray<BodyState> MoonEphemeris;
        [ReadOnly] public NativeArray<double> EphemerisTimes;
        [ReadOnly] public NativeArray<double3> MoonVelocities;
        public int MoonCount;
        public int PlaneMapping;
        public int ReferenceFrameIndex;

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

        [WriteOnly] public NativeArray<SegmentBoundaryState> SegmentBoundaries;
        public NativeReference<int> SegmentBoundaryCount;

        public NativeArray<long> ProfileCounters;

        public double CheckpointIntervalSeconds;
        [WriteOnly] public NativeArray<TrajectoryCheckpoint> Checkpoints;
        public NativeReference<int> CheckpointCount;

        public void Execute()
        {
            int totalPoints = 0;
            int ephemIdx = 0;
            int cpCount = 0;
            double nextCheckpointTime = StartTime + CheckpointIntervalSeconds;
            int bCount = 0;

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

            if (SegmentBoundaries.IsCreated && SegmentBoundaries.Length > 0)
            {
                SegmentBoundaries[0] = new SegmentBoundaryState
                {
                    Position = currentPos,
                    Velocity = currentVel,
                    Time = currentTime
                };
                bCount = 1;
            }

            if (Checkpoints.IsCreated && Checkpoints.Length > 0)
            {
                Checkpoints[cpCount++] = new TrajectoryCheckpoint
                {
                    Position = currentPos, Velocity = currentVel,
                    Time = currentTime, NodeVersion = 0
                };
            }

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
                        ResolveReferenceFrameState(currentTime, out double3 framePos, out double3 frameVel);
                        double3 localPos = currentPos - framePos;
                        double3 localVel = currentVel - frameVel;
                        double3 dv = CalculateWorldDeltaV(localPos, localVel, currentNode);
                        currentVel += dv;
                        totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints, currentPos, currentTime, 1);

                        if (currentNode.HasEngine != 0 && currentNode.IsInstant == 0)
                        {
                            currentTime = currentNode.StartTime + currentNode.Duration;
                        }

                        if (SegmentBoundaries.IsCreated && bCount < SegmentBoundaries.Length)
                        {
                            SegmentBoundaries[bCount++] = new SegmentBoundaryState
                            {
                                Position = currentPos,
                                Velocity = currentVel,
                                Time = currentTime
                            };
                        }

                        if (Checkpoints.IsCreated && cpCount < Checkpoints.Length)
                        {
                            Checkpoints[cpCount++] = new TrajectoryCheckpoint
                            {
                                Position = currentPos, Velocity = currentVel,
                                Time = currentTime, NodeVersion = seg + 1
                            };
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
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_MAJOR_STEPS]++;

                    if (iterCount > dynamicIterLimit)
                    {
                        trajectoryLimit = true;
                        break;
                    }

                    // Prepare body positions for step sizing (reused by first substep)
                    var stepData = PrepareSubstepData(currentTime, currentPos, ref ephemIdx);

                    // Adaptive step from SubstepData (min distance to any body)
                    const double referenceDistance = 1e8;
                    double adaptiveFactor = math.clamp(
                        stepData.MinDistToAnyBody / referenceDistance, 0.1, 10.0);
                    double adaptiveMajorStep = majorStep * adaptiveFactor;
                    double stepTime = math.min(adaptiveMajorStep, targetTime - currentTime);
                    if (stepTime <= 0.0) break;

                    int internalSteps = CalculateAdaptiveSubsteps(stepTime, substepLimit);
                    internalSteps = math.min(internalSteps, MaxSubstepsPerSegment);
                    double internalDt = stepTime / internalSteps;

                    bool aborted = false;
                    for (int k = 0; k < internalSteps; k++)
                    {
                        if (ProfileCounters.IsCreated) ProfileCounters[PC_SUBSTEPS]++;
                        safetyCounter++;
                        if (safetyCounter > MaxStepsPerSegment)
                        {
                            trajectoryLimit = true;
                            aborted = true;
                            break;
                        }

                        // Fresh cache per substep (moon positions advance)
                        if (k > 0)
                            stepData = PrepareSubstepData(currentTime, currentPos, ref ephemIdx);

                        var res = RK4Step(currentPos, currentVel, internalDt, ref stepData);
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

                    while (currentTime >= nextCheckpointTime && cpCount < Checkpoints.Length)
                    {
                        Checkpoints[cpCount++] = new TrajectoryCheckpoint
                        {
                            Position = currentPos, Velocity = currentVel,
                            Time = currentTime, NodeVersion = seg
                        };
                        nextCheckpointTime += CheckpointIntervalSeconds;
                    }

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
                    ResolveReferenceFrameState(currentTime, out double3 framePos, out double3 frameVel);
                    double3 localPos = currentPos - framePos;
                    double3 localVel = currentVel - frameVel;
                    double3 dv = CalculateWorldDeltaV(localPos, localVel, currentNode);
                    currentVel += dv;

                    totalPoints = AddPoint(OutputPoints, totalPoints, MaxPoints,
                        currentPos, currentTime, 1);

                    if (currentNode.HasEngine != 0 && currentNode.IsInstant == 0)
                    {
                        currentTime = currentNode.StartTime + currentNode.Duration;
                    }

                    if (SegmentBoundaries.IsCreated && bCount < SegmentBoundaries.Length)
                    {
                        SegmentBoundaries[bCount++] = new SegmentBoundaryState
                        {
                            Position = currentPos,
                            Velocity = currentVel,
                            Time = currentTime
                        };
                    }

                    if (Checkpoints.IsCreated && cpCount < Checkpoints.Length)
                    {
                        Checkpoints[cpCount++] = new TrajectoryCheckpoint
                        {
                            Position = currentPos, Velocity = currentVel,
                            Time = currentTime, NodeVersion = seg + 1
                        };
                    }
                }

                if (totalPoints >= MaxPoints) break;
            }

            if (CheckpointCount.IsCreated) CheckpointCount.Value = cpCount;
            PointCount.Value = totalPoints;
            CalculationStatus.Value = 1;
            if (SegmentBoundaryCount.IsCreated)
                SegmentBoundaryCount.Value = bCount;
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

        private SubstepData PrepareSubstepData(double time, double3 shipPos, ref int ephemIdx)
        {
            SubstepData data;
            data.MoonCount = MoonCount;
            data.Moon0 = default; data.Moon1 = default; data.Moon2 = default; data.Moon3 = default;

            double minDistSq = math.lengthsq(shipPos - JupiterPosition);

            if (MoonCount > 0 && MoonEphemeris.Length > 0 && EphemerisTimes.Length > 0)
            {
                int timesLen = EphemerisTimes.Length;
                if (ephemIdx >= timesLen) ephemIdx = timesLen - 1;
                if (ephemIdx < 0) ephemIdx = 0;

                while (ephemIdx < timesLen - 2 && EphemerisTimes[ephemIdx + 1] < time)
                {
                    ephemIdx++;
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_EPHEM_SEARCH]++;
                }
                while (ephemIdx > 0 && EphemerisTimes[ephemIdx] > time)
                {
                    ephemIdx--;
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_EPHEM_SEARCH]++;
                }

                int idx = ephemIdx;
                if (idx >= 0 && idx < timesLen - 1)
                {
                    double t0 = EphemerisTimes[idx];
                    double t1 = EphemerisTimes[idx + 1];
                    int b0 = idx * MoonCount;
                    int b1 = (idx + 1) * MoonCount;

                    for (int m = 0; m < MoonCount; m++)
                    {
                        if (ProfileCounters.IsCreated) ProfileCounters[PC_HERMITE]++;
                        double3 moonPos = AccelerationEvaluator.HermiteInterpolate(
                            MoonEphemeris[b0 + m].Position,
                            MoonVelocities[b0 + m],
                            MoonEphemeris[b1 + m].Position,
                            MoonVelocities[b1 + m],
                            t0, t1, time);

                        double sgp = MoonEphemeris[b0 + m].StandardGravitationalParameter;
                        double moonDistSq = math.lengthsq(moonPos - shipPos);
                        if (moonDistSq < minDistSq) minDistSq = moonDistSq;

                        BodyState ms = new BodyState { Position = moonPos, StandardGravitationalParameter = sgp };
                        if (m == 0) data.Moon0 = ms;
                        else if (m == 1) data.Moon1 = ms;
                        else if (m == 2) data.Moon2 = ms;
                        else if (m == 3) data.Moon3 = ms;
                    }
                }
                else if (idx >= 0 && idx < timesLen)
                {
                    int b = idx * MoonCount;
                    for (int m = 0; m < MoonCount; m++)
                    {
                        double3 moonPos = MoonEphemeris[b + m].Position;
                        double sgp = MoonEphemeris[b + m].StandardGravitationalParameter;
                        double moonDistSq = math.lengthsq(moonPos - shipPos);
                        if (moonDistSq < minDistSq) minDistSq = moonDistSq;

                        BodyState ms = new BodyState { Position = moonPos, StandardGravitationalParameter = sgp };
                        if (m == 0) data.Moon0 = ms;
                        else if (m == 1) data.Moon1 = ms;
                        else if (m == 2) data.Moon2 = ms;
                        else if (m == 3) data.Moon3 = ms;
                    }
                }
            }

            data.MinDistToAnyBody = math.sqrt(minDistSq);
            return data;
        }

        private IntegrationResult RK4Step(double3 pos, double3 vel, double dt, ref SubstepData data)
        {
            if (ProfileCounters.IsCreated) ProfileCounters[PC_EVAL_ACCEL] += 4;

            double halfDt = dt * 0.5;
            double sixthDt = dt / 6.0;

            double3 k1Pos = vel;
            double3 k1Vel = EvaluateAccelerationCached(pos, ref data);

            double3 k2Pos = vel + k1Vel * halfDt;
            double3 k2Vel = EvaluateAccelerationCached(pos + k1Pos * halfDt, ref data);

            double3 k3Pos = vel + k2Vel * halfDt;
            double3 k3Vel = EvaluateAccelerationCached(pos + k2Pos * halfDt, ref data);

            double3 k4Pos = vel + k3Vel * dt;
            double3 k4Vel = EvaluateAccelerationCached(pos + k3Pos * dt, ref data);

            double3 newPos = pos + ((k1Pos + 2.0 * k2Pos + 2.0 * k3Pos + k4Pos) * sixthDt);
            double3 newVel = vel + ((k1Vel + 2.0 * k2Vel + 2.0 * k3Vel + k4Vel) * sixthDt);

            return new IntegrationResult { Position = newPos, Velocity = newVel };
        }

        private double3 EvaluateAccelerationCached(double3 pos, ref SubstepData data)
        {
            double3 total = double3.zero;

            if (JupiterSGP > 0.0)
            {
                total += BodyGravity(pos, JupiterPosition, JupiterSGP);
            }

            if (data.MoonCount > 0)
            {
                total += BodyGravity(pos, data.Moon0.Position, data.Moon0.StandardGravitationalParameter);
                if (data.MoonCount > 1)
                    total += BodyGravity(pos, data.Moon1.Position, data.Moon1.StandardGravitationalParameter);
                if (data.MoonCount > 2)
                    total += BodyGravity(pos, data.Moon2.Position, data.Moon2.StandardGravitationalParameter);
                if (data.MoonCount > 3)
                    total += BodyGravity(pos, data.Moon3.Position, data.Moon3.StandardGravitationalParameter);
            }

            return total;
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

        private void ResolveReferenceFrameState(double time, out double3 framePos, out double3 frameVel)
        {
            if (ReferenceFrameIndex <= 0 || MoonCount == 0 || MoonEphemeris.Length == 0 || EphemerisTimes.Length == 0)
            {
                framePos = JupiterPosition;
                frameVel = double3.zero;
                return;
            }

            int moonIndex = math.clamp(ReferenceFrameIndex - 1, 0, MoonCount - 1);

            int timesLen = EphemerisTimes.Length;
            int idx = 0;
            while (idx < timesLen - 2 && EphemerisTimes[idx + 1] < time) idx++;
            while (idx > 0 && EphemerisTimes[idx] > time) idx--;

            int b0 = idx * MoonCount + moonIndex;
            int b1 = math.min(idx + 1, timesLen - 1) * MoonCount + moonIndex;

            framePos = AccelerationEvaluator.HermiteInterpolate(
                MoonEphemeris[b0].Position,
                MoonVelocities[b0],
                MoonEphemeris[b1].Position,
                MoonVelocities[b1],
                EphemerisTimes[idx],
                EphemerisTimes[math.min(idx + 1, timesLen - 1)],
                time);

            frameVel = MoonVelocities[b0];
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

        private struct SubstepData
        {
            public BodyState Moon0, Moon1, Moon2, Moon3;
            public int MoonCount;
            public double MinDistToAnyBody;
        }
    }

    public struct SegmentBoundaryState
    {
        public double3 Position;
        public double3 Velocity;
        public double Time;
    }
}

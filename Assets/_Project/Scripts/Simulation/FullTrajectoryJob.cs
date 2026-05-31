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

        public int HotNodeIndex;
        public double HotCheckpointInterval;

        // ─── State-based restart ─────────────────────────────────────
        public int StartEphemerisIndex;   // -1 = с начала, >=0 = стартовый индекс эфемерид
        public int EphemerisVersion;      // версия эфемерид для этого расчёта

        // ─── Adaptive integrator settings ─────────────────────────────
        public double RelTol;             // относительный tolerance (позиция+скорость)
        public double AbsTol;             // абсолютный tolerance (метры)
        public double MinStepSeconds;     // минимальный шаг
        public double MaxStepSeconds;     // максимальный шаг в пустоте

        // ─── Approximate body radii for event caps ────────────────────
        public double JupiterRadius;
        public double MoonRadius;

        public void Execute()
        {
            int totalPoints = 0;
            int ephemIdx = StartEphemerisIndex >= 0 ? StartEphemerisIndex : 0;
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
                    Time = currentTime, NodeVersion = 0,
                    EphemerisVersion = EphemerisVersion,
                    EphemerisIndex = ephemIdx
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

                // Hot zone: denser checkpoint interval around active edit
                if (HotNodeIndex >= 0 && seg == HotNodeIndex)
                    nextCheckpointTime = currentTime + HotCheckpointInterval;

                bool trajectoryLimit = false;
                int iterCount = 0;
                int safetyCounter = 0;
                double dt = math.min(MaxStepSeconds, targetTime - currentTime);

                while (currentTime < targetTime && !trajectoryLimit)
                {
                    iterCount++;
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_MAJOR_STEPS]++;

                    if (iterCount > dynamicIterLimit)
                    {
                        trajectoryLimit = true;
                        break;
                    }

                    if (safetyCounter++ > MaxStepsPerSegment)
                    {
                        trajectoryLimit = true;
                        break;
                    }

                    // Single adaptive step with error control
                    bool stepAccepted = TryAdvanceStep(
                        ref currentPos, ref currentVel, ref currentTime,
                        targetTime, ref ephemIdx,
                        ref dt);

                    if (!IsFinite(currentPos) || !IsFinite(currentVel))
                    {
                        trajectoryLimit = true;
                        break;
                    }

                    // Only record points after accepted steps
                    if (!stepAccepted) continue;

                    // Checkpoint recording
                    while (currentTime >= nextCheckpointTime && cpCount < Checkpoints.Length)
                    {
                        Checkpoints[cpCount++] = new TrajectoryCheckpoint
                        {
                            Position = currentPos, Velocity = currentVel,
                            Time = currentTime, NodeVersion = seg,
                            EphemerisVersion = EphemerisVersion,
                            EphemerisIndex = ephemIdx
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
                            Time = currentTime, NodeVersion = seg + 1,
                            EphemerisVersion = EphemerisVersion,
                            EphemerisIndex = ephemIdx
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

        // ─── Dormand–Prince 5(4) step ─────────────────────────────────
        /// <summary>
        /// Выполняет один шаг Dormand–Prince 5(4) с контролем ошибки.
        /// Возвращает результат 5-го порядка и оценки ошибок по позиции и скорости.
        /// </summary>
        private IntegrationResult DoPri5Step(
            double3 pos, double3 vel, double dt,
            ref SubstepData data,
            out double errPos, out double errVel)
        {
            if (ProfileCounters.IsCreated) ProfileCounters[PC_EVAL_ACCEL] += 7;

            // Dormand–Prince 5(4) coefficients (Butcher tableau)
            // 7 stages, FSAL = false (we don't reuse last evaluation)
            const double a21 = 1.0 / 5.0;
            const double a31 = 3.0 / 40.0;
            const double a32 = 9.0 / 40.0;
            const double a41 = 44.0 / 45.0;
            const double a42 = -56.0 / 15.0;
            const double a43 = 32.0 / 9.0;
            const double a51 = 19372.0 / 6561.0;
            const double a52 = -25360.0 / 2187.0;
            const double a53 = 64448.0 / 6561.0;
            const double a54 = -212.0 / 729.0;
            const double a61 = 9017.0 / 3168.0;
            const double a62 = -355.0 / 33.0;
            const double a63 = 46732.0 / 5247.0;
            const double a64 = 49.0 / 176.0;
            const double a65 = -5103.0 / 18656.0;
            const double a71 = 35.0 / 384.0;
            const double a73 = 500.0 / 1113.0;
            const double a74 = 125.0 / 192.0;
            const double a75 = -2187.0 / 6784.0;
            const double a76 = 11.0 / 84.0;

            // Stage evaluations
            double3 k1v = EvaluateAccelerationCached(pos, ref data);
            double3 k1p = vel;

            double3 pos2 = pos + k1p * (dt * a21);
            double3 vel2 = vel + k1v * (dt * a21);
            double3 a2 = EvaluateAccelerationCached(pos2, ref data);

            double3 pos3 = pos + (k1p * (dt * a31) + vel2 * (dt * a32));
            double3 vel3 = vel + (k1v * (dt * a31) + a2 * (dt * a32));
            double3 a3 = EvaluateAccelerationCached(pos3, ref data);

            double3 pos4 = pos + (k1p * (dt * a41) + vel2 * (dt * a42) + vel3 * (dt * a43));
            double3 vel4 = vel + (k1v * (dt * a41) + a2 * (dt * a42) + a3 * (dt * a43));
            double3 a4 = EvaluateAccelerationCached(pos4, ref data);

            double3 pos5 = pos + (k1p * (dt * a51) + vel2 * (dt * a52) + vel3 * (dt * a53) + vel4 * (dt * a54));
            double3 vel5 = vel + (k1v * (dt * a51) + a2 * (dt * a52) + a3 * (dt * a53) + a4 * (dt * a54));
            double3 a5 = EvaluateAccelerationCached(pos5, ref data);

            double3 pos6 = pos + (k1p * (dt * a61) + vel2 * (dt * a62) + vel3 * (dt * a63) + vel4 * (dt * a64) + vel5 * (dt * a65));
            double3 vel6 = vel + (k1v * (dt * a61) + a2 * (dt * a62) + a3 * (dt * a63) + a4 * (dt * a64) + a5 * (dt * a65));
            double3 a6 = EvaluateAccelerationCached(pos6, ref data);

            double3 pos7 = pos + (k1p * (dt * a71) + vel3 * (dt * a73) + vel4 * (dt * a74) + vel5 * (dt * a75) + vel6 * (dt * a76));
            double3 vel7 = vel + (k1v * (dt * a71) + a3 * (dt * a73) + a4 * (dt * a74) + a5 * (dt * a75) + a6 * (dt * a76));
            double3 a7 = EvaluateAccelerationCached(pos7, ref data);

            // 5th order weights (using b_i coefficients)
            const double b1 = 35.0 / 384.0;
            const double b3 = 500.0 / 1113.0;
            const double b4 = 125.0 / 192.0;
            const double b5 = -2187.0 / 6784.0;
            const double b6 = 11.0 / 84.0;

            double3 fifthPos = pos + (k1p * (dt * b1) + vel3 * (dt * b3) + vel4 * (dt * b4) + vel5 * (dt * b5) + vel6 * (dt * b6));
            double3 fifthVel = vel + (k1v * (dt * b1) + a3 * (dt * b3) + a4 * (dt * b4) + a5 * (dt * b5) + a6 * (dt * b6));

            // 4th order weights (for error estimation)
            const double bs1 = 5179.0 / 57600.0;
            const double bs3 = 7571.0 / 16695.0;
            const double bs4 = 393.0 / 640.0;
            const double bs5 = -92097.0 / 339200.0;
            const double bs6 = 187.0 / 2100.0;
            const double bs7 = 1.0 / 40.0;

            double3 fourthPos = pos + (k1p * (dt * bs1) + vel3 * (dt * bs3) + vel4 * (dt * bs4) + vel5 * (dt * bs5) + vel6 * (dt * bs6) + vel7 * (dt * bs7));
            double3 fourthVel = vel + (k1v * (dt * bs1) + a3 * (dt * bs3) + a4 * (dt * bs4) + a5 * (dt * bs5) + a6 * (dt * bs6) + a7 * (dt * bs7));

            // Error = difference between 5th and 4th order
            errPos = math.length(fifthPos - fourthPos);
            errVel = math.length(fifthVel - fourthVel);

            return new IntegrationResult { Position = fifthPos, Velocity = fifthVel };
        }

        // ─── Event-aware stepping caps ──────────────────────────────
        /// <summary>
        /// Вычисляет ограничения шага по геометрии, манёврам и границам сегментов.
        /// </summary>
        private double ComputeEventCaps(
            double3 pos, double3 vel, double currentTime,
            double targetTime, ref SubstepData data)
        {
            double dt = targetTime - currentTime;

            // 1. Geometry: no more than 1/10 of time to closest approach
            double minDist = data.MinDistToAnyBody;
            double speed = math.length(vel);
            double timeToClosest = minDist / math.max(speed, 0.1);
            double dtGeom = timeToClosest * 0.1;

            // 2. Sphere of influence: tighter step near bodies
            if (JupiterRadius > 0.0 && minDist < JupiterRadius * 10.0)
                dtGeom = math.min(dtGeom, 60.0);
            if (MoonRadius > 0.0 && minDist < MoonRadius * 10.0)
                dtGeom = math.min(dtGeom, 10.0);

            // 3. Maneuver: within 1 hour before/after — step no larger than time to maneuver * 0.01
            double dtManeuver = dt;
            double timeToManeuver = targetTime - currentTime;
            if (timeToManeuver > 0.0 && timeToManeuver < 3600.0)
                dtManeuver = timeToManeuver * 0.01;

            // 4. Segment boundary: don't overshoot targetTime
            double dtRemaining = dt;

            // Combine all caps
            double capped = math.min(dtGeom, math.min(dtManeuver, dtRemaining));
            return math.max(capped, MinStepSeconds);
        }

        // ─── Adaptive integrator (single step) ──────────────────────
        /// <summary>
        /// Делает один адаптивный шаг Dormand–Prince 5(4) с контролем ошибки
        /// и event-aware caps. Возвращает false если шаг не принят (reject).
        /// Не содержит цикл по времени — внешний цикл управляет итерациями.
        /// </summary>
        private bool TryAdvanceStep(
            ref double3 pos, ref double3 vel, ref double time,
            double targetTime, ref int ephemIdx,
            ref double dt)
        {
            // Get fresh ephemeris data for current position/time
            var stepData = PrepareSubstepData(time, pos, ref ephemIdx);

            // Apply event-aware caps
            double cappedDt = ComputeEventCaps(pos, vel, time, targetTime, ref stepData);
            dt = math.min(dt, cappedDt);
            if (time + dt > targetTime) dt = targetTime - time;

            // DoPri5 step with error estimation
            var result = DoPri5Step(pos, vel, dt, ref stepData, out double errPos, out double errVel);

            // Scaled error
            double scalePos = AbsTol + RelTol * math.max(math.length(pos), math.length(result.Position));
            double scaleVel = AbsTol + RelTol * math.max(math.length(vel), math.length(result.Velocity));
            double normalizedError = math.max(errPos / scalePos, errVel / scaleVel);

            if (normalizedError <= 1.0)
            {
                // ACCEPT step
                pos = result.Position;
                vel = result.Velocity;
                time += dt;

                // Increase next step with safety factor
                double stepScale = math.clamp(
                    0.9 * math.pow(1.0 / math.max(normalizedError, 1e-10), 0.2),
                    0.2, 5.0);
                dt = math.clamp(dt * stepScale, MinStepSeconds, MaxStepSeconds);
                return true;
            }
            else
            {
                // REJECT — retry with smaller step (Pi control formula)
                double rejectScale = math.clamp(
                    0.9 * math.pow(1.0 / math.max(normalizedError, 1e-10), 0.2),
                    0.1, 0.5);
                dt = math.max(dt * rejectScale, MinStepSeconds);
                if (dt <= MinStepSeconds)
                {
                    // Force-accept at minimum step
                    pos = result.Position;
                    vel = result.Velocity;
                    time += dt;
                    return true;
                }
                return false;
            }
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
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Galilego.Core;

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
            double3 fsalAccel = double3.zero;
            bool fsalValid = false;
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
                        // После манёвра сбросить FSAL, т.к. скорость изменилась
                        fsalValid = false;
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

                    // Single adaptive step with error control (FSAL: k7 → k1 propagation)
                    bool stepAccepted = TryAdvanceStep(
                        ref currentPos, ref currentVel, ref currentTime,
                        targetTime, ref ephemIdx,
                        ref dt, ref fsalAccel, ref fsalValid);

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
                    // После манёвра сбросить FSAL
                    fsalValid = false;

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
        /// Каждая стадия вычисляет гравитацию в правильное время (t + c_i * dt),
        /// чтобы учитывать движение лун внутри шага.
        /// FSAL (First Same As Last): k7 текущего шага = k1 следующего.
        /// </summary>
        private IntegrationResult DoPri5Step(
            double3 pos, double3 vel, double time, double dt, int hintEphemIdx,
            out double errPos, out double errVel,
            double3 fsalAccel, out double3 lastAccel)
        {
            // Dormand–Prince 5(4) coefficients - using shared constants
            // See DOPRI5Coefficients.cs for single source of truth
            const double a21 = DOPRI5Coefficients.a21;
            const double a31 = DOPRI5Coefficients.a31;
            const double a32 = DOPRI5Coefficients.a32;
            const double a41 = DOPRI5Coefficients.a41;
            const double a42 = DOPRI5Coefficients.a42;
            const double a43 = DOPRI5Coefficients.a43;
            const double a51 = DOPRI5Coefficients.a51;
            const double a52 = DOPRI5Coefficients.a52;
            const double a53 = DOPRI5Coefficients.a53;
            const double a54 = DOPRI5Coefficients.a54;
            const double a61 = DOPRI5Coefficients.a61;
            const double a62 = DOPRI5Coefficients.a62;
            const double a63 = DOPRI5Coefficients.a63;
            const double a64 = DOPRI5Coefficients.a64;
            const double a65 = DOPRI5Coefficients.a65;
            const double a71 = DOPRI5Coefficients.a71;
            const double a73 = DOPRI5Coefficients.a73;
            const double a74 = DOPRI5Coefficients.a74;
            const double a75 = DOPRI5Coefficients.a75;
            const double a76 = DOPRI5Coefficients.a76;

            // Stage evaluations with time-dependent gravity
            // c-values: 0, 1/5, 3/10, 4/5, 8/9, 1, 1
            // FSAL: k1v берётся из переданного fsalAccel (k7 предыдущего шага)
            // Локальная копия hint — внешний ephemIdx не мутируется стадиями DoPri5
            int localHint = hintEphemIdx;
            double3 k1v = fsalAccel;
            double3 k1p = vel;
            
            // DEBUG: Log first step
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[FTJ_DEBUG] DoPri5Step called: pos=({pos.x:F3}, {pos.y:F3}, {pos.z:F3}), vel=({vel.x:F3}, {vel.y:F3}, {vel.z:F3}), dt={dt:F3}");
                UnityEngine.Debug.Log($"[FTJ_DEBUG] k1p=({k1p.x:F3}, {k1p.y:F3}, {k1p.z:F3}), k1v=({k1v.x:F3}, {k1v.y:F3}, {k1v.z:F3})");
            }

            double3 pos2 = pos + k1p * (dt * a21);
            double3 vel2 = vel + k1v * (dt * a21);
            double3 a2 = EvaluateAccelerationAt(pos2, time + dt * (1.0/5.0), ref localHint);
            
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[FTJ_DEBUG] Stage 2: pos2=({pos2.x:F3}, {pos2.y:F3}, {pos2.z:F3}), vel2=({vel2.x:F3}, {vel2.y:F3}, {vel2.z:F3})");
            }

            double3 pos3 = pos + (k1p * (dt * a31) + vel2 * (dt * a32));
            double3 vel3 = vel + (k1v * (dt * a31) + a2 * (dt * a32));
            double3 a3 = EvaluateAccelerationAt(pos3, time + dt * (3.0/10.0), ref localHint);

            double3 pos4 = pos + (k1p * (dt * a41) + vel2 * (dt * a42) + vel3 * (dt * a43));
            double3 vel4 = vel + (k1v * (dt * a41) + a2 * (dt * a42) + a3 * (dt * a43));
            double3 a4 = EvaluateAccelerationAt(pos4, time + dt * (4.0/5.0), ref localHint);

            double3 pos5 = pos + (k1p * (dt * a51) + vel2 * (dt * a52) + vel3 * (dt * a53) + vel4 * (dt * a54));
            double3 vel5 = vel + (k1v * (dt * a51) + a2 * (dt * a52) + a3 * (dt * a53) + a4 * (dt * a54));
            double3 a5 = EvaluateAccelerationAt(pos5, time + dt * (8.0/9.0), ref localHint);

            double3 pos6 = pos + (k1p * (dt * a61) + vel2 * (dt * a62) + vel3 * (dt * a63) + vel4 * (dt * a64) + vel5 * (dt * a65));
            double3 vel6 = vel + (k1v * (dt * a61) + a2 * (dt * a62) + a3 * (dt * a63) + a4 * (dt * a64) + a5 * (dt * a65));
            double3 a6 = EvaluateAccelerationAt(pos6, time + dt, ref localHint);

            double3 pos7 = pos + (k1p * (dt * a71) + vel3 * (dt * a73) + vel4 * (dt * a74) + vel5 * (dt * a75) + vel6 * (dt * a76));
            double3 vel7 = vel + (k1v * (dt * a71) + a3 * (dt * a73) + a4 * (dt * a74) + a5 * (dt * a75) + a6 * (dt * a76));
            double3 a7 = EvaluateAccelerationAt(pos7, time + dt, ref localHint);

            // 5th order weights - using shared constants
            const double b1 = DOPRI5Coefficients.b1;
            const double b3 = DOPRI5Coefficients.b3;
            const double b4 = DOPRI5Coefficients.b4;
            const double b5 = DOPRI5Coefficients.b5;
            const double b6 = DOPRI5Coefficients.b6;

            double3 fifthPos = pos + (k1p * (dt * b1) + vel3 * (dt * b3) + vel4 * (dt * b4) + vel5 * (dt * b5) + vel6 * (dt * b6));
            double3 fifthVel = vel + (k1v * (dt * b1) + a3 * (dt * b3) + a4 * (dt * b4) + a5 * (dt * b5) + a6 * (dt * b6));
            
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[FTJ_DEBUG] Result: fifthPos=({fifthPos.x:F3}, {fifthPos.y:F3}, {fifthPos.z:F3}), fifthVel=({fifthVel.x:F3}, {fifthVel.y:F3}, {fifthVel.z:F3})");
            }

            // 4th order weights (for error estimation) - using shared constants
            const double bs1 = DOPRI5Coefficients.bStar1;
            const double bs3 = DOPRI5Coefficients.bStar3;
            const double bs4 = DOPRI5Coefficients.bStar4;
            const double bs5 = DOPRI5Coefficients.bStar5;
            const double bs6 = DOPRI5Coefficients.bStar6;
            const double bs7 = DOPRI5Coefficients.bStar7;

            double3 fourthPos = pos + (k1p * (dt * bs1) + vel3 * (dt * bs3) + vel4 * (dt * bs4) + vel5 * (dt * bs5) + vel6 * (dt * bs6) + vel7 * (dt * bs7));
            double3 fourthVel = vel + (k1v * (dt * bs1) + a3 * (dt * bs3) + a4 * (dt * bs4) + a5 * (dt * bs5) + a6 * (dt * bs6) + a7 * (dt * bs7));

            // Error = difference between 5th and 4th order
            errPos = math.length(fifthPos - fourthPos);
            errVel = math.length(fifthVel - fourthVel);

            // FSAL: k7 этого шага = k1 следующего шага (если принят)
            lastAccel = a7;

            return new IntegrationResult { Position = fifthPos, Velocity = fifthVel };
        }

        // ─── Event-aware stepping caps ──────────────────────────────
        private double ComputeEventCaps(
            double3 pos, double3 vel, double currentTime,
            double targetTime, ref SubstepData data)
        {
            double dt = targetTime - currentTime;

            // If both JupiterRadius and MoonRadius are 0, event caps are disabled
            // Return dt unchanged (no geometric capping)
            if (JupiterRadius <= 0.0 && MoonRadius <= 0.0)
            {
                return dt;
            }

            double minDist = data.MinDistToAnyBody;
            double speed = math.length(vel);
            double timeToClosest = minDist / math.max(speed, 0.1);
            double dtGeom = timeToClosest * 0.1;

            if (JupiterRadius > 0.0 && minDist < JupiterRadius * 10.0)
                dtGeom = math.min(dtGeom, 60.0);
            if (MoonRadius > 0.0 && minDist < MoonRadius * 10.0)
                dtGeom = math.min(dtGeom, 10.0);

            double dtManeuver = dt;
            double timeToManeuver = targetTime - currentTime;
            if (timeToManeuver > 0.0 && timeToManeuver < 3600.0)
                dtManeuver = timeToManeuver * 0.01;

            double capped = math.min(dtGeom, math.min(dtManeuver, dt));
            return math.max(capped, MinStepSeconds);
        }

        // ─── Adaptive integrator (single step, FSAL-aware) ──────────
        private bool TryAdvanceStep(
            ref double3 pos, ref double3 vel, ref double time,
            double targetTime, ref int ephemIdx,
            ref double dt,
            ref double3 fsalAccel, ref bool fsalValid)
        {
            // PrepareSubstepData only for MinDistToAnyBody (event caps)
            var stepData = PrepareSubstepData(time, pos, ref ephemIdx);

            // Apply event-aware caps
            double cappedDt = ComputeEventCaps(pos, vel, time, targetTime, ref stepData);
            dt = math.min(dt, cappedDt);
            if (time + dt > targetTime) dt = targetTime - time;

            // FSAL: если не валиден (первый шаг, после reject или после манёвра),
            // вычисляем k1 явно. Иначе fsalAccel содержит k7 предыдущего принятого шага.
            if (!fsalValid)
            {
                fsalAccel = EvaluateAccelerationAt(pos, time, ref ephemIdx);
                fsalValid = true;
            }

            // DoPri5: k1 = fsalAccel, остальные 6 стадий вычисляются внутри
            var result = DoPri5Step(pos, vel, time, dt, ephemIdx, out double errPos, out double errVel,
                fsalAccel, out double3 newLastAccel);

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

                // FSAL: k7 принятого шага = k1 следующего
                fsalAccel = newLastAccel;
                fsalValid = true;

                double stepScale = math.clamp(
                    0.9 * math.pow(1.0 / math.max(normalizedError, 1e-10), 0.2),
                    0.2, 5.0);
                dt = math.clamp(dt * stepScale, MinStepSeconds, MaxStepSeconds);
                return true;
            }
            else
            {
                double rejectScale = math.clamp(
                    0.9 * math.pow(1.0 / math.max(normalizedError, 1e-10), 0.2),
                    0.1, 0.5);
                dt = math.max(dt * rejectScale, MinStepSeconds);
                if (dt <= MinStepSeconds)
                {
                    // Force-accept с минимальным шагом
                    pos = result.Position;
                    vel = result.Velocity;
                    time += dt;

                    // FSAL: k7 force-accepted шага = k1 следующего
                    fsalAccel = newLastAccel;
                    fsalValid = true;
                    return true;
                }

                // REJECT: FSAL невалиден для следующей попытки
                fsalValid = false;
                return false;
            }
        }

        // ─── Time-dependent acceleration evaluation ─────────────────
        /// <summary>
        /// Вычисляет сумму гравитационных ускорений от Юпитера и всех лун
        /// для заданного корабля и времени t. Луны интерполируются через
        /// Hermite по эфемеридной таблице на момент t.
        /// </summary>
        private double3 EvaluateAccelerationAt(double3 pos, double t, ref int hintIdx)
        {
            if (ProfileCounters.IsCreated) ProfileCounters[PC_EVAL_ACCEL]++;
            
            double3 total = double3.zero;

            if (JupiterSGP > 0.0)
            {
                total += AccelerationEvaluator.BodyGravity(pos, JupiterPosition, JupiterSGP);
            }

            if (MoonCount > 0 && MoonEphemeris.Length > 0 && EphemerisTimes.Length > 0)
            {
                int timesLen = EphemerisTimes.Length;
                int idx = math.clamp(hintIdx, 0, timesLen - 2);
                while (idx < timesLen - 2 && EphemerisTimes[idx + 1] < t)
                {
                    idx++;
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_EPHEM_SEARCH]++;
                }

                hintIdx = idx; // Update hint for next call

                double t0 = EphemerisTimes[idx];
                double t1 = EphemerisTimes[math.min(idx + 1, timesLen - 1)];
                int b0 = idx * MoonCount;
                int b1 = math.min(idx + 1, timesLen - 1) * MoonCount;

                int mCount = MoonCount; // FIXED: Use all moons, not just first 4
                for (int m = 0; m < mCount; m++)
                {
                    if (ProfileCounters.IsCreated) ProfileCounters[PC_HERMITE]++;
                    double3 moonPos = AccelerationEvaluator.HermiteInterpolate(
                        MoonEphemeris[b0 + m].Position, MoonVelocities[b0 + m],
                        MoonEphemeris[b1 + m].Position, MoonVelocities[b1 + m],
                        t0, t1, t);
                    double moonMu = MoonEphemeris[b0 + m].StandardGravitationalParameter;
                    double3 moonAccel = AccelerationEvaluator.BodyGravity(pos, moonPos, moonMu);
                    total += moonAccel;
                }
            }

            return total;
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
                
                // Simplified: only compute MinDistToAnyBody without full Hermite interpolation
                // Use linear approximation for distance calculation (good enough for event caps)
                // Linear interpolation error is acceptable here because:
                // - MinDistToAnyBody is only used for geometric step limiting (event caps)
                // - Not used for acceleration evaluation (which requires Hermite precision)
                // - At ephemerisStep=18000s (5h), error ~5% of distance is acceptable for caps
                // - At ephemerisStep=60s (1min), error is negligible
                if (idx >= 0 && idx < timesLen - 1)
                {
                    double t0 = EphemerisTimes[idx];
                    double t1 = EphemerisTimes[idx + 1];
                    double alpha = (time - t0) / math.max(t1 - t0, 1e-10);
                    alpha = math.clamp(alpha, 0.0, 1.0);
                    
                    int b0 = idx * MoonCount;
                    int b1 = (idx + 1) * MoonCount;

                    for (int m = 0; m < MoonCount; m++)
                    {
                        // Linear interpolation for distance check (faster than Hermite)
                        double3 moonPos = math.lerp(
                            MoonEphemeris[b0 + m].Position,
                            MoonEphemeris[b1 + m].Position,
                            alpha);

                        double moonDistSq = math.lengthsq(moonPos - shipPos);
                        if (moonDistSq < minDistSq) minDistSq = moonDistSq;
                    }
                }
                else if (idx >= 0 && idx < timesLen)
                {
                    int b = idx * MoonCount;
                    for (int m = 0; m < MoonCount; m++)
                    {
                        double3 moonPos = MoonEphemeris[b + m].Position;
                        double moonDistSq = math.lengthsq(moonPos - shipPos);
                        if (moonDistSq < minDistSq) minDistSq = moonDistSq;
                    }
                }
            }

            data.MinDistToAnyBody = math.sqrt(minDistSq);
            return data;
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

            frameVel = AccelerationEvaluator.HermiteInterpolateVelocity(
                MoonEphemeris[b0].Position,
                MoonVelocities[b0],
                MoonEphemeris[b1].Position,
                MoonVelocities[b1],
                EphemerisTimes[idx],
                EphemerisTimes[math.min(idx + 1, timesLen - 1)],
                time);
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
            // Only used for MinDistToAnyBody calculation in event caps
            // Moon positions are computed on-demand via EvaluateAccelerationAt
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
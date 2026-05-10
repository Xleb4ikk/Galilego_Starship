using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Physics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Maneuver trajectory evaluator with deterministic rendering.
    /// 
    /// Architecture:
    ///   1. Prediction runs in coroutine, builds into back buffer
    ///   2. On completion, atomic swap to front buffer
    ///   3. LateUpdate renders from front buffer only
    /// 
    /// This eliminates:
    ///   - Coroutine/render race conditions
    ///   - Partial trajectory visibility
    ///   - Stale frame artifacts
    ///   - Visual drift during rebuild
    /// </summary>
    [RequireComponent(typeof(LineRenderer))]
    public class ManeuverEvaluator : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;

        [Header("Visualization")]
        [SerializeField] private GameObject timeMarkerPrefab;
        private GameObject timeMarkerInstance;
        private List<LineRenderer> segmentLines = new List<LineRenderer>();

        [Header("Prediction Settings")]
        [SerializeField] private double predictionStepSeconds = 2.0d;
        [SerializeField] private double maxPredictionSubstepSeconds = 1.0d;
        [SerializeField] private int maxSubstepsPerSegment = 1024;
        [SerializeField] private int maxTrajectoryPoints = 5000;
        [SerializeField] private double defaultPredictionLengthSeconds = 3600d;
        [SerializeField] private double maxPredictionLengthSeconds = 86400d * 30d;

        [Header("Performance")]
        [SerializeField] private int maxStepsPerFrame = 512;
        [SerializeField] private float debounceTime = 0.05f;
        [SerializeField] private int maxPointsPerLine = 512;

        // Trajectory data
        private List<Vector3d> fullTrajectoryPoints = new List<Vector3d>();
        private List<double> fullTrajectoryTimes = new List<double>();

        // Back buffer for building (no partial rendering)
        private Vector3[] backBufferPoints;
        private double[] backBufferTimes;
        private int backBufferCount;

        // Frame-locked reference frame
        private ReferenceFrameTarget lockedReferenceFrame;

        private Material solidMaterial;
        private Material dashedMaterial;
        private Vector3[] positionsBuffer = new Vector3[0];

        private FlightPlan flightPlan = new FlightPlan();
        private Coroutine calculationCoroutine;
        private bool isDirty = false;
        private float dirtyTimer = 0f;

        private void Start()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();

            if (!IsInvoking(nameof(RequestRecalculation)))
            {
                float jitter = UnityEngine.Random.Range(0f, 0.5f);
                Invoke(nameof(RequestRecalculation), 1f + jitter);
            }
        }

        private void Update()
        {
            if (isDirty)
            {
                dirtyTimer += Time.unscaledDeltaTime;
                if (dirtyTimer >= debounceTime)
                {
                    isDirty = false;
                    dirtyTimer = 0f;
                    RequestRecalculation();
                }
            }

            UpdateVisibility();
        }

        private void LateUpdate()
        {
            UpdateMarkerPosition();
        }

        private void UpdateVisibility()
        {
            if (universeManager == null) return;

            bool showLines = universeManager.CameraMode == SpaceCameraMode.OrbitMap;

            foreach (var line in segmentLines)
            {
                if (line != null && line.gameObject.activeSelf != showLines && line.positionCount > 0)
                {
                    line.gameObject.SetActive(showLines);
                }

                if (showLines && line != null && line.positionCount > 1)
                {
                    float width = universeManager.ResolveWorldLineWidthForPixels(2.25f, 0.02f);
                    line.startWidth = width;
                    line.endWidth = width;
                    line.alignment = LineAlignment.View;
                    line.numCornerVertices = 6;
                    line.numCapVertices = 6;
                }
            }

            if (timeMarkerInstance != null)
            {
                timeMarkerInstance.SetActive(showLines);
            }
        }

        public void MarkAsDirty()
        {
            if (!isDirty)
            {
                dirtyTimer = 0f;
            }
            isDirty = true;
        }

        public void RequestRecalculation()
        {
            if (Time.realtimeSinceStartup < 1f)
            {
                if (!IsInvoking(nameof(RequestRecalculation)))
                {
                    float jitter = UnityEngine.Random.Range(0f, 0.5f);
                    Invoke(nameof(RequestRecalculation), 1f + jitter);
                }
                return;
            }

            if (calculationCoroutine != null)
                StopCoroutine(calculationCoroutine);
            calculationCoroutine = StartCoroutine(CalculateFullTrajectoryCoroutine());
        }

        /// <summary>
        /// Main prediction coroutine.
        /// Builds trajectory into back buffer, then atomically swaps.
        /// NO partial rendering during build.
        /// </summary>
        private IEnumerator CalculateFullTrajectoryCoroutine()
        {
            if (universeManager == null || universeManager.ShipBody == null)
                yield break;

            ClearLines();
            fullTrajectoryPoints.Clear();
            fullTrajectoryTimes.Clear();

            // Lock reference frame for entire prediction
            lockedReferenceFrame = universeManager.ActiveReferenceFrame;

            flightPlan.Nodes.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));

            Vector3d currentPos = universeManager.ShipBody.Position;
            Vector3d currentVel = universeManager.ShipBody.Velocity;
            double currentTime = universeManager.SimulationTimeSeconds;

            double majorStep = Math.Max(1e-6d, predictionStepSeconds);
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);

            // Calculate prediction horizon
            double requestedPrediction = flightPlan.PredictionLengthSeconds;
            double cappedMax = Math.Max(10d, maxPredictionLengthSeconds);
            double effectivePrediction = requestedPrediction > 0d
                ? Math.Min(requestedPrediction, cappedMax)
                : Math.Min(Math.Max(10d, defaultPredictionLengthSeconds), cappedMax);
            double endTime = currentTime + effectivePrediction;

            // Initialize back buffer
            int estimatedCapacity = Math.Min(maxTrajectoryPoints,
                (int)(effectivePrediction / majorStep) + 100);
            InitializeBackBuffer(estimatedCapacity);

            int totalPoints = 0;
            int totalStepsInFrame = 0;
            int safetyCounter = 0;
            const int SAFETY_LIMIT = 100000;

            // Frame state with NO stale fallback
            Vector3d framePos = Vector3d.Zero;
            Vector3d frameVel = Vector3d.Zero;

            for (int i = 0; i <= flightPlan.Nodes.Count; i++)
            {
                double targetTime;
                ManeuverNode currentNode = null;

                if (i < flightPlan.Nodes.Count)
                {
                    currentNode = flightPlan.Nodes[i];
                    targetTime = currentNode.StartTime;
                }
                else
                {
                    targetTime = endTime;
                }

                // Handle past nodes
                if (targetTime <= currentTime)
                {
                    if (currentNode != null)
                    {
                        if (!TryUpdateFrameState(ref framePos, ref frameVel, currentTime))
                        {
                            Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting.");
                            yield break;
                        }

                        Vector3d relativePos = currentPos - framePos;
                        Vector3d relativeVel = currentVel - frameVel;
                        Vector3d dv = FlightPlan.CalculateWorldDeltaV(relativePos, relativeVel, currentNode);
                        currentVel += dv;
                    }
                    continue;
                }

                if (currentTime >= endTime) break;

                // Update frame state at segment start
                if (!TryUpdateFrameState(ref framePos, ref frameVel, currentTime))
                {
                    Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting.");
                    yield break;
                }

                bool isDashedSegment = (i > 0) && flightPlan.Nodes[i - 1].TotalDeltaV > 0.001;

                // Add initial point
                Vector3d relativePos = currentPos - framePos;
                AddPointToBackBuffer(totalPoints, universeManager.ToUnityOffset(relativePos), currentTime);
                fullTrajectoryPoints.Add(currentPos);
                fullTrajectoryTimes.Add(currentTime);
                totalPoints++;

                bool trajectoryLimitReached = false;
                int iterLimit = 0;
                const int ITER_LIMIT = 5000;

                while (currentTime < targetTime)
                {
                    iterLimit++;
                    if (iterLimit > ITER_LIMIT)
                    {
                        Debug.LogError("ManeuverEvaluator: Outer integration iter limit reached");
                        trajectoryLimitReached = true;
                        break;
                    }

                    double stepTime = Math.Min(majorStep, targetTime - currentTime);
                    if (stepTime <= 0) break;

                    int internalSteps = CalculateAdaptiveSubsteps(currentPos, stepTime, substepLimit);
                    double internalDt = stepTime / internalSteps;

                    bool abortedBySafety = false;
                    for (int k = 0; k < internalSteps; k++)
                    {
                        safetyCounter++;
                        if (safetyCounter > SAFETY_LIMIT)
                        {
                            Debug.LogError("ManeuverEvaluator: Trajectory safety stop");
                            trajectoryLimitReached = true;
                            abortedBySafety = true;
                            break;
                        }

                        var res = PhysicsSolver.RK4(
                            currentPos, currentVel, currentTime, internalDt,
                            universeManager.EvaluateShipAccelerationAt);

                        currentPos = res.Position;
                        currentVel = res.Velocity;
                        currentTime += internalDt;

                        if (!currentPos.IsFinite || !currentVel.IsFinite)
                        {
                            Debug.LogError($"ManeuverEvaluator: Invalid physics state at t={currentTime}");
                            trajectoryLimitReached = true;
                            abortedBySafety = true;
                            break;
                        }

                        totalStepsInFrame++;

                        if (totalStepsInFrame >= maxStepsPerFrame)
                        {
                            yield return null;
                            totalStepsInFrame = 0;
                        }
                    }

                    if (abortedBySafety) break;

                    // Update frame state for sample point
                    if (!TryUpdateFrameState(ref framePos, ref frameVel, currentTime))
                    {
                        Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting.");
                        trajectoryLimitReached = true;
                        break;
                    }

                    // Add sample point
                    relativePos = currentPos - framePos;
                    if (!relativePos.IsFinite || !currentPos.IsFinite || !currentVel.IsFinite)
                    {
                        Debug.LogError($"ManeuverEvaluator: NaN detected at time {currentTime}");
                        trajectoryLimitReached = true;
                        break;
                    }

                    AddPointToBackBuffer(totalPoints, universeManager.ToUnityOffset(relativePos), currentTime);
                    fullTrajectoryPoints.Add(currentPos);
                    fullTrajectoryTimes.Add(currentTime);
                    totalPoints++;

                    if (totalPoints >= maxTrajectoryPoints)
                    {
                        Debug.LogWarning("ManeuverEvaluator: Trajectory point limit reached");
                        trajectoryLimitReached = true;
                    }

                    if (trajectoryLimitReached) break;
                }

                // Apply maneuver Δv
                if (currentNode != null)
                {
                    if (!TryUpdateFrameState(ref framePos, ref frameVel, currentTime))
                    {
                        Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting.");
                        yield break;
                    }

                    Vector3d relativePos = currentPos - framePos;
                    Vector3d relativeVel = currentVel - frameVel;
                    Vector3d dv = FlightPlan.CalculateWorldDeltaV(relativePos, relativeVel, currentNode);
                    currentVel += dv;
                }

                if (totalPoints >= maxTrajectoryPoints) break;
            }

            // Complete build and render (atomic swap)
            CompleteBackBuffer(totalPoints);
            calculationCoroutine = null;
        }

        /// <summary>
        /// Update frame state with NO stale fallback.
        /// Returns false if frame state cannot be obtained.
        /// </summary>
        private bool TryUpdateFrameState(ref Vector3d framePos, ref Vector3d frameVel, double time)
        {
            if (universeManager.TryGetReferenceStateAtTime(
                lockedReferenceFrame, time,
                out _, out framePos, out frameVel,
                out _, out _, out _))
            {
                return true;
            }

            // Hard fail - do NOT use stale frame
            return false;
        }

        private void InitializeBackBuffer(int capacity)
        {
            backBufferPoints = new Vector3[capacity];
            backBufferTimes = new double[capacity];
            backBufferCount = 0;
        }

        private void AddPointToBackBuffer(int index, Vector3 point, double time)
        {
            if (index < backBufferPoints.Length)
            {
                backBufferPoints[index] = point;
                backBufferTimes[index] = time;
            }
        }

        /// <summary>
        /// Complete back buffer and render atomically.
        /// NO partial rendering - only called when build is complete.
        /// </summary>
        private void CompleteBackBuffer(int count)
        {
            if (count == 0) return;

            // Ensure we have enough LineRenderers
            int lineCount = (count + maxPointsPerLine - 1) / maxPointsPerLine;
            EnsureLineCount(lineCount);

            // Distribute points across LineRenderers
            int pointIndex = 0;
            for (int lineIdx = 0; lineIdx < lineCount; lineIdx++)
            {
                int pointsInLine = Math.Min(maxPointsPerLine, count - pointIndex);
                var line = segmentLines[lineIdx];

                line.positionCount = pointsInLine;

                // Copy points to positions buffer
                if (positionsBuffer.Length < pointsInLine)
                {
                    positionsBuffer = new Vector3[pointsInLine];
                }

                for (int i = 0; i < pointsInLine; i++)
                {
                    positionsBuffer[i] = backBufferPoints[pointIndex + i];
                }

                line.SetPositions(positionsBuffer);
                pointIndex += pointsInLine;
            }
        }

        private void EnsureLineCount(int count)
        {
            while (segmentLines.Count < count)
            {
                GameObject obj = new GameObject("ManeuverSegment_" + segmentLines.Count);
                obj.transform.SetParent(GetTrajectoryParent());
                obj.layer = ResolveTrajectoryLayer();
                var lr = obj.AddComponent<LineRenderer>();
                lr.useWorldSpace = false;
                lr.startWidth = 0.15f;
                lr.endWidth = 0.15f;
                lr.alignment = LineAlignment.View;
                lr.numCornerVertices = 6;
                lr.numCapVertices = 6;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                segmentLines.Add(lr);
            }
        }

        public bool TryGetTrajectoryPositionAtTime(double targetTime, out Vector3d position)
        {
            position = Vector3d.Zero;
            if (fullTrajectoryPoints == null || fullTrajectoryPoints.Count == 0 ||
                fullTrajectoryTimes == null || fullTrajectoryTimes.Count == 0)
                return false;

            for (int i = 0; i < fullTrajectoryTimes.Count - 1; i++)
            {
                if (targetTime >= fullTrajectoryTimes[i] && targetTime <= fullTrajectoryTimes[i + 1])
                {
                    double t = (targetTime - fullTrajectoryTimes[i]) /
                               (fullTrajectoryTimes[i + 1] - fullTrajectoryTimes[i]);
                    position = Vector3d.Lerp(fullTrajectoryPoints[i], fullTrajectoryPoints[i + 1], t);
                    return true;
                }
            }

            if (fullTrajectoryPoints.Count > 0)
            {
                position = fullTrajectoryPoints[fullTrajectoryPoints.Count - 1];
                return true;
            }

            return false;
        }

        public void UpdateMarkerPosition()
        {
            if (flightPlan == null || universeManager == null) return;

            double targetTime = universeManager.TrajectoryPreviewEndTime;

            Vector3d pos = Vector3d.Zero;
            bool found = false;

            for (int i = 0; i < fullTrajectoryTimes.Count - 1; i++)
            {
                if (targetTime >= fullTrajectoryTimes[i] && targetTime <= fullTrajectoryTimes[i + 1])
                {
                    double t = (targetTime - fullTrajectoryTimes[i]) /
                               (fullTrajectoryTimes[i + 1] - fullTrajectoryTimes[i]);
                    pos = Vector3d.Lerp(fullTrajectoryPoints[i], fullTrajectoryPoints[i + 1], t);
                    found = true;
                    break;
                }
            }

            if (!found && fullTrajectoryPoints.Count > 0)
            {
                pos = fullTrajectoryPoints[fullTrajectoryPoints.Count - 1];
                found = true;
            }

            if (found)
            {
                if (timeMarkerInstance == null)
                {
                    if (timeMarkerPrefab != null)
                    {
                        timeMarkerInstance = Instantiate(timeMarkerPrefab, GetTrajectoryParent());
                    }
                    else
                    {
                        timeMarkerInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        timeMarkerInstance.transform.SetParent(GetTrajectoryParent());
                        timeMarkerInstance.transform.localScale = Vector3.one * 0.5f;
                        timeMarkerInstance.GetComponent<Renderer>().material.color = Color.yellow;
                    }
                    timeMarkerInstance.layer = ResolveTrajectoryLayer();
                }

                ReferenceFrameTarget frame = lockedReferenceFrame;
                if (universeManager.TryGetReferenceStateAtTime(frame, targetTime, out _, out Vector3d framePos, out _, out _, out _, out _))
                {
                    universeManager.ApplyVisualPosition(timeMarkerInstance.transform, framePos);
                    timeMarkerInstance.transform.localPosition = universeManager.ToUnityOffset(pos - framePos);
                }

                float markerScale = universeManager.ResolveWorldLineWidthForPixels(5f, 0.01f);
                if (!float.IsNaN(markerScale) && !float.IsInfinity(markerScale) && markerScale > 0.00001f)
                {
                    timeMarkerInstance.transform.localScale = Vector3.one * markerScale;
                }
            }
        }

        private Transform GetTrajectoryParent()
        {
            if (universeManager != null)
            {
                var field = universeManager.GetType().GetField("trajectoryVisualRoot",
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (field != null)
                {
                    var val = field.GetValue(universeManager) as Transform;
                    if (val != null) return val;
                }
            }
            return transform;
        }

        private void ClearLines()
        {
            foreach (var line in segmentLines)
                if (line != null) line.gameObject.SetActive(false);
        }

        private double ResolveSubstepLimitSeconds(double majorStepSeconds)
        {
            double configuredLimit = maxPredictionSubstepSeconds > 0d 
                ? maxPredictionSubstepSeconds 
                : (universeManager != null ? universeManager.RecommendedSolverStepSeconds : 0d);
            if (configuredLimit <= 0d) configuredLimit = majorStepSeconds;
            return Math.Min(configuredLimit, majorStepSeconds);
        }

        private int CalculateAdaptiveSubsteps(Vector3d position, double majorStep, double baseSubstep)
        {
            double clampedSubstep = Math.Max(1e-9, Math.Min(baseSubstep, majorStep));
            int steps = (int)Math.Ceiling(majorStep / clampedSubstep);
            return Math.Max(1, Math.Min(steps, maxSubstepsPerSegment));
        }

        public FlightPlan GetFlightPlan() => flightPlan;

        private static int ResolveTrajectoryLayer()
        {
            int layer = LayerMask.NameToLayer("Trajectory");
            return layer >= 0 ? layer : 0;
        }
    }
}
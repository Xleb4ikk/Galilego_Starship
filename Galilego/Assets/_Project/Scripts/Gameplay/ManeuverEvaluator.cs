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
        [SerializeField] private double predictionStepSeconds = 10.0d;
        [SerializeField] private double maxPredictionSubstepSeconds = 5.0d;
        [SerializeField] private int maxSubstepsPerSegment = 256;
        [SerializeField] private int maxTrajectoryPoints = 2000;
        [SerializeField] private double defaultPredictionLengthSeconds = 7200d;
        [SerializeField] private double maxPredictionLengthSeconds = 86400d;

        [Header("Performance")]
        [SerializeField] private int maxStepsPerFrame = 10000;
        [SerializeField] private bool forceSynchronousCalculation = false;
        [SerializeField] private float debounceTime = 0.5f;
        [SerializeField] private int maxPointsPerLine = 512;

        // Trajectory data
        private List<Vector3d> fullTrajectoryPoints = new List<Vector3d>();
        private List<double> fullTrajectoryTimes = new List<double>();

        // Back buffer for building (no partial rendering)
        private Vector3[] backBufferPoints;
        private double[] backBufferTimes;
        private bool[] backBufferIsDashed;
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
            
            // If a calculation is in progress, stop it immediately
            if (calculationCoroutine != null)
            {
                Debug.Log("[ManeuverEvaluator] Stopping previous calculation due to parameter change");
                StopCoroutine(calculationCoroutine);
                calculationCoroutine = null;
            }
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

            // Don't start a new calculation if one is already running
            if (calculationCoroutine != null)
            {
                Debug.Log("[ManeuverEvaluator] Calculation already in progress, skipping request");
                return;
            }

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

            Debug.Log($"[ManeuverEvaluator] Starting trajectory calculation with {flightPlan.Nodes.Count} maneuvers");

            ClearLines();
            fullTrajectoryPoints.Clear();
            fullTrajectoryTimes.Clear();
            
            Debug.Log($"[ManeuverEvaluator] Cleared trajectory buffers: points={fullTrajectoryPoints.Count}, times={fullTrajectoryTimes.Count}");

            // Lock reference frame for entire prediction
            lockedReferenceFrame = universeManager.ActiveReferenceFrame;

            flightPlan.Nodes.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));

            Vector3d currentPos = universeManager.ShipBody.Position;
            Vector3d currentVel = universeManager.ShipBody.Velocity;
            double currentTime = universeManager.SimulationTimeSeconds;

            Debug.Log($"[ManeuverEvaluator] Initial state: pos={currentPos.Magnitude:F0}m, vel={currentVel.Magnitude:F2}m/s, time={currentTime:F1}s");

            double majorStep = Math.Max(1e-6d, predictionStepSeconds);
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);

            // Calculate prediction horizon
            double requestedPrediction = flightPlan.PredictionLengthSeconds;
            double cappedMax = Math.Min(maxPredictionLengthSeconds, 86400d); // Force max 1 day
            double effectivePrediction = requestedPrediction > 0d
                ? Math.Min(requestedPrediction, cappedMax)
                : Math.Min(Math.Max(10d, defaultPredictionLengthSeconds), cappedMax);
            double endTime = currentTime + effectivePrediction;
            
            // Force reset if prediction is too long
            if (requestedPrediction > cappedMax)
            {
                Debug.LogWarning($"[ManeuverEvaluator] Prediction length {requestedPrediction:F0}s exceeds maximum {cappedMax:F0}s, capping to {cappedMax:F0}s");
                flightPlan.PredictionLengthSeconds = cappedMax;
            }

            // Initialize back buffer — allocate enough for the full prediction
            int backBufferCapacity = (int)(effectivePrediction / Math.Max(1e-6d, majorStep)) + 100;
            backBufferCapacity = Math.Max(1000, Math.Min(maxTrajectoryPoints, backBufferCapacity));
            InitializeBackBuffer(backBufferCapacity);
            
            Debug.Log($"[ManeuverEvaluator] Back buffer capacity: {backBufferCapacity}, prediction length: {effectivePrediction:F0}s");

            // Dynamic ITER_LIMIT: enough for worst-case prediction @ 1s per major step
            int dynamicIterLimit = Math.Max(5000,
                (int)(effectivePrediction / Math.Max(1e-6d, majorStep)) + 100);
            const int ITER_LIMIT_ABSOLUTE_MAX = 2000000;
            dynamicIterLimit = Math.Min(dynamicIterLimit, ITER_LIMIT_ABSOLUTE_MAX);

            int totalPoints = 0;
            int totalStepsInFrame = 0;
            const int SAFETY_LIMIT_PER_SEGMENT = 500000;

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

                        Vector3d relPos = currentPos - framePos;
                        Vector3d relVel = currentVel - frameVel;
                        Vector3d dv = FlightPlan.CalculateWorldDeltaV(relPos, relVel, currentNode);
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

                // Segment is dashed if it comes AFTER a maneuver with non-zero deltaV
                // i=0: first segment (before first maneuver) - solid
                // i=1: segment after first maneuver - dashed if maneuver[0] had deltaV
                // i=2: segment after second maneuver - dashed if maneuver[1] had deltaV
                bool isDashedSegment = false;
                if (i > 0 && i - 1 < flightPlan.Nodes.Count)
                {
                    isDashedSegment = flightPlan.Nodes[i - 1].TotalDeltaV > 0.001;
                }
                
                Debug.Log($"[ManeuverEvaluator] Segment {i}: isDashed={isDashedSegment}, time={currentTime:F1}s");

                // Add initial point
                Vector3d relativePos = currentPos - framePos;
                Vector3 unityPos = universeManager.ToUnityOffset(relativePos);
                
                if (totalPoints == 0)
                {
                    Debug.Log($"[ManeuverEvaluator] First point: currentPos={currentPos.Magnitude:F0}m, framePos={framePos.Magnitude:F0}m, relativePos={relativePos.Magnitude:F0}m, unityPos={unityPos.magnitude:F2}");
                }
                
                AddPointToBackBuffer(totalPoints, unityPos, currentTime, isDashedSegment);
                fullTrajectoryPoints.Add(currentPos);
                fullTrajectoryTimes.Add(currentTime);
                totalPoints++;

                bool trajectoryLimitReached = false;
                int iterLimit = 0;
                int safetyCounter = 0;

                while (currentTime < targetTime && !trajectoryLimitReached)
                {
                    iterLimit++;
                    if (iterLimit > dynamicIterLimit)
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
                        if (safetyCounter > SAFETY_LIMIT_PER_SEGMENT)
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

                        if (!forceSynchronousCalculation && totalStepsInFrame >= maxStepsPerFrame)
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

                    Vector3 unitySamplePos = universeManager.ToUnityOffset(relativePos);
                    
                    if (totalPoints == 1)
                    {
                        Debug.Log($"[ManeuverEvaluator] Second point: currentPos={currentPos.Magnitude:F0}m, framePos={framePos.Magnitude:F0}m, relativePos={relativePos.Magnitude:F0}m, unityPos={unitySamplePos.magnitude:F2}");
                    }
                    
                    AddPointToBackBuffer(totalPoints, unitySamplePos, currentTime, isDashedSegment);
                    fullTrajectoryPoints.Add(currentPos);
                    fullTrajectoryTimes.Add(currentTime);
                    totalPoints++;

                    if (totalPoints >= maxTrajectoryPoints)
                    {
                        Debug.LogWarning($"ManeuverEvaluator: Trajectory point limit ({maxTrajectoryPoints}) reached. Stopping calculation.");
                        trajectoryLimitReached = true;
                    }

                    if (totalPoints >= backBufferCapacity)
                    {
                        Debug.LogWarning("ManeuverEvaluator: Back buffer capacity reached");
                        trajectoryLimitReached = true;
                    }

                    if (trajectoryLimitReached) break;
                }

                // Apply maneuver Δv — only if trajectory integration completed successfully
                if (currentNode != null && !trajectoryLimitReached)
                {
                    if (!TryUpdateFrameState(ref framePos, ref frameVel, currentTime))
                    {
                        Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting.");
                        yield break;
                    }

                    Vector3d maneuverRelPos = currentPos - framePos;
                    Vector3d maneuverRelVel = currentVel - frameVel;
                    Vector3d dv = FlightPlan.CalculateWorldDeltaV(maneuverRelPos, maneuverRelVel, currentNode);
                    
                    Debug.Log($"[ManeuverEvaluator] Applying maneuver #{i}: Δv={dv.Magnitude:F2} m/s at t={currentTime:F1}s");
                    Debug.Log($"  Before: pos={currentPos.Magnitude:F0}m, vel={currentVel.Magnitude:F2}m/s, pos={currentPos}");
                    Debug.Log($"  Frame: framePos={framePos.Magnitude:F0}m, frameVel={frameVel.Magnitude:F2}m/s");
                    Debug.Log($"  Relative: relPos={maneuverRelPos.Magnitude:F0}m, relVel={maneuverRelVel.Magnitude:F2}m/s");
                    Debug.Log($"  DeltaV components: prograde={currentNode.DvPrograde:F2}, normal={currentNode.DvNormal:F2}, radial={currentNode.DvRadial:F2}");
                    
                    currentVel += dv;
                    
                    Debug.Log($"  After: vel={currentVel.Magnitude:F2}m/s, relVelAfter={ (currentVel - frameVel).Magnitude:F2}m/s");
                    // Verify that position hasn't changed dramatically after Δv
                    double distToJupiter = currentPos.Magnitude;
                    Debug.Log($"  Post-Δv position check: distToJupiter={distToJupiter:F0}m, distInUnits={distToJupiter / universeManager.MetersPerUnityUnit:F2}");

                    // Add a point immediately after the maneuver with the new velocity
                    // This ensures the orbit visually intersects at the maneuver point
                    // (the pre-maneuver and post-maneuver orbits should cross here)
                    if (totalPoints < backBufferCapacity)
                    {
                        bool isPostManeuverDashed = currentNode.TotalDeltaV > 0.001;
                        Vector3d postManeuverRelPos = currentPos - framePos;
                        Vector3 unityPostManeuverPos = universeManager.ToUnityOffset(postManeuverRelPos);
                        
                        Debug.Log($"  Post-maneuver point: currentPos={currentPos.Magnitude:F0}m, framePos={framePos.Magnitude:F0}m, relPos={postManeuverRelPos.Magnitude:F0}m, unityPos={unityPostManeuverPos.magnitude:F2}, isDashed={isPostManeuverDashed}");
                        
                        AddPointToBackBuffer(totalPoints, unityPostManeuverPos, currentTime, isPostManeuverDashed);
                        fullTrajectoryPoints.Add(currentPos);
                        fullTrajectoryTimes.Add(currentTime);
                        totalPoints++;
                    }
                }

                // Don't break the segment loop — post-maneuver segments still need
                // to run (for Δv application and their initial points, even if
                // subsequent integration will be limited). The back buffer
                // silently drops points beyond capacity.
                if (totalPoints >= backBufferCapacity) trajectoryLimitReached = true;
            }

            // Complete build and render (atomic swap)
            CompleteBackBuffer(totalPoints);
            Debug.Log($"[ManeuverEvaluator] Trajectory calculation complete: {totalPoints} points generated");
            calculationCoroutine = null;
        }

        /// <summary>
        /// Update frame state with NO stale fallback.
        /// Returns false if frame state cannot be obtained.
        /// </summary>
        private bool TryUpdateFrameState(ref Vector3d framePos, ref Vector3d frameVel, double time)
        {
            Vector3d oldFramePos = framePos;
            if (universeManager.TryGetReferenceStateAtTime(
                lockedReferenceFrame, time,
                out _, out framePos, out frameVel,
                out _, out _, out _))
            {
                if (oldFramePos.SqrMagnitude == 0 && framePos.SqrMagnitude > 0)
                {
                    Debug.Log($"[ManeuverEvaluator] Frame state updated: framePos={framePos.Magnitude:F0}m, frameVel={frameVel.Magnitude:F2}m/s at t={time:F1}s");
                }
                return true;
            }

            // Hard fail - do NOT use stale frame
            return false;
        }

        private void InitializeBackBuffer(int capacity)
        {
            backBufferPoints = new Vector3[capacity];
            backBufferTimes = new double[capacity];
            backBufferIsDashed = new bool[capacity];
            backBufferCount = 0;
            
            Debug.Log($"[ManeuverEvaluator] Initialized back buffer with capacity {capacity}");
        }

        private void AddPointToBackBuffer(int index, Vector3 point, double time, bool isDashed)
        {
            if (index < backBufferPoints.Length)
            {
                backBufferPoints[index] = point;
                backBufferTimes[index] = time;
                backBufferIsDashed[index] = isDashed;
            }
        }

        /// <summary>
        /// Complete back buffer and render atomically.
        /// NO partial rendering - only called when build is complete.
        /// Groups consecutive points by segment type (solid vs dashed).
        /// </summary>
        private void CompleteBackBuffer(int count)
        {
            if (count == 0) return;

            int validCount = Math.Min(count, backBufferPoints.Length);
            if (validCount == 0) return;

            Debug.Log($"[ManeuverEvaluator] CompleteBackBuffer: rendering {validCount} points, reference frame: {lockedReferenceFrame}");

            // Debug: sample trajectory points and their distance from origin
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.Append($"[ManeuverPoints] count={validCount}");
            double minDistAll = double.MaxValue;
            double minDistDashed = double.MaxValue;
            int minIdxAll = -1;
            int minIdxDashed = -1;
            int sampleInterval = Math.Max(1, validCount / 8);
            for (int i = 0; i < validCount; i += sampleInterval)
            {
                float dist = backBufferPoints[i].magnitude;
                sb.Append($" | [{i}] pos={backBufferPoints[i]} dist={dist:F2}");
                if (dist < minDistAll) { minDistAll = dist; minIdxAll = i; }
                if (backBufferIsDashed[i] && dist < minDistDashed) { minDistDashed = dist; minIdxDashed = i; }
            }
            // Also scan all dashed points at full resolution for minimum distance
            for (int i = 0; i < validCount; i++)
            {
                if (backBufferIsDashed[i])
                {
                    float dist = backBufferPoints[i].magnitude;
                    if (dist < minDistDashed) { minDistDashed = dist; minIdxDashed = i; }
                }
            }
            sb.Append($" | minDist(all)={minDistAll:F2} at idx={minIdxAll}");
            sb.Append($" | minDist(dashed)={minDistDashed:F2} at idx={minIdxDashed}");
            Debug.Log(sb.ToString());

            // Find contiguous runs of same segment type
            List<(int start, int end, bool isDashed)> runs = new List<(int, int, bool)>();
            int runStart = 0;
            for (int i = 1; i < validCount; i++)
            {
                if (backBufferIsDashed[i] != backBufferIsDashed[runStart])
                {
                    runs.Add((runStart, i - 1, backBufferIsDashed[runStart]));
                    runStart = i;
                }
            }
            runs.Add((runStart, validCount - 1, backBufferIsDashed[runStart]));

            Debug.Log($"[ManeuverEvaluator] Rendering {runs.Count} runs:");
            foreach (var run in runs)
            {
                int runLength = run.end - run.start + 1;
                Debug.Log($"  Run: points {run.start}-{run.end} ({runLength} points), isDashed={run.isDashed}");
            }

            // Calculate required LineRenderers
            int totalLines = 0;
            foreach (var run in runs)
            {
                int runLength = run.end - run.start + 1;
                totalLines += (runLength + maxPointsPerLine - 1) / maxPointsPerLine;
            }

            EnsureLineCount(totalLines);

            // Render each run
            int lineIdx = 0;
            foreach (var run in runs)
            {
                int runLength = run.end - run.start + 1;
                int linesForRun = (runLength + maxPointsPerLine - 1) / maxPointsPerLine;

                for (int l = 0; l < linesForRun; l++)
                {
                    int pointsInLine = Math.Min(maxPointsPerLine, runLength - l * maxPointsPerLine);
                    var line = segmentLines[lineIdx];

                    line.positionCount = pointsInLine;
                    if (positionsBuffer.Length < pointsInLine) positionsBuffer = new Vector3[pointsInLine];

                    for (int i = 0; i < pointsInLine; i++)
                    {
                        positionsBuffer[i] = backBufferPoints[run.start + l * maxPointsPerLine + i];
                    }

                    line.SetPositions(positionsBuffer);
                    line.material = run.isDashed ? (dashedMaterial ?? CreateDefaultDashedMaterial()) : (solidMaterial ?? CreateDefaultSolidMaterial());
                    line.gameObject.SetActive(true);

                    lineIdx++;
                }
            }

            // Hide unused LineRenderers
            for (int i = lineIdx; i < segmentLines.Count; i++)
            {
                if (segmentLines[i] != null)
                {
                    segmentLines[i].positionCount = 0;
                    segmentLines[i].gameObject.SetActive(false);
                }
            }
        }

        private Material CreateDefaultSolidMaterial()
        {
            Shader shader = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            solidMaterial = new Material(shader);
            solidMaterial.color = new Color(0.5f, 0.5f, 1f, 0.9f);
            return solidMaterial;
        }

        private Material CreateDefaultDashedMaterial()
        {
            Shader shader = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            dashedMaterial = new Material(shader);
            dashedMaterial.color = new Color(0.3f, 1f, 0.3f, 0.9f);
            return dashedMaterial;
        }

        private void EnsureLineCount(int count)
        {
            while (segmentLines.Count < count)
            {
                GameObject obj = new GameObject("ManeuverSegment_" + segmentLines.Count);
                obj.transform.SetParent(GetTrajectoryParent(), false);
                obj.layer = ResolveTrajectoryLayer();
                var lr = obj.AddComponent<LineRenderer>();
                lr.useWorldSpace = false;  // Use local space relative to parent (reference frame)
                lr.startWidth = 0.15f;
                lr.endWidth = 0.15f;
                lr.alignment = LineAlignment.View;
                lr.numCornerVertices = 6;
                lr.numCapVertices = 6;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                segmentLines.Add(lr);
                
                Debug.Log($"[ManeuverEvaluator] Created LineRenderer with useWorldSpace=false");
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
                        timeMarkerInstance.transform.SetParent(GetTrajectoryParent(), false);
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
            Transform parent = null;
            if (universeManager != null && universeManager.TrajectoryVisualRoot != null)
                parent = universeManager.TrajectoryVisualRoot;
            else
                parent = transform;
                
            Debug.Log($"[ManeuverEvaluator] Trajectory parent: {parent.name}, position: {parent.position}, localPosition: {parent.localPosition}");
            return parent;
        }

        private void ClearLines()
        {
            Debug.Log($"[ManeuverEvaluator] Clearing {segmentLines.Count} line renderers");
            
            foreach (var line in segmentLines)
            {
                if (line != null && line.gameObject != null)
                {
                    UnityEngine.Object.Destroy(line.gameObject);
                }
            }
            
            segmentLines.Clear();
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
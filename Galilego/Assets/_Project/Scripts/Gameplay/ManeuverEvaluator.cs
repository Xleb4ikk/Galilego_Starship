using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Physics;

namespace Galilego.Gameplay
{
    [RequireComponent(typeof(LineRenderer))]
    public class ManeuverEvaluator : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;

        [Header("Visualization")]
        [SerializeField] private GameObject lineRendererPrefab;
        [SerializeField] private GameObject timeMarkerPrefab;
        private GameObject timeMarkerInstance;
        private List<LineRenderer> segmentLines = new List<LineRenderer>();
        
        private List<Vector3d> fullTrajectoryPoints = new List<Vector3d>();
        private List<double> fullTrajectoryTimes = new List<double>();

        [Header("Budgeting")]
        [Tooltip("Максимальное количество шагов интеграции за один кадр")]
        [SerializeField] private int maxStepsPerFrame = 256;
        [Tooltip("Задержка перед пересчетом при изменении UI (Debounce)")]
        [SerializeField] private float debounceTime = 0.15f;

        [Header("Prediction Settings")]
        [SerializeField] private double predictionStepSeconds = 30.0d;

        [Tooltip("Максимальная длина внутреннего шага интеграции")]
        [SerializeField] private double maxPredictionSubstepSeconds = 1.0d;

        [Tooltip("Максимальное количество внутренних шагов на один сегмент")]
        [SerializeField] private int maxSubstepsPerSegment = 1024;

        [Tooltip("Допустимая ошибка интеграции в метрах")]
        [SerializeField] private double toleranceMeters = 5.0d;

        [Tooltip("Предохранитель от миллионов точек")]
        [SerializeField] private int maxTrajectoryPoints = 5000;

        [Tooltip("Длина прогноза по умолчанию, если FlightPlan.PredictionLengthSeconds не задан")]
        [SerializeField] private double defaultPredictionLengthSeconds = 3600d;

        [Tooltip("Верхняя граница длины прогноза (сек.), согласована с окном планировщика (~30 суток)")]
        [SerializeField] private double maxPredictionLengthSeconds = 86400d * 30d;

        [Tooltip("Максимальное количество точек на один LineRenderer")] 
        [SerializeField] private int maxPointsPerLine = 256;

        [Tooltip("Максимальное число чанков (LineRenderer flush) за один кадр")]
        [SerializeField] private int maxChunkFlushPerFrame = 3;

        private Material solidMaterial;
        private Material dashedMaterial;
        // Reusable buffer to avoid allocations when calling LineRenderer.SetPositions
        private Vector3[] positionsBuffer = new Vector3[0];

        private FlightPlan flightPlan = new FlightPlan();
        private Coroutine calculationCoroutine;
        private bool isDirty = false;
        private float dirtyTimer = 0f;

        private void Start()
        {
            if (universeManager == null) universeManager = FindAnyObjectByType<UniverseManager>();
            // Delay initial heavy calculation slightly to avoid startup hitches
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

        private void UpdateVisibility()
        {
            if (universeManager == null) return;

            // Orbits should only be visible in OrbitMap mode
            bool showLines = universeManager.CameraMode == SpaceCameraMode.OrbitMap;

            foreach (var line in segmentLines)
            {
                if (line != null && line.gameObject.activeSelf != showLines && line.positionCount > 0)
                {
                    line.gameObject.SetActive(showLines);
                }

                if (showLines && line != null && line.positionCount > 1)
                {
                    // Keep maneuver trajectory readable at any zoom level (Principia-like feel)
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
                if (showLines) UpdateMarkerPosition();
            }
        }

        public void MarkAsDirty()
        {
            // Request recalculation via debounce — do not start immediate heavy work
            // Start the debounce timer only on the first dirty mark so that
            // subsequent per-frame MarkAsDirty() calls (e.g. while dragging)
            // don't reset the timer and prevent the debounce from firing.
            if (!isDirty)
            {
                dirtyTimer = 0f;
            }
            isDirty = true;
        }

        public void RequestRecalculation()
        {
            // If we're still in the first second of application start, defer heavy work once
            if (Time.realtimeSinceStartup < 1f)
            {
                if (!IsInvoking(nameof(RequestRecalculation)))
                {
                    float jitter = UnityEngine.Random.Range(0f, 0.5f);
                    Invoke(nameof(RequestRecalculation), 1f + jitter);
                }
                return;
            }

            if (calculationCoroutine != null) StopCoroutine(calculationCoroutine);
            calculationCoroutine = StartCoroutine(CalculateFullTrajectoryCoroutine());
        }

        private IEnumerator CalculateFullTrajectoryCoroutine()
        {
            if (universeManager == null || universeManager.ShipBody == null) yield break;

            ClearLines();
            fullTrajectoryPoints.Clear();
            fullTrajectoryTimes.Clear();
            
            flightPlan.Nodes.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));

            Vector3d currentPos = universeManager.ShipBody.Position;
            Vector3d currentVel = universeManager.ShipBody.Velocity;
            double currentTime = universeManager.SimulationTimeSeconds;
            ReferenceFrameTarget referenceFrame = universeManager.ActiveReferenceFrame;

            double majorStep = Math.Max(1e-6d, predictionStepSeconds);
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);
            
            int segmentIndex = 0;
            int totalStepsInFrame = 0;
            int chunkFlushCount = 0; // number of chunk flushes performed this coroutine slice

            // Длина нарисованной траектории = горизонт из планировщика (время до конца «будущей» дуги)
            double requestedPrediction = flightPlan != null ? flightPlan.PredictionLengthSeconds : 0d;
            double cappedMax = Math.Max(10d, maxPredictionLengthSeconds);
            double effectivePrediction = requestedPrediction > 0d
                ? Math.Min(requestedPrediction, cappedMax)
                : Math.Min(Math.Max(10d, defaultPredictionLengthSeconds), cappedMax);
            double endTime = currentTime + effectivePrediction;

            // Safety counter to avoid runaway integration
            int safetyCounter = 0;
            const int SAFETY_LIMIT = 100000;

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

                if (targetTime <= currentTime) 
                {
                    if (currentNode != null)
                    {
                        Vector3d dv = FlightPlan.CalculateWorldDeltaV(currentPos, currentVel, currentNode);
                        currentVel += dv;
                    }
                    continue;
                }

                if (currentTime >= endTime) break;

                LineRenderer line = null;
                List<Vector3> points = null;

                if (universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out Vector3d framePos, out _, out _, out _, out _))
                {
                    // Create first chunk/line for this segment
                    line = GetOrCreateLine(segmentIndex++);
                    points = new List<Vector3>(maxPointsPerLine);
                    ApplySegmentStyle(line, i > 0);
                    points.Add(universeManager.ToUnityOffset(currentPos - framePos));
                    universeManager.ApplyVisualPosition(line.transform, framePos);
                }

                fullTrajectoryPoints.Add(currentPos);
                fullTrajectoryTimes.Add(currentTime);

                bool trajectoryLimitReached = false;

                // Prevent outer while-loop runaway by limiting iterations per segment
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

                    int internalSteps = CalculateAdaptiveSubsteps(
                        currentPos,
                        stepTime,
                        substepLimit
                    );

                    double internalDt = stepTime / internalSteps;

                        bool abortedBySafety = false;
                        for (int k = 0; k < internalSteps; k++)
                        {
                            // count each RK4 call towards safety to avoid infinite loops
                            safetyCounter++;
                            if (safetyCounter > SAFETY_LIMIT)
                            {
                                Debug.LogError("ManeuverEvaluator: Trajectory safety stop");
                                trajectoryLimitReached = true;
                                abortedBySafety = true;
                                break;
                            }

                            var res = PhysicsSolver.RK4(
                                currentPos,
                                currentVel,
                                currentTime,
                                internalDt,
                                universeManager.EvaluateShipAccelerationAt
                            );

                            currentPos = res.Position;
                            currentVel = res.Velocity;
                            currentTime += internalDt;

                            // Guard: abort if integration produced invalid numbers
                            if (!currentPos.IsFinite || !currentVel.IsFinite)
                            {
                                Debug.LogError($"ManeuverEvaluator: Invalid physics state at t={currentTime} — aborting segment.");
                                trajectoryLimitReached = true;
                                abortedBySafety = true;
                                break;
                            }

                            totalStepsInFrame++;

                            if (totalStepsInFrame >= maxStepsPerFrame)
                            {
                                // Flush current visual points to the LineRenderer without allocating a new array
                                FlushLine(line, points);

                                chunkFlushCount++;
                                if (chunkFlushCount >= maxChunkFlushPerFrame)
                                {
                                    yield return null;
                                    chunkFlushCount = 0;
                                }
                                else
                                {
                                    // still yield occasionally to keep UI responsive
                                    yield return null;
                                }

                                totalStepsInFrame = 0;
                            }
                        }

                        if (abortedBySafety) break;

                        // After internal substeps complete (one major step), add a single sample for rendering and trajectory
                        if (universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out Vector3d fp, out _, out _, out _, out _))
                        {
                            Vector3d relativePos = currentPos - fp;

                            // CRITICAL: stop if any coordinates became invalid — avoid NaN propagation to visuals
                            if (!relativePos.IsFinite || !currentPos.IsFinite || !currentVel.IsFinite)
                            {
                                Debug.LogError($"ManeuverEvaluator: NaN detected at time {currentTime}. Aborting trajectory segment.");
                                trajectoryLimitReached = true;
                                break;
                            }

                            Vector3 sample = universeManager.ToUnityOffset(relativePos);
                            if (points == null)
                            {
                                line = GetOrCreateLine(segmentIndex++);
                                points = new List<Vector3>(maxPointsPerLine);
                                ApplySegmentStyle(line, i > 0);
                            }

                            points.Add(sample);

                            // If this chunk is full, flush and start a new LineRenderer for next chunk
                            if (points.Count >= maxPointsPerLine)
                            {
                                FlushLine(line, points);
                                chunkFlushCount++;
                                if (chunkFlushCount >= maxChunkFlushPerFrame)
                                {
                                    yield return null;
                                    chunkFlushCount = 0;
                                }

                                line = GetOrCreateLine(segmentIndex++);
                                points = new List<Vector3>(maxPointsPerLine);
                                ApplySegmentStyle(line, i > 0);

                                // carry on; we will add next sample on next major step
                            }
                        }

                        fullTrajectoryPoints.Add(currentPos);
                        fullTrajectoryTimes.Add(currentTime);

                        // Limit total number of trajectory points to avoid runaway memory/CPU
                        if (fullTrajectoryPoints.Count >= maxTrajectoryPoints)
                        {
                            Debug.LogWarning("ManeuverEvaluator: Trajectory point limit reached");
                            trajectoryLimitReached = true;
                        }

                        if (trajectoryLimitReached) break;
                }

                // Flush remaining points of the last chunk for this segment
                FlushLine(line, points);
                chunkFlushCount++;
                if (chunkFlushCount >= maxChunkFlushPerFrame)
                {
                    yield return null;
                    chunkFlushCount = 0;
                }

                if (currentNode != null)
                {
                    Vector3d dv = FlightPlan.CalculateWorldDeltaV(currentPos, currentVel, currentNode);
                    currentVel += dv;
                }
            }

            UpdateMarkerPosition();
            calculationCoroutine = null;
        }

        public bool TryGetTrajectoryPositionAtTime(double targetTime, out Vector3d position)
        {
            position = Vector3d.Zero;
            if (fullTrajectoryPoints == null || fullTrajectoryPoints.Count == 0 || fullTrajectoryTimes == null || fullTrajectoryTimes.Count == 0)
                return false;

            for (int i = 0; i < fullTrajectoryTimes.Count - 1; i++)
            {
                if (targetTime >= fullTrajectoryTimes[i] && targetTime <= fullTrajectoryTimes[i + 1])
                {
                    double t = (targetTime - fullTrajectoryTimes[i]) / (fullTrajectoryTimes[i + 1] - fullTrajectoryTimes[i]);
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
            
            // Конец отрезка траектории = «сейчас» симуляции + горизонт предсказания из планировщика
            double targetTime = universeManager.TrajectoryPreviewEndTime;
            
            Vector3d pos = Vector3d.Zero;
            bool found = false;
            
            for (int i = 0; i < fullTrajectoryTimes.Count - 1; i++)
            {
                if (targetTime >= fullTrajectoryTimes[i] && targetTime <= fullTrajectoryTimes[i+1])
                {
                    double t = (targetTime - fullTrajectoryTimes[i]) / (fullTrajectoryTimes[i+1] - fullTrajectoryTimes[i]);
                    pos = Vector3d.Lerp(fullTrajectoryPoints[i], fullTrajectoryPoints[i+1], t);
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
                    if (timeMarkerPrefab != null) timeMarkerInstance = Instantiate(timeMarkerPrefab, GetTrajectoryParent());
                    else
                    {
                        timeMarkerInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        timeMarkerInstance.transform.SetParent(GetTrajectoryParent());
                        timeMarkerInstance.transform.localScale = Vector3.one * 0.5f;
                        timeMarkerInstance.GetComponent<Renderer>().material.color = Color.yellow;
                    }
                    timeMarkerInstance.layer = ResolveTrajectoryLayer();
                }
                
                ReferenceFrameTarget frame = universeManager.ActiveReferenceFrame;
                if (universeManager.TryGetReferenceStateAtTime(frame, targetTime, out _, out Vector3d framePos, out _, out _, out _, out _))
                {
                    universeManager.ApplyVisualPosition(timeMarkerInstance.transform, framePos);
                    timeMarkerInstance.transform.localPosition = universeManager.ToUnityOffset(pos - framePos);
                }

                // Keep marker readable at any zoom level (OrbitMap)
                float markerScale = universeManager.ResolveWorldLineWidthForPixels(5f, 0.01f);
                if (!float.IsNaN(markerScale) && !float.IsInfinity(markerScale) && markerScale > 0.00001f)
                {
                    timeMarkerInstance.transform.localScale = Vector3.one * markerScale;
                }
            }
        }

        private void ApplySegmentStyle(LineRenderer line, bool isDashed)
        {
            if (isDashed)
            {
                if (dashedMaterial == null)
                {
                    dashedMaterial = new Material(Shader.Find("Custom/DashedLine"));
                    dashedMaterial.SetColor("_Color", new Color(1, 0.7f, 0, 0.8f));
                    dashedMaterial.SetFloat("_DashSize", 0.5f);
                    dashedMaterial.SetFloat("_GapSize", 0.5f);
                    dashedMaterial.SetFloat("_Tiling", 30f);
                }
                line.material = dashedMaterial;
                line.textureMode = LineTextureMode.Tile;
            }
            else
            {
                if (solidMaterial == null)
                {
                    solidMaterial = new Material(Shader.Find("Sprites/Default"));
                    solidMaterial.color = Color.cyan;
                }
                line.material = solidMaterial;
            }
        }

        private LineRenderer GetOrCreateLine(int index)
        {
            if (index >= segmentLines.Count)
            {
                GameObject obj = new GameObject("ManeuverSegment_" + index);
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
                return lr;
            }

            var existing = segmentLines[index];
            if (existing != null)
            {
                if (!existing.gameObject.activeSelf) existing.gameObject.SetActive(true);
                existing.positionCount = 0;
            }

            return existing;
        }

        private Transform GetTrajectoryParent()
        {
            if (universeManager != null)
            {
                var field = universeManager.GetType().GetField("trajectoryVisualRoot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
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
            double configuredLimit = maxPredictionSubstepSeconds > 0d ? maxPredictionSubstepSeconds : (universeManager != null ? universeManager.RecommendedSolverStepSeconds : 0d);
            if (configuredLimit <= 0d) configuredLimit = majorStepSeconds;
            return Math.Min(configuredLimit, majorStepSeconds);
        }

        private int CalculateAdaptiveSubsteps(
            Vector3d position,
            double majorStep,
            double baseSubstep)
        {
            // altitudeFactor: give more resolution near massive bodies (smaller altitude -> smaller steps)
            double altitudeFactor = Math.Max(1.0, position.Magnitude / 1000000.0);

            // desiredStep scales with sqrt of tolerance to give diminishing returns for very small tolerances
            double desiredStep = Math.Sqrt(Math.Max(1e-9, toleranceMeters)) * altitudeFactor;

            // clamp desiredStep to sane bounds
            desiredStep = Math.Min(Math.Max(desiredStep, baseSubstep), majorStep);

            int steps = (int)Math.Ceiling(majorStep / desiredStep);

            return Math.Max(1, Math.Min(steps, maxSubstepsPerSegment));
        }

        private void EnsurePositionsBuffer(int required)
        {
            if (positionsBuffer == null || positionsBuffer.Length < required)
            {
                int newSize = Math.Max(required, positionsBuffer != null ? positionsBuffer.Length * 2 : 256);
                positionsBuffer = new Vector3[newSize];
            }
        }

        private void FlushLine(LineRenderer line, List<Vector3> points)
        {
            if (line == null || points == null || points.Count == 0) return;

            EnsurePositionsBuffer(points.Count);
            points.CopyTo(positionsBuffer);

            // Prepare an array of the exact length expected by LineRenderer.SetPositions
            Vector3[] toSet;
            if (positionsBuffer.Length == points.Count)
            {
                toSet = positionsBuffer;
            }
            else
            {
                toSet = new Vector3[points.Count];
                Array.Copy(positionsBuffer, toSet, points.Count);
            }

            line.positionCount = points.Count;
            line.SetPositions(toSet);
        }

        public FlightPlan GetFlightPlan() => flightPlan;

        private static int ResolveTrajectoryLayer()
        {
            int layer = LayerMask.NameToLayer("Trajectory");
            return layer >= 0 ? layer : 0;
        }
    }
}

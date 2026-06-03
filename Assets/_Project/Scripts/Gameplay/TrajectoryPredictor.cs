using System;
using System.Collections;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;

namespace Galilego.Gameplay
{
    [RequireComponent(typeof(LineRenderer))]
    public sealed class TrajectoryPredictor : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private LineRenderer lineRenderer;

        [Header("Prediction")]
        [SerializeField] private int predictionSteps = 2000;
        [SerializeField] private double predictionStepSeconds = 2d;
        [SerializeField] private double maxPredictionSubstepSeconds = 0d;

        public double TotalPredictionDurationSeconds => predictionSteps * predictionStepSeconds;

        [Header("Performance")]
        [SerializeField] private bool autoRefresh = true;
        [SerializeField] private float refreshIntervalSeconds = 0.15f;
        [SerializeField] private int stepsPerBatch = 128;

        private Coroutine rebuildCoroutine;
        private Vector3[] cachedLocalPoints = Array.Empty<Vector3>();
        private bool refreshQueued;
        private float nextRefreshTime;

        private void Awake()
        {
            ResolveReferences();
            ConfigureLineRenderer();
        }

        private void OnEnable()
        {
            ForceRefresh();
        }

        private void OnDisable()
        {
            if (rebuildCoroutine != null)
            {
                StopCoroutine(rebuildCoroutine);
                rebuildCoroutine = null;
            }

            refreshQueued = false;

            if (lineRenderer != null)
                lineRenderer.positionCount = 0;
        }

        private void Update()
        {
            UpdateLineWidth();

            if (!autoRefresh)
            {
                return;
            }

            if (Time.unscaledTime < nextRefreshTime)
            {
                return;
            }

            nextRefreshTime = Time.unscaledTime + Mathf.Max(0.01f, refreshIntervalSeconds);
            RequestRefresh();
        }

        private void UpdateLineWidth()
        {
            if (lineRenderer == null || universeManager == null)
            {
                return;
            }

            // Calculate width based on average distance from camera to line points
            float avgDistance = 0f;
            if (lineRenderer.positionCount > 0)
            {
                Transform parent = transform;
                Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
                
                // Sample a few points along the line to get average distance
                int sampleCount = Mathf.Min(5, lineRenderer.positionCount);
                for (int i = 0; i < sampleCount; i++)
                {
                    int idx = (i * lineRenderer.positionCount) / Mathf.Max(1, sampleCount);
                    Vector3 localPos = lineRenderer.GetPosition(idx);
                    Vector3 worldPos = parent.TransformPoint(localPos);
                    avgDistance += Vector3.Distance(camPos, worldPos);
                }
                avgDistance /= sampleCount;
            }
            else
            {
                avgDistance = 1000f; // fallback
            }
            
            // Calculate width for constant screen-space size
            float pixelHeight = Camera.main != null ? Camera.main.pixelHeight : 1080f;
            float fov = Camera.main != null ? Camera.main.fieldOfView : 60f;
            float frustumHeight = 2f * avgDistance * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
            float width = frustumHeight * 2.5f / pixelHeight;
            
            // Clamp width to reasonable range
            width = Mathf.Clamp(width, 0.01f, 0.5f);
            
            lineRenderer.startWidth = width;
            lineRenderer.endWidth = width;
        }

        public void ForceRefresh()
        {
            nextRefreshTime = Time.unscaledTime + Mathf.Max(0.01f, refreshIntervalSeconds);
            RequestRefresh();
        }

        public void RequestRefresh()
        {
            ResolveReferences();

            if (lineRenderer == null || universeManager == null)
            {
                return;
            }

            refreshQueued = true;
            if (rebuildCoroutine == null)
            {
                rebuildCoroutine = StartCoroutine(RebuildTrajectoryCoroutine());
            }
        }

        public void Configure(UniverseManager manager, LineRenderer renderer)
        {
            universeManager = manager;
            lineRenderer = renderer;
            ConfigureLineRenderer();
        }

        public void ConfigurePrediction(
            int steps,
            double stepSeconds,
            double maxSubstepSeconds,
            bool shouldAutoRefresh,
            float refreshInterval,
            int batchSize)
        {
            predictionSteps = Math.Max(1, steps);
            predictionStepSeconds = Math.Max(1e-6d, stepSeconds);
            maxPredictionSubstepSeconds = Math.Max(0d, maxSubstepSeconds);
            autoRefresh = shouldAutoRefresh;
            refreshIntervalSeconds = Mathf.Max(0.01f, refreshInterval);
            stepsPerBatch = Math.Max(1, batchSize);
        }

        private IEnumerator RebuildTrajectoryCoroutine()
        {
            while (refreshQueued)
            {
                refreshQueued = false;

                // Wait for parent transform to be set (avoids one-frame visual offset
                // when parent is assigned after Awake/OnEnable)
                int parentWaitFrames = 0;
                while (transform.parent == null && parentWaitFrames < 60)
                {
                    yield return null;
                    parentWaitFrames++;
                }

                if (!TryGetPredictionState(out Vector3d startPosition, out Vector3d startVelocity, out double startTimeSeconds))
                {
                    lineRenderer.positionCount = 0;
                    break;
                }

                ReferenceFrameTarget referenceFrame = universeManager.ActiveReferenceFrame;
                if (!universeManager.TryGetReferenceStateAtTime(
                    referenceFrame,
                    startTimeSeconds,
                    out _,
                    out Vector3d startFramePosition,
                    out _,
                    out _,
                    out _,
                    out _))
                {
                    lineRenderer.positionCount = 0;
                    break;
                }

                // ── Clear stale line BEFORE moving transform (prevents phantom orbit) ──
                lineRenderer.positionCount = 0;

                universeManager.ApplyVisualPosition(transform, startFramePosition);
                transform.rotation = Quaternion.identity;

                Vector3 firstPointLocal = universeManager.ToUnityOffset(startPosition - startFramePosition);

                int clampedSteps = Math.Max(1, predictionSteps);
                EnsurePointCapacity(clampedSteps + 1);

                cachedLocalPoints[0] = firstPointLocal;

                Vector3d predictedPosition = startPosition;
                Vector3d predictedVelocity = startVelocity;
                double predictedTimeSeconds = startTimeSeconds;
                double majorStepSeconds = Math.Max(1e-6d, predictionStepSeconds);
                double substepLimitSeconds = ResolveSubstepLimitSeconds(majorStepSeconds);

                int pointsWritten = 1;

                for (int stepIndex = 0; stepIndex < clampedSteps; stepIndex++)
                {
                    int internalStepCount = Math.Max(1, (int)Math.Ceiling(majorStepSeconds / substepLimitSeconds));
                    double internalStepSeconds = majorStepSeconds / internalStepCount;

                    for (int internalStepIndex = 0; internalStepIndex < internalStepCount; internalStepIndex++)
                    {
                        IntegrationResult stepResult = PhysicsSolver.RK4(
                            predictedPosition,
                            predictedVelocity,
                            predictedTimeSeconds,
                            internalStepSeconds,
                            universeManager.EvaluateShipAccelerationAt);

                        predictedPosition = stepResult.Position;
                        predictedVelocity = stepResult.Velocity;
                        predictedTimeSeconds += internalStepSeconds;

                        if (!predictedPosition.IsFinite || !predictedVelocity.IsFinite)
                        {
                            ApplyLine(0);
                            rebuildCoroutine = null;
                            yield break;
                        }
                    }

                    // ── Per-point frame position (same pattern as ManeuverEvaluator.CompleteBackBuffer) ──
                    // Each trajectory point uses framePos AT THAT SAMPLE TIME,
                    // so the trajectory shows frame-relative motion (loops in moving frames).
                    Vector3d framePosAtTime = startFramePosition;
                    universeManager.TryGetReferenceStateAtTime(
                        referenceFrame, predictedTimeSeconds,
                        out _, out framePosAtTime, out _,
                        out _, out _, out _);
                    cachedLocalPoints[pointsWritten] = universeManager.ToUnityOffset(predictedPosition - framePosAtTime);
                    pointsWritten++;
                }
            }

            rebuildCoroutine = null;
        }

        private bool TryGetPredictionState(out Vector3d startPosition, out Vector3d startVelocity, out double startTimeSeconds)
        {
            startPosition = Vector3d.Zero;
            startVelocity = Vector3d.Zero;
            startTimeSeconds = 0d;

            if (universeManager == null || universeManager.ShipBody == null)
            {
                return false;
            }

            CelestialBody shipBody = universeManager.ShipBody;
            startPosition = shipBody.Position;
            startVelocity = shipBody.Velocity;
            startTimeSeconds = universeManager.SimulationTimeSeconds;
            return true;
        }

        private void ApplyLine(int pointCount)
        {
            if (lineRenderer == null)
            {
                return;
            }

            lineRenderer.positionCount = pointCount;
            for (int i = 0; i < pointCount; i++)
            {
                lineRenderer.SetPosition(i, cachedLocalPoints[i]);
            }
        }

        private void EnsurePointCapacity(int requiredCapacity)
        {
            if (cachedLocalPoints.Length >= requiredCapacity)
            {
                return;
            }

            cachedLocalPoints = new Vector3[requiredCapacity];
        }

        private double ResolveSubstepLimitSeconds(double majorStepSeconds)
        {
            double configuredLimit = maxPredictionSubstepSeconds > 0d
                ? maxPredictionSubstepSeconds
                : universeManager.RecommendedSolverStepSeconds;

            if (configuredLimit <= 0d)
            {
                configuredLimit = majorStepSeconds;
            }

            return Math.Min(configuredLimit, majorStepSeconds);
        }

        private void ResolveReferences()
        {
            if (lineRenderer == null)
            {
                lineRenderer = GetComponent<LineRenderer>();
            }

            if (universeManager == null)
            {
                universeManager = FindAnyObjectByType<UniverseManager>();
            }
        }

        private void ConfigureLineRenderer()
        {
            if (lineRenderer != null)
            {
                lineRenderer.useWorldSpace = false;
            }
        }

        private void OnValidate()
        {
            if (predictionSteps < 1)
            {
                predictionSteps = 1;
            }

            if (predictionStepSeconds <= 0d)
            {
                predictionStepSeconds = 0.1d;
            }

            if (stepsPerBatch < 1)
            {
                stepsPerBatch = 1;
            }

            ResolveReferences();
            ConfigureLineRenderer();
        }
    }
}

using System;
using System.Collections;
using UnityEngine;

namespace Galilego.Physics
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
        }

        private void Update()
        {
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

        private IEnumerator RebuildTrajectoryCoroutine()
        {
            while (refreshQueued)
            {
                refreshQueued = false;

                if (!TryGetPredictionState(out Vector3d startPosition, out Vector3d startVelocity, out double startTimeSeconds))
                {
                    lineRenderer.positionCount = 0;
                    break;
                }

                Vector3 anchorPosition = universeManager.ToUnityPosition(startPosition);
                transform.SetPositionAndRotation(anchorPosition, Quaternion.identity);

                int clampedSteps = Math.Max(1, predictionSteps);
                EnsurePointCapacity(clampedSteps + 1);

                cachedLocalPoints[0] = Vector3.zero;

                Vector3d predictedPosition = startPosition;
                Vector3d predictedVelocity = startVelocity;
                double predictedTimeSeconds = startTimeSeconds;
                double majorStepSeconds = Math.Max(1e-6d, predictionStepSeconds);
                double substepLimitSeconds = ResolveSubstepLimitSeconds(majorStepSeconds);

                int pointsWritten = 1;
                int stepsSinceYield = 0;
                bool abortedForNewRefresh = false;

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
                    }

                    cachedLocalPoints[pointsWritten] = universeManager.ToUnityOffset(predictedPosition - startPosition);
                    pointsWritten++;
                    stepsSinceYield++;

                    if (stepsSinceYield >= Mathf.Max(1, stepsPerBatch))
                    {
                        stepsSinceYield = 0;
                        yield return null;

                        if (refreshQueued)
                        {
                            abortedForNewRefresh = true;
                            break;
                        }
                    }
                }

                if (abortedForNewRefresh)
                {
                    continue;
                }

                ApplyLine(pointsWritten);
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

            if (refreshIntervalSeconds < 0.01f)
            {
                refreshIntervalSeconds = 0.01f;
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

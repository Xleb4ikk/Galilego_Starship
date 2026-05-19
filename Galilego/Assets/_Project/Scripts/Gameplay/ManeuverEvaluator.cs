using System;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Physics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Gameplay
{
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
        [SerializeField] private int maxSubstepsPerSegment = 4096;
        [SerializeField] private int maxTrajectoryPoints = 10000;
        [SerializeField] private double defaultPredictionLengthSeconds = 7200d;
        [SerializeField] private double maxPredictionLengthSeconds = 315360000d;

        [Header("Performance")]
        [SerializeField] private int maxStepsPerFrame = 10000;
        [SerializeField] private bool forceSynchronousCalculation = false;
        [SerializeField] private float debounceTime = 0.5f;
        [SerializeField] private int maxPointsPerLine = 512;

        private List<Vector3d> fullTrajectoryPoints = new List<Vector3d>();
        private List<double> fullTrajectoryTimes = new List<double>();

        private Vector3[] backBufferPoints;
        private double[] backBufferTimes;
        private bool[] backBufferIsDashed;
        private int backBufferCount;

        private ReferenceFrameTarget lockedReferenceFrame;

        private Material solidMaterial;
        private Material dashedMaterial;
        private Vector3[] positionsBuffer = new Vector3[0];

        private FlightPlan flightPlan = new FlightPlan();
        private bool isDirty = false;
        private float dirtyTimer = 0f;

        private JobHandle ephemerisJobHandle;
        private JobHandle trajectoryJobHandle;
        private bool isJobRunning = false;

        private NativeArray<MoonOrbitData> nativeMoonOrbits;
        private NativeArray<ManeuverNodeData> nativeNodeData;
        private NativeArray<double> nativeEphemerisTimes;
        private NativeArray<BodyState> nativeEphemerisResults;
        private NativeArray<double3> nativeEphemerisVelocities;
        private NativeArray<TrajectoryPoint> nativeTrajectoryOutput;
        private NativeReference<int> nativePointCount;
        private NativeReference<int> nativeCalcStatus;

        private void Start()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();

            var ownRenderer = GetComponent<LineRenderer>();
            if (ownRenderer != null) ownRenderer.enabled = false;

            if (!IsInvoking(nameof(RequestRecalculation)))
            {
                float jitter = UnityEngine.Random.Range(0f, 0.5f);
                Invoke(nameof(RequestRecalculation), 1f + jitter);
            }
        }

        private void OnDestroy()
        {
            CompleteAndDisposeJobs();
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

            if (isJobRunning)
            {
                if (trajectoryJobHandle.IsCompleted)
                {
                    CompleteJobAndSwap();
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
                if (line != null && line.positionCount > 0)
                {
                    if (line.gameObject.activeSelf != showLines)
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
                if (timeMarkerInstance.activeSelf != showLines)
                    timeMarkerInstance.SetActive(showLines);
            }
        }

        public void MarkAsDirty()
        {
            if (!isDirty)
                dirtyTimer = 0f;
            isDirty = true;

            if (isJobRunning)
            {
                CompleteAndDisposeJobs();
            }
        }

        public void MarkAsDirtyLightweight()
        {
            isDirty = true;
            dirtyTimer = 0f;
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

            if (isJobRunning) return;

            if (universeManager == null || universeManager.ShipBody == null)
                return;

            CompleteAndDisposeJobs();
            ClearLines();

            fullTrajectoryPoints.Clear();
            fullTrajectoryTimes.Clear();

            lockedReferenceFrame = universeManager.ActiveReferenceFrame;

            ScheduleJobs();
        }

        private void ScheduleJobs()
        {
            flightPlan.Nodes.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));

            double3 startPos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
            double3 startVel = JobTypeConversion.ToDouble3(universeManager.ShipBody.Velocity);
            double startTime = universeManager.SimulationTimeSeconds;

            double requestedPrediction = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;
            double effectivePrediction = Math.Min(requestedPrediction, maxPredictionLengthSeconds);
            double endTime = startTime + effectivePrediction;

            int nodeCount = flightPlan.Nodes.Count;

            int adjustedMaxPoints = Math.Max(maxTrajectoryPoints,
                (int)(effectivePrediction / 600.0) + 2);
            double adaptiveStep = effectivePrediction / Math.Max(1, (int)(adjustedMaxPoints * 0.9));
            double majorStep = Math.Max(1e-6d, Math.Max(predictionStepSeconds, adaptiveStep));
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);
            int maxSubsteps = flightPlan.MaxStepsPerSegment > 0
                ? flightPlan.MaxStepsPerSegment
                : maxSubstepsPerSegment;

            int nodeDataAlloc = Math.Max(1, nodeCount);
            nativeNodeData = new NativeArray<ManeuverNodeData>(nodeDataAlloc, Allocator.Persistent);
            for (int i = 0; i < nodeCount; i++)
                nativeNodeData[i] = JobTypeConversion.ToNodeData(flightPlan.Nodes[i]);

            double3 jupiterPos = JobTypeConversion.ToDouble3(universeManager.JupiterPosition);
            double jupiterSGP = universeManager.JupiterSGP;
            int planeMapping = universeManager.CurrentPlaneMapping == AstrodynamicPlaneMapping.UnityXyPlaneZUp ? 0 : 1;

            int moonCount = universeManager.MoonRailCount;
            nativeMoonOrbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.Persistent);
            if (moonCount > 0)
            {
                var tempOrbits = new MoonOrbitData[moonCount];
                universeManager.FillMoonOrbitData(tempOrbits, 0, moonCount, startTime);
                for (int i = 0; i < moonCount; i++)
                    nativeMoonOrbits[i] = tempOrbits[i];
            }

            double predictionSpan = endTime - startTime;
            double ephemerisStep = Math.Min(5.0 * 3600.0,
                Math.Max(60.0, predictionSpan / 20000.0));
            int ephemerisSampleCount = Math.Max(2, (int)(predictionSpan / ephemerisStep) + 1);
            ephemerisSampleCount = Math.Min(ephemerisSampleCount, 50000);

            nativeEphemerisTimes = new NativeArray<double>(ephemerisSampleCount, Allocator.Persistent);
            for (int i = 0; i < ephemerisSampleCount; i++)
                nativeEphemerisTimes[i] = startTime + i * ephemerisStep;
            nativeEphemerisTimes[ephemerisSampleCount - 1] = endTime;

            nativeEphemerisResults = new NativeArray<BodyState>(
                ephemerisSampleCount * moonCount, Allocator.Persistent);
            nativeEphemerisVelocities = new NativeArray<double3>(
                ephemerisSampleCount * moonCount, Allocator.Persistent);

            nativeTrajectoryOutput = new NativeArray<TrajectoryPoint>(
                adjustedMaxPoints, Allocator.Persistent);
            nativePointCount = new NativeReference<int>(0, Allocator.Persistent);
            nativeCalcStatus = new NativeReference<int>(0, Allocator.Persistent);

            if (moonCount > 0)
            {
                var moonJob = new MoonEphemerisJob
                {
                    SampleTimes = nativeEphemerisTimes,
                    MoonOrbits = nativeMoonOrbits,
                    Results = nativeEphemerisResults,
                    JupiterPosition = jupiterPos,
                    PlaneMapping = planeMapping
                };
                ephemerisJobHandle = moonJob.Schedule(ephemerisSampleCount, 64);

                var velJob = new EphemerisVelocityJob
                {
                    SampleTimes = nativeEphemerisTimes,
                    MoonStates = nativeEphemerisResults,
                    Velocities = nativeEphemerisVelocities,
                    MoonCount = moonCount
                };
                ephemerisJobHandle = velJob.Schedule(ephemerisSampleCount, 64, ephemerisJobHandle);
            }
            else
            {
                ephemerisJobHandle = default;
            }

            var trajectoryJob = new FullTrajectoryJob
            {
                Nodes = nativeNodeData,
                MoonEphemeris = nativeEphemerisResults,
                EphemerisTimes = nativeEphemerisTimes,
                MoonVelocities = nativeEphemerisVelocities,
                MoonCount = moonCount,
                PlaneMapping = planeMapping,

                StartPos = startPos,
                StartVel = startVel,
                StartTime = startTime,

                JupiterPosition = jupiterPos,
                JupiterSGP = jupiterSGP,

                MajorStepSeconds = majorStep,
                SubstepLimitSeconds = substepLimit,
                MaxSubstepsPerSegment = maxSubsteps,
                MaxPoints = adjustedMaxPoints,
                MaxStepsPerSegment = (int)Math.Min(
                    ((long)(effectivePrediction / Math.Max(1e-6, majorStep)) + 1) * maxSubsteps + 100000,
                    int.MaxValue / 2),

                PredictionLengthSeconds = requestedPrediction,
                MaxPredictionLengthSeconds = maxPredictionLengthSeconds,

                OutputPoints = nativeTrajectoryOutput,
                PointCount = nativePointCount,
                CalculationStatus = nativeCalcStatus
            };

            trajectoryJobHandle = trajectoryJob.Schedule(ephemerisJobHandle);
            JobHandle.ScheduleBatchedJobs();
            isJobRunning = true;
        }

        private void CompleteJobAndSwap()
        {
            if (!isJobRunning) return;

            trajectoryJobHandle.Complete();

            int count = nativePointCount.Value;
            int status = nativeCalcStatus.Value;

            if (count > 0 && status == 1)
            {
                InitializeBackBuffer(count);

                Vector3d firstFramePos = Vector3d.Zero;
                Vector3d firstFrameVel = Vector3d.Zero;
                if (count > 0)
                {
                    double firstTime = nativeTrajectoryOutput[0].Time;
                    TryUpdateFrameState(ref firstFramePos, ref firstFrameVel, firstTime);
                }

                for (int i = 0; i < count && i < backBufferPoints.Length; i++)
                {
                    var pt = nativeTrajectoryOutput[i];
                    Vector3d absPos = JobTypeConversion.ToVector3d(pt.Position);

                    Vector3d framePos = firstFramePos;
                    Vector3d frameVel = firstFrameVel;
                    TryUpdateFrameState(ref framePos, ref frameVel, pt.Time);

                    Vector3d relPos = absPos - framePos;
                    backBufferPoints[i] = universeManager.ToUnityOffset(relPos);
                    backBufferTimes[i] = pt.Time;
                    backBufferIsDashed[i] = pt.IsDashed != 0;

                    fullTrajectoryPoints.Add(absPos);
                    fullTrajectoryTimes.Add(pt.Time);
                }
                backBufferCount = count;

                CompleteBackBuffer(count);
            }

            DisposeJobResources();
            isJobRunning = false;
            UpdateVisibility();
        }

        private void DisposeJobResources()
        {
            if (nativeMoonOrbits.IsCreated) nativeMoonOrbits.Dispose();
            if (nativeNodeData.IsCreated) nativeNodeData.Dispose();
            if (nativeEphemerisTimes.IsCreated) nativeEphemerisTimes.Dispose();
            if (nativeEphemerisResults.IsCreated) nativeEphemerisResults.Dispose();
            if (nativeEphemerisVelocities.IsCreated) nativeEphemerisVelocities.Dispose();
            if (nativeTrajectoryOutput.IsCreated) nativeTrajectoryOutput.Dispose();
            if (nativePointCount.IsCreated) nativePointCount.Dispose();
            if (nativeCalcStatus.IsCreated) nativeCalcStatus.Dispose();
        }

        private void CompleteAndDisposeJobs()
        {
            if (isJobRunning)
            {
                trajectoryJobHandle.Complete();
            }
            DisposeJobResources();
            isJobRunning = false;
        }

        private bool TryUpdateFrameState(ref Vector3d framePos, ref Vector3d frameVel, double time)
        {
            if (universeManager.TryGetReferenceStateAtTime(
                lockedReferenceFrame, time,
                out _, out framePos, out frameVel,
                out _, out _, out _))
            {
                return true;
            }
            return false;
        }

        private void InitializeBackBuffer(int capacity)
        {
            backBufferPoints = new Vector3[capacity];
            backBufferTimes = new double[capacity];
            backBufferIsDashed = new bool[capacity];
            backBufferCount = 0;
        }

        private void CompleteBackBuffer(int count)
        {
            if (count == 0) return;

            int validCount = Math.Min(count, backBufferPoints.Length);
            if (validCount == 0) return;

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

            int totalLines = 0;
            foreach (var run in runs)
            {
                int runLength = run.end - run.start + 1;
                totalLines += (runLength + maxPointsPerLine - 1) / maxPointsPerLine;
            }

            EnsureLineCount(totalLines);

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
                    if (positionsBuffer.Length < pointsInLine)
                        positionsBuffer = new Vector3[pointsInLine];

                    for (int i = 0; i < pointsInLine; i++)
                    {
                        positionsBuffer[i] = backBufferPoints[run.start + l * maxPointsPerLine + i];
                    }

                    line.SetPositions(positionsBuffer);
                    line.material = run.isDashed
                        ? (dashedMaterial ?? CreateDefaultDashedMaterial())
                        : (solidMaterial ?? CreateDefaultSolidMaterial());
                    line.gameObject.SetActive(true);
                    lineIdx++;
                }
            }

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
                    timeMarkerInstance.transform.localScale = Vector3.one * markerScale;
            }
        }

        private Transform GetTrajectoryParent()
        {
            Transform parent = null;
            if (universeManager != null && universeManager.TrajectoryVisualRoot != null)
                parent = universeManager.TrajectoryVisualRoot;
            else
                parent = transform;
            return parent;
        }

        private void ClearLines()
        {
            foreach (var line in segmentLines)
            {
                if (line != null && line.gameObject != null)
                    UnityEngine.Object.Destroy(line.gameObject);
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

        public FlightPlan GetFlightPlan() => flightPlan;

        private static int ResolveTrajectoryLayer()
        {
            int layer = LayerMask.NameToLayer("Trajectory");
            return layer >= 0 ? layer : 0;
        }
    }
}

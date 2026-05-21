using System;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Core;
using Galilego.Simulation;
using Galilego.Universe;
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
        private List<bool> segmentIsDashed = new List<bool>();
        private LineRenderer ballisticLine;

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
        private List<bool> fullTrajectoryIsDashed = new List<bool>();

        private Vector3[] backBufferPoints;
        private double[] backBufferTimes;
        private bool[] backBufferIsDashed;
        private int backBufferCount;
        private int lastClipStartIdx = -1;

        private ReferenceFrameTarget lockedReferenceFrame;

        private Material solidMaterial;
        private Material dashedMaterial;
        private Material ballisticMaterial;
        private Vector3[] positionsBuffer = new Vector3[0];

        // ─── Moon prediction ────────────────────────────────────────────────────
        [Header("Moon Prediction")]
        [SerializeField] private int moonPredictionSamples = 64;

        private class MoonPredictionCache
        {
            public LineRenderer Line;
            public GameObject Marker;
            public Material MarkerMaterial;
            public double CachedEndTime = double.MinValue;
            public int CachedMoonCount = -1;
            public Vector3[] LocalPositionsBuffer;
            public bool ColorDirty = true;
        }

        private List<MoonPredictionCache> moonPredictionCaches = new List<MoonPredictionCache>();
        private Transform moonPredictionRoot;
        private bool moonPredictionNeedsRebuild = true;

        private FlightPlan flightPlan = new FlightPlan();
        private bool isDirty = false;
        private float dirtyTimer = 0f;

        private JobHandle ephemerisJobHandle;
        private JobHandle trajectoryJobHandle;
        private bool isJobRunning = false;

        // Cache to skip recalculation when inputs haven't changed
        private double3? cachedStartPos;
        private double3? cachedStartVel;
        private double? cachedStartTime;
        private double? cachedPredictionLength;
        private ReferenceFrameTarget cachedReferenceFrame = ReferenceFrameTarget.Jupiter;
        private List<ManeuverNodeData> cachedNodeData = new List<ManeuverNodeData>();

        // Cached marker position for use during scrubbing (when fullTrajectoryPoints is cleared)
        private Vector3d cachedMarkerPos;
        private double cachedMarkerTime;
        private bool hasCachedMarkerPos;

        // Segment-level cache: per-boundary states for partial recalculation
        private List<SegmentBoundaryState> cachedBoundaries = new List<SegmentBoundaryState>();
        private bool hasPartialCacheHit = false;
        private int partialStartSegment = 0;
        private double3 cachedPartialStartPos;
        private double3 cachedPartialStartVel;
        private double cachedPartialStartTime;

        private NativeArray<MoonOrbitData> nativeMoonOrbits;
        private NativeArray<ManeuverNodeData> nativeNodeData;
        private NativeArray<ManeuverNodeData> nativeBallisticNodeData;
        private NativeArray<double> nativeEphemerisTimes;
        private NativeArray<BodyState> nativeEphemerisResults;
        private NativeArray<double3> nativeEphemerisVelocities;
        private NativeArray<TrajectoryPoint> nativeTrajectoryOutput;
        private NativeArray<TrajectoryPoint> nativeBallisticOutput;
        private NativeArray<SegmentBoundaryState> nativeBoundaries;
        private NativeReference<int> nativePointCount;
        private NativeReference<int> nativeBallisticPointCount;
        private NativeReference<int> nativeBoundaryCount;
        private NativeReference<int> nativeCalcStatus;
        private NativeReference<int> nativeBallisticCalcStatus;

        // Ballistic job (no maneuvers) needs its own boundary containers
        private NativeArray<SegmentBoundaryState> nativeBallisticBoundaries;
        private NativeReference<int> nativeBallisticBoundaryCount;

        private Vector3[] ballisticPositions;
        private double[] ballisticTimesData;
        private int ballisticCount;

        private void OnEnable()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();

            if (universeManager != null)
            {
                universeManager.ActiveReferenceFrameChanged -= HandleActiveReferenceFrameChanged;
                universeManager.ActiveReferenceFrameChanged += HandleActiveReferenceFrameChanged;
            }
        }

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

        private void OnDisable()
        {
            if (universeManager != null)
                universeManager.ActiveReferenceFrameChanged -= HandleActiveReferenceFrameChanged;

            foreach (var line in segmentLines)
            {
                if (line != null && line.gameObject != null)
                    line.gameObject.SetActive(false);
            }
            HideMoonPredictionVisuals();
        }

        private void OnDestroy()
        {
            CompleteAndDisposeJobs();
            ClearLines();
            ClearMoonPredictionVisuals();
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
                    if (ScrubbingActive)
                        skipClearOnNextRecalc = true;
                    RequestRecalculation();
                }
            }

            if (isJobRunning)
            {
                if (trajectoryJobHandle.IsCompleted)
                {
                    CompleteJobAndSwap();
                    // If scrubbing is still changing values, schedule next job
                    // without clearing lines (keeps old trajectory visible)
                    if (isDirty)
                    {
                        isDirty = false;
                        dirtyTimer = 0f;
                        skipClearOnNextRecalc = true;
                        hasPartialCacheHit = false;
                        fullTrajectoryPoints.Clear();
                        fullTrajectoryTimes.Clear();
                        fullTrajectoryIsDashed.Clear();
                        ScheduleJobs();
                        UpdateCache();
                    }
                }
            }

            UpdateTrajectoryClip();
            UpdateVisibility();
        }

        private void LateUpdate()
        {
            UpdateMarkerPosition();
            UpdateMoonPredictionVisuals();
            ShrinkPassedSegments();
        }

        private void ShrinkPassedSegments()
        {
            double simTime = universeManager.SimulationTimeSeconds;

            // Clip maneuver trajectory segments
            if (backBufferTimes != null && backBufferCount >= 2)
            {
                int newStartIdx = Array.BinarySearch(backBufferTimes, 0, backBufferCount, simTime);
                if (newStartIdx < 0) newStartIdx = ~newStartIdx;
                if (newStartIdx >= backBufferCount) newStartIdx = backBufferCount - 1;

                if (newStartIdx != lastClipStartIdx)
                {
                    CompleteBackBuffer(backBufferCount);
                }
            }

            // Clip ballistic prediction line
            if (ballisticTimesData != null && ballisticCount >= 2 && ballisticLine != null)
            {
                int bStart = Array.BinarySearch(ballisticTimesData, 0, ballisticCount, simTime);
                if (bStart < 0) bStart = ~bStart;
                if (bStart >= ballisticCount) bStart = ballisticCount - 1;

                int bRemaining = ballisticCount - bStart;
                if (bRemaining < 2)
                {
                    ballisticLine.positionCount = 0;
                }
                else
                {
                    var clipped = new Vector3[bRemaining];
                    Array.Copy(ballisticPositions, bStart, clipped, 0, bRemaining);
                    ballisticLine.positionCount = bRemaining;
                    ballisticLine.SetPositions(clipped);
                }
            }
        }

        private void UpdateTrajectoryClip()
        {
            if (backBufferTimes == null || backBufferCount < 2) return;

            double simTime = universeManager.SimulationTimeSeconds;
            int newStartIdx = Array.BinarySearch(backBufferTimes, 0, backBufferCount, simTime);
            if (newStartIdx < 0) newStartIdx = ~newStartIdx;
            if (newStartIdx >= backBufferCount) newStartIdx = backBufferCount - 1;

            if (newStartIdx != lastClipStartIdx)
            {
                CompleteBackBuffer(backBufferCount);
            }
        }

        private void UpdateVisibility()
        {
            if (universeManager == null) return;

            bool inOrbitMap = universeManager.CameraMode == SpaceCameraMode.OrbitMap;
            bool hasPrediction = universeManager.TrajectoryPreviewEndTime > universeManager.SimulationTimeSeconds + 0.5;
            bool hasNodes = flightPlan != null && flightPlan.Nodes.Count > 0;

            // Ballistic prediction line (purple, no maneuvers): shown when slider is active
            bool showBallistic = inOrbitMap && hasPrediction;
            if (ballisticLine != null && ballisticLine.gameObject.activeSelf != showBallistic)
                ballisticLine.gameObject.SetActive(showBallistic);
            if (showBallistic && ballisticLine != null)
            {
                float w = universeManager.ResolveWorldLineWidthForPixels(2.25f, 0.02f);
                ballisticLine.startWidth = w;
                ballisticLine.endWidth = w;
                ballisticLine.alignment = LineAlignment.View;
                ballisticLine.numCornerVertices = 6;
                ballisticLine.numCapVertices = 6;
            }

            // Maneuver trajectory segments: hide solid (replaced by ballistic line), show dashed burns only
            bool showSolid = false;
            bool showDashed = inOrbitMap && hasNodes;

            for (int li = 0; li < segmentLines.Count; li++)
            {
                var line = segmentLines[li];
                if (line != null && line.positionCount > 0)
                {
                    bool isDashed = li < segmentIsDashed.Count && segmentIsDashed[li];
                    bool showThis = isDashed ? showDashed : showSolid;
                    if (line.gameObject.activeSelf != showThis)
                        line.gameObject.SetActive(showThis);
                }

                if (line != null && line.positionCount > 1)
                {
                    bool isDashed = li < segmentIsDashed.Count && segmentIsDashed[li];
                    bool showWidth = isDashed ? showDashed : showSolid;
                    if (showWidth)
                    {
                        float width = universeManager.ResolveWorldLineWidthForPixels(2.25f, 0.02f);
                        line.startWidth = width;
                        line.endWidth = width;
                        line.alignment = LineAlignment.View;
                        line.numCornerVertices = 6;
                        line.numCapVertices = 6;
                    }
                }
            }

            bool showAny = showSolid || showDashed;
            if (timeMarkerInstance != null)
            {
                if (timeMarkerInstance.activeSelf != showAny)
                    timeMarkerInstance.SetActive(showAny);
            }

            foreach (var cache in moonPredictionCaches)
            {
                if (cache.Line != null && cache.Line.gameObject.activeSelf != showAny)
                    cache.Line.gameObject.SetActive(showAny);
                if (cache.Marker != null && cache.Marker.activeSelf != showAny)
                    cache.Marker.SetActive(showAny);
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
            if (!isDirty)
                dirtyTimer = 0f;
            isDirty = true;
        }

        private void HandleActiveReferenceFrameChanged(ReferenceFrameTarget _)
        {
            MarkAsDirty();
        }

        private bool MatchesCachedInput()
        {
            if (!cachedStartPos.HasValue || !cachedStartVel.HasValue || !cachedStartTime.HasValue)
                return false;

            if (universeManager.ActiveReferenceFrame != cachedReferenceFrame)
                return false;

            double3 pos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
            double3 vel = JobTypeConversion.ToDouble3(universeManager.ShipBody.Velocity);
            double time = universeManager.SimulationTimeSeconds;

            if (math.distance(pos, cachedStartPos.Value) > 0.001) return false;
            if (math.distance(vel, cachedStartVel.Value) > 0.001) return false;
            if (math.abs(time - cachedStartTime.Value) > 0.001) return false;

            double requestedPrediction = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;
            if (cachedPredictionLength.HasValue &&
                math.abs(requestedPrediction - cachedPredictionLength.Value) > 0.5)
                return false;

            int nodeCount = flightPlan.Nodes.Count;
            if (nodeCount != cachedNodeData.Count) return false;

            for (int i = 0; i < nodeCount; i++)
            {
                var cur = JobTypeConversion.ToNodeData(flightPlan.Nodes[i]);
                var cached = cachedNodeData[i];
                if (math.abs(cur.StartTime - cached.StartTime) > 0.01) return false;
                if (math.abs(cur.DvPrograde - cached.DvPrograde) > 0.001) return false;
                if (math.abs(cur.DvNormal - cached.DvNormal) > 0.001) return false;
                if (math.abs(cur.DvRadial - cached.DvRadial) > 0.001) return false;
                if (cur.IsInstant != cached.IsInstant) return false;
                if (cur.HasEngine != cached.HasEngine) return false;
            }

            return true;
        }

        private void UpdateCache()
        {
            cachedStartPos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
            cachedStartVel = JobTypeConversion.ToDouble3(universeManager.ShipBody.Velocity);
            cachedStartTime = universeManager.SimulationTimeSeconds;
            cachedPredictionLength = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;

            cachedReferenceFrame = lockedReferenceFrame;

            cachedNodeData.Clear();
            for (int i = 0; i < flightPlan.Nodes.Count; i++)
                cachedNodeData.Add(JobTypeConversion.ToNodeData(flightPlan.Nodes[i]));
        }

        private bool skipClearOnNextRecalc;
        public bool ScrubbingActive { get; set; }

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

            // Skip recalculation if inputs haven't changed meaningfully
            if (MatchesCachedInput())
            {
                isDirty = false;
                return;
            }

            // Try partial recalc: only recompute affected suffix of trajectory
            hasPartialCacheHit = false;
            TryFindPartialRestartPoint();

            CompleteAndDisposeJobs();
            if (!skipClearOnNextRecalc)
                ClearLines();
            skipClearOnNextRecalc = false;

            if (hasPartialCacheHit)
            {
                TrimTrajectoryToBoundary();
            }
            else
            {
                fullTrajectoryPoints.Clear();
                fullTrajectoryTimes.Clear();
                fullTrajectoryIsDashed.Clear();
            }

            lockedReferenceFrame = universeManager.ActiveReferenceFrame;

            ScheduleJobs();

            if (forceSynchronousCalculation)
            {
                CompleteJobAndSwap();
            }
        }

        private void TryFindPartialRestartPoint()
        {
            hasPartialCacheHit = false;

            if (cachedBoundaries.Count == 0) return;

            // Ship must be at the same start state
            double3 pos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
            double3 vel = JobTypeConversion.ToDouble3(universeManager.ShipBody.Velocity);
            if (math.distance(pos, cachedBoundaries[0].Position) > 0.001) return;
            if (math.distance(vel, cachedBoundaries[0].Velocity) > 0.001) return;

            double requestedPrediction = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;
            if (cachedPredictionLength.HasValue &&
                math.abs(requestedPrediction - cachedPredictionLength.Value) > 0.5) return;

            int nodeCount = flightPlan.Nodes.Count;
            if (nodeCount != cachedNodeData.Count) return;

            // Find first changed node (by idx, sorted by time)
            int changedIdx = -1;
            for (int i = 0; i < nodeCount; i++)
            {
                var cur = JobTypeConversion.ToNodeData(flightPlan.Nodes[i]);
                var cached = cachedNodeData[i];
                if (math.abs(cur.StartTime - cached.StartTime) > 0.01)
                {
                    // Time changed — sort order might have changed, can't partial restart
                    return;
                }
                if (math.abs(cur.DvPrograde - cached.DvPrograde) > 0.001 ||
                    math.abs(cur.DvNormal - cached.DvNormal) > 0.001 ||
                    math.abs(cur.DvRadial - cached.DvRadial) > 0.001 ||
                    cur.IsInstant != cached.IsInstant ||
                    cur.HasEngine != cached.HasEngine)
                {
                    changedIdx = i;
                    break;
                }
            }

            if (changedIdx <= 0) return;

            // We have a cache hit — start from cachedBoundaries[changedIdx]
            // which is state AFTER node[changedIdx-1]'s Δv
            int boundaryIdx = Math.Min(changedIdx, cachedBoundaries.Count - 1);
            var boundary = cachedBoundaries[boundaryIdx];
            hasPartialCacheHit = true;
            partialStartSegment = changedIdx;
            cachedPartialStartPos = boundary.Position;
            cachedPartialStartVel = boundary.Velocity;
            cachedPartialStartTime = boundary.Time;
        }

        private void TrimTrajectoryToBoundary()
        {
            if (fullTrajectoryPoints.Count == 0) return;

            double boundaryTime = cachedPartialStartTime;
            int trimIdx = -1;
            for (int i = 0; i < fullTrajectoryTimes.Count; i++)
            {
                if (fullTrajectoryTimes[i] >= boundaryTime - 0.5 && fullTrajectoryTimes[i] <= boundaryTime + 0.5)
                {
                    trimIdx = i;
                    break;
                }
            }

            if (trimIdx >= 0 && trimIdx + 1 < fullTrajectoryPoints.Count)
            {
                int keepCount = trimIdx;
                fullTrajectoryPoints.RemoveRange(keepCount, fullTrajectoryPoints.Count - keepCount);
                fullTrajectoryTimes.RemoveRange(keepCount, fullTrajectoryTimes.Count - keepCount);
                fullTrajectoryIsDashed.RemoveRange(keepCount, fullTrajectoryIsDashed.Count - keepCount);
            }
        }

        private void ScheduleJobs()
        {
            flightPlan.Nodes.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));

            // Always compute endTime from the original ship start time
            double baseStartTime = universeManager.SimulationTimeSeconds;
            double requestedPrediction = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;
            double effectivePrediction = Math.Min(requestedPrediction, maxPredictionLengthSeconds);
            double endTime = baseStartTime + effectivePrediction;

            // Determine start state — either fresh or from cache
            double3 startPos;
            double3 startVel;
            double startTime;
            int nodeCount = flightPlan.Nodes.Count;
            int jobNodeOffset = 0;

            if (hasPartialCacheHit)
            {
                startPos = cachedPartialStartPos;
                startVel = cachedPartialStartVel;
                startTime = cachedPartialStartTime;
                jobNodeOffset = partialStartSegment;
            }
            else
            {
                startPos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
                startVel = JobTypeConversion.ToDouble3(universeManager.ShipBody.Velocity);
                startTime = baseStartTime;
            }

            double remainingPrediction = Math.Max(0.0, endTime - startTime);

            int jobNodeCount = nodeCount - jobNodeOffset;
            int adjustedMaxPoints = Math.Max(maxTrajectoryPoints,
                (int)(effectivePrediction / 600.0) + 2);
            double adaptiveStep = effectivePrediction / Math.Max(1, (int)(adjustedMaxPoints * 0.9));
            double majorStep = Math.Max(1e-6d, Math.Max(predictionStepSeconds, adaptiveStep));
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);
            int maxSubsteps = flightPlan.MaxStepsPerSegment > 0
                ? flightPlan.MaxStepsPerSegment
                : maxSubstepsPerSegment;

            int nodeDataAlloc = Math.Max(1, jobNodeCount);
            nativeNodeData = new NativeArray<ManeuverNodeData>(nodeDataAlloc, Allocator.Persistent);
            for (int i = 0; i < jobNodeCount; i++)
                nativeNodeData[i] = JobTypeConversion.ToNodeData(flightPlan.Nodes[jobNodeOffset + i]);

            nativeBallisticNodeData = new NativeArray<ManeuverNodeData>(0, Allocator.Persistent);

            double3 jupiterPos = JobTypeConversion.ToDouble3(universeManager.JupiterPosition);
            double jupiterSGP = universeManager.JupiterSGP;
            int planeMapping = universeManager.CurrentPlaneMapping == AstrodynamicPlaneMapping.UnityXyPlaneZUp ? 0 : 1;

            int moonCount = universeManager.MoonRailCount;
            nativeMoonOrbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.Persistent);
            if (moonCount > 0)
            {
                var tempOrbits = new MoonOrbitData[moonCount];
                universeManager.FillMoonOrbitData(tempOrbits, 0, moonCount, baseStartTime);
                for (int i = 0; i < moonCount; i++)
                    nativeMoonOrbits[i] = tempOrbits[i];
            }

            double predictionSpan = endTime - baseStartTime;
            double ephemerisStep = Math.Min(5.0 * 3600.0,
                Math.Max(60.0, predictionSpan / 20000.0));
            int ephemerisSampleCount = Math.Max(2, (int)(predictionSpan / ephemerisStep) + 1);
            ephemerisSampleCount = Math.Min(ephemerisSampleCount, 50000);

            nativeEphemerisTimes = new NativeArray<double>(ephemerisSampleCount, Allocator.Persistent);
            for (int i = 0; i < ephemerisSampleCount; i++)
                nativeEphemerisTimes[i] = baseStartTime + i * ephemerisStep;
            nativeEphemerisTimes[ephemerisSampleCount - 1] = endTime;

            nativeEphemerisResults = new NativeArray<BodyState>(
                ephemerisSampleCount * moonCount, Allocator.Persistent);
            nativeEphemerisVelocities = new NativeArray<double3>(
                ephemerisSampleCount * moonCount, Allocator.Persistent);

            nativeTrajectoryOutput = new NativeArray<TrajectoryPoint>(
                adjustedMaxPoints, Allocator.Persistent);
            nativeBallisticOutput = new NativeArray<TrajectoryPoint>(
                adjustedMaxPoints, Allocator.Persistent);
            nativePointCount = new NativeReference<int>(0, Allocator.Persistent);
            nativeBallisticPointCount = new NativeReference<int>(0, Allocator.Persistent);
            nativeBoundaries = new NativeArray<SegmentBoundaryState>(
                nodeCount + 2, Allocator.Persistent);
            nativeBoundaryCount = new NativeReference<int>(0, Allocator.Persistent);
            nativeCalcStatus = new NativeReference<int>(0, Allocator.Persistent);
            nativeBallisticCalcStatus = new NativeReference<int>(0, Allocator.Persistent);

            nativeBallisticBoundaries = new NativeArray<SegmentBoundaryState>(1, Allocator.Persistent);
            nativeBallisticBoundaryCount = new NativeReference<int>(0, Allocator.Persistent);

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

            var ballisticTrajectoryJob = new FullTrajectoryJob
            {
                Nodes = nativeBallisticNodeData, // empty — no maneuvers, pure ballistic
                MoonEphemeris = nativeEphemerisResults,
                EphemerisTimes = nativeEphemerisTimes,
                MoonVelocities = nativeEphemerisVelocities,
                MoonCount = moonCount,
                PlaneMapping = planeMapping,
                ReferenceFrameIndex = (int)lockedReferenceFrame,

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

                PredictionLengthSeconds = remainingPrediction > 0.0 ? remainingPrediction : requestedPrediction,
                MaxPredictionLengthSeconds = maxPredictionLengthSeconds,

                OutputPoints = nativeBallisticOutput,
                PointCount = nativeBallisticPointCount,
                CalculationStatus = nativeBallisticCalcStatus,

                SegmentBoundaries = nativeBallisticBoundaries,
                SegmentBoundaryCount = nativeBallisticBoundaryCount
            };

            var ballisticHandle = ballisticTrajectoryJob.Schedule(ephemerisJobHandle);

            var trajectoryJob = new FullTrajectoryJob
            {
                Nodes = nativeNodeData,
                MoonEphemeris = nativeEphemerisResults,
                EphemerisTimes = nativeEphemerisTimes,
                MoonVelocities = nativeEphemerisVelocities,
                MoonCount = moonCount,
                PlaneMapping = planeMapping,
                ReferenceFrameIndex = (int)lockedReferenceFrame,

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

                PredictionLengthSeconds = remainingPrediction > 0.0 ? remainingPrediction : requestedPrediction,
                MaxPredictionLengthSeconds = maxPredictionLengthSeconds,

                OutputPoints = nativeTrajectoryOutput,
                PointCount = nativePointCount,
                CalculationStatus = nativeCalcStatus,

                SegmentBoundaries = nativeBoundaries,
                SegmentBoundaryCount = nativeBoundaryCount
            };

            trajectoryJobHandle = trajectoryJob.Schedule(ballisticHandle);
            JobHandle.ScheduleBatchedJobs();
            isJobRunning = true;

            UpdateCache();
        }

        private void CompleteJobAndSwap()
        {
            if (!isJobRunning) return;

            trajectoryJobHandle.Complete();

            // Read boundary states from job, mapping to global indices
            int boundaryCount = nativeBoundaryCount.IsCreated ? nativeBoundaryCount.Value : 0;
            if (nativeBoundaries.IsCreated)
            {
                if (hasPartialCacheHit)
                {
                    // Replace suffix: keep 0..partialStartSegment-1, replace from partialStartSegment
                    int replaceIdx = partialStartSegment;
                    for (int i = 0; i < boundaryCount && i < nativeBoundaries.Length; i++)
                    {
                        int globalIdx = replaceIdx + i;
                        while (cachedBoundaries.Count <= globalIdx)
                            cachedBoundaries.Add(default);
                        cachedBoundaries[globalIdx] = nativeBoundaries[i];
                    }
                    // Trim stale entries beyond new boundary count
                    int totalGlobal = replaceIdx + boundaryCount;
                    if (cachedBoundaries.Count > totalGlobal)
                        cachedBoundaries.RemoveRange(totalGlobal, cachedBoundaries.Count - totalGlobal);
                }
                else
                {
                    // Full recalc — replace entirely
                    cachedBoundaries.Clear();
                    for (int i = 0; i < boundaryCount && i < nativeBoundaries.Length; i++)
                        cachedBoundaries.Add(nativeBoundaries[i]);
                }
            }

            // In partial recalc, fill backBuffer with cached prefix + new suffix
            int count = nativePointCount.Value;
            int status = nativeCalcStatus.Value;

            int existingPointCount = fullTrajectoryPoints.Count;
            int totalPoints = existingPointCount + count;

            if (count > 0 && status == 1)
            {
                InitializeBackBuffer(totalPoints);

                Vector3d firstFramePos = Vector3d.Zero;
                Vector3d firstFrameVel = Vector3d.Zero;

                // Fill cached prefix (points before the changed boundary)
                int bufIdx = 0;
                for (int i = 0; i < existingPointCount && bufIdx < backBufferPoints.Length; i++)
                {
                    double ptTime = fullTrajectoryTimes[i];
                    Vector3d absPos = fullTrajectoryPoints[i];
                    TryUpdateFrameState(ref firstFramePos, ref firstFrameVel, ptTime);
                    Vector3d relPos = absPos - firstFramePos;
                    backBufferPoints[bufIdx] = universeManager.ToUnityOffset(relPos);
                    backBufferTimes[bufIdx] = ptTime;
                    backBufferIsDashed[bufIdx] = i < fullTrajectoryIsDashed.Count && fullTrajectoryIsDashed[i];
                    bufIdx++;
                }

                // Fill new suffix (from job output), skip first point if it duplicates last cached point
                int newStartIdx = 0;
                if (existingPointCount > 0 && count > 0)
                {
                    double lastCachedTime = fullTrajectoryTimes[existingPointCount - 1];
                    double firstNewTime = nativeTrajectoryOutput[0].Time;
                    if (Math.Abs(lastCachedTime - firstNewTime) < 0.5)
                        newStartIdx = 1;
                }

                for (int i = newStartIdx; i < count && bufIdx < backBufferPoints.Length; i++)
                {
                    var pt = nativeTrajectoryOutput[i];
                    Vector3d absPos = JobTypeConversion.ToVector3d(pt.Position);
                    TryUpdateFrameState(ref firstFramePos, ref firstFrameVel, pt.Time);
                    Vector3d relPos = absPos - firstFramePos;
                    backBufferPoints[bufIdx] = universeManager.ToUnityOffset(relPos);
                    backBufferTimes[bufIdx] = pt.Time;
                    backBufferIsDashed[bufIdx] = pt.IsDashed != 0;

                    fullTrajectoryPoints.Add(absPos);
                    fullTrajectoryTimes.Add(pt.Time);
                    fullTrajectoryIsDashed.Add(pt.IsDashed != 0);
                    bufIdx++;
                }
                backBufferCount = bufIdx;

                CompleteBackBuffer(backBufferCount);
            }

            // Process ballistic trajectory (no maneuvers) for the purple prediction line
            int ballisticCountJob = nativeBallisticPointCount.IsCreated ? nativeBallisticPointCount.Value : 0;
            int ballisticStatus = nativeBallisticCalcStatus.IsCreated ? nativeBallisticCalcStatus.Value : 0;
            if (ballisticCountJob > 0 && ballisticStatus == 1 && nativeBallisticOutput.IsCreated)
            {
                int bCount = Math.Min(ballisticCountJob, nativeBallisticOutput.Length);
                ballisticPositions = new Vector3[bCount];
                ballisticTimesData = new double[bCount];
                Vector3d framePos = Vector3d.Zero, frameVel = Vector3d.Zero;
                for (int i = 0; i < bCount; i++)
                {
                    var pt = nativeBallisticOutput[i];
                    TryUpdateFrameState(ref framePos, ref frameVel, pt.Time);
                    Vector3d absPos = JobTypeConversion.ToVector3d(pt.Position);
                    Vector3d relPos = absPos - framePos;
                    ballisticPositions[i] = universeManager.ToUnityOffset(relPos);
                    ballisticTimesData[i] = pt.Time;
                }
                ballisticCount = bCount;
                UpdateBallisticLine();
            }

            DisposeJobResources();
            isJobRunning = false;
            // Cache last trajectory end point for marker during scrubbing
            hasCachedMarkerPos = fullTrajectoryPoints.Count > 0;
            if (hasCachedMarkerPos)
            {
                cachedMarkerPos = fullTrajectoryPoints[fullTrajectoryPoints.Count - 1];
                cachedMarkerTime = fullTrajectoryTimes[fullTrajectoryTimes.Count - 1];
            }
            UpdateVisibility();
        }

        private void DisposeJobResources()
        {
            if (nativeMoonOrbits.IsCreated) nativeMoonOrbits.Dispose();
            if (nativeNodeData.IsCreated) nativeNodeData.Dispose();
            if (nativeBallisticNodeData.IsCreated) nativeBallisticNodeData.Dispose();
            if (nativeEphemerisTimes.IsCreated) nativeEphemerisTimes.Dispose();
            if (nativeEphemerisResults.IsCreated) nativeEphemerisResults.Dispose();
            if (nativeEphemerisVelocities.IsCreated) nativeEphemerisVelocities.Dispose();
            if (nativeTrajectoryOutput.IsCreated) nativeTrajectoryOutput.Dispose();
            if (nativeBallisticOutput.IsCreated) nativeBallisticOutput.Dispose();
            if (nativeBoundaries.IsCreated) nativeBoundaries.Dispose();
            if (nativePointCount.IsCreated) nativePointCount.Dispose();
            if (nativeBallisticPointCount.IsCreated) nativeBallisticPointCount.Dispose();
            if (nativeBoundaryCount.IsCreated) nativeBoundaryCount.Dispose();
            if (nativeCalcStatus.IsCreated) nativeCalcStatus.Dispose();
            if (nativeBallisticCalcStatus.IsCreated) nativeBallisticCalcStatus.Dispose();
            if (nativeBallisticBoundaries.IsCreated) nativeBallisticBoundaries.Dispose();
            if (nativeBallisticBoundaryCount.IsCreated) nativeBallisticBoundaryCount.Dispose();
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

            // Skip past points: only show future trajectory ahead of current ship time
            double simTime = universeManager.SimulationTimeSeconds;
            int startIdx = 0;
            for (int i = 0; i < validCount; i++)
            {
                if (backBufferTimes[i] >= simTime)
                {
                    startIdx = i;
                    break;
                }
            }
            int remainingCount = validCount - startIdx;
            lastClipStartIdx = startIdx;
            if (remainingCount < 2) return;

            List<(int start, int end, bool isDashed)> runs = new List<(int, int, bool)>();
            int runStart = startIdx;
            for (int i = startIdx + 1; i < validCount; i++)
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
                    segmentIsDashed[lineIdx] = run.isDashed;
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
                if (i < segmentIsDashed.Count)
                    segmentIsDashed[i] = false;
            }
        }

        private Material CreateDefaultSolidMaterial()
        {
            Shader shader = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            solidMaterial = new Material(shader);
            solidMaterial.color = new Color(0.5f, 0.5f, 1f, 0.9f);
            return solidMaterial;
        }

        private Material CreateDefaultBallisticMaterial()
        {
            Shader shader = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            ballisticMaterial = new Material(shader);
            ballisticMaterial.color = new Color(0.5f, 0.5f, 1f, 0.9f);
            return ballisticMaterial;
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
                segmentIsDashed.Add(false);
            }
        }

        private void EnsureBallisticLine()
        {
            if (ballisticLine != null) return;
            GameObject obj = new GameObject("BallisticPrediction");
            obj.transform.SetParent(GetTrajectoryParent(), false);
            obj.layer = ResolveTrajectoryLayer();
            var lr = obj.AddComponent<LineRenderer>();
            lr.useWorldSpace = false;
            lr.startWidth = 0.2f;
            lr.endWidth = 0.2f;
            lr.alignment = LineAlignment.View;
            lr.numCornerVertices = 6;
            lr.numCapVertices = 6;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.material = ballisticMaterial ?? CreateDefaultBallisticMaterial();
            lr.enabled = true;
            ballisticLine = lr;
        }

        private void UpdateBallisticLine()
        {
            if (ballisticPositions == null || ballisticCount < 2)
            {
                if (ballisticLine != null) ballisticLine.positionCount = 0;
                return;
            }
            EnsureBallisticLine();
            ballisticLine.positionCount = ballisticCount;
            ballisticLine.SetPositions(ballisticPositions);
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

            if (!found && hasCachedMarkerPos)
            {
                pos = cachedMarkerPos;
                targetTime = cachedMarkerTime;
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
            segmentIsDashed.Clear();

            if (ballisticLine != null)
            {
                UnityEngine.Object.Destroy(ballisticLine.gameObject);
                ballisticLine = null;
            }
            ballisticPositions = null;
            ballisticCount = 0;
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

        public int GetRecommendedMaxSteps()
        {
            if (universeManager == null || flightPlan == null) return 0;
            double requestedPrediction = flightPlan.PredictionLengthSeconds > 0d
                ? flightPlan.PredictionLengthSeconds
                : defaultPredictionLengthSeconds;
            double effectivePrediction = Math.Min(requestedPrediction, maxPredictionLengthSeconds);
            int adjustedMaxPoints = Math.Max(maxTrajectoryPoints,
                (int)(effectivePrediction / 600.0) + 2);
            double adaptiveStep = effectivePrediction / Math.Max(1, (int)(adjustedMaxPoints * 0.9));
            double majorStep = Math.Min(
                Math.Max(1e-6d, Math.Max(predictionStepSeconds, adaptiveStep)),
                600.0);
            double substepLimit = ResolveSubstepLimitSeconds(majorStep);
            return (int)Math.Ceiling(majorStep / Math.Max(1e-9, substepLimit));
        }

        private static int ResolveTrajectoryLayer()
        {
            int layer = LayerMask.NameToLayer("Trajectory");
            return layer >= 0 ? layer : 0;
        }

        // ─── Moon prediction visuals ────────────────────────────────────────────

        private Transform GetMoonPredictionRoot()
        {
            if (moonPredictionRoot == null)
            {
                Transform parent = GetTrajectoryParent();
                GameObject rootObj = new GameObject("MoonPredictions");
                rootObj.transform.SetParent(parent, false);
                rootObj.transform.localPosition = Vector3.zero;
                rootObj.transform.localRotation = Quaternion.identity;
                rootObj.transform.localScale = Vector3.one;
                rootObj.layer = ResolveTrajectoryLayer();
                moonPredictionRoot = rootObj.transform;
            }
            return moonPredictionRoot;
        }

        private void EnsureMoonPredictionCaches(int count)
        {
            int currentCount = moonPredictionCaches.Count;

            if (currentCount == count && !moonPredictionNeedsRebuild)
                return;

            // Remove excess caches
            while (moonPredictionCaches.Count > count)
            {
                int idx = moonPredictionCaches.Count - 1;
                var cache = moonPredictionCaches[idx];
                if (cache.Line != null && cache.Line.gameObject != null)
                    UnityEngine.Object.Destroy(cache.Line.gameObject);
                if (cache.Marker != null)
                    UnityEngine.Object.Destroy(cache.Marker);
                if (cache.MarkerMaterial != null)
                    UnityEngine.Object.Destroy(cache.MarkerMaterial);
                moonPredictionCaches.RemoveAt(idx);
            }

            // Create missing caches
            while (moonPredictionCaches.Count < count)
            {
                int idx = moonPredictionCaches.Count;
                Transform root = GetMoonPredictionRoot();

                // Line renderer
                GameObject lineObj = new GameObject($"MoonPredictionLine_{idx}");
                lineObj.transform.SetParent(root, false);
                lineObj.transform.localPosition = Vector3.zero;
                lineObj.transform.localRotation = Quaternion.identity;
                lineObj.layer = ResolveTrajectoryLayer();
                LineRenderer line = lineObj.AddComponent<LineRenderer>();
                line.useWorldSpace = false;
                line.loop = false;
                line.positionCount = 0;
                line.startWidth = 0.1f;
                line.endWidth = 0.1f;
                line.alignment = LineAlignment.View;
                line.numCornerVertices = 4;
                line.numCapVertices = 4;
                line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                line.receiveShadows = false;
                line.gameObject.SetActive(false);

                // Marker sphere
                GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                marker.name = $"MoonPredictionMarker_{idx}";
                marker.transform.SetParent(root, false);
                marker.transform.localScale = Vector3.one * 0.4f;
                marker.layer = ResolveTrajectoryLayer();
                Renderer markerRenderer = marker.GetComponent<Renderer>();
                Material markerMat;
                if (markerRenderer.sharedMaterial != null)
                    markerMat = new Material(markerRenderer.sharedMaterial);
                else
                    markerMat = new Material(GetDefaultMarkerShader());
                markerMat.color = Color.white;
                markerRenderer.material = markerMat;
                marker.SetActive(false);

                var collision = marker.GetComponent<Collider>();
                if (collision != null) UnityEngine.Object.Destroy(collision);

                var cache = new MoonPredictionCache
                {
                    Line = line,
                    Marker = marker,
                    MarkerMaterial = markerMat,
                    CachedEndTime = double.MinValue,
                    CachedMoonCount = -1
                };
                moonPredictionCaches.Add(cache);
            }

            moonPredictionNeedsRebuild = false;
        }

        private static Shader GetDefaultMarkerShader()
        {
            return Shader.Find("Universal Render Pipeline/Unlit") ??
                   Shader.Find("Unlit/Color") ??
                   Shader.Find("Sprites/Default") ??
                   Shader.Find("Standard");
        }

        private void UpdateMoonPredictionVisuals()
        {
            if (universeManager == null || flightPlan == null)
            {
                HideMoonPredictionVisuals();
                return;
            }

            double simTime = universeManager.SimulationTimeSeconds;
            double endTime = universeManager.TrajectoryPreviewEndTime;

            if (endTime <= simTime + 0.5)
            {
                HideMoonPredictionVisuals();
                return;
            }

            int moonCount = universeManager.MoonCount;
            if (moonCount == 0)
            {
                HideMoonPredictionVisuals();
                return;
            }

            EnsureMoonPredictionCaches(moonCount);
            bool show = universeManager.CameraMode == SpaceCameraMode.OrbitMap;
            ReferenceFrameTarget frame = universeManager.ActiveReferenceFrame;
            int samples = Math.Max(2, moonPredictionSamples);

            float lineWidth = universeManager.ResolveWorldLineWidthForPixels(1.5f, 0.01f);
            float markerScale = universeManager.ResolveWorldLineWidthForPixels(4f, 0.008f);
            if (float.IsNaN(markerScale) || float.IsInfinity(markerScale) || markerScale <= 0.00001f)
                markerScale = 0.4f;

            // ── Precompute sample times and frame positions at each time ──────────
            // Same as spacecraft trajectory: each point uses framePos AT THAT TIME,
            // so (moonPos_t - framePos_t) creates realistic paths (loops etc.) in moving frames.
            double[] sampleTimes = new double[samples];
            Vector3d[] framePosAtTime = new Vector3d[samples];
            for (int j = 0; j < samples; j++)
            {
                sampleTimes[j] = simTime + (endTime - simTime) * (j / (double)(samples - 1));
                if (!universeManager.TryGetReferenceStateAtTime(
                        frame, sampleTimes[j],
                        out _, out framePosAtTime[j], out _,
                        out _, out _, out _))
                {
                    framePosAtTime[j] = universeManager.JupiterPosition;
                }
            }

            // Position root at frame pos at current simTime (same convention as moonOrbitRoot)
            universeManager.ApplyVisualPosition(GetMoonPredictionRoot(), framePosAtTime[0]);

            for (int i = 0; i < moonCount; i++)
            {
                var cache = moonPredictionCaches[i];

                // ── Material / color setup (only when slider changed) ────────────
                bool endTimeChanged = Math.Abs(endTime - cache.CachedEndTime) > 1.0;
                if (endTimeChanged || cache.ColorDirty)
                {
                    cache.CachedEndTime = endTime;
                    cache.ColorDirty = false;

                    Color moonColor = universeManager.GetMoonOrbitColor(i);
                    cache.MarkerMaterial.color = moonColor;

                    var lineMaterial = cache.Line.material;
                    bool isDefaultMat = lineMaterial == null ||
                                        lineMaterial.name == "Default-Material" ||
                                        lineMaterial.name.StartsWith("Default");
                    if (isDefaultMat)
                    {
                        lineMaterial = new Material(GetDefaultMarkerShader());
                        cache.Line.material = lineMaterial;
                    }
                    if (lineMaterial.HasProperty("_BaseColor"))
                        lineMaterial.SetColor("_BaseColor", moonColor);
                    else if (lineMaterial.HasProperty("_Color"))
                        lineMaterial.SetColor("_Color", moonColor);
                    else
                        lineMaterial.color = moonColor;

                    cache.Line.positionCount = samples;
                }

                // ── Compute moon positions and convert to frame-relative ─────────
                // Each point uses per-point frame position: moonPos_at_t - framePos_at_t
                // This mirrors how the spacecraft trajectory renders in CompleteBackBuffer
                // and produces realistic looping paths when reference frame moves.
                if (cache.Line != null)
                {
                    if (cache.LocalPositionsBuffer == null || cache.LocalPositionsBuffer.Length < samples)
                        cache.LocalPositionsBuffer = new Vector3[samples];

                    for (int j = 0; j < samples; j++)
                    {
                        if (universeManager.TryGetMoonPositionAtTime(i, sampleTimes[j], out Vector3d moonPos))
                        {
                            cache.LocalPositionsBuffer[j] = universeManager.ToUnityOffset(moonPos - framePosAtTime[j]);
                        }
                        else
                        {
                            cache.LocalPositionsBuffer[j] = Vector3.zero;
                        }
                    }

                    cache.Line.SetPositions(cache.LocalPositionsBuffer);
                    cache.Line.startWidth = lineWidth;
                    cache.Line.endWidth = lineWidth;
                    if (cache.Line.gameObject.activeSelf != show)
                        cache.Line.gameObject.SetActive(show);
                }

                // ── Marker at endTime with per-point frame position ──────────────
                if (cache.Marker != null)
                {
                    if (universeManager.TryGetMoonPositionAtTime(i, endTime, out Vector3d endMoonPos))
                    {
                        Vector3d endFramePos = framePosAtTime[samples - 1];
                        cache.Marker.transform.localPosition = universeManager.ToUnityOffset(endMoonPos - endFramePos);
                        cache.Marker.transform.localScale = Vector3.one * markerScale;
                    }
                    if (cache.Marker.activeSelf != show)
                        cache.Marker.SetActive(show);
                }
            }
        }

        private void HideMoonPredictionVisuals()
        {
            foreach (var cache in moonPredictionCaches)
            {
                if (cache.Line != null) cache.Line.gameObject.SetActive(false);
                if (cache.Marker != null) cache.Marker.SetActive(false);
            }
        }

        private void ClearMoonPredictionVisuals()
        {
            if (moonPredictionRoot != null)
            {
                UnityEngine.Object.Destroy(moonPredictionRoot.gameObject);
                moonPredictionRoot = null;
            }
            moonPredictionCaches.Clear();
            moonPredictionNeedsRebuild = true;
        }
    }
}

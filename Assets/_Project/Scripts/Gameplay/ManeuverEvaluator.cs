using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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

        [Header("Adaptive Integrator")]
        [SerializeField] private double integratorRelTol = 1e-8;
        [SerializeField] private double integratorAbsTol = 1e-1;
        [SerializeField] private double integratorMinStep = 0.1;
        [SerializeField] private double integratorMaxStep = 600.0;
        [SerializeField] private double integratorJupiterRadius = 7.0e7;
        [SerializeField] private double integratorMoonRadius = 2.0e6;

        // Tighter tolerances for close flyby / maneuver windows
        private const double FlybyRelTol = 1e-10;
        private const double FlybyAbsTol = 1e-2;
        private const double FlybyMinStep = 0.001;
        private const double FlybyMaxStep = 10.0;

        private const double ManeuverRelTol = 1e-9;
        private const double ManeuverAbsTol = 1e-3;
        private const double ManeuverMinStep = 0.01;
        private const double ManeuverMaxStep = 60.0;

        [Header("LOD Settings")]
        [SerializeField] private double lodNearTime = 86400.0;          // 24 часа
        [SerializeField] private double lodMidTime = 2592000.0;         // 30 дней
        [SerializeField] private double lodErrorTolerance = 0.01f;      // макс отклонение полилинии (Unity units)

        [Header("Moon Prediction Performance")]
        [SerializeField] private float moonVisRebuildIntervalReal = 0.5f;  // реальные секунды между пересчётами лунных орбит

        // Временные буферы для LOD
        private Vector3[] _lodPoints;
        private double[] _lodTimes;
        private bool[] _lodIsDashed;

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
        [SerializeField] private int moonPredictionSamplesPerOrbit = 96;

        // Reusable arrays to avoid per-frame GC allocations in UpdateMoonPredictionVisuals
        private double[] _moonSampleTimesCache = Array.Empty<double>();
        private Vector3d[] _moonFramePosCache = Array.Empty<Vector3d>();

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
        private float _moonVisLastRebuildRealTime = -999f;
        private float _moonPredLastRebuildRealTime = -999f;

        private FlightPlan flightPlan = new FlightPlan();
        private bool isDirty = false;
        private float dirtyTimer = 0f;

        private JobHandle ephemerisJobHandle;
        private JobHandle trajectoryJobHandle;
        private JobHandle ballisticJobHandle;
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
        private int cachedPartialStartVersion;

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

        // Checkpoints
        private NativeArray<TrajectoryCheckpoint> nativeMvrCheckpoints;
        private NativeArray<TrajectoryCheckpoint> nativeBalCheckpoints;
        private NativeReference<int> nativeMvrCheckpointCount;
        private NativeReference<int> nativeBalCheckpointCount;
        private List<TrajectoryCheckpoint> cachedCheckpoints = new List<TrajectoryCheckpoint>();
        private int cacheEpoch = 0;
        private int cachedPartialEpoch = -1;

        // Ballistic visual cache
        private List<Vector3d> cachedBallisticPoints = new List<Vector3d>();
        private List<double> cachedBallisticTimes = new List<double>();

        // Profiling
        private NativeArray<long> nativeMvrProfileCounters;
        private NativeArray<long> nativeBalProfileCounters;
        private Stopwatch jobTimer = new Stopwatch();
        private long lastJobTicks;

        // ─── Ephemeris version for checkpoint validation ──────────────
        private int _ephemerisRevision = 0;
        private int _cachedEphemerisRevision = -1;
        private int _cachedPartialEphemerisRevision = -1;
        private int _cachedPartialEphemerisIndex = -1;

        private Vector3[] ballisticPositions;
        private double[] ballisticTimesData;
        private int ballisticCount;
        private Vector3[] _ballisticClipBuffer = new Vector3[0];

        // ═══════════════════════════════════════════════════════════════════════════
        // REQUEST VERSIONING & PREVIEW SYSTEM
        // ═══════════════════════════════════════════════════════════════════════════

        /// <summary>
        /// Счётчик запросов. Инкрементируется при каждом изменении манёвра.
        /// </summary>
        private ulong _calculationRequestRevision = 0;

        /// <summary>
        /// Ревизия последнего успешно применённого точного результата.
        /// </summary>
        private ulong _lastAppliedRevision = 0;

        /// <summary>
        /// Ревизия текущего выполняющегося job.
        /// </summary>
        private ulong _runningJobRevision = 0;

        /// <summary>
        /// Состояние отображаемой траектории.
        /// </summary>
        private enum VisualizationState
        {
            Empty,           // Нет траектории
            ShowingExact,    // Показываем точный результат
            ShowingPreview,  // Показываем быстрый preview
            Computing        // Идёт расчёт, показываем предыдущий результат
        }

        private VisualizationState _vizState = VisualizationState.Empty;

        // Exact result buffer (последний точный результат)
        private List<Vector3d> _exactTrajectoryPoints = new List<Vector3d>();
        private List<double> _exactTrajectoryTimes = new List<double>();
        private List<bool> _exactTrajectoryIsDashed = new List<bool>();

        // Preview buffer (для быстрых интерполяций)
        private List<Vector3d> _previewTrajectoryPoints = new List<Vector3d>();
        private List<double> _previewTrajectoryTimes = new List<double>();
        private List<bool> _previewTrajectoryIsDashed = new List<bool>();

        // Параметры последнего exact result (база для интерполяции)
        private struct ExactResultSnapshot
        {
            public double DvPrograde;
            public double DvNormal;
            public double DvRadial;
            public int ManeuverIndex;
            public double ManeuverTime;
            public int TrajectoryPointIndex; // индекс точки манёвра в массиве
            
            public bool IsValid => ManeuverIndex >= 0;
        }

        // Храним snapshot для каждого манёвра
        private Dictionary<int, ExactResultSnapshot> _exactSnapshots = new Dictionary<int, ExactResultSnapshot>();
        
        // Для обратной совместимости
        private ExactResultSnapshot _lastExactSnapshot => _exactSnapshots.Count > 0 ? _exactSnapshots.Values.First() : default;

        // Threshold: если Δv изменился больше этого значения, preview не создаём
        private const double MAX_DV_DELTA_FOR_PREVIEW_MS = 500.0; // 500 m/s

        // ─── Moon prediction cache ──────────────────────────────────
        private double _cachedMoonEndTime = -1.0;
        private double _cachedMoonSimTime = -1.0;
        private ReferenceFrameTarget _cachedMoonFrame = ReferenceFrameTarget.Jupiter;
        private MoonOrbitData[] _moonOrbitDataCache = null; // Reusable buffer to avoid GC allocation

        // ─── MoonPredictionLinesJob (Burst parallel) ────────────────────────
        private JobHandle _moonPredJobHandle;
        private bool _moonPredJobRunning = false;
        private NativeArray<float3> _moonPredResults;
        private NativeArray<double> _moonPredSampleTimes;
        private NativeArray<double3> _moonPredFramePositions;
        private NativeArray<MoonOrbitData> _moonPredOrbits;
        private int _moonPredSamplesPerMoon;
        private int _moonPredMoonCount;
        private double _moonPredSimTime;
        private double _moonPredEndTime;
        private ReferenceFrameTarget _moonPredFrame;
        private bool _moonPredNeedsRebuild = true;

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
            DisposeAllNativeBuffers();
            ClearLines();
            ClearMoonPredictionVisuals();
        }

        private void Update()
        {
            if (isDirty)
            {
                dirtyTimer += Time.unscaledDeltaTime;
                // При активном scrubbing пересчитываем немедленно без debounce
                bool shouldRecalc = ScrubbingActive || dirtyTimer >= debounceTime;
                if (shouldRecalc)
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

            ScheduleMoonPredictionJob();
            UpdateTrajectoryClip();
            UpdateVisibility();
        }

        private void LateUpdate()
        {
            CompleteMoonPredictionJob();
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
                    if (_ballisticClipBuffer.Length < bRemaining)
                        _ballisticClipBuffer = new Vector3[bRemaining];
                    Array.Copy(ballisticPositions, bStart, _ballisticClipBuffer, 0, bRemaining);
                    ballisticLine.positionCount = bRemaining;
                    ballisticLine.SetPositions(_ballisticClipBuffer);
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
                // Calculate width based on average distance from camera to line points
                float avgDistance = 0f;
                if (ballisticPositions != null && ballisticCount > 0)
                {
                    Transform parent = GetTrajectoryParent();
                    Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
                    
                    int sampleCount = Mathf.Min(10, ballisticCount);
                    for (int i = 0; i < sampleCount; i++)
                    {
                        int idx = (i * ballisticCount) / sampleCount;
                        Vector3 worldPos = parent.TransformPoint(ballisticPositions[idx]);
                        avgDistance += Vector3.Distance(camPos, worldPos);
                    }
                    avgDistance /= sampleCount;
                }
                else
                {
                    avgDistance = 1000f;
                }
                
                float pixelHeight = Camera.main != null ? Camera.main.pixelHeight : 1080f;
                float fov = Camera.main != null ? Camera.main.fieldOfView : 60f;
                float frustumHeight = 2f * avgDistance * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
                float w = frustumHeight * 2.25f / pixelHeight;
                w = Mathf.Clamp(w, 0.01f, 0.5f);
                
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
                        // Calculate width based on average distance from camera to line points
                        float avgDistance = 0f;
                        Transform parent = GetTrajectoryParent();
                        Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
                        
                        int sampleCount = Mathf.Min(5, line.positionCount);
                        for (int i = 0; i < sampleCount; i++)
                        {
                            int idx = (i * line.positionCount) / Mathf.Max(1, sampleCount);
                            Vector3 localPos = line.GetPosition(idx);
                            Vector3 worldPos = parent.TransformPoint(localPos);
                            avgDistance += Vector3.Distance(camPos, worldPos);
                        }
                        avgDistance /= sampleCount;
                        
                        float pixelHeight = Camera.main != null ? Camera.main.pixelHeight : 1080f;
                        float fov = Camera.main != null ? Camera.main.fieldOfView : 60f;
                        float frustumHeight = 2f * avgDistance * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
                        float width = frustumHeight * 2.25f / pixelHeight;
                        width = Mathf.Clamp(width, 0.01f, 0.5f);
                        
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

        // ═══════════════════════════════════════════════════════════════════════════
        // INSTANT PREVIEW SYSTEM
        // ═══════════════════════════════════════════════════════════════════════════

        /// <summary>
        /// Запрос на мгновенное обновление preview при изменении Δv слайдером.
        /// Вызывается БЕЗ debounce для немедленного визуального отклика.
        /// </summary>
        /// <param name="maneuverIndex">Индекс редактируемого манёвра</param>
        /// <param name="dvPrograde">Новое значение Δv prograde (m/s)</param>
        /// <param name="dvNormal">Новое значение Δv normal (m/s)</param>
        /// <param name="dvRadial">Новое значение Δv radial (m/s)</param>
        public void RequestInstantPreview(int maneuverIndex, double dvPrograde, double dvNormal, double dvRadial)
        {
            // Increment request counter
            _calculationRequestRevision++;
            
            // Try to generate fast preview (currently disabled)
            bool previewCreated = TryGenerateInstantPreview(maneuverIndex, dvPrograde, dvNormal, dvRadial);
            
            if (previewCreated)
            {
                _vizState = VisualizationState.ShowingPreview;
            }
            
            // Always request exact recalculation (with debounce)
            MarkAsDirtyLightweight();
        }

        /// <summary>
        /// Попытка создать быстрый интерполированный preview траектории.
        /// Возвращает true если preview успешно создан.
        /// </summary>
        private bool TryGenerateInstantPreview(int maneuverIndex, double dvPrograde, double dvNormal, double dvRadial)
        {
            // ВРЕМЕННО ОТКЛЮЧЕНО: preview интерполяция работает некорректно в вращающейся системе отсчёта
            // Вместо этого просто оставляем текущую траекторию видимой до готовности exact result
            return false;
            
            /* TODO: Реализовать корректную интерполяцию с учётом:
             * 1. Вращающейся системы отсчёта (rotating reference frame)
             * 2. Гравитационного влияния
             * 3. Frame transformations
             * 
             * Пока просто используем версионирование для отбрасывания устаревших результатов
             */
        }

        /// <summary>
        /// Генерирует линейно интерполированный preview траектории.
        /// ЭТО ГРУБОЕ ПРИБЛИЖЕНИЕ только для визуального сглаживания UI.
        /// </summary>
        private void GenerateLinearPreview(ExactResultSnapshot snapshot, double newDvPrograde, double newDvNormal, double newDvRadial, double dvDelta)
        {
            _previewTrajectoryPoints.Clear();
            _previewTrajectoryTimes.Clear();
            _previewTrajectoryIsDashed.Clear();
            
            int maneuverPtIdx = snapshot.TrajectoryPointIndex;
            
            if (maneuverPtIdx < 0 || maneuverPtIdx >= _exactTrajectoryPoints.Count)
            {
                // Fallback: копируем exact result без изменений
                _previewTrajectoryPoints.AddRange(_exactTrajectoryPoints);
                _previewTrajectoryTimes.AddRange(_exactTrajectoryTimes);
                _previewTrajectoryIsDashed.AddRange(_exactTrajectoryIsDashed);
                return;
            }
            
            // Копируем точки ДО манёвра без изменений
            for (int i = 0; i < maneuverPtIdx; i++)
            {
                _previewTrajectoryPoints.Add(_exactTrajectoryPoints[i]);
                _previewTrajectoryTimes.Add(_exactTrajectoryTimes[i]);
                _previewTrajectoryIsDashed.Add(_exactTrajectoryIsDashed[i]);
            }
            
            // Вычисляем изменение Δv в мировых координатах
            double dvDeltaPrograde = newDvPrograde - snapshot.DvPrograde;
            double dvDeltaNormal = newDvNormal - snapshot.DvNormal;
            double dvDeltaRadial = newDvRadial - snapshot.DvRadial;
            
            Vector3d dvWorldDelta = ComputeWorldDeltaVChange(maneuverPtIdx, 
                dvDeltaPrograde, dvDeltaNormal, dvDeltaRadial);
            
            // Применяем линейную аппроксимацию к точкам ПОСЛЕ манёвра
            double maneuverTime = _exactTrajectoryTimes[maneuverPtIdx];
            
            for (int i = maneuverPtIdx; i < _exactTrajectoryPoints.Count; i++)
            {
                double timeSinceManeuver = _exactTrajectoryTimes[i] - maneuverTime;
                
                // Простая формула: новая позиция ≈ старая позиция + deltaV * время
                // (игнорируем гравитацию - это только визуальный preview)
                Vector3d approximateOffset = dvWorldDelta * timeSinceManeuver;
                
                _previewTrajectoryPoints.Add(_exactTrajectoryPoints[i] + approximateOffset);
                _previewTrajectoryTimes.Add(_exactTrajectoryTimes[i]);
                _previewTrajectoryIsDashed.Add(_exactTrajectoryIsDashed[i]);
            }
        }

        /// <summary>
        /// Вычисляет изменение Δv в мировых координатах.
        /// </summary>
        private Vector3d ComputeWorldDeltaVChange(int trajectoryPointIdx, 
            double dvDeltaPrograde, double dvDeltaNormal, double dvDeltaRadial)
        {
            if (trajectoryPointIdx < 0 || trajectoryPointIdx >= _exactTrajectoryPoints.Count)
                return Vector3d.Zero;
            
            Vector3d position = _exactTrajectoryPoints[trajectoryPointIdx];
            
            // Вычисляем velocity из соседних точек (численная производная)
            Vector3d velocity;
            if (trajectoryPointIdx + 1 < _exactTrajectoryPoints.Count)
            {
                double dt = _exactTrajectoryTimes[trajectoryPointIdx + 1] - _exactTrajectoryTimes[trajectoryPointIdx];
                if (dt > 0.01)
                    velocity = (_exactTrajectoryPoints[trajectoryPointIdx + 1] - position) / dt;
                else
                    velocity = Vector3d.Zero;
            }
            else if (trajectoryPointIdx > 0)
            {
                double dt = _exactTrajectoryTimes[trajectoryPointIdx] - _exactTrajectoryTimes[trajectoryPointIdx - 1];
                if (dt > 0.01)
                    velocity = (position - _exactTrajectoryPoints[trajectoryPointIdx - 1]) / dt;
                else
                    velocity = Vector3d.Zero;
            }
            else
            {
                velocity = Vector3d.Zero;
            }
            
            // Вычисляем орбитальный базис
            if (!OrbitalBasis.TryComputeBasis(position, velocity, 
                out Vector3d radial, out Vector3d normal, out Vector3d prograde))
            {
                return Vector3d.Zero;
            }
            
            return prograde * dvDeltaPrograde + normal * dvDeltaNormal + radial * dvDeltaRadial;
        }

        /// <summary>
        /// Применяет preview траекторию к визуализации (LineRenderer).
        /// </summary>
        private void ApplyPreviewToVisualization()
        {
            if (_previewTrajectoryPoints.Count == 0)
                return;
            
            // Используем ту же логику рендеринга, что и для exact result
            int totalPoints = _previewTrajectoryPoints.Count;
            InitializeBackBuffer(totalPoints);
            
            Vector3d firstFramePos = Vector3d.Zero;
            Vector3d firstFrameVel = Vector3d.Zero;
            
            for (int i = 0; i < totalPoints && i < backBufferPoints.Length; i++)
            {
                double ptTime = _previewTrajectoryTimes[i];
                Vector3d absPos = _previewTrajectoryPoints[i];
                
                TryUpdateFrameState(ref firstFramePos, ref firstFrameVel, ptTime);
                Vector3d relPos = absPos - firstFramePos;
                
                backBufferPoints[i] = universeManager.ToUnityOffset(relPos);
                backBufferTimes[i] = ptTime;
                backBufferIsDashed[i] = _previewTrajectoryIsDashed[i];
            }
            
            backBufferCount = totalPoints;
            CompleteBackBuffer(backBufferCount);
        }

        private void InvalidateCache()
        {
            cacheEpoch++;
            cachedPartialEpoch = -1;
            hasPartialCacheHit = false;
            cachedCheckpoints.Clear();
            cachedBoundaries.Clear();
            cachedBallisticPoints.Clear();
            cachedBallisticTimes.Clear();
            cachedStartPos = null;
            cachedStartVel = null;
            cachedStartTime = null;
            cachedPredictionLength = null;
            cachedNodeData.Clear();
            _cachedEphemerisRevision = -1;
        }

        /// <summary>
        /// Вызывать при любом изменении орбит луны, источника эфемерид или таблиц.
        /// Увеличивает ревизию, что делает все старые чекпоинты невалидными.
        /// </summary>
        public void InvalidateEphemerisRevision()
        {
            _ephemerisRevision++;
        }

        private void HandleActiveReferenceFrameChanged(ReferenceFrameTarget newFrame)
        {
            // Траектория всегда рассчитывается относительно Jupiter (lockedReferenceFrame = Jupiter)
            // Но визуализация должна показываться в текущем ActiveReferenceFrame
            
            // Пересчитать визуализацию существующих точек с новым frame
            if (fullTrajectoryPoints.Count > 0)
            {
                RetransformTrajectoryForNewFrame();
            }
            
            if (cachedBallisticPoints.Count > 0)
            {
                RetransformBallisticForNewFrame();
            }
            
            // Обновить cachedReferenceFrame чтобы MatchesCachedInput не триггерил пересчёт
            // Траектория в абсолютных координатах не меняется при смене frame, только визуализация
            cachedReferenceFrame = newFrame;
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
            cachedPartialEpoch = cacheEpoch;

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

            // ═══ NEW: Assign revision to this exact calculation request ═══
            _runningJobRevision = _calculationRequestRevision;
            _vizState = VisualizationState.Computing;

            // Try partial recalc: only recompute affected suffix of trajectory
            hasPartialCacheHit = false;
            _cachedEphemerisRevision = _ephemerisRevision;
            TryFindPartialRestartPoint();

            CompleteAndDisposeJobs();
            if (!skipClearOnNextRecalc)
                ClearLines();
            skipClearOnNextRecalc = false;

            if (hasPartialCacheHit)
            {
                TrimTrajectoryToBoundary();
                TrimBallisticToBoundary();
            }
            else
            {
                fullTrajectoryPoints.Clear();
                fullTrajectoryTimes.Clear();
                fullTrajectoryIsDashed.Clear();
                cachedBallisticPoints.Clear();
                cachedBallisticTimes.Clear();
            }

            // Всегда планировать траектории относительно Jupiter (главное тело)
            // ActiveReferenceFrame используется только для визуализации
            lockedReferenceFrame = ReferenceFrameTarget.Jupiter;

            ScheduleJobs();

            if (forceSynchronousCalculation)
            {
                CompleteJobAndSwap();
            }
        }

        private void TryFindPartialRestartPoint()
        {
            hasPartialCacheHit = false;

            // Epoch mismatch means cache was globally invalidated (e.g. reference frame change)
            if (cacheEpoch != cachedPartialEpoch) return;

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

            if (changedIdx < 0) return;

            bool found = false;
            TrajectoryCheckpoint bestCp = default;
            for (int i = cachedCheckpoints.Count - 1; i >= 0; i--)
            {
                var cp = cachedCheckpoints[i];
                if (cp.NodeVersion <= changedIdx)
                {
                    bestCp = cp;
                    found = true;
                    break;
                }
            }
            if (!found) return;

            // Validate checkpoint versions against current state
            if (bestCp.NodeVersion > changedIdx) return;          // node version mismatch (older version is fine)
            if (bestCp.EphemerisVersion != _cachedEphemerisRevision) return; // ephemeris version mismatch

            hasPartialCacheHit = true;
            partialStartSegment = changedIdx;
            cachedPartialStartPos = bestCp.Position;
            cachedPartialStartVel = bestCp.Velocity;
            cachedPartialStartTime = bestCp.Time;
            cachedPartialStartVersion = bestCp.NodeVersion;
            _cachedPartialEphemerisRevision = bestCp.EphemerisVersion;
            _cachedPartialEphemerisIndex = bestCp.EphemerisIndex;
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

            int keepCp = cachedCheckpoints.FindLastIndex(cp => cp.Time <= cachedPartialStartTime);
            if (keepCp >= 0 && keepCp + 1 < cachedCheckpoints.Count)
                cachedCheckpoints.RemoveRange(keepCp + 1, cachedCheckpoints.Count - keepCp - 1);
        }

        private void TrimBallisticToBoundary()
        {
            if (cachedBallisticPoints.Count == 0) return;
            double boundaryTime = cachedPartialStartTime;
            int trimIdx = -1;
            for (int i = 0; i < cachedBallisticTimes.Count; i++)
            {
                if (cachedBallisticTimes[i] >= boundaryTime - 0.5 && cachedBallisticTimes[i] <= boundaryTime + 0.5)
                {
                    trimIdx = i;
                    break;
                }
            }
            if (trimIdx >= 0 && trimIdx + 1 < cachedBallisticPoints.Count)
            {
                int keepCount = trimIdx;
                cachedBallisticPoints.RemoveRange(keepCount, cachedBallisticPoints.Count - keepCount);
                cachedBallisticTimes.RemoveRange(keepCount, cachedBallisticTimes.Count - keepCount);
            }
        }

        private NativeArray<T> ResizeBuffer<T>(NativeArray<T> existing, int requiredSize) where T : unmanaged
        {
            if (existing.IsCreated && existing.Length >= requiredSize)
                return existing;
            if (existing.IsCreated)
                existing.Dispose();
            return new NativeArray<T>(requiredSize, Allocator.Persistent);
        }

        // ─── LOD: Ramer–Douglas–Peucker (индексная версия, без копирования) ────
        /// <summary>
        /// Упрощает полилинию через RDP, работая по индексам [start..end].
        /// Записывает результат в dst начиная с dstStartIdx.
        /// Возвращает количество записанных точек.
        /// </summary>
        private int SimplifyLineRDP(
            Vector3[] srcPoints, double[] srcTimes, bool[] srcIsDashed,
            int start, int end, double errorTol,
            Vector3[] dstPoints, double[] dstTimes, bool[] dstIsDashed,
            int dstStartIdx)
        {
            int count = end - start + 1;
            if (count <= 2)
            {
                for (int i = 0; i < count; i++)
                {
                    dstPoints[dstStartIdx + i] = srcPoints[start + i];
                    dstTimes[dstStartIdx + i] = srcTimes[start + i];
                    dstIsDashed[dstStartIdx + i] = srcIsDashed[start + i];
                }
                return count;
            }

            // Найти точку с максимальным отклонением от линии (start..end)
            double maxDist = 0.0;
            int maxIdx = start;
            Vector3 lineStart = srcPoints[start];
            Vector3 lineEnd = srcPoints[end];
            Vector3 lineDir = lineEnd - lineStart;
            float lineLenSq = lineDir.sqrMagnitude;

            for (int i = start + 1; i < end; i++)
            {
                double dist;
                if (lineLenSq < 1e-12f)
                {
                    dist = (srcPoints[i] - lineStart).magnitude;
                }
                else
                {
                    Vector3 ap = srcPoints[i] - lineStart;
                    float t = Vector3.Dot(ap, lineDir) / lineLenSq;
                    t = Mathf.Clamp01(t);
                    Vector3 proj = lineStart + lineDir * t;
                    dist = (srcPoints[i] - proj).magnitude;
                }

                if (dist > maxDist)
                {
                    maxDist = dist;
                    maxIdx = i;
                }
            }

            if (maxDist <= (double)errorTol)
            {
                // Упростить: оставить только первую и последнюю
                dstPoints[dstStartIdx] = lineStart;
                dstTimes[dstStartIdx] = srcTimes[start];
                dstIsDashed[dstStartIdx] = srcIsDashed[start];

                dstPoints[dstStartIdx + 1] = lineEnd;
                dstTimes[dstStartIdx + 1] = srcTimes[end];
                dstIsDashed[dstStartIdx + 1] = srcIsDashed[end];
                return 2;
            }

            // Разделить и рекурсивно упростить
            int leftCount = SimplifyLineRDP(
                srcPoints, srcTimes, srcIsDashed,
                start, maxIdx, errorTol,
                dstPoints, dstTimes, dstIsDashed,
                dstStartIdx);

            int rightCount = SimplifyLineRDP(
                srcPoints, srcTimes, srcIsDashed,
                maxIdx, end, errorTol,
                dstPoints, dstTimes, dstIsDashed,
                dstStartIdx + leftCount - 1); // -1 чтобы не дублировать maxIdx

            return leftCount + rightCount - 1;
        }

        /// <summary>
        /// Строит LOD-буфер из backBufferPoints с упрощением по ошибке.
        /// Разбивает на временные окна: ближнее (точнее), среднее, дальнее (грубее).
        /// </summary>
        private int BuildLODBuffer(int srcStartIdx, int srcCount)
        {
            double simTime = universeManager.SimulationTimeSeconds;

            // Убедиться, что буферы достаточного размера
            if (_lodPoints == null || _lodPoints.Length < srcCount)
                _lodPoints = new Vector3[srcCount];
            if (_lodTimes == null || _lodTimes.Length < srcCount)
                _lodTimes = new double[srcCount];
            if (_lodIsDashed == null || _lodIsDashed.Length < srcCount)
                _lodIsDashed = new bool[srcCount];

            int lodTotal = 0;

            // Разбить на окна: [start..end1), [end1..end2), [end2..end)
            int nearEnd = srcStartIdx;
            int midEnd = srcStartIdx;

            // Найти границы по времени
            for (int i = srcStartIdx; i < srcStartIdx + srcCount; i++)
            {
                double dt = backBufferTimes[i] - simTime;
                if (dt <= lodNearTime)
                    nearEnd = i;
                if (dt <= lodMidTime)
                    midEnd = i;
            }
            if (nearEnd <= srcStartIdx) nearEnd = srcStartIdx + 1;
            if (midEnd <= nearEnd) midEnd = nearEnd + 1;
            if (midEnd >= srcStartIdx + srcCount) midEnd = srcStartIdx + srcCount - 1;

            // Ближнее окно: минимальный tolerance (точнее)
            int nearCount = SimplifyLineRDP(
                backBufferPoints, backBufferTimes, backBufferIsDashed,
                srcStartIdx, nearEnd, lodErrorTolerance * 0.5,
                _lodPoints, _lodTimes, _lodIsDashed, 0);
            lodTotal = nearCount;

            // Среднее окно: средний tolerance
            int midCount = SimplifyLineRDP(
                backBufferPoints, backBufferTimes, backBufferIsDashed,
                nearEnd, midEnd, lodErrorTolerance,
                _lodPoints, _lodTimes, _lodIsDashed, lodTotal - 1);
            lodTotal += midCount - 1; // -1 чтобы не дублировать nearEnd

            // Дальнее окно: больший tolerance
            int farCount = SimplifyLineRDP(
                backBufferPoints, backBufferTimes, backBufferIsDashed,
                midEnd, srcStartIdx + srcCount - 1, lodErrorTolerance * 4.0,
                _lodPoints, _lodTimes, _lodIsDashed, lodTotal - 1);
            lodTotal += farCount - 1;

            return lodTotal;
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
            MoonOrbitData[] tempOrbits = null;
            if (moonCount > 0)
            {
                tempOrbits = new MoonOrbitData[moonCount];
                universeManager.FillMoonOrbitData(tempOrbits, 0, moonCount, baseStartTime);
            }
            nativeMoonOrbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.Persistent);
            if (moonCount > 0)
            {
                for (int i = 0; i < moonCount; i++)
                    nativeMoonOrbits[i] = tempOrbits[i];
            }

            double predictionSpan = endTime - baseStartTime;
            double ephemerisStep = Math.Min(5.0 * 3600.0,
                Math.Max(60.0, predictionSpan / 20000.0));
            int ephemerisSampleCount = Math.Max(2, (int)(predictionSpan / ephemerisStep) + 1);
            ephemerisSampleCount = Math.Min(ephemerisSampleCount, 50000);

            nativeEphemerisTimes = ResizeBuffer(nativeEphemerisTimes, ephemerisSampleCount);
            for (int i = 0; i < ephemerisSampleCount; i++)
                nativeEphemerisTimes[i] = baseStartTime + i * ephemerisStep;
            nativeEphemerisTimes[ephemerisSampleCount - 1] = endTime;

            int flatEphemerisCount = ephemerisSampleCount * moonCount;
            nativeEphemerisResults = ResizeBuffer(nativeEphemerisResults, flatEphemerisCount);
            nativeEphemerisVelocities = ResizeBuffer(nativeEphemerisVelocities, flatEphemerisCount);

            nativeTrajectoryOutput = ResizeBuffer(nativeTrajectoryOutput, adjustedMaxPoints);
            nativeBallisticOutput = ResizeBuffer(nativeBallisticOutput, adjustedMaxPoints);
            if (!nativePointCount.IsCreated) nativePointCount = new NativeReference<int>(0, Allocator.Persistent);
            if (!nativeBallisticPointCount.IsCreated) nativeBallisticPointCount = new NativeReference<int>(0, Allocator.Persistent);
            nativeBoundaries = ResizeBuffer(nativeBoundaries, nodeCount + 2);
            if (!nativeBoundaryCount.IsCreated) nativeBoundaryCount = new NativeReference<int>(0, Allocator.Persistent);

            double checkpointInterval = 21600.0;
            int maxCheckpoints = (int)(effectivePrediction / checkpointInterval) + nodeCount + 10;
            maxCheckpoints = Math.Min(maxCheckpoints, 10000);
            nativeMvrCheckpoints = ResizeBuffer(nativeMvrCheckpoints, maxCheckpoints);
            nativeBalCheckpoints = ResizeBuffer(nativeBalCheckpoints, maxCheckpoints);
            if (!nativeMvrCheckpointCount.IsCreated) nativeMvrCheckpointCount = new NativeReference<int>(0, Allocator.Persistent);
            if (!nativeBalCheckpointCount.IsCreated) nativeBalCheckpointCount = new NativeReference<int>(0, Allocator.Persistent);
            if (!nativeCalcStatus.IsCreated) nativeCalcStatus = new NativeReference<int>(0, Allocator.Persistent);
            if (!nativeBallisticCalcStatus.IsCreated) nativeBallisticCalcStatus = new NativeReference<int>(0, Allocator.Persistent);

            nativeBallisticBoundaries = ResizeBuffer(nativeBallisticBoundaries, 1);
            if (!nativeBallisticBoundaryCount.IsCreated) nativeBallisticBoundaryCount = new NativeReference<int>(0, Allocator.Persistent);

            // Reset output refs before scheduling (clean slate for reuse)
            nativePointCount.Value = 0;
            nativeBallisticPointCount.Value = 0;
            nativeBoundaryCount.Value = 0;
            nativeBallisticBoundaryCount.Value = 0;
            nativeCalcStatus.Value = 0;
            nativeBallisticCalcStatus.Value = 0;

            // Allocate profile counters
            nativeMvrProfileCounters = ResizeBuffer(nativeMvrProfileCounters, FullTrajectoryJob.PC_COUNT);
            nativeBalProfileCounters = ResizeBuffer(nativeBalProfileCounters, FullTrajectoryJob.PC_COUNT);
            for (int i = 0; i < FullTrajectoryJob.PC_COUNT; i++)
            {
                nativeMvrProfileCounters[i] = 0;
                nativeBalProfileCounters[i] = 0;
            }

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
                ephemerisJobHandle = moonJob.Schedule(flatEphemerisCount, 64);

                var velJob = new EphemerisVelocityJob
                {
                    SampleTimes = nativeEphemerisTimes,
                    MoonStates = nativeEphemerisResults,
                    Velocities = nativeEphemerisVelocities,
                    MoonCount = moonCount
                };
                ephemerisJobHandle = velJob.Schedule(flatEphemerisCount, 64, ephemerisJobHandle);
            }
            else
            {
                ephemerisJobHandle = default;
            }

            // Choose integrator tolerances based on flight regime
            double useRelTol = integratorRelTol;
            double useAbsTol = integratorAbsTol;
            double useMinStep = integratorMinStep;
            double useMaxStep = integratorMaxStep;

            // Check for close flyby: min distance to any body from ship start position
            double3 shipPos = JobTypeConversion.ToDouble3(universeManager.ShipBody.Position);
            double minDistToBody = math.length(shipPos - jupiterPos);
            if (moonCount > 0)
            {
                // Rough check: use already-computed tempOrbits for moon positions at baseStartTime
                for (int mi = 0; mi < moonCount; mi++)
                {
                    var orbit = tempOrbits[mi];
                    double3 moonPos = AccelerationEvaluator.EvaluateMoonPosition(
                        ref orbit, baseStartTime, planeMapping);
                    double d = math.length(shipPos - moonPos);
                    if (d < minDistToBody) minDistToBody = d;
                }
            }

            // < 10 Jupiter radii → close flyby regime
            if (minDistToBody < integratorJupiterRadius * 10.0)
            {
                useRelTol = FlybyRelTol;
                useAbsTol = FlybyAbsTol;
                useMinStep = FlybyMinStep;
                useMaxStep = FlybyMaxStep;
            }

            // Check if any maneuver is within 1 hour from start → maneuver regime (overrides flyby)
            for (int ni = 0; ni < flightPlan.Nodes.Count; ni++)
            {
                double timeToManeuver = flightPlan.Nodes[ni].StartTime - baseStartTime;
                if (timeToManeuver > 0 && timeToManeuver < 3600.0)
                {
                    useRelTol = ManeuverRelTol;
                    useAbsTol = ManeuverAbsTol;
                    useMinStep = ManeuverMinStep;
                    useMaxStep = ManeuverMaxStep;
                    break;
                }
            }

            int ephemerisVersion = _cachedEphemerisRevision;
            int startEphemerisIndex = hasPartialCacheHit ? _cachedPartialEphemerisIndex : -1;

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
                SegmentBoundaryCount = nativeBallisticBoundaryCount,
                ProfileCounters = nativeBalProfileCounters,

                CheckpointIntervalSeconds = 21600.0,
                Checkpoints = nativeBalCheckpoints,
                CheckpointCount = nativeBalCheckpointCount,
                HotNodeIndex = hasPartialCacheHit ? partialStartSegment : -1,
                HotCheckpointInterval = 60.0,

                // State-based restart
                StartEphemerisIndex = startEphemerisIndex,
                EphemerisVersion = ephemerisVersion,

                // Adaptive integrator settings (regime-aware)
                RelTol = useRelTol,
                AbsTol = useAbsTol,
                MinStepSeconds = useMinStep,
                MaxStepSeconds = useMaxStep,
                JupiterRadius = integratorJupiterRadius,
                MoonRadius = integratorMoonRadius
            };

            jobTimer.Restart();
            ballisticJobHandle = ballisticTrajectoryJob.Schedule(ephemerisJobHandle);

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
                SegmentBoundaryCount = nativeBoundaryCount,
                ProfileCounters = nativeMvrProfileCounters,

                CheckpointIntervalSeconds = 21600.0,
                Checkpoints = nativeMvrCheckpoints,
                CheckpointCount = nativeMvrCheckpointCount,
                HotNodeIndex = hasPartialCacheHit ? partialStartSegment : -1,
                HotCheckpointInterval = 60.0,

                // State-based restart
                StartEphemerisIndex = startEphemerisIndex,
                EphemerisVersion = ephemerisVersion,

                // Adaptive integrator settings (regime-aware)
                RelTol = useRelTol,
                AbsTol = useAbsTol,
                MinStepSeconds = useMinStep,
                MaxStepSeconds = useMaxStep,
                JupiterRadius = integratorJupiterRadius,
                MoonRadius = integratorMoonRadius
            };

            var maneuverHandle = trajectoryJob.Schedule(ephemerisJobHandle);
            trajectoryJobHandle = JobHandle.CombineDependencies(ballisticJobHandle, maneuverHandle);
            JobHandle.ScheduleBatchedJobs();
            isJobRunning = true;

            UpdateCache();
        }

        private void CompleteJobAndSwap()
        {
            if (!isJobRunning) return;

            // ═══ NEW: Check if this result is still relevant ═══
            if (_runningJobRevision < _lastAppliedRevision)
            {
                // Этот job завершился, но его результат уже устарел
                trajectoryJobHandle.Complete();
                DisposeJobResources();
                isJobRunning = false;
                return;
            }
            
            // При scrubbing принимаем любой результат, даже если пришёл более новый запрос
            if (!ScrubbingActive && _runningJobRevision < _calculationRequestRevision)
            {
                // Пока job считался, пришёл новый запрос - результат устарел
                trajectoryJobHandle.Complete();
                DisposeJobResources();
                isJobRunning = false;
                return;
            }
            
            // Job актуален (или scrubbing активен) - применяем результат
            _lastAppliedRevision = _runningJobRevision;

            trajectoryJobHandle.Complete();
            jobTimer.Stop();
            lastJobTicks = jobTimer.ElapsedTicks;
            LogProfileData();

            // Read checkpoints from job — replace stale suffix, keep valid prefix
            int cpCount = nativeMvrCheckpointCount.IsCreated ? nativeMvrCheckpointCount.Value : 0;
            if (nativeMvrCheckpoints.IsCreated && cpCount > 0)
            {
                if (hasPartialCacheHit)
                {
                    // Remove checkpoints in the restarted region (NodeVersion >= partialStartSegment)
                    int staleStart = cachedCheckpoints.FindIndex(cp => cp.NodeVersion >= partialStartSegment);
                    if (staleStart >= 0)
                        cachedCheckpoints.RemoveRange(staleStart, cachedCheckpoints.Count - staleStart);

                    // Append new tail with offset NodeVersion
                    for (int i = 0; i < cpCount && i < nativeMvrCheckpoints.Length; i++)
                    {
                        var cp = nativeMvrCheckpoints[i];
                        cp.NodeVersion += partialStartSegment;
                        cachedCheckpoints.Add(cp);
                    }
                }
                else
                {
                    cachedCheckpoints.Clear();
                    for (int i = 0; i < cpCount && i < nativeMvrCheckpoints.Length; i++)
                        cachedCheckpoints.Add(nativeMvrCheckpoints[i]);
                }
            }

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

            // Process ballistic trajectory — append new suffix to cache
            int ballisticCountJob = nativeBallisticPointCount.IsCreated ? nativeBallisticPointCount.Value : 0;
            int ballisticStatus = nativeBallisticCalcStatus.IsCreated ? nativeBallisticCalcStatus.Value : 0;
            if (ballisticCountJob > 0 && ballisticStatus == 1 && nativeBallisticOutput.IsCreated)
            {
                int existingBallisticCount = cachedBallisticPoints.Count;

                // Skip first new point if it duplicates last cached point
                int newStartIdx = 0;
                if (existingBallisticCount > 0 && ballisticCountJob > 0)
                {
                    double lastCachedTime = cachedBallisticTimes[existingBallisticCount - 1];
                    double firstNewTime = nativeBallisticOutput[0].Time;
                    if (Math.Abs(lastCachedTime - firstNewTime) < 0.5)
                        newStartIdx = 1;
                }

                for (int i = newStartIdx; i < ballisticCountJob && i < nativeBallisticOutput.Length; i++)
                {
                    var pt = nativeBallisticOutput[i];
                    Vector3d absPos = JobTypeConversion.ToVector3d(pt.Position);
                    cachedBallisticPoints.Add(absPos);
                    cachedBallisticTimes.Add(pt.Time);
                }

                // Build visual arrays from cache
                int bCount = cachedBallisticPoints.Count;
                ballisticPositions = new Vector3[bCount];
                ballisticTimesData = new double[bCount];
                Vector3d framePos = Vector3d.Zero, frameVel = Vector3d.Zero;
                for (int i = 0; i < bCount; i++)
                {
                    TryUpdateFrameState(ref framePos, ref frameVel, cachedBallisticTimes[i]);
                    Vector3d relPos = cachedBallisticPoints[i] - framePos;
                    ballisticPositions[i] = universeManager.ToUnityOffset(relPos);
                    ballisticTimesData[i] = cachedBallisticTimes[i];
                }
                ballisticCount = bCount;
                UpdateBallisticLine();
            }
            
            // DEBUG: Find true closest approach (check EVERY point, not every 10th)
            if (fullTrajectoryPoints.Count > 0)
            {
                double minDistToMoon = double.MaxValue;
                int closestMoonIndex = -1;
                double closestTime = 0;
                Vector3d closestShipPos = Vector3d.Zero;
                Vector3d closestMoonPos = Vector3d.Zero;
                
                int moonCount = universeManager.MoonRailCount;
                for (int i = 0; i < fullTrajectoryPoints.Count; i++) // Check EVERY point
                {
                    double t = fullTrajectoryTimes[i];
                    Vector3d shipPos = fullTrajectoryPoints[i];
                    
                    for (int m = 0; m < moonCount; m++)
                    {
                        if (universeManager.TryGetMoonPositionAtTime(m, t, out Vector3d moonPos))
                        {
                            double dist = (shipPos - moonPos).Magnitude;
                            if (dist < minDistToMoon)
                            {
                                minDistToMoon = dist;
                                closestMoonIndex = m;
                                closestTime = t;
                                closestShipPos = shipPos;
                                closestMoonPos = moonPos;
                            }
                        }
                    }
                }
                
                // Log ANY close approach < 100,000 km
                if (closestMoonIndex >= 0 && minDistToMoon < 100e6)
                {
                    string moonName = universeManager.GetMoonName(closestMoonIndex);
                    double moonRadius = universeManager.GetMoonRadius(closestMoonIndex);
                    double altitude = minDistToMoon - moonRadius;
                    double soiRadius = universeManager.GetMoonSOI(closestMoonIndex);
                    
                    string severity = altitude <= 0 ? "IMPACT!!!" :
                                     altitude < soiRadius ? "FLYBY" :
                                     "distant";
                    
                    if (altitude < soiRadius * 5) // Log only interesting cases
                    {
                        UnityEngine.Debug.Log($"[{severity}] {moonName}: " +
                            $"altitude={altitude/1e3:F0} km (SOI={soiRadius/1e3:F0} km) at t={closestTime:F1}s");
                    }
                }
            }

            DisposeJobResources();
            isJobRunning = false;
            
            // ═══ NEW: Save exact result for future preview interpolation ═══
            _exactTrajectoryPoints.Clear();
            _exactTrajectoryPoints.AddRange(fullTrajectoryPoints);
            _exactTrajectoryTimes.Clear();
            _exactTrajectoryTimes.AddRange(fullTrajectoryTimes);
            _exactTrajectoryIsDashed.Clear();
            _exactTrajectoryIsDashed.AddRange(fullTrajectoryIsDashed);
            
            // Save snapshot of ALL maneuvers
            CaptureExactResultSnapshot();
            
            _vizState = VisualizationState.ShowingExact;
            
            // Cache last trajectory end point for marker during scrubbing
            hasCachedMarkerPos = fullTrajectoryPoints.Count > 0;
            if (hasCachedMarkerPos)
            {
                cachedMarkerPos = fullTrajectoryPoints[fullTrajectoryPoints.Count - 1];
                cachedMarkerTime = fullTrajectoryTimes[fullTrajectoryTimes.Count - 1];
            }
            UpdateVisibility();
        }

        /// <summary>
        /// Захватывает параметры всех манёвров для будущих preview.
        /// </summary>
        private ExactResultSnapshot CaptureExactResultSnapshot()
        {
            if (flightPlan == null || flightPlan.Nodes.Count == 0)
                return default;
            
            // Сохраняем snapshot для ВСЕХ манёвров
            _exactSnapshots.Clear();
            
            for (int i = 0; i < flightPlan.Nodes.Count; i++)
            {
                var node = flightPlan.Nodes[i];
                
                // Находим индекс точки манёвра в траектории
                int trajectoryIdx = FindTrajectoryPointNearTime(node.StartTime);
                
                var snapshot = new ExactResultSnapshot
                {
                    DvPrograde = node.DvPrograde,
                    DvNormal = node.DvNormal,
                    DvRadial = node.DvRadial,
                    ManeuverIndex = i,
                    ManeuverTime = node.StartTime,
                    TrajectoryPointIndex = trajectoryIdx
                };
                
                _exactSnapshots[i] = snapshot;
            }
            
            // Возвращаем первый snapshot для обратной совместимости
            return _exactSnapshots.Count > 0 ? _exactSnapshots[0] : default;
        }

        /// <summary>
        /// Находит индекс точки траектории ближайшей к заданному времени.
        /// </summary>
        private int FindTrajectoryPointNearTime(double targetTime)
        {
            if (_exactTrajectoryTimes.Count == 0)
                return -1;
            
            int idx = _exactTrajectoryTimes.BinarySearch(targetTime);
            if (idx < 0)
                idx = ~idx; // Индекс вставки
            
            return Math.Min(idx, _exactTrajectoryTimes.Count - 1);
        }

        private void DisposeJobResources()
        {
            // Dispose small/frequently-changing arrays; keep large buffers for reuse
            if (nativeMoonOrbits.IsCreated) nativeMoonOrbits.Dispose();
            if (nativeNodeData.IsCreated) nativeNodeData.Dispose();
            if (nativeBallisticNodeData.IsCreated) nativeBallisticNodeData.Dispose();
            // These large buffers persist across frames — only recreate when size grows
            // nativeEphemerisTimes, nativeEphemerisResults, nativeEphemerisVelocities,
            // nativeTrajectoryOutput, nativeBallisticOutput, nativeBoundaries,
            // nativeBallisticBoundaries, nativePointCount, nativeBallisticPointCount,
            // nativeBoundaryCount, nativeCalcStatus, nativeBallisticCalcStatus,
            // nativeBallisticBoundaryCount are reused
        }

        private void DisposeAllNativeBuffers()
        {
            DisposeMoonPredictionBuffers();
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
            if (nativeMvrProfileCounters.IsCreated) nativeMvrProfileCounters.Dispose();
            if (nativeBalProfileCounters.IsCreated) nativeBalProfileCounters.Dispose();
            if (nativeMvrCheckpoints.IsCreated) nativeMvrCheckpoints.Dispose();
            if (nativeBalCheckpoints.IsCreated) nativeBalCheckpoints.Dispose();
            if (nativeMvrCheckpointCount.IsCreated) nativeMvrCheckpointCount.Dispose();
            if (nativeBalCheckpointCount.IsCreated) nativeBalCheckpointCount.Dispose();
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

        private void LogProfileData()
        {
            if (!nativeBalProfileCounters.IsCreated || !nativeMvrProfileCounters.IsCreated)
                return;

            long[] bal = new long[FullTrajectoryJob.PC_COUNT];
            long[] mvr = new long[FullTrajectoryJob.PC_COUNT];
            for (int i = 0; i < FullTrajectoryJob.PC_COUNT; i++)
            {
                bal[i] = nativeBalProfileCounters[i];
                mvr[i] = nativeMvrProfileCounters[i];
            }

            double elapsedMs = (double)lastJobTicks / Stopwatch.Frequency * 1000.0;

            string partialTag = hasPartialCacheHit ? " [PARTIAL]" : "";
            string cpInfo = hasPartialCacheHit
                ? $" cpTime={cachedPartialStartTime:F1}s cpVer={cachedPartialStartVersion}"
                : "";

            UnityEngine.Debug.Log(
                $"[FTJ]{partialTag} BAL: major={bal[0]} substeps={bal[1]} evalAccel={bal[2]} hermit={bal[3]} ephemSearch={bal[4]}\n" +
                $"[FTJ]{partialTag} MVR: major={mvr[0]} substeps={mvr[1]} evalAccel={mvr[2]} hermit={mvr[3]} ephemSearch={mvr[4]}\n" +
                $"[FTJ]{partialTag} TIMER: {elapsedMs:F1}ms (bal+maneuver combined){cpInfo}");
        }

        private bool TryUpdateFrameState(ref Vector3d framePos, ref Vector3d frameVel, double time)
        {
            // Для визуализации всегда использовать ТЕКУЩИЙ ActiveReferenceFrame,
            // независимо от того, в каком frame планировалась траектория (lockedReferenceFrame)
            if (universeManager.TryGetReferenceStateAtTime(
                universeManager.ActiveReferenceFrame, time,
                out _, out framePos, out frameVel,
                out _, out _, out _))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Пересчитывает визуализацию траектории при смене reference frame.
        /// Использует уже рассчитанные absolute coordinates, только меняет трансформацию.
        /// </summary>
        private void RetransformTrajectoryForNewFrame()
        {
            int count = fullTrajectoryPoints.Count;
            if (count == 0) return;
            
            InitializeBackBuffer(count);
            
            Vector3d framePos = Vector3d.Zero;
            Vector3d frameVel = Vector3d.Zero;
            
            // Используем ТЕКУЩИЙ ActiveReferenceFrame для визуализации, а не lockedReferenceFrame
            ReferenceFrameTarget visualFrame = universeManager.ActiveReferenceFrame;
            
            for (int i = 0; i < count && i < backBufferPoints.Length; i++)
            {
                double ptTime = fullTrajectoryTimes[i];
                Vector3d absPos = fullTrajectoryPoints[i];
                
                // Получить позицию frame для визуализации
                universeManager.TryGetReferenceStateAtTime(
                    visualFrame, ptTime,
                    out _, out framePos, out frameVel,
                    out _, out _, out _);
                
                Vector3d relPos = absPos - framePos;
                
                backBufferPoints[i] = universeManager.ToUnityOffset(relPos);
                backBufferTimes[i] = ptTime;
                backBufferIsDashed[i] = i < fullTrajectoryIsDashed.Count && fullTrajectoryIsDashed[i];
            }
            backBufferCount = count;
            
            CompleteBackBuffer(backBufferCount);
        }

        /// <summary>
        /// Пересчитывает визуализацию баллистической траектории при смене reference frame.
        /// </summary>
        private void RetransformBallisticForNewFrame()
        {
            int bCount = cachedBallisticPoints.Count;
            if (bCount == 0) return;
            
            ballisticPositions = new Vector3[bCount];
            ballisticTimesData = new double[bCount];
            
            Vector3d framePos = Vector3d.Zero;
            Vector3d frameVel = Vector3d.Zero;
            
            // Используем ТЕКУЩИЙ ActiveReferenceFrame для визуализации, а не lockedReferenceFrame
            ReferenceFrameTarget visualFrame = universeManager.ActiveReferenceFrame;
            
            for (int i = 0; i < bCount; i++)
            {
                // Получить позицию frame для визуализации
                universeManager.TryGetReferenceStateAtTime(
                    visualFrame, cachedBallisticTimes[i],
                    out _, out framePos, out frameVel,
                    out _, out _, out _);
                
                Vector3d relPos = cachedBallisticPoints[i] - framePos;
                ballisticPositions[i] = universeManager.ToUnityOffset(relPos);
                ballisticTimesData[i] = cachedBallisticTimes[i];
            }
            ballisticCount = bCount;
            UpdateBallisticLine();
        }

        private void InitializeBackBuffer(int capacity)
        {
            backBufferPoints = new Vector3[capacity];
            backBufferTimes = new double[capacity];
            backBufferIsDashed = new bool[capacity];
            backBufferCount = 0;
        }

        // ─── LOD-интеграция в CompleteBackBuffer ──────────────────────
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

            // Apply LOD simplification to reduce point count for rendering
            int lodCount = BuildLODBuffer(startIdx, remainingCount);
            if (lodCount < 2) return;

            // Use LOD arrays for the rest of the pipeline
            // We need to work with _lodPoints[0..lodCount] instead of backBufferPoints
            // Build runs based on LOD data
            List<(int start, int end, bool isDashed)> runs = new List<(int, int, bool)>();
            int runStart = 0;
            for (int i = 1; i < lodCount; i++)
            {
                if (_lodIsDashed[i] != _lodIsDashed[runStart])
                {
                    runs.Add((runStart, i - 1, _lodIsDashed[runStart]));
                    runStart = i;
                }
            }
            runs.Add((runStart, lodCount - 1, _lodIsDashed[runStart]));

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
                        positionsBuffer[i] = _lodPoints[run.start + l * maxPointsPerLine + i];
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

                // Calculate marker scale based on distance from camera
                Transform parent = GetTrajectoryParent();
                Vector3 worldPos = parent.TransformPoint(timeMarkerInstance.transform.localPosition);
                Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
                float distance = Vector3.Distance(camPos, worldPos);
                
                float pixelHeight = Camera.main != null ? Camera.main.pixelHeight : 1080f;
                float fov = Camera.main != null ? Camera.main.fieldOfView : 60f;
                float frustumHeight = 2f * distance * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
                float markerScale = frustumHeight * 5f / pixelHeight;
                markerScale = Mathf.Clamp(markerScale, 0.01f, 1.0f);
                
                if (!float.IsNaN(markerScale) && !float.IsInfinity(markerScale) && markerScale > 0.00001f)
                    timeMarkerInstance.transform.localScale = Vector3.one * markerScale;
            }
        }

        private Transform GetTrajectoryParent()
        {
            if (universeManager != null && universeManager.ManeuverTrajectoryRoot != null)
                return universeManager.ManeuverTrajectoryRoot;
            if (universeManager != null && universeManager.TrajectoryVisualRoot != null)
                return universeManager.TrajectoryVisualRoot;
            return transform;
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
            cachedBallisticPoints.Clear();
            cachedBallisticTimes.Clear();
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

        /// <summary>
        /// Tries to get the segment boundary state at the specified index.
        /// Segment boundaries represent the state after each maneuver node's delta-v is applied.
        /// </summary>
        /// <param name="index">The segment index (0-based)</param>
        /// <param name="boundary">The boundary state if found</param>
        /// <returns>True if the boundary exists, false otherwise</returns>
        public bool TryGetSegmentBoundaryState(int index, out SegmentBoundaryState boundary)
        {
            boundary = default;
            
            if (index < 0 || index >= cachedBoundaries.Count)
                return false;
            
            boundary = cachedBoundaries[index];
            return true;
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

            // Если job ещё выполняется — не делаем полный пересчёт
            if (_moonPredJobRunning)
                return;

            EnsureMoonPredictionCaches(moonCount);
            bool show = universeManager.CameraMode == SpaceCameraMode.OrbitMap;
            ReferenceFrameTarget frame = universeManager.ActiveReferenceFrame;
            // ── Dynamic sample count ──────────────────────────────────────────────
            double predictionSpan = endTime - simTime;
            int samples;
            {
                const int kMaxSamples = 16384;
                double shortestPeriod = double.PositiveInfinity;
                // Reuse buffer to avoid GC allocation
                if (_moonOrbitDataCache == null || _moonOrbitDataCache.Length < moonCount)
                    _moonOrbitDataCache = new MoonOrbitData[moonCount];
                universeManager.FillMoonOrbitData(_moonOrbitDataCache, 0, moonCount, simTime);
                var tmpOrbits = _moonOrbitDataCache;
                for (int _i = 0; _i < moonCount; _i++)
                {
                    double _sma = tmpOrbits[_i].SemiMajorAxis;
                    double _mu  = tmpOrbits[_i].GravitationalParameter;
                    if (_sma > 0 && _mu > 0)
                    {
                        double _period = 2.0 * Math.PI * Math.Sqrt(_sma * _sma * _sma / _mu);
                        if (_period > 1.0 && _period < shortestPeriod)
                            shortestPeriod = _period;
                    }
                }
                if (double.IsInfinity(shortestPeriod))
                {
                    samples = Math.Max(2, moonPredictionSamples);
                }
                else
                {
                    double orbitsInSpan = predictionSpan / shortestPeriod;
                    int needed = (int)Math.Ceiling(orbitsInSpan * Math.Max(1, moonPredictionSamplesPerOrbit));
                    samples = Math.Max(moonPredictionSamples, needed);
                    samples = Math.Max(2, Math.Min(kMaxSamples, samples));
                }
            }

            // lineWidth and markerScale will be calculated per-line based on distance
            float markerScale = 0.4f; // default fallback

            // ── Precompute sample times and frame positions at each time ──────────
            // Same as spacecraft trajectory: each point uses framePos AT THAT TIME,
            // so (moonPos_t - framePos_t) creates realistic paths (loops etc.) in moving frames.
            // Use cached arrays to avoid per-frame GC pressure.
            if (_moonSampleTimesCache.Length < samples)
            {
                _moonSampleTimesCache = new double[samples];
                _moonFramePosCache    = new Vector3d[samples];
            }
            double[] sampleTimes    = _moonSampleTimesCache;
            Vector3d[] framePosAtTime = _moonFramePosCache;
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

            // ── Cache check: early return if inputs haven't changed ──────────────
            bool endTimeChanged = Math.Abs(endTime - _cachedMoonEndTime) > 1.0;
            bool frameChanged = frame != _cachedMoonFrame;
            bool timeElapsed = (Time.unscaledTime - _moonVisLastRebuildRealTime) >= moonVisRebuildIntervalReal;
            bool needsRebuild = endTimeChanged || frameChanged || timeElapsed;
            
            if (!needsRebuild)
            {
                // Sync visibility only
                for (int i = 0; i < moonCount; i++)
                {
                    var c = moonPredictionCaches[i];
                    if (c.Line != null && c.Line.gameObject.activeSelf != show)
                        c.Line.gameObject.SetActive(show);
                    if (c.Marker != null && c.Marker.activeSelf != show)
                        c.Marker.SetActive(show);
                }
                return;
            }

            for (int i = 0; i < moonCount; i++)
            {
                var cache = moonPredictionCaches[i];

                // ── Material / color setup (only when slider changed) ────────────
                bool cacheEndTimeChanged = Math.Abs(endTime - cache.CachedEndTime) > 1.0;
                if (cacheEndTimeChanged || cache.ColorDirty)
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

                    cache.Line.positionCount = samples; // Set position count BEFORE SetPositions
                    cache.Line.SetPositions(cache.LocalPositionsBuffer);
                    
                    // Calculate width based on average distance from camera to line points
                    float avgDistance = 0f;
                    Transform parent = GetTrajectoryParent();
                    Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;
                    int sampleCount = Mathf.Min(5, samples);
                    for (int j = 0; j < sampleCount; j++)
                    {
                        int idx = (j * samples) / Mathf.Max(1, sampleCount);
                        Vector3 worldPos = parent.TransformPoint(cache.LocalPositionsBuffer[idx]);
                        avgDistance += Vector3.Distance(camPos, worldPos);
                    }
                    avgDistance /= sampleCount;
                    
                    float pixelHeight = Camera.main != null ? Camera.main.pixelHeight : 1080f;
                    float fov = Camera.main != null ? Camera.main.fieldOfView : 60f;
                    float frustumHeight = 2f * avgDistance * Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);
                    float lineWidth = frustumHeight * 1.5f / pixelHeight;
                    lineWidth = Mathf.Clamp(lineWidth, 0.01f, 0.5f);
                    
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
            
            // Update cache after successful recalculation
            _cachedMoonEndTime = endTime;
            _cachedMoonSimTime = simTime;
            _cachedMoonFrame = frame;
            _moonVisLastRebuildRealTime = Time.unscaledTime;
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

        // ─── MoonPredictionLinesJob: Schedule ─────────────────────────
        private void ScheduleMoonPredictionJob()
        {
            if (_moonPredJobRunning) return;

            if (universeManager == null) return;

            double simTime = universeManager.SimulationTimeSeconds;
            double endTime = universeManager.TrajectoryPreviewEndTime;
            if (endTime <= simTime + 0.5) return;

            int moonCount = universeManager.MoonCount;
            if (moonCount == 0) return;

            // Проверить, нужно ли пересчитывать
            bool rebuild = _moonPredNeedsRebuild ||
                Math.Abs(endTime - _moonPredEndTime) > 1.0 ||
                universeManager.ActiveReferenceFrame != _moonPredFrame ||
                (Time.unscaledTime - _moonPredLastRebuildRealTime) >= moonVisRebuildIntervalReal;
            if (!rebuild) return;

            int samples = 0;
            {
                const int kMaxSamples = 16384;
                double shortestPeriod = double.PositiveInfinity;
                if (_moonOrbitDataCache == null || _moonOrbitDataCache.Length < moonCount)
                    _moonOrbitDataCache = new MoonOrbitData[moonCount];
                universeManager.FillMoonOrbitData(_moonOrbitDataCache, 0, moonCount, simTime);
                var tmpOrbits = _moonOrbitDataCache;
                for (int _i = 0; _i < moonCount; _i++)
                {
                    double _sma = tmpOrbits[_i].SemiMajorAxis;
                    double _mu  = tmpOrbits[_i].GravitationalParameter;
                    if (_sma > 0 && _mu > 0)
                    {
                        double _period = 2.0 * Math.PI * Math.Sqrt(_sma * _sma * _sma / _mu);
                        if (_period > 1.0 && _period < shortestPeriod)
                            shortestPeriod = _period;
                    }
                }
                double predictionSpan = endTime - simTime;
                if (double.IsInfinity(shortestPeriod))
                    samples = Math.Max(2, moonPredictionSamples);
                else
                {
                    double orbitsInSpan = predictionSpan / shortestPeriod;
                    int needed = (int)Math.Ceiling(orbitsInSpan * Math.Max(1, moonPredictionSamplesPerOrbit));
                    samples = Math.Max(moonPredictionSamples, needed);
                    samples = Math.Max(2, Math.Min(kMaxSamples, samples));
                }
            }

            ReferenceFrameTarget frame = universeManager.ActiveReferenceFrame;

            // Выделить/переиспользовать нативные буферы
            int total = moonCount * samples;
            if (!_moonPredResults.IsCreated || _moonPredResults.Length < total)
            {
                if (_moonPredResults.IsCreated) _moonPredResults.Dispose();
                _moonPredResults = new NativeArray<float3>(total, Allocator.Persistent);
            }
            if (!_moonPredSampleTimes.IsCreated || _moonPredSampleTimes.Length < samples)
            {
                if (_moonPredSampleTimes.IsCreated) _moonPredSampleTimes.Dispose();
                _moonPredSampleTimes = new NativeArray<double>(samples, Allocator.Persistent);
                if (_moonPredFramePositions.IsCreated) _moonPredFramePositions.Dispose();
                _moonPredFramePositions = new NativeArray<double3>(samples, Allocator.Persistent);
            }
            if (!_moonPredOrbits.IsCreated || _moonPredOrbits.Length < moonCount)
            {
                if (_moonPredOrbits.IsCreated) _moonPredOrbits.Dispose();
                _moonPredOrbits = new NativeArray<MoonOrbitData>(moonCount, Allocator.Persistent);
            }

            // Заполнить времена и позиции фрейма (дёшево, Kepler тут нет)
            for (int j = 0; j < samples; j++)
            {
                double t = simTime + (endTime - simTime) * (j / (double)(samples - 1));
                _moonPredSampleTimes[j] = t;
                if (!universeManager.TryGetReferenceStateAtTime(
                        frame, t, out _, out Vector3d fp, out _, out _, out _, out _))
                    fp = universeManager.JupiterPosition;
                _moonPredFramePositions[j] = new double3(fp.X, fp.Y, fp.Z);
            }

            // Заполнить орбиты
            if (_moonOrbitDataCache == null || _moonOrbitDataCache.Length < moonCount)
                _moonOrbitDataCache = new MoonOrbitData[moonCount];
            universeManager.FillMoonOrbitData(_moonOrbitDataCache, 0, moonCount, simTime);
            for (int i = 0; i < moonCount; i++)
                _moonPredOrbits[i] = _moonOrbitDataCache[i];

            _moonPredSamplesPerMoon = samples;
            _moonPredMoonCount = moonCount;
            _moonPredSimTime = simTime;
            _moonPredEndTime = endTime;
            _moonPredLastRebuildRealTime = Time.unscaledTime;
            _moonPredFrame = frame;
            _moonPredNeedsRebuild = false;

            int planeMapping = universeManager.CurrentPlaneMapping ==
                AstrodynamicPlaneMapping.UnityXyPlaneZUp ? 0 : 1;

            var job = new MoonPredictionLinesJob
            {
                Orbits = _moonPredOrbits,
                SampleTimes = _moonPredSampleTimes,
                FramePositions = _moonPredFramePositions,
                JupiterPosition = new double3(
                    universeManager.JupiterPosition.X,
                    universeManager.JupiterPosition.Y,
                    universeManager.JupiterPosition.Z),
                SamplesPerMoon = samples,
                PlaneMapping = planeMapping,
                Results = _moonPredResults
            };

            _moonPredJobHandle = job.Schedule(total, 64);
            _moonPredJobRunning = true;
        }

        // ─── MoonPredictionLinesJob: Complete and apply ──────────────
        private void CompleteMoonPredictionJob()
        {
            if (!_moonPredJobRunning) return;
            if (!_moonPredJobHandle.IsCompleted) return;

            _moonPredJobHandle.Complete();
            _moonPredJobRunning = false;

            // Синхронизировать кэш-переменные, чтобы UpdateMoonPredictionVisuals
            // не запускал повторный пересчёт сразу после завершения джоба
            _cachedMoonSimTime = _moonPredSimTime;
            _cachedMoonEndTime = _moonPredEndTime;
            _cachedMoonFrame   = _moonPredFrame;
            _moonVisLastRebuildRealTime = Time.unscaledTime;

            // Проверить, что кэши LineRenderer существуют
            int moonCount = _moonPredMoonCount;
            if (moonCount == 0 || moonCount > moonPredictionCaches.Count) return;

            bool show = universeManager != null &&
                universeManager.CameraMode == SpaceCameraMode.OrbitMap;

            for (int i = 0; i < moonCount; i++)
            {
                var cache = moonPredictionCaches[i];
                if (cache.Line == null) continue;

                int start = i * _moonPredSamplesPerMoon;
                if (cache.LocalPositionsBuffer == null ||
                    cache.LocalPositionsBuffer.Length < _moonPredSamplesPerMoon)
                    cache.LocalPositionsBuffer = new Vector3[_moonPredSamplesPerMoon];

                for (int j = 0; j < _moonPredSamplesPerMoon; j++)
                    cache.LocalPositionsBuffer[j] = _moonPredResults[start + j];

                cache.Line.positionCount = _moonPredSamplesPerMoon;
                cache.Line.SetPositions(cache.LocalPositionsBuffer);

                if (cache.Line.gameObject.activeSelf != show)
                    cache.Line.gameObject.SetActive(show);
            }
        }

        // ─── Dispose MoonPredictionLinesJob buffers ──────────────────
        private void DisposeMoonPredictionBuffers()
        {
            if (_moonPredJobRunning)
            {
                _moonPredJobHandle.Complete();
                _moonPredJobRunning = false;
            }
            if (_moonPredResults.IsCreated) _moonPredResults.Dispose();
            if (_moonPredSampleTimes.IsCreated) _moonPredSampleTimes.Dispose();
            if (_moonPredFramePositions.IsCreated) _moonPredFramePositions.Dispose();
            if (_moonPredOrbits.IsCreated) _moonPredOrbits.Dispose();
        }
    }
}

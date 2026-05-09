using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Galilego.Gameplay;
using UnityEngine.SceneManagement;

namespace Galilego.Physics
{
    public enum ReferenceFrameTarget
    {
        Jupiter = 0,
        Io = 1,
        Europa = 2,
        Ganymede = 3,
        Callisto = 4
    }

    public enum AstrodynamicPlaneMapping
    {
        UnityXzPlaneYUp = 0,
        UnityXyPlaneZUp = 1
    }

    public enum SpaceCameraMode
    {
        ShipFocus = 0,
        OrbitMap = 1
    }

    public sealed class UniverseManager : MonoBehaviour
    {
        [Header("Jupiter")]
        [SerializeField] private Transform jupiterTransform;
        [SerializeField] private double jupiterMass = 1.89813e27d;
        [SerializeField] private double jupiterStandardGravitationalParameter = 1.266865319e17d;
        [SerializeField] private double jupiterRadius = 6.9911e7d;
        [SerializeField] private Vector3d jupiterRealPosition = Vector3d.Zero;
        [SerializeField] private Vector3 jupiterNorthLocalDirection = Vector3.up;

        [Header("Ship")]
        [SerializeField] private ShipSettings ship = new ShipSettings();

        [Header("Moon Rails")]
        [SerializeField] private List<MoonRail> moonRails = new List<MoonRail>();

        [Header("Scene")]
        [SerializeField] private Transform worldContainer;

        [Header("Camera Modes")]
        [SerializeField] private Camera celestialCamera;
        [SerializeField] private Camera shipOverlayCamera;
        [SerializeField] private SpaceCameraMode cameraMode = SpaceCameraMode.ShipFocus;

        public SpaceCameraMode CameraMode => cameraMode;
[SerializeField] private bool enableCameraToggle = true;
        [SerializeField] private double shipFocusDistanceMeters = 24d;
        [SerializeField] private double shipFocusHeightMeters = 8d;
        [SerializeField] private float shipFocusFieldOfView = 45f;
        [SerializeField] private float shipFocusYawDegrees;
        [SerializeField] private float shipFocusPitchDegrees = 12f;
        [SerializeField] private double minShipFocusDistanceMeters = 0.5d;
        [SerializeField] private double maxShipFocusDistanceMeters = 5e7d;
        [SerializeField] private float shipZoomSensitivity = 0.18f;
        [SerializeField] private float orbitMapYawDegrees;
        [SerializeField] private float orbitMapPadding = 1.25f;
        [SerializeField] private float orbitMapPitchDegrees = 80f;
        [SerializeField] private float minOrbitMapOrthographicSize = 10f;
        [SerializeField] private float maxOrbitMapZoomMultiplier = 6f;
        [SerializeField] private float orbitMapZoomSensitivity = 0.12f;
        [SerializeField] private float orbitMapFieldOfView = 45f;
        [SerializeField] private float orbitMapMinMoonScreenRadius = 5f;
        [SerializeField] private float orbitMapMinJupiterScreenRadius = 18f;
        [SerializeField] private double orbitMapSurfaceClearanceMeters = 500d;
        [SerializeField] private float cameraOrbitSensitivity = 0.18f;
        [SerializeField] private float cameraRotationSmoothing = 14f;
        [SerializeField] private float cameraZoomSmoothing = 10f;
        [SerializeField] private ReferenceFrameTarget orbitMapFocusTarget = ReferenceFrameTarget.Jupiter;

        [Header("Visual Scale")]
        [SerializeField] private double visualDistanceMultiplier = 0.1d;

        [Header("Simulation")]
        [SerializeField] private double simulationTimeSeconds;
        [SerializeField] private double timeScale = 1d;
        [SerializeField] private double maxSolverStepSeconds = 1d;
        [SerializeField] private double metersPerUnityUnit = 100000d;
        [SerializeField] private double floatingOriginThreshold = 5000d;

        [Header("Reference Frames")]
        [SerializeField] private ReferenceFrameTarget selectedReferenceFrame = ReferenceFrameTarget.Jupiter;
        [SerializeField] private AstrodynamicPlaneMapping astrodynamicPlaneMapping = AstrodynamicPlaneMapping.UnityXzPlaneYUp;
        [SerializeField] private bool autoSelectSphereOfInfluence;
        [SerializeField] private bool enableReferenceFrameHotkeys = true;
        [SerializeField] private KeyCode cycleReferenceFrameKey = KeyCode.Tab;

        [Header("Trajectory Visuals")]
        [SerializeField] private bool showShipTrajectory = true;
        [SerializeField] private bool showMoonOrbits = true;
        [SerializeField] private int shipPredictionSteps = 1200;
        [SerializeField] private double shipPredictionStepSeconds = 10d;
        [SerializeField] private float shipPredictionRefreshInterval = 0.2f;
        [SerializeField] private int shipPredictionStepsPerBatch = 128;
        [SerializeField] private int moonOrbitSamples = 256;
        [SerializeField] private float shipTrajectoryWidth = 0.2f;
        [SerializeField] private float moonOrbitWidth = 0.08f;
        [SerializeField] private float moonOrbitScreenWidth = 3f;
        [SerializeField] private float moonOrbitTailAlpha = 1f;
        [SerializeField] private float moonOrbitAheadAlpha = 0.08f;
        [SerializeField] [Range(0f, 1f)] private float moonOrbitHistoryFraction = 1f;
        [SerializeField] private Color shipTrajectoryColor = new Color(0.25f, 0.95f, 1f, 0.95f);
        [SerializeField] private Color moonOrbitColor = new Color(0.65f, 0.85f, 1f, 0.4f);

        [Header("Telemetry Overlay")]
        [SerializeField] private bool showTelemetryOverlay = true;
        [SerializeField] private Rect telemetryOverlayRect = new Rect(12f, 12f, 460f, 420f);
        [SerializeField] private Vector2 telemetryOverlayMinSize = new Vector2(320f, 220f);
        [SerializeField] private Vector2 telemetryOverlayMaxSize = new Vector2(900f, 760f);

        [Header("Debug")]
        [SerializeField] private bool warnOnInvalidCoordinates = true;

        private bool hasWarnedAboutInvalidCoordinates;

        private readonly List<CelestialBody> moonBodies = new List<CelestialBody>();
        private readonly List<LineRenderer> moonOrbitRenderers = new List<LineRenderer>();

        private CelestialBody jupiterBody;
        private CelestialBody shipBody;
        private Vector3d floatingOriginOffset = Vector3d.Zero;
        private Transform trajectoryVisualRoot;
        private Transform moonOrbitRoot;
        private Transform orbitMapMarkerRoot;
        private LineRenderer jupiterOrbitRenderer;
        private TrajectoryPredictor shipTrajectoryPredictor;
        private Material runtimeLineMaterial;
        private Material runtimeOrbitMapMarkerMaterial;
        private ReferenceFrameTarget lastActiveReferenceFrame = ReferenceFrameTarget.Jupiter;
        private float orbitMapOrthographicSize;
        private bool cameraStateInitialized;
        private double smoothedShipFocusDistanceMeters;
        private float smoothedShipFocusYawDegrees;
        private float smoothedShipFocusPitchDegrees;
        private float smoothedOrbitMapYawDegrees;
        private float smoothedOrbitMapPitchDegrees;
        private float smoothedOrbitMapOrthographicSize;
        private Vector2 telemetryOverlayScroll;
        private bool isResizingTelemetryOverlay;
        private Vector2 telemetryResizeStartMouse;
        private Vector2 telemetryResizeStartSize;
        private readonly Dictionary<ReferenceFrameTarget, Transform> orbitMapMarkers = new Dictionary<ReferenceFrameTarget, Transform>();
        private readonly Dictionary<ReferenceFrameTarget, Renderer> orbitMapMarkerRenderers = new Dictionary<ReferenceFrameTarget, Renderer>();
        private readonly List<Renderer> rendererBuffer = new List<Renderer>();

        private const float MinimumCameraNearClipPlane = 0.0000001f;
        private const float MaximumCelestialNearClipPlane = 0.01f;
        private const float MarkerVisibleScreenRadiusMultiplier = 3f;
        private const double DefaultOrbitMapSurfaceClearanceMeters = 500d;
        private const double MinimumOrbitMapSurfaceClearanceMeters = 50d;
        private const double MaximumOrbitMapSurfaceClearanceMeters = 2500d;
        private const double OrbitMapSurfaceClearanceRadiusFraction = 0.0002d;

        public event Action<ReferenceFrameTarget> ActiveReferenceFrameChanged;

        public IReadOnlyList<CelestialBody> MoonBodies => moonBodies;
        public CelestialBody ShipBody => shipBody;
        public Transform ShipVisualTransform => ship != null ? ship.VisualTransform : null;
        public double SimulationTimeSeconds => simulationTimeSeconds;
        public double RecommendedSolverStepSeconds => maxSolverStepSeconds;
        public double MetersPerUnityUnit => GetMetersPerVisualUnit();
        public Vector3d FloatingOriginOffset => floatingOriginOffset;
        public ReferenceFrameTarget SelectedReferenceFrame => selectedReferenceFrame;
        public ReferenceFrameTarget ActiveReferenceFrame => ResolveActiveReferenceFrameTarget();
        public bool IsAutoSphereOfInfluenceSelectionEnabled => autoSelectSphereOfInfluence;
        public Vector3 AstrodynamicNorthUnityDirection => ToUnityDirection(ConvertAstrodynamicToSimulationFrame(new Vector3d(0d, 0d, 1d)));
        public Vector3 AstrodynamicEastUnityDirection => ToUnityDirection(ConvertAstrodynamicToSimulationFrame(new Vector3d(1d, 0d, 0d)));

        // Preview time offset for UI live-preview (seconds). When non-zero, visuals
        // and orbit samples will be computed for SimulationTimeSeconds + this offset.
        private double previewTimeOffsetSeconds = 0d;
        public double PreviewTimeOffsetSeconds { get => previewTimeOffsetSeconds; set => previewTimeOffsetSeconds = value; }
        /// <summary>
        /// End of plotted maneuver trajectory in simulation timeline (simNow + planner horizon).
        /// Does not animate planets; only visuals that explicitly read this time use it (e.g. marker at path end).
        /// </summary>
        public double TrajectoryPreviewEndTime => simulationTimeSeconds + Math.Max(0d, previewTimeOffsetSeconds);

        public double PreviewAbsoluteTime => TrajectoryPreviewEndTime;

        public float MoonOrbitHistoryFraction
        {
            get => moonOrbitHistoryFraction;
            set => moonOrbitHistoryFraction = Mathf.Clamp01(value);
        }

        private void Awake()
        {
            ResolveCameraReferences();
            DetachCameraRigFromShipIfNeeded();
            InitializeBodies();
            ApplyFloatingOriginIfNeeded();
            SyncAllVisuals();
            EnsureTrajectoryVisuals();
            lastActiveReferenceFrame = ResolveActiveReferenceFrameTarget();
            ApplyCameraMode();
        }

        private void Reset()
        {
            if (moonRails.Count == 0)
            {
                LoadJplGalileanMoonRails();
            }
        }

        private void FixedUpdate()
        {
            EnsureInitialized();

            double frameDt = Time.fixedDeltaTime * timeScale;
            int stepCount = GetSolverStepCount(frameDt);
            if (stepCount > 2048)
            {
                Debug.LogWarning($"UniverseManager: clamping solver stepCount from {stepCount} to 2048 to avoid long frame/blocking.");
                stepCount = 2048;
            }
            double stepDt = frameDt / stepCount;

            for (int i = 0; i < stepCount; i++)
            {
                StepSimulation(stepDt);
            }

            ApplyFloatingOriginIfNeeded();
            SyncAllVisuals();
        }

        private void Update()
        {
            if (enableReferenceFrameHotkeys && WasCycleReferenceFramePressed())
            {
                CycleReferenceFrame();
            }

            if (enableCameraToggle && WasCameraTogglePressed())
            {
                ToggleCameraMode();
            }

            HandleCameraInput();
            RefreshReferenceFrameChange();
        }

        private void LateUpdate()
        {
            ApplyCameraMode();
            if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                RebuildMoonOrbitLines();
            }

            RefreshTrajectoryLineStyles();
        }

        private void OnGUI()
        {
            if (!showTelemetryOverlay)
            {
                return;
            }

            telemetryOverlayRect = GUILayout.Window(
                913751,
                telemetryOverlayRect,
                DrawTelemetryWindow,
                "Navigation Frame");
            telemetryOverlayRect = ClampTelemetryOverlayRect(telemetryOverlayRect);
        }

        public void SyncVisualFromRealCoordinates()
        {
            SyncAllVisuals();
        }

        public void ApplyVisualPosition(Transform target, Vector3d realPosition)
        {
            if (target == null)
            {
                return;
            }

            if (!realPosition.IsFinite)
            {
                WarnInvalidCoordinates(target.name, realPosition);
                return;
            }

            if (worldContainer != null && target.IsChildOf(worldContainer))
            {
                Vector3 localPosition = ToUnityLocalPosition(realPosition);
                if (!IsFinite(localPosition))
                {
                    WarnInvalidCoordinates(target.name, realPosition);
                    return;
                }

                target.localPosition = localPosition;
                return;
            }

            Vector3 worldPosition = ToUnityPosition(realPosition);
            if (!IsFinite(worldPosition))
            {
                WarnInvalidCoordinates(target.name, realPosition);
                return;
            }

            target.position = worldPosition;
        }

        public Vector3 ToUnityPosition(Vector3d realPosition)
        {
            Vector3d localPosition = (realPosition - floatingOriginOffset) / GetMetersPerVisualUnit();
            return new Vector3((float)localPosition.X, (float)localPosition.Y, (float)localPosition.Z);
        }

        public Vector3 ToUnityLocalPosition(Vector3d realPosition)
        {
            Vector3d localPosition = realPosition / GetMetersPerVisualUnit();
            return new Vector3((float)localPosition.X, (float)localPosition.Y, (float)localPosition.Z);
        }

        public Vector3 ToUnityOffset(Vector3d realOffset)
        {
            Vector3d scaledOffset = realOffset / GetMetersPerVisualUnit();
            return new Vector3((float)scaledOffset.X, (float)scaledOffset.Y, (float)scaledOffset.Z);
        }

        public Vector3 ToUnityDirection(Vector3d realDirection)
        {
            if (!realDirection.IsFinite || realDirection.SqrMagnitude <= 0d)
            {
                return Vector3.zero;
            }

            Vector3 direction = new Vector3((float)realDirection.X, (float)realDirection.Y, (float)realDirection.Z);
            return direction.sqrMagnitude > 0f ? direction.normalized : Vector3.zero;
        }

        public void SelectReferenceFrame(ReferenceFrameTarget target)
        {
            bool changed = selectedReferenceFrame != target || autoSelectSphereOfInfluence;
            selectedReferenceFrame = target;
            autoSelectSphereOfInfluence = false;

            if (changed)
            {
                NotifyReferenceFrameStateChanged();
            }
        }

        public void SelectReferenceFrame(int targetIndex)
        {
            if (!Enum.IsDefined(typeof(ReferenceFrameTarget), targetIndex))
            {
                return;
            }

            SelectReferenceFrame((ReferenceFrameTarget)targetIndex);
        }

        public void SetAutoSphereOfInfluenceSelection(bool enabled)
        {
            bool changed = autoSelectSphereOfInfluence != enabled;
            autoSelectSphereOfInfluence = enabled;

            if (changed)
            {
                NotifyReferenceFrameStateChanged();
            }
        }

        public void CycleReferenceFrame()
        {
            int next = ((int)selectedReferenceFrame + 1) % Enum.GetValues(typeof(ReferenceFrameTarget)).Length;
            SelectReferenceFrame((ReferenceFrameTarget)next);
        }

        public void ToggleCameraMode()
        {
            SetCameraMode(cameraMode == SpaceCameraMode.ShipFocus
                ? SpaceCameraMode.OrbitMap
                : SpaceCameraMode.ShipFocus);
        }

        public void SetCameraMode(SpaceCameraMode mode)
        {
            if (cameraMode == mode)
            {
                return;
            }

            cameraMode = mode;
            if (cameraMode == SpaceCameraMode.ShipFocus)
            {
                ApplyFloatingOriginIfNeeded();
                SyncAllVisuals();
            }

            ApplyCameraMode();
            EnsureTrajectoryVisuals();
            if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                RebuildMoonOrbitLines();
            }
        }

        public void FocusOrbitMapOn(ReferenceFrameTarget target)
        {
            if (!Enum.IsDefined(typeof(ReferenceFrameTarget), target))
            {
                return;
            }

            orbitMapFocusTarget = target;
            cameraMode = SpaceCameraMode.OrbitMap;

            double focusRadiusMeters = ResolveReferenceRadius(target);
            double targetSizeMeters = target == ReferenceFrameTarget.Jupiter
                ? jupiterRadius * 3d
                : Math.Max(focusRadiusMeters * 6d, 5e6d);

            orbitMapOrthographicSize = ClampOrbitMapOrthographicSize(MetersToVisualUnitsFloat(targetSizeMeters));
            smoothedOrbitMapOrthographicSize = orbitMapOrthographicSize;
            RebuildMoonOrbitLines();
            ApplyCameraMode();
        }

        public bool TryGetShipRelativeState(
            ReferenceFrameTarget target,
            out string frameName,
            out Vector3d relativePosition,
            out Vector3d relativeVelocity,
            out double referenceStandardGravitationalParameter,
            out double referenceRadius,
            out double sphereOfInfluenceRadius)
        {
            EnsureInitialized();

            relativePosition = Vector3d.Zero;
            relativeVelocity = Vector3d.Zero;

            if (!TryGetReferenceState(
                target,
                out frameName,
                out Vector3d framePosition,
                out Vector3d frameVelocity,
                out referenceStandardGravitationalParameter,
                out referenceRadius,
                out sphereOfInfluenceRadius))
            {
                return false;
            }

            relativePosition = shipBody.Position - framePosition;
            relativeVelocity = shipBody.Velocity - frameVelocity;
            return true;
        }

        public OrbitalElements GetShipOrbitAround(ReferenceFrameTarget target)
        {
            if (!TryGetShipRelativeState(
                target,
                out _,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out double referenceStandardGravitationalParameter,
                out _,
                out _))
            {
                return OrbitalElements.Invalid;
            }

            return OrbitalElements.FromState(
                ConvertSimulationToAstrodynamicFrame(relativePosition),
                ConvertSimulationToAstrodynamicFrame(relativeVelocity),
                referenceStandardGravitationalParameter);
        }

        private void ApplyVisualScale(Transform target, double realRadiusMeters)
        {
            if (target == null)
            {
                return;
            }

            if (realRadiusMeters <= 0d)
            {
                return;
            }

            double metersPerVisual = GetMetersPerVisualUnit();
            if (metersPerVisual <= 0d)
            {
                return;
            }

            // Unity primitive sphere has radius 0.5 at localScale == 1, so we need diameter in units.
            double desiredDiameterInUnits = (realRadiusMeters / metersPerVisual) * 2d;
            if (double.IsNaN(desiredDiameterInUnits) || double.IsInfinity(desiredDiameterInUnits))
            {
                Debug.LogWarning($"UniverseManager: invalid scale computed for '{target.name}': {desiredDiameterInUnits}");
                return;
            }

            float uniformScale = (float)desiredDiameterInUnits;

            // Clamp to reasonable bounds to avoid disappearing or exploding objects at runtime.
            const float minScale = 0.000000001f;
            const float maxScale = 10000f;
            float clamped = Mathf.Clamp(uniformScale, minScale, maxScale);
            if (!Mathf.Approximately(clamped, uniformScale))
            {
                Debug.LogWarning($"UniverseManager: clamped visual scale for '{target.name}' from {uniformScale} to {clamped}");
            }

            // Only set scale if target or its descendants contain a renderer; otherwise skip to avoid scaling empty parents.
            bool hasRenderer = target.GetComponentInChildren<Renderer>(true) != null;
            if (!hasRenderer)
            {
                Debug.LogWarning($"UniverseManager: skipped scaling '{target.name}' because no Renderer found in children.");
                return;
            }
            // Prefer scaling based on the actual mesh bounds if available so arbitrary meshes scale correctly.
            MeshFilter meshFilter = target.GetComponentInChildren<MeshFilter>(true);
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                Transform meshTransform = meshFilter.transform;
                Bounds meshBounds = meshFilter.sharedMesh.bounds;
                float meshRadiusLocal = Math.Max(meshBounds.extents.x, Math.Max(meshBounds.extents.y, meshBounds.extents.z));
                if (meshRadiusLocal > 0f)
                {
                    // Current world radius = meshRadiusLocal * lossyScale
                    Vector3 lossy = meshTransform.lossyScale;
                    float currentWorldScale = Math.Max(lossy.x, Math.Max(lossy.y, lossy.z));
                    float currentWorldRadius = meshRadiusLocal * currentWorldScale;

                    double desiredWorldRadius = realRadiusMeters / metersPerVisual;
                    if (currentWorldRadius <= 0f)
                    {
                        Debug.LogWarning($"UniverseManager: currentWorldRadius is zero for '{meshTransform.name}', skipping mesh-based scale.");
                    }
                    else
                    {
                        float scaleMultiplier = (float)(desiredWorldRadius / currentWorldRadius);
                        float finalMultiplier = Mathf.Clamp(scaleMultiplier, minScale, maxScale);

                        if (!Mathf.Approximately(finalMultiplier, scaleMultiplier))
                        {
                            Debug.LogWarning($"UniverseManager: clamped mesh-based multiplier for '{meshTransform.name}' from {scaleMultiplier} to {finalMultiplier}");
                        }

                        if (!Mathf.Approximately(finalMultiplier, 1f))
                        {
                            Vector3 newLocalScale = meshTransform.localScale * finalMultiplier;
                            meshTransform.localScale = newLocalScale;
                            Debug.Log($"UniverseManager: scaled mesh '{meshTransform.name}' to localScale={newLocalScale} (desiredWorldRadius={desiredWorldRadius})");
                        }

                        return;
                    }
                }
            }

            // Fallback: assume model radius 0.5 at localScale == 1
            Vector3 fallbackScale = new Vector3(clamped, clamped, clamped);
            if (!Approximately(target.localScale, fallbackScale))
            {
                target.localScale = fallbackScale;
                Debug.Log($"UniverseManager: scaled '{target.name}' fallback. uniformScale={clamped}");
            }
        }

        public Vector3d EvaluateShipAccelerationAt(Vector3d shipPosition, double sampleTimeSeconds)
        {
            EnsureInitialized();
            return EvaluateShipAcceleration(shipPosition, sampleTimeSeconds);
        }

        private void InitializeBodies()
        {
            jupiterBody = new CelestialBody(
                jupiterMass,
                jupiterRealPosition,
                Vector3d.Zero,
                jupiterStandardGravitationalParameter);
            shipBody = new CelestialBody(ship.Mass, ship.InitialPosition, ship.InitialVelocity);

            moonBodies.Clear();
            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                rail.ApplyPeriapsisAndApoapsis();
                rail.SyncMassFromGravitationalParameter();
                rail.UpdateInfluenceRadii(ResolveJupiterMassForInfluence());
                moonBodies.Add(new CelestialBody(
                    rail.Mass,
                    Vector3d.Zero,
                    Vector3d.Zero,
                    rail.ResolveStandardGravitationalParameter()));
            }

            UpdateMoonBodies(simulationTimeSeconds);
        }

        private void OnValidate()
        {
            if (maxSolverStepSeconds <= 0d)
            {
                maxSolverStepSeconds = 1d;
            }

            if (visualDistanceMultiplier <= 0d)
            {
                visualDistanceMultiplier = 0.1d;
            }

            if (metersPerUnityUnit <= 0d)
            {
                metersPerUnityUnit = 1d;
            }

            if (floatingOriginThreshold <= 0d)
            {
                floatingOriginThreshold = 5000d;
            }

            if (ship != null && ship.VisualRadiusMeters <= 0d)
            {
                ship.VisualRadiusMeters = 3d;
            }

            if (shipFocusDistanceMeters <= 0d)
            {
                shipFocusDistanceMeters = 24d;
            }

            if (shipFocusHeightMeters < 0d)
            {
                shipFocusHeightMeters = 0d;
            }

            if (minShipFocusDistanceMeters <= 0d)
            {
                minShipFocusDistanceMeters = 0.5d;
            }

            if (maxShipFocusDistanceMeters <= minShipFocusDistanceMeters)
            {
                maxShipFocusDistanceMeters = minShipFocusDistanceMeters * 2d;
            }

            shipFocusPitchDegrees = Mathf.Clamp(shipFocusPitchDegrees, -85f, 85f);

            if (shipFocusFieldOfView <= 0f)
            {
                shipFocusFieldOfView = 45f;
            }

            if (orbitMapPadding < 1f)
            {
                orbitMapPadding = 1f;
            }

            orbitMapPitchDegrees = NormalizeDegrees(orbitMapPitchDegrees);

            if (minOrbitMapOrthographicSize <= 0f)
            {
                minOrbitMapOrthographicSize = 10f;
            }

            if (maxOrbitMapZoomMultiplier < 1f)
            {
                maxOrbitMapZoomMultiplier = 1f;
            }

            shipZoomSensitivity = Mathf.Clamp(shipZoomSensitivity, 0.01f, 0.95f);
            orbitMapZoomSensitivity = Mathf.Clamp(orbitMapZoomSensitivity, 0.01f, 0.95f);

            if (orbitMapFieldOfView <= 0f)
            {
                orbitMapFieldOfView = 45f;
            }

            if (orbitMapMinMoonScreenRadius <= 0f)
            {
                orbitMapMinMoonScreenRadius = 5f;
            }

            if (orbitMapMinJupiterScreenRadius <= 0f)
            {
                orbitMapMinJupiterScreenRadius = 18f;
            }

            if (orbitMapSurfaceClearanceMeters <= 0d)
            {
                orbitMapSurfaceClearanceMeters = DefaultOrbitMapSurfaceClearanceMeters;
            }

            if (cameraOrbitSensitivity <= 0f)
            {
                cameraOrbitSensitivity = 0.18f;
            }

            if (cameraRotationSmoothing <= 0f)
            {
                cameraRotationSmoothing = 14f;
            }

            if (cameraZoomSmoothing <= 0f)
            {
                cameraZoomSmoothing = 10f;
            }

            if (!Enum.IsDefined(typeof(ReferenceFrameTarget), orbitMapFocusTarget))
            {
                orbitMapFocusTarget = ReferenceFrameTarget.Jupiter;
            }

            if (shipPredictionSteps < 1)
            {
                shipPredictionSteps = 1;
            }

            if (shipPredictionStepSeconds <= 0d)
            {
                shipPredictionStepSeconds = 0.1d;
            }

            if (shipPredictionRefreshInterval < 0.01f)
            {
                shipPredictionRefreshInterval = 0.01f;
            }

            if (shipPredictionStepsPerBatch < 1)
            {
                shipPredictionStepsPerBatch = 1;
            }

            if (moonOrbitSamples < 16)
            {
                moonOrbitSamples = 16;
            }

            if (shipTrajectoryWidth <= 0f)
            {
                shipTrajectoryWidth = 0.2f;
            }

            if (moonOrbitWidth <= 0f)
            {
                moonOrbitWidth = 0.08f;
            }

            if (moonOrbitScreenWidth <= 0f)
            {
                moonOrbitScreenWidth = 3f;
            }

            moonOrbitTailAlpha = Mathf.Clamp01(moonOrbitTailAlpha);
            moonOrbitAheadAlpha = Mathf.Clamp01(moonOrbitAheadAlpha);
            moonOrbitHistoryFraction = Mathf.Clamp01(moonOrbitHistoryFraction);

            telemetryOverlayMinSize.x = Mathf.Max(240f, telemetryOverlayMinSize.x);
            telemetryOverlayMinSize.y = Mathf.Max(160f, telemetryOverlayMinSize.y);
            telemetryOverlayMaxSize.x = Mathf.Max(telemetryOverlayMinSize.x, telemetryOverlayMaxSize.x);
            telemetryOverlayMaxSize.y = Mathf.Max(telemetryOverlayMinSize.y, telemetryOverlayMaxSize.y);
            telemetryOverlayRect.width = Mathf.Clamp(telemetryOverlayRect.width, telemetryOverlayMinSize.x, telemetryOverlayMaxSize.x);
            telemetryOverlayRect.height = Mathf.Clamp(telemetryOverlayRect.height, telemetryOverlayMinSize.y, telemetryOverlayMaxSize.y);

            if (jupiterStandardGravitationalParameter <= 0d)
            {
                jupiterStandardGravitationalParameter = PhysicsSolver.MassToStandardGravitationalParameter(jupiterMass);
            }

            if (jupiterRadius <= 0d)
            {
                jupiterRadius = 6.9911e7d;
            }

            for (int i = 0; i < moonRails.Count; i++)
            {
                moonRails[i].ApplyPeriapsisAndApoapsis();
                moonRails[i].SyncMassFromGravitationalParameter();
                moonRails[i].UpdateInfluenceRadii(ResolveJupiterMassForInfluence());
            }

            // Avoid creating GameObjects or calling SendMessage during OnValidate.
        }

        private void EnsureInitialized()
        {
            if (shipBody == null || jupiterBody == null || moonBodies.Count != moonRails.Count)
            {
                InitializeBodies();
            }

            jupiterBody.SetState(jupiterRealPosition, Vector3d.Zero);
        }

        private void UpdateMoonBodies(double timeSeconds)
        {
            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                CelestialBody body = moonBodies[i];

                EvaluateMoonState(rail, timeSeconds, out Vector3d position, out Vector3d velocity);
                body.SetState(position, velocity);
            }
        }

        private void EvaluateMoonState(MoonRail rail, double timeSeconds, out Vector3d position, out Vector3d velocity)
        {
            double semiMajorAxis = Math.Max(rail.ResolveSemiMajorAxis(), 1d);
            double eccentricity = Clamp(rail.ResolveEccentricity(), 0d, 0.999d);
            double inclination = DegreesToRadians(rail.InclinationDegrees);
            double ascendingNode = DegreesToRadians(rail.LongitudeOfAscendingNodeDegrees);
            double periapsis = DegreesToRadians(rail.ArgumentOfPeriapsisDegrees);
            double meanAnomalyAtEpoch = DegreesToRadians(rail.MeanAnomalyAtEpochDegrees);

            double gravitationalParameter = jupiterStandardGravitationalParameter + rail.ResolveStandardGravitationalParameter();
            double meanMotion = Math.Sqrt(gravitationalParameter / (semiMajorAxis * semiMajorAxis * semiMajorAxis));
            double meanAnomaly = NormalizeAngle(meanAnomalyAtEpoch + (meanMotion * (timeSeconds - rail.EpochTimeSeconds)));
            double eccentricAnomaly = SolveEccentricAnomaly(meanAnomaly, eccentricity);

            double cosE = Math.Cos(eccentricAnomaly);
            double sinE = Math.Sin(eccentricAnomaly);
            double radius = semiMajorAxis * (1d - (eccentricity * cosE));
            double orbitalYScale = Math.Sqrt(1d - (eccentricity * eccentricity));

            Vector3d orbitalPosition = new Vector3d(
                semiMajorAxis * (cosE - eccentricity),
                semiMajorAxis * orbitalYScale * sinE,
                0d);

            double velocityFactor = Math.Sqrt(gravitationalParameter * semiMajorAxis) / radius;
            Vector3d orbitalVelocity = new Vector3d(
                -velocityFactor * sinE,
                velocityFactor * orbitalYScale * cosE,
                0d);

            position = jupiterRealPosition + ConvertAstrodynamicToSimulationFrame(RotateOrbitalToWorld(orbitalPosition, ascendingNode, inclination, periapsis));
            velocity = ConvertAstrodynamicToSimulationFrame(RotateOrbitalToWorld(orbitalVelocity, ascendingNode, inclination, periapsis));
        }

        private void StepSimulation(double dt)
        {
            double stepStartTime = simulationTimeSeconds;
            IntegrationResult shipStep = PhysicsSolver.RK4(shipBody, stepStartTime, dt, EvaluateShipAcceleration);

            if (!shipStep.Position.IsFinite || !shipStep.Velocity.IsFinite)
            {
                WarnInvalidCoordinates(nameof(shipBody), shipStep.Position);
                enabled = false;
                return;
            }

            simulationTimeSeconds = stepStartTime + dt;
            shipBody.SetState(shipStep.Position, shipStep.Velocity);
            UpdateMoonBodies(simulationTimeSeconds);
        }

        private Vector3d EvaluateShipAcceleration(Vector3d shipPosition, double sampleTimeSeconds)
        {
            Vector3d totalAcceleration = PhysicsSolver.CalculateAccelerationFromStandardGravitationalParameter(
                shipPosition,
                jupiterRealPosition,
                jupiterStandardGravitationalParameter);

            for (int i = 0; i < moonRails.Count; i++)
            {
                EvaluateMoonState(moonRails[i], sampleTimeSeconds, out Vector3d moonPosition, out _);
                totalAcceleration += PhysicsSolver.CalculateAccelerationFromStandardGravitationalParameter(
                    shipPosition,
                    moonPosition,
                    moonRails[i].ResolveStandardGravitationalParameter());
            }

            return totalAcceleration;
        }

        /// <summary>
        /// Try to compute ship state (position, velocity) at an arbitrary sample time
        /// by integrating from the current simulation state. Returns false if ship
        /// is unavailable or integration produced invalid values.
        /// </summary>
        public bool TryGetShipStateAtTime(double sampleTimeSeconds, out Vector3d position, out Vector3d velocity)
        {
            position = Vector3d.Zero; velocity = Vector3d.Zero;
            if (shipBody == null)
            {
                return false;
            }

            // Start from current simulated state (do not modify actual shipBody)
            position = shipBody.Position;
            velocity = shipBody.Velocity;
            double currentTime = simulationTimeSeconds;

            double dtTotal = sampleTimeSeconds - currentTime;
            if (Math.Abs(dtTotal) <= 1e-9) return true;

            double remaining = Math.Abs(dtTotal);
            double dir = Math.Sign(dtTotal);
            double majorStep = Math.Max(1e-6d, maxSolverStepSeconds);

            int iterCount = 0;
            const int iterSafetyLimit = 1000000; // very large cap to avoid runaway

            while (remaining > 0d)
            {
                double step = Math.Min(remaining, majorStep) * dir;
                var res = PhysicsSolver.RK4(position, velocity, currentTime, step, EvaluateShipAcceleration);
                position = res.Position; velocity = res.Velocity; currentTime += step; remaining -= Math.Abs(step);

                iterCount++;
                if (iterCount > iterSafetyLimit)
                {
                    Debug.LogError($"UniverseManager: TryGetShipStateAtTime aborted — exceeded iteration safety limit ({iterSafetyLimit}). sampleTimeSeconds={sampleTimeSeconds}");
                    return false;
                }

                if (!position.IsFinite || !velocity.IsFinite)
                {
                    return false;
                }
            }

            return true;
        }

        private void DrawTelemetryWindow(int windowId)
        {
            ReferenceFrameTarget activeFrame = ResolveActiveReferenceFrameTarget();

            telemetryOverlayScroll = GUILayout.BeginScrollView(telemetryOverlayScroll, false, true);

            GUILayout.BeginHorizontal();
            bool autoSoi = GUILayout.Toggle(autoSelectSphereOfInfluence, "Auto SOI frame", GUILayout.Width(130f));
            if (autoSoi != autoSelectSphereOfInfluence)
            {
                SetAutoSphereOfInfluenceSelection(autoSoi);
            }

            GUILayout.FlexibleSpace();
            GUILayout.Label($"Camera: {cameraMode}");
            GUILayout.EndHorizontal();

            GUILayout.Space(4f);
            GUILayout.Label("Bodies");
            DrawTelemetryTargetRow(ReferenceFrameTarget.Jupiter, activeFrame);
            DrawTelemetryTargetRow(ReferenceFrameTarget.Io, activeFrame);
            DrawTelemetryTargetRow(ReferenceFrameTarget.Europa, activeFrame);
            DrawTelemetryTargetRow(ReferenceFrameTarget.Ganymede, activeFrame);
            DrawTelemetryTargetRow(ReferenceFrameTarget.Callisto, activeFrame);

            if (TryGetShipRelativeState(
                activeFrame,
                out string frameName,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out double frameMu,
                out double frameRadius,
                out double frameSoi))
            {
                double distance = relativePosition.Magnitude;
                double speed = relativeVelocity.Magnitude;
                double radialSpeed = distance > 0d ? Vector3d.Dot(relativePosition, relativeVelocity) / distance : 0d;
                double tangentialSpeedSquared = Math.Max(0d, relativeVelocity.SqrMagnitude - (radialSpeed * radialSpeed));
                double altitude = distance - frameRadius;

                GUILayout.Space(8f);
                GUILayout.Label($"Frame: {frameName}  |  mu {FormatMu(frameMu)}");
                GUILayout.Label($"r: {FormatDistance(distance)}  alt: {FormatDistance(altitude)}");
                GUILayout.Label($"v: {FormatSpeed(speed)}  radial: {FormatSpeed(radialSpeed)}  tangential: {FormatSpeed(Math.Sqrt(tangentialSpeedSquared))}");
                if (!double.IsInfinity(frameSoi) && frameSoi > 0d)
                {
                    GUILayout.Label($"SOI: {FormatDistance(frameSoi)}  fill: {Math.Min(999d, distance / frameSoi):0.000}");
                }

                OrbitalElements orbit = GetShipOrbitAround(activeFrame);
                if (orbit.IsValid)
                {
                    GUILayout.Space(6f);
                    GUILayout.Label("Ship orbit");
                    GUILayout.Label($"a: {FormatDistance(orbit.SemiMajorAxis)}  e: {orbit.Eccentricity:0.000000}");
                    GUILayout.Label($"Pe: {FormatDistance(orbit.PeriapsisDistance)}  Ap: {FormatDistance(orbit.ApoapsisDistance)}");
                    GUILayout.Label($"Pe alt: {FormatDistance(orbit.PeriapsisDistance - frameRadius)}  Ap alt: {FormatDistance(orbit.ApoapsisDistance - frameRadius)}");
                    GUILayout.Label($"i: {orbit.InclinationDegrees:0.000} deg  LAN: {orbit.LongitudeOfAscendingNodeDegrees:0.000} deg");
                    GUILayout.Label($"arg Pe: {orbit.ArgumentOfPeriapsisDegrees:0.000} deg  true: {orbit.TrueAnomalyDegrees:0.000} deg");
                    GUILayout.Label($"period: {FormatDuration(orbit.OrbitalPeriodSeconds)}  energy: {orbit.SpecificOrbitalEnergy:0.###} J/kg");
                }
            }

            GUILayout.Space(6f);
            GUILayout.Label("Relative ship state");
            foreach (ReferenceFrameTarget target in Enum.GetValues(typeof(ReferenceFrameTarget)))
            {
                if (TryGetShipRelativeState(
                    target,
                    out string name,
                    out Vector3d stateRelativePosition,
                    out Vector3d stateRelativeVelocity,
                    out _,
                    out _,
                    out _))
                {
                    GUILayout.Label($"{name}: r {FormatDistance(stateRelativePosition.Magnitude)}  v {FormatSpeed(stateRelativeVelocity.Magnitude)}");
                }
            }

            GUILayout.EndScrollView();
            DrawTelemetryResizeHandle();
            GUI.DragWindow(new Rect(0f, 0f, telemetryOverlayRect.width - 18f, 20f));
        }

        private void DrawTelemetryTargetRow(ReferenceFrameTarget target, ReferenceFrameTarget activeFrame)
        {
            GUILayout.BeginHorizontal();
            string name = target.ToString();
            string frameLabel = target == activeFrame ? $"[{name}]" : name;

            if (GUILayout.Button(frameLabel, GUILayout.MinWidth(72f)))
            {
                SelectReferenceFrame(target);
            }

            string focusLabel = orbitMapFocusTarget == target && cameraMode == SpaceCameraMode.OrbitMap
                ? "Focused"
                : "Focus";

            if (GUILayout.Button(focusLabel, GUILayout.Width(64f)))
            {
                FocusOrbitMapOn(target);
            }

            if (TryGetReferenceState(
                target,
                out _,
                out Vector3d bodyPosition,
                out _,
                out _,
                out double radius,
                out _))
            {
                GUILayout.Label($"r {FormatDistance(radius)}  d {FormatDistance((shipBody.Position - bodyPosition).Magnitude)}");
            }

            GUILayout.EndHorizontal();
        }

        private void DrawTelemetryResizeHandle()
        {
            const float handleSize = 16f;
            Rect handleRect = new Rect(
                telemetryOverlayRect.width - handleSize - 3f,
                telemetryOverlayRect.height - handleSize - 3f,
                handleSize,
                handleSize);

            GUI.Box(handleRect, "//");
            EditorGUIUtilityAddResizeCursor(handleRect);

            Event currentEvent = Event.current;
            if (currentEvent == null)
            {
                return;
            }

            if (currentEvent.type == EventType.MouseDown && currentEvent.button == 0 && handleRect.Contains(currentEvent.mousePosition))
            {
                isResizingTelemetryOverlay = true;
                telemetryResizeStartMouse = GUIUtility.GUIToScreenPoint(currentEvent.mousePosition);
                telemetryResizeStartSize = new Vector2(telemetryOverlayRect.width, telemetryOverlayRect.height);
                currentEvent.Use();
            }

            if (!isResizingTelemetryOverlay)
            {
                return;
            }

            if (currentEvent.type == EventType.MouseDrag)
            {
                Vector2 currentMouse = GUIUtility.GUIToScreenPoint(currentEvent.mousePosition);
                Vector2 delta = currentMouse - telemetryResizeStartMouse;
                telemetryOverlayRect.width = Mathf.Clamp(
                    telemetryResizeStartSize.x + delta.x,
                    telemetryOverlayMinSize.x,
                    Mathf.Min(telemetryOverlayMaxSize.x, Screen.width - telemetryOverlayRect.x));
                telemetryOverlayRect.height = Mathf.Clamp(
                    telemetryResizeStartSize.y + delta.y,
                    telemetryOverlayMinSize.y,
                    Mathf.Min(telemetryOverlayMaxSize.y, Screen.height - telemetryOverlayRect.y));
                currentEvent.Use();
            }
            else if (currentEvent.type == EventType.MouseUp || currentEvent.rawType == EventType.MouseUp)
            {
                isResizingTelemetryOverlay = false;
                currentEvent.Use();
            }
        }

        private static void EditorGUIUtilityAddResizeCursor(Rect rect)
        {
#if UNITY_EDITOR
            UnityEditor.EditorGUIUtility.AddCursorRect(rect, UnityEditor.MouseCursor.ResizeUpLeft);
#endif
        }

        private Rect ClampTelemetryOverlayRect(Rect rect)
        {
            float maxWidth = Mathf.Min(telemetryOverlayMaxSize.x, Mathf.Max(telemetryOverlayMinSize.x, Screen.width));
            float maxHeight = Mathf.Min(telemetryOverlayMaxSize.y, Mathf.Max(telemetryOverlayMinSize.y, Screen.height));
            rect.width = Mathf.Clamp(rect.width, telemetryOverlayMinSize.x, maxWidth);
            rect.height = Mathf.Clamp(rect.height, telemetryOverlayMinSize.y, maxHeight);
            rect.x = Mathf.Clamp(rect.x, 0f, Mathf.Max(0f, Screen.width - rect.width));
            rect.y = Mathf.Clamp(rect.y, 0f, Mathf.Max(0f, Screen.height - rect.height));
            return rect;
        }

        private void RefreshReferenceFrameChange()
        {
            ReferenceFrameTarget activeFrame = ResolveActiveReferenceFrameTarget();
            if (activeFrame == lastActiveReferenceFrame)
            {
                return;
            }

            lastActiveReferenceFrame = activeFrame;
            RebuildReferenceFrameVisuals();
            ActiveReferenceFrameChanged?.Invoke(activeFrame);
        }

        private void NotifyReferenceFrameStateChanged()
        {
            lastActiveReferenceFrame = ResolveActiveReferenceFrameTarget();
            RebuildReferenceFrameVisuals();
            ActiveReferenceFrameChanged?.Invoke(lastActiveReferenceFrame);
        }

        private void RebuildReferenceFrameVisuals()
        {
            RebuildMoonOrbitLines();

            if (shipTrajectoryPredictor != null)
            {
                shipTrajectoryPredictor.ForceRefresh();
            }
        }

        private bool WasCameraTogglePressed()
        {
            Keyboard keyboard = Keyboard.current;
            return keyboard != null && keyboard.mKey.wasPressedThisFrame;
        }

        private bool WasCycleReferenceFramePressed()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null)
            {
                switch (cycleReferenceFrameKey)
                {
                    case KeyCode.Tab:
                        return keyboard.tabKey.wasPressedThisFrame;
                    case KeyCode.M:
                        return keyboard.mKey.wasPressedThisFrame;
                }
            }

#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKeyDown(cycleReferenceFrameKey);
#else
            return false;
#endif
        }

        private void HandleCameraInput()
        {
            Mouse mouse = Mouse.current;
            if (mouse == null)
            {
                return;
            }

            if (mouse.rightButton.isPressed)
            {
                Vector2 delta = mouse.delta.ReadValue();
                if (delta.sqrMagnitude > 0f)
                {
                    ApplyCameraOrbit(delta);
                }
            }

            float scrollY = mouse.scroll.ReadValue().y;
            if (Mathf.Abs(scrollY) > 0.01f)
            {
                ApplyCameraZoom(NormalizeScrollSteps(scrollY));
            }
        }

        private void ApplyCameraOrbit(Vector2 mouseDelta)
        {
            if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                orbitMapYawDegrees = NormalizeDegrees(orbitMapYawDegrees + (mouseDelta.x * cameraOrbitSensitivity));
                orbitMapPitchDegrees = NormalizeDegrees(orbitMapPitchDegrees - (mouseDelta.y * cameraOrbitSensitivity));
                return;
            }

            shipFocusYawDegrees += mouseDelta.x * cameraOrbitSensitivity;
            shipFocusPitchDegrees = Mathf.Clamp(
                shipFocusPitchDegrees - (mouseDelta.y * cameraOrbitSensitivity),
                -85f,
                85f);
        }

        private void ApplyCameraZoom(float scrollSteps)
        {
            if (Mathf.Approximately(scrollSteps, 0f))
            {
                return;
            }

            if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                if (orbitMapOrthographicSize <= 0f)
                {
                    orbitMapOrthographicSize = ResolveDefaultOrbitMapOrthographicSize();
                }

                float sensitivity = Mathf.Clamp(orbitMapZoomSensitivity, 0.01f, 0.95f);
                float zoomFactor = Mathf.Pow(1f - sensitivity, scrollSteps);
                orbitMapOrthographicSize = ClampOrbitMapOrthographicSize(orbitMapOrthographicSize * zoomFactor);
                return;
            }

            double minDistance = ResolveMinShipFocusDistanceMeters();
            double maxDistance = ResolveMaxShipFocusDistanceMeters(minDistance);
            double sensitivityMultiplier = Math.Max(0.01d, Math.Min(0.95d, shipZoomSensitivity));
            double distanceFactor = Math.Pow(1d - sensitivityMultiplier, scrollSteps);
            shipFocusDistanceMeters = Clamp(shipFocusDistanceMeters * distanceFactor, minDistance, maxDistance);
        }

        private static float NormalizeScrollSteps(float rawScrollY)
        {
            return Mathf.Abs(rawScrollY) > 10f ? rawScrollY / 120f : rawScrollY;
        }

        private void EnsureCameraStateInitialized()
        {
            if (cameraStateInitialized)
            {
                return;
            }

            double minDistance = ResolveMinShipFocusDistanceMeters();
            double maxDistance = ResolveMaxShipFocusDistanceMeters(minDistance);
            shipFocusDistanceMeters = Clamp(shipFocusDistanceMeters, minDistance, maxDistance);
            shipFocusPitchDegrees = Mathf.Clamp(shipFocusPitchDegrees, -85f, 85f);

            if (orbitMapOrthographicSize <= 0f)
            {
                orbitMapOrthographicSize = ResolveDefaultOrbitMapOrthographicSize();
            }

            orbitMapPitchDegrees = NormalizeDegrees(orbitMapPitchDegrees);
            orbitMapOrthographicSize = ClampOrbitMapOrthographicSize(orbitMapOrthographicSize);

            smoothedShipFocusDistanceMeters = shipFocusDistanceMeters;
            smoothedShipFocusYawDegrees = shipFocusYawDegrees;
            smoothedShipFocusPitchDegrees = shipFocusPitchDegrees;
            smoothedOrbitMapYawDegrees = orbitMapYawDegrees;
            smoothedOrbitMapPitchDegrees = orbitMapPitchDegrees;
            smoothedOrbitMapOrthographicSize = orbitMapOrthographicSize;
            cameraStateInitialized = true;
        }

        private void UpdateSmoothedCameraState()
        {
            EnsureCameraStateInitialized();

            double minDistance = ResolveMinShipFocusDistanceMeters();
            double maxDistance = ResolveMaxShipFocusDistanceMeters(minDistance);
            shipFocusDistanceMeters = Clamp(shipFocusDistanceMeters, minDistance, maxDistance);
            shipFocusPitchDegrees = Mathf.Clamp(shipFocusPitchDegrees, -85f, 85f);

            if (orbitMapOrthographicSize <= 0f)
            {
                orbitMapOrthographicSize = ResolveDefaultOrbitMapOrthographicSize();
            }

            orbitMapPitchDegrees = NormalizeDegrees(orbitMapPitchDegrees);
            orbitMapOrthographicSize = ClampOrbitMapOrthographicSize(orbitMapOrthographicSize);

            float rotationAlpha = ExpSmoothingAlpha(cameraRotationSmoothing);
            float zoomAlpha = ExpSmoothingAlpha(cameraZoomSmoothing);

            smoothedShipFocusYawDegrees = Mathf.LerpAngle(smoothedShipFocusYawDegrees, shipFocusYawDegrees, rotationAlpha);
            smoothedShipFocusPitchDegrees = Mathf.Lerp(smoothedShipFocusPitchDegrees, shipFocusPitchDegrees, rotationAlpha);
            smoothedShipFocusDistanceMeters = Lerp(smoothedShipFocusDistanceMeters, shipFocusDistanceMeters, zoomAlpha);

            smoothedOrbitMapYawDegrees = Mathf.LerpAngle(smoothedOrbitMapYawDegrees, orbitMapYawDegrees, rotationAlpha);
            smoothedOrbitMapPitchDegrees = Mathf.LerpAngle(smoothedOrbitMapPitchDegrees, orbitMapPitchDegrees, rotationAlpha);
            smoothedOrbitMapOrthographicSize = Mathf.Lerp(smoothedOrbitMapOrthographicSize, orbitMapOrthographicSize, zoomAlpha);
        }

        private static float ExpSmoothingAlpha(float smoothing)
        {
            float deltaTime = Time.unscaledDeltaTime;
            if (deltaTime <= 0f || float.IsNaN(deltaTime) || float.IsInfinity(deltaTime))
            {
                return 1f;
            }

            return 1f - Mathf.Exp(-Mathf.Max(0.01f, smoothing) * deltaTime);
        }

        private static float NormalizeDegrees(float degrees)
        {
            if (float.IsNaN(degrees) || float.IsInfinity(degrees))
            {
                return 0f;
            }

            return Mathf.Repeat(degrees + 180f, 360f) - 180f;
        }

        private static double Lerp(double from, double to, float t)
        {
            double clampedT = Mathf.Clamp01(t);
            return from + ((to - from) * clampedT);
        }

        private void ApplyCameraMode()
        {
            ResolveCameraReferences();
            if (celestialCamera == null)
            {
                return;
            }

            ConfigureCameraRendering();
            UpdateSmoothedCameraState();

            if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                ApplyOrbitMapCamera();
            }
            else
            {
                ApplyShipFocusCamera();
            }

            UpdateMoonOrbitVisibility();
            UpdateOrbitMapMarkers();
        }

        private void ResolveCameraReferences()
        {
            if (celestialCamera == null)
            {
                GameObject skyCameraObject = GameObject.Find("SkyCamera");
                if (skyCameraObject != null)
                {
                    celestialCamera = skyCameraObject.GetComponent<Camera>();
                }
            }

            if (shipOverlayCamera == null)
            {
                GameObject shipCameraObject = GameObject.Find("ShipCamera");
                if (shipCameraObject != null)
                {
                    shipOverlayCamera = shipCameraObject.GetComponent<Camera>();
                }
            }

            if (celestialCamera == null)
            {
                celestialCamera = Camera.main;
            }

            DetachCameraRigFromShipIfNeeded();
        }

        private void DetachCameraRigFromShipIfNeeded()
        {
            Transform shipTransform = ship != null ? ship.VisualTransform : null;
            if (shipTransform == null)
            {
                return;
            }

            DetachCameraTransformFromShip(celestialCamera, shipTransform);
            DetachCameraTransformFromShip(shipOverlayCamera, shipTransform);
        }

        private static void DetachCameraTransformFromShip(Camera camera, Transform shipTransform)
        {
            if (camera == null || shipTransform == null || !camera.transform.IsChildOf(shipTransform))
            {
                return;
            }

            Transform root = camera.transform;
            while (root.parent != null && root.parent != shipTransform && root.parent.IsChildOf(shipTransform))
            {
                root = root.parent;
            }

            if (root.parent == shipTransform)
            {
                root.SetParent(null, true);
            }
        }

        private void ConfigureCameraRendering()
        {
            celestialCamera.enabled = true;
            int shipVisualLayer = ResolveShipVisualLayer();
            celestialCamera.cullingMask = BuildLayerMaskIncludingLayer(
                shipVisualLayer,
                "Default",
                "Celestial",
                "Trajectory",
                "Ship");
            celestialCamera.eventMask = 0;

            AudioListener celestialListener = celestialCamera.GetComponent<AudioListener>();
            if (celestialListener != null)
            {
                celestialListener.enabled = true;
            }

            if (shipOverlayCamera == null)
            {
                return;
            }

            shipOverlayCamera.enabled = cameraMode == SpaceCameraMode.ShipFocus;
            shipOverlayCamera.cullingMask = BuildLayerMaskIncludingLayer(shipVisualLayer, "Ship");
            shipOverlayCamera.eventMask = 0;
            shipOverlayCamera.clearFlags = CameraClearFlags.Depth;

            AudioListener shipListener = shipOverlayCamera.GetComponent<AudioListener>();
            if (shipListener != null)
            {
                shipListener.enabled = false;
            }
        }

        private void ApplyOrbitMapCamera()
        {
            float defaultOrthographicSize = ResolveDefaultOrbitMapOrthographicSize();
            if (orbitMapOrthographicSize <= 0f)
            {
                orbitMapOrthographicSize = defaultOrthographicSize;
            }

            float orthographicSize = ClampOrbitMapOrthographicSize(smoothedOrbitMapOrthographicSize);
            smoothedOrbitMapOrthographicSize = orthographicSize;

            Vector3 center = ResolveOrbitMapFocusPosition(orthographicSize);
            float fieldOfView = Mathf.Clamp(orbitMapFieldOfView, 15f, 80f);
            float distance = ResolvePerspectiveDistanceForViewSize(orthographicSize, fieldOfView);
            Quaternion orbitRotation = Quaternion.Euler(smoothedOrbitMapPitchDegrees, smoothedOrbitMapYawDegrees, 0f);
            Vector3 viewOffset = orbitRotation * Vector3.back;
            if (!IsUsableDirection(viewOffset))
            {
                viewOffset = Vector3.back;
            }

            Vector3 cameraPosition = center + (viewOffset.normalized * distance);
            Quaternion cameraRotation = CreateSafeLookRotation(
                center - cameraPosition,
                orbitRotation * Vector3.up,
                celestialCamera.transform.rotation);

            ApplyCameraPose(cameraPosition, cameraRotation);

            celestialCamera.orthographic = false;
            celestialCamera.fieldOfView = fieldOfView;
            float focusedRadiusUnits = MetersToVisualUnitsFloat(ResolveReferenceRadius(orbitMapFocusTarget));
            float focusSurfaceDistance = Mathf.Max(0f, distance - focusedRadiusUnits);
            celestialCamera.nearClipPlane = ResolveCameraNearClipPlane(focusSurfaceDistance);
            celestialCamera.farClipPlane = Mathf.Max(
                ResolveRequiredCelestialFarClipPlane(cameraPosition),
                distance + (ResolveOrbitMapRadiusUnits() * 4f) + (orthographicSize * 8f));
        }

        private void ApplyShipFocusCamera()
        {
            if (ship == null || ship.VisualTransform == null)
            {
                ApplyOrbitMapCamera();
                return;
            }

            Vector3 target = ship.VisualTransform.position;
            shipFocusPitchDegrees = Mathf.Clamp(shipFocusPitchDegrees, -85f, 85f);
            double minDistance = ResolveMinShipFocusDistanceMeters();
            double maxDistance = ResolveMaxShipFocusDistanceMeters(minDistance);
            shipFocusDistanceMeters = Clamp(shipFocusDistanceMeters, minDistance, maxDistance);

            Quaternion orbitRotation = Quaternion.Euler(smoothedShipFocusPitchDegrees, smoothedShipFocusYawDegrees, 0f);
            Vector3 cameraBack = orbitRotation * Vector3.back;
            Vector3 cameraUp = orbitRotation * Vector3.up;
            if (!IsUsableDirection(cameraBack))
            {
                cameraBack = Vector3.back;
            }

            if (!IsUsableDirection(cameraUp))
            {
                cameraUp = Vector3.up;
            }

            double stableDistanceMeters = Math.Max(ResolveMinShipFocusDistanceMeters(), smoothedShipFocusDistanceMeters);
            float distance = Mathf.Max(1e-9f, MetersToVisualUnitsFloat(stableDistanceMeters));
            float height = Mathf.Max(0f, MetersToVisualUnitsFloat(shipFocusHeightMeters));
            Vector3 cameraPosition = target + (cameraBack.normalized * distance) + (cameraUp.normalized * height);
            Vector3 lookDirection = target - cameraPosition;

            if (!IsUsableDirection(lookDirection))
            {
                cameraPosition = target + (Vector3.back * distance);
                lookDirection = target - cameraPosition;
                cameraUp = Vector3.up;
            }

            Quaternion cameraRotation = CreateSafeLookRotation(
                lookDirection,
                cameraUp,
                celestialCamera.transform.rotation);
            ApplyCameraPose(cameraPosition, cameraRotation);

            celestialCamera.orthographic = false;
            celestialCamera.fieldOfView = shipFocusFieldOfView;
            float cameraToShipUnits = Mathf.Max(1e-12f, Vector3.Distance(cameraPosition, target));
            // Without this, celestial near-plane is clamped up to MaximumCelestialNearClipPlane (world units).
            // With scene scale (~1e5 m per Unity unit) the ship sits much closer than that and gets clipped.
            float celestialNear = ResolveCameraNearClipPlane(ResolveNearestCelestialSurfaceDistanceUnits(cameraPosition));
            float shipSafeNear = Mathf.Max(MinimumCameraNearClipPlane, cameraToShipUnits * 0.08f);

            // Compute raw near/far and ensure we never produce an extreme near/far ratio
            // which would destroy depth buffer precision. Guarantee near/far <= MaxNearFarRatio.
            float rawFar = ResolveRequiredCelestialFarClipPlane(cameraPosition);
            float rawNear = Mathf.Min(celestialNear, shipSafeNear);
            const float MaxNearFarRatio = 100000f;
            celestialCamera.nearClipPlane = Mathf.Max(rawNear, rawFar / MaxNearFarRatio);
            celestialCamera.farClipPlane = rawFar;

            if (shipOverlayCamera != null)
            {
                shipOverlayCamera.orthographic = false;
                shipOverlayCamera.fieldOfView = shipFocusFieldOfView;
                // Ship overlay uses the original ship-safe near so the ship remains visible
                // even if the celestial camera had its near adjusted for depth precision.
                shipOverlayCamera.nearClipPlane = Mathf.Max(MinimumCameraNearClipPlane, shipSafeNear * 0.98f);
                shipOverlayCamera.farClipPlane = Mathf.Max(
                    cameraToShipUnits * 4f + MetersToVisualUnitsFloat(Math.Max(1d, ship.VisualRadiusMeters * 16d)),
                    celestialCamera.farClipPlane,
                    MetersToVisualUnitsFloat(1000d));
            }
        }

        private void ApplyCameraPose(Vector3 position, Quaternion rotation)
        {
            celestialCamera.transform.SetPositionAndRotation(position, rotation);

            if (shipOverlayCamera != null)
            {
                shipOverlayCamera.transform.SetPositionAndRotation(position, rotation);
            }
        }

        private float ResolveDefaultOrbitMapOrthographicSize()
        {
            return Mathf.Max(minOrbitMapOrthographicSize, ResolveOrbitMapRadiusUnits() * orbitMapPadding);
        }

        private static float ResolvePerspectiveDistanceForViewSize(float halfHeight, float fieldOfView)
        {
            float fovRadians = Mathf.Deg2Rad * Mathf.Clamp(fieldOfView, 1f, 120f);
            float tangent = Mathf.Tan(fovRadians * 0.5f);
            if (tangent <= 0.0001f)
            {
                return Mathf.Max(halfHeight, 10f);
            }

            return Mathf.Max(halfHeight / tangent, 10f);
        }

        private Vector3 ResolveOrbitMapFocusPosition(float viewHalfHeight)
        {
            if (TryGetReferenceStateAtTime(
                orbitMapFocusTarget,
                simulationTimeSeconds,
                out _,
                out Vector3d focusPosition,
                out _,
                out _,
                out _,
                out _))
            {
                Vector3 focusWorld = ToUnityPosition(focusPosition);
                if (orbitMapFocusTarget != ReferenceFrameTarget.Jupiter)
                {
                    Vector3 jupiterWorld = jupiterTransform != null
                        ? jupiterTransform.position
                        : ToUnityPosition(jupiterRealPosition);
                    float focusDistance = Vector3.Distance(focusWorld, jupiterWorld);
                    if (focusDistance > 0.001f)
                    {
                        float t = Mathf.InverseLerp(focusDistance * 0.35f, focusDistance * 1.15f, viewHalfHeight);
                        return Vector3.Lerp(focusWorld, jupiterWorld, Mathf.Clamp01(t));
                    }
                }

                return focusWorld;
            }

            return jupiterTransform != null ? jupiterTransform.position : ToUnityPosition(jupiterRealPosition);
        }

        private float ClampOrbitMapOrthographicSize(float size)
        {
            float defaultSize = ResolveDefaultOrbitMapOrthographicSize();
            float minSize = ResolveMinOrbitMapViewHalfHeight();
            float maxSize = Mathf.Max(defaultSize * Mathf.Max(1f, maxOrbitMapZoomMultiplier), minSize * 2f);
            return Mathf.Clamp(size, minSize, maxSize);
        }

        private float ResolveMinOrbitMapViewHalfHeight()
        {
            float focusedRadiusUnits = MetersToVisualUnitsFloat(ResolveReferenceRadius(orbitMapFocusTarget));
            float fieldOfView = Mathf.Clamp(orbitMapFieldOfView, 15f, 80f);
            float surfaceClearanceUnits = MetersToVisualUnitsFloat(
                ResolveOrbitMapSurfaceClearanceMeters(orbitMapFocusTarget));
            float minimumCameraDistance = Mathf.Max(0.001f, focusedRadiusUnits + surfaceClearanceUnits);
            float minimumHalfHeight = minimumCameraDistance * Mathf.Tan(Mathf.Deg2Rad * fieldOfView * 0.5f);

            return Mathf.Max(0.001f, minimumHalfHeight);
        }

        private float ResolveJupiterRadiusUnits()
        {
            double metersPerVisual = GetMetersPerVisualUnit();
            if (metersPerVisual <= 0d)
            {
                return 1f;
            }

            return (float)Math.Max(1d, jupiterRadius / metersPerVisual);
        }

        private double ResolveOrbitMapSurfaceClearanceMeters(ReferenceFrameTarget target)
        {
            double configuredClearance = orbitMapSurfaceClearanceMeters > 0d
                ? orbitMapSurfaceClearanceMeters
                : DefaultOrbitMapSurfaceClearanceMeters;
            double radiusBasedClearance = ResolveReferenceRadius(target) * OrbitMapSurfaceClearanceRadiusFraction;
            double preferredClearance = Math.Min(configuredClearance, radiusBasedClearance);
            return Clamp(
                preferredClearance,
                MinimumOrbitMapSurfaceClearanceMeters,
                MaximumOrbitMapSurfaceClearanceMeters);
        }

        private double ResolveReferenceRadius(ReferenceFrameTarget target)
        {
            if (target == ReferenceFrameTarget.Jupiter)
            {
                return Math.Max(1d, jupiterRadius);
            }

            int moonIndex = FindMoonIndex(target);
            if (moonIndex >= 0 && moonIndex < moonRails.Count && moonRails[moonIndex] != null)
            {
                return Math.Max(1d, moonRails[moonIndex].Radius);
            }

            return Math.Max(1d, jupiterRadius);
        }

        private void UpdateOrbitMapMarkers()
        {
            EnsureOrbitMapMarkers();
            bool shouldShow = celestialCamera != null && celestialCamera.enabled;
            SetOrbitMapMarkersActive(shouldShow);

            if (!shouldShow)
            {
                return;
            }

            UpdateOrbitMapMarker(ReferenceFrameTarget.Jupiter, jupiterRealPosition, jupiterRadius, orbitMapMinJupiterScreenRadius);
            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                if (rail == null)
                {
                    continue;
                }

                ReferenceFrameTarget target = ResolveRailReferenceFrameTarget(i);
                if (TryGetReferenceStateAtTime(
                    target,
                    simulationTimeSeconds,
                    out _,
                    out Vector3d moonPosition,
                    out _,
                    out _,
                    out _,
                    out _))
                {
                    UpdateOrbitMapMarker(target, moonPosition, rail.Radius, orbitMapMinMoonScreenRadius);
                }
            }
        }

        private void EnsureOrbitMapMarkers()
        {
            Transform visualParent = worldContainer != null ? worldContainer : transform;
            if (orbitMapMarkerRoot == null)
            {
                orbitMapMarkerRoot = FindChildByName(visualParent, "Orbit_Map_Markers");
                if (orbitMapMarkerRoot == null)
                {
                    GameObject rootObject = new GameObject("Orbit_Map_Markers");
                    orbitMapMarkerRoot = rootObject.transform;
                    orbitMapMarkerRoot.SetParent(visualParent, false);
                }
            }
            else if (orbitMapMarkerRoot.parent != visualParent)
            {
                orbitMapMarkerRoot.SetParent(visualParent, false);
            }

            foreach (ReferenceFrameTarget target in Enum.GetValues(typeof(ReferenceFrameTarget)))
            {
                EnsureOrbitMapMarker(target);
            }
        }

        private void EnsureOrbitMapMarker(ReferenceFrameTarget target)
        {
            if (orbitMapMarkers.TryGetValue(target, out Transform existingMarker) && existingMarker != null)
            {
                return;
            }

            Transform markerTransform = orbitMapMarkerRoot != null
                ? FindChildByName(orbitMapMarkerRoot, $"{target}_MapMarker")
                : null;

            GameObject markerObject;
            if (markerTransform != null)
            {
                markerObject = markerTransform.gameObject;
            }
            else
            {
                markerObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                markerObject.name = $"{target}_MapMarker";
                markerObject.transform.SetParent(orbitMapMarkerRoot, false);
                Collider markerCollider = markerObject.GetComponent<Collider>();
                if (markerCollider != null)
                {
                    Destroy(markerCollider);
                }
            }

            int celestialLayer = LayerMask.NameToLayer("Celestial");
            if (celestialLayer >= 0)
            {
                markerObject.layer = celestialLayer;
            }

            Renderer markerRenderer = markerObject.GetComponent<Renderer>();
            if (markerRenderer != null)
            {
                markerRenderer.sharedMaterial = GetOrCreateOrbitMapMarkerMaterial();
                markerRenderer.allowOcclusionWhenDynamic = false;
            }

            orbitMapMarkers[target] = markerObject.transform;
            orbitMapMarkerRenderers[target] = markerRenderer;
        }

        private void UpdateOrbitMapMarker(
            ReferenceFrameTarget target,
            Vector3d realPosition,
            double realRadiusMeters,
            float minScreenRadiusPixels)
        {
            if (!orbitMapMarkers.TryGetValue(target, out Transform marker) || marker == null)
            {
                return;
            }

            ApplyVisualPosition(marker, realPosition);
            float physicalRadiusUnits = MetersToVisualUnitsFloat(realRadiusMeters);
            Vector3 bodyWorldPosition = marker.position;
            float distanceAlongView = Vector3.Dot(
                bodyWorldPosition - celestialCamera.transform.position,
                celestialCamera.transform.forward);
            if (distanceAlongView <= celestialCamera.nearClipPlane)
            {
                marker.gameObject.SetActive(false);
                return;
            }

            float physicalScreenRadius = ResolveScreenRadiusForWorldRadius(bodyWorldPosition, physicalRadiusUnits);
            if (physicalScreenRadius >= Mathf.Max(1f, minScreenRadiusPixels) * MarkerVisibleScreenRadiusMultiplier)
            {
                marker.gameObject.SetActive(false);
                return;
            }

            float markerRadiusUnits = Mathf.Max(
                ResolveWorldRadiusForScreenPixels(bodyWorldPosition, minScreenRadiusPixels),
                0.001f);
            Vector3 frontOffset = -celestialCamera.transform.forward * Mathf.Max(
                physicalRadiusUnits + (markerRadiusUnits * 0.05f),
                markerRadiusUnits);
            marker.position = bodyWorldPosition + frontOffset;
            marker.localScale = Vector3.one * (markerRadiusUnits * 2f);
            marker.gameObject.SetActive(true);

            if (orbitMapMarkerRenderers.TryGetValue(target, out Renderer markerRenderer) && markerRenderer != null)
            {
                ApplyOrbitMapMarkerColor(markerRenderer, target);
            }
        }

        private void SetOrbitMapMarkersActive(bool active)
        {
            if (orbitMapMarkerRoot != null)
            {
                orbitMapMarkerRoot.gameObject.SetActive(active);
            }
        }

        private float ResolveCameraNearClipPlane(float nearestSurfaceDistanceUnits)
        {
            float minimumNearClip = Mathf.Max(
                MinimumCameraNearClipPlane,
                MetersToVisualUnitsFloat(1d));

            if (float.IsNaN(nearestSurfaceDistanceUnits) || float.IsInfinity(nearestSurfaceDistanceUnits))
            {
                return minimumNearClip;
            }

            if (nearestSurfaceDistanceUnits <= 0f)
            {
                return minimumNearClip;
            }

            return Mathf.Clamp(
                nearestSurfaceDistanceUnits * 0.1f,
                minimumNearClip,
                MaximumCelestialNearClipPlane);
        }

        private float ResolveNearestCelestialSurfaceDistanceUnits(Vector3 cameraPosition)
        {
            float nearestDistance = float.PositiveInfinity;
            IncludeCelestialSurfaceDistance(ref nearestDistance, cameraPosition, jupiterRealPosition, jupiterRadius);

            for (int i = 0; i < moonBodies.Count && i < moonRails.Count; i++)
            {
                if (moonBodies[i] == null || moonRails[i] == null)
                {
                    continue;
                }

                IncludeCelestialSurfaceDistance(ref nearestDistance, cameraPosition, moonBodies[i].Position, moonRails[i].Radius);
            }

            return float.IsInfinity(nearestDistance) ? MetersToVisualUnitsFloat(100d) : nearestDistance;
        }

        private void IncludeCelestialSurfaceDistance(
            ref float nearestDistance,
            Vector3 cameraPosition,
            Vector3d realPosition,
            double radiusMeters)
        {
            Vector3 bodyPosition = ToUnityPosition(realPosition);
            if (!IsFinite(bodyPosition))
            {
                return;
            }

            float radiusUnits = MetersToVisualUnitsFloat(radiusMeters);
            float surfaceDistance = Mathf.Max(0f, Vector3.Distance(cameraPosition, bodyPosition) - radiusUnits);
            nearestDistance = Mathf.Min(nearestDistance, surfaceDistance);
        }

        private float ResolveRequiredCelestialFarClipPlane(Vector3 cameraPosition)
        {
            float farClip = 1000f;
            IncludeCelestialFarClipDistance(ref farClip, cameraPosition, jupiterRealPosition, jupiterRadius);

            for (int i = 0; i < moonBodies.Count && i < moonRails.Count; i++)
            {
                if (moonBodies[i] == null || moonRails[i] == null)
                {
                    continue;
                }

                IncludeCelestialFarClipDistance(ref farClip, cameraPosition, moonBodies[i].Position, moonRails[i].Radius);
            }

            return Mathf.Clamp(farClip * 1.25f, 1000f, 100000f);
        }

        private void IncludeCelestialFarClipDistance(
            ref float farClip,
            Vector3 cameraPosition,
            Vector3d realPosition,
            double radiusMeters)
        {
            Vector3 bodyPosition = ToUnityPosition(realPosition);
            if (!IsFinite(bodyPosition))
            {
                return;
            }

            float radiusUnits = MetersToVisualUnitsFloat(radiusMeters);
            farClip = Mathf.Max(farClip, Vector3.Distance(cameraPosition, bodyPosition) + radiusUnits);
        }

        private float ResolveWorldRadiusForScreenPixels(Vector3 worldPosition, float screenPixels)
        {
            if (celestialCamera == null)
            {
                return 0f;
            }

            float pixelHeight = Mathf.Max(1f, celestialCamera.pixelHeight);
            float clampedPixels = Mathf.Max(1f, screenPixels);
            if (celestialCamera.orthographic)
            {
                return ((celestialCamera.orthographicSize * 2f) / pixelHeight) * clampedPixels;
            }

            float distanceAlongView = Vector3.Dot(
                worldPosition - celestialCamera.transform.position,
                celestialCamera.transform.forward);
            if (distanceAlongView <= celestialCamera.nearClipPlane)
            {
                return 0f;
            }

            float fovRadians = Mathf.Deg2Rad * Mathf.Clamp(celestialCamera.fieldOfView, 1f, 120f);
            float worldHeight = 2f * Mathf.Max(0.001f, distanceAlongView) * Mathf.Tan(fovRadians * 0.5f);
            return (worldHeight / pixelHeight) * clampedPixels;
        }

        private float ResolveScreenRadiusForWorldRadius(Vector3 worldPosition, float worldRadius)
        {
            if (celestialCamera == null || worldRadius <= 0f)
            {
                return 0f;
            }

            float pixelHeight = Mathf.Max(1f, celestialCamera.pixelHeight);
            if (celestialCamera.orthographic)
            {
                float pixelsPerWorldUnit = pixelHeight / Mathf.Max(0.001f, celestialCamera.orthographicSize * 2f);
                return worldRadius * pixelsPerWorldUnit;
            }

            float distanceAlongView = Vector3.Dot(
                worldPosition - celestialCamera.transform.position,
                celestialCamera.transform.forward);
            if (distanceAlongView <= celestialCamera.nearClipPlane)
            {
                return 0f;
            }

            float fovRadians = Mathf.Deg2Rad * Mathf.Clamp(celestialCamera.fieldOfView, 1f, 120f);
            float worldHeight = 2f * Mathf.Max(0.001f, distanceAlongView) * Mathf.Tan(fovRadians * 0.5f);
            float perspectivePixelsPerWorldUnit = pixelHeight / Mathf.Max(0.001f, worldHeight);
            return worldRadius * perspectivePixelsPerWorldUnit;
        }

        private void ApplyOrbitMapMarkerColor(Renderer markerRenderer, ReferenceFrameTarget target)
        {
            Color color = ResolveOrbitMapMarkerColor(target);
            MaterialPropertyBlock propertyBlock = new MaterialPropertyBlock();
            markerRenderer.GetPropertyBlock(propertyBlock);
            propertyBlock.SetColor("_Color", color);
            propertyBlock.SetColor("_BaseColor", color);
            markerRenderer.SetPropertyBlock(propertyBlock);
        }

        private static Color ResolveOrbitMapMarkerColor(ReferenceFrameTarget target)
        {
            switch (target)
            {
                case ReferenceFrameTarget.Jupiter:
                    return new Color(1f, 0.78f, 0.48f, 1f);
                case ReferenceFrameTarget.Io:
                    return new Color(1f, 0.9f, 0.45f, 1f);
                case ReferenceFrameTarget.Europa:
                    return new Color(0.82f, 0.92f, 1f, 1f);
                case ReferenceFrameTarget.Ganymede:
                    return new Color(0.7f, 0.72f, 0.68f, 1f);
                case ReferenceFrameTarget.Callisto:
                    return new Color(0.62f, 0.5f, 0.42f, 1f);
                default:
                    return Color.white;
            }
        }

        private double ResolveMinShipFocusDistanceMeters()
        {
            double visualRadius = ship != null ? Math.Max(0d, ship.VisualRadiusMeters) : 0d;
            double surfaceClearance = visualRadius > 0d ? Math.Max(0.35d, visualRadius * 0.15d) : 0d;
            return Math.Max(minShipFocusDistanceMeters, visualRadius + surfaceClearance);
        }

        private double ResolveMaxShipFocusDistanceMeters(double minDistance)
        {
            return Math.Max(maxShipFocusDistanceMeters, minDistance * 2d);
        }

        private float ResolveOrbitMapRadiusUnits()
        {
            EnsureInitialized();

            double radiusMeters = Math.Max(jupiterRadius, 1d);
            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                if (rail == null)
                {
                    continue;
                }

                double distanceMeters = i < moonBodies.Count && moonBodies[i] != null
                    ? (moonBodies[i].Position - jupiterRealPosition).Magnitude
                    : rail.ResolveSemiMajorAxis() * (1d + rail.ResolveEccentricity());

                radiusMeters = Math.Max(radiusMeters, distanceMeters + Math.Max(rail.Radius, 0d));
            }

            if (shipBody != null)
            {
                radiusMeters = Math.Max(radiusMeters, (shipBody.Position - jupiterRealPosition).Magnitude);
            }

            double metersPerVisual = GetMetersPerVisualUnit();
            if (metersPerVisual <= 0d)
            {
                return 1f;
            }

            return (float)Math.Max(1d, radiusMeters / metersPerVisual);
        }

        private float MetersToVisualUnitsFloat(double meters)
        {
            double metersPerVisual = GetMetersPerVisualUnit();
            if (metersPerVisual <= 0d || double.IsNaN(meters) || double.IsInfinity(meters))
            {
                return 0f;
            }

            double units = Math.Max(0d, meters / metersPerVisual);
            return (float)Math.Min(units, float.MaxValue);
        }

        private static int BuildLayerMask(params string[] layerNames)
        {
            int mask = 0;
            for (int i = 0; i < layerNames.Length; i++)
            {
                int layer = LayerMask.NameToLayer(layerNames[i]);
                if (layer >= 0)
                {
                    mask |= 1 << layer;
                }
            }

            return mask;
        }

        private static int BuildLayerMaskIncludingLayer(int requiredLayer, params string[] layerNames)
        {
            int mask = BuildLayerMask(layerNames);
            if (requiredLayer >= 0)
            {
                mask |= 1 << requiredLayer;
            }

            return mask;
        }

        private int ResolveShipVisualLayer()
        {
            if (ship != null && ship.VisualTransform != null)
            {
                return ship.VisualTransform.gameObject.layer;
            }

            int shipLayer = LayerMask.NameToLayer("Ship");
            if (shipLayer >= 0)
            {
                return shipLayer;
            }

            return gameObject.layer;
        }

        private static bool IsUsableDirection(Vector3 value)
        {
            return IsFinite(value) && value.sqrMagnitude > 1e-18f;
        }

        private static Quaternion CreateSafeLookRotation(Vector3 forward, Vector3 up, Quaternion fallback)
        {
            if (!IsUsableDirection(forward))
            {
                return fallback;
            }

            Vector3 normalizedForward = forward.normalized;
            Vector3 normalizedUp = IsUsableDirection(up) ? up.normalized : Vector3.up;
            if (Mathf.Abs(Vector3.Dot(normalizedForward, normalizedUp)) > 0.999f)
            {
                normalizedUp = Mathf.Abs(Vector3.Dot(normalizedForward, Vector3.up)) > 0.999f
                    ? Vector3.forward
                    : Vector3.up;
            }

            return Quaternion.LookRotation(normalizedForward, normalizedUp);
        }

        private void ApplyTidallyLockedMoonVisualRotation(MoonRail rail, Transform visual, CelestialBody tempMoon)
        {
            if (rail == null || visual == null || tempMoon == null)
            {
                return;
            }

            // Direction from moon towards Jupiter in simulation space
            Vector3d toJupiter = jupiterRealPosition - tempMoon.Position;
            if (!toJupiter.IsFinite || toJupiter.SqrMagnitude <= 0d)
            {
                return;
            }

            // Convert to Unity direction and build a safe rotation that points the forward
            // axis of the visual towards Jupiter while keeping a sensible 'up' direction.
            Vector3 forward = ToUnityDirection(toJupiter);
            Vector3 up = jupiterNorthLocalDirection;

            Quaternion target = CreateSafeLookRotation(forward, up, visual.rotation);
            visual.rotation = target;
        }

        private ReferenceFrameTarget ResolveActiveReferenceFrameTarget()
        {
            if (!autoSelectSphereOfInfluence || shipBody == null)
            {
                return selectedReferenceFrame;
            }

            ReferenceFrameTarget nearestTarget = ReferenceFrameTarget.Jupiter;
            double nearestDistance = double.PositiveInfinity;

            foreach (ReferenceFrameTarget target in Enum.GetValues(typeof(ReferenceFrameTarget)))
            {
                if (target == ReferenceFrameTarget.Jupiter)
                {
                    continue;
                }

                if (!TryGetShipRelativeState(
                    target,
                    out _,
                    out Vector3d relativePosition,
                    out _,
                    out _,
                    out _,
                    out double soiRadius))
                {
                    continue;
                }

                double distance = relativePosition.Magnitude;
                if (soiRadius > 0d && distance <= soiRadius && distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestTarget = target;
                }
            }

            return nearestTarget;
        }

        public bool TryGetReferenceState(
            ReferenceFrameTarget target,
            out string frameName,
            out Vector3d framePosition,
            out Vector3d frameVelocity,
            out double frameStandardGravitationalParameter,
            out double frameRadius,
            out double sphereOfInfluenceRadius)
        {
            EnsureInitialized();

            if (target == ReferenceFrameTarget.Jupiter)
            {
                frameName = "Jupiter";
                framePosition = jupiterRealPosition;
                frameVelocity = Vector3d.Zero;
                frameStandardGravitationalParameter = jupiterStandardGravitationalParameter;
                frameRadius = jupiterRadius;
                sphereOfInfluenceRadius = double.PositiveInfinity;
                return true;
            }

            int moonIndex = FindMoonIndex(target);
            if (moonIndex < 0 || moonIndex >= moonBodies.Count || moonIndex >= moonRails.Count)
            {
                frameName = target.ToString();
                framePosition = Vector3d.Zero;
                frameVelocity = Vector3d.Zero;
                frameStandardGravitationalParameter = 0d;
                frameRadius = 0d;
                sphereOfInfluenceRadius = 0d;
                return false;
            }

            MoonRail rail = moonRails[moonIndex];
            CelestialBody moonBody = moonBodies[moonIndex];
            frameName = string.IsNullOrWhiteSpace(rail.Name) ? target.ToString() : rail.Name;
            framePosition = moonBody.Position;
            frameVelocity = moonBody.Velocity;
            frameStandardGravitationalParameter = rail.ResolveStandardGravitationalParameter();
            frameRadius = rail.Radius;
            sphereOfInfluenceRadius = rail.SphereOfInfluenceRadius;
            return true;
        }

        public bool TryGetReferenceStateAtTime(
            ReferenceFrameTarget target,
            double sampleTimeSeconds,
            out string frameName,
            out Vector3d framePosition,
            out Vector3d frameVelocity,
            out double frameStandardGravitationalParameter,
            out double frameRadius,
            out double sphereOfInfluenceRadius)
        {
            frameName = target.ToString();
            framePosition = Vector3d.Zero;
            frameVelocity = Vector3d.Zero;
            frameStandardGravitationalParameter = 0d;
            frameRadius = 0d;
            sphereOfInfluenceRadius = 0d;

            EnsureInitialized();

            if (target == ReferenceFrameTarget.Jupiter)
            {
                frameName = "Jupiter";
                framePosition = jupiterRealPosition;
                frameVelocity = Vector3d.Zero;
                frameStandardGravitationalParameter = jupiterStandardGravitationalParameter;
                frameRadius = jupiterRadius;
                sphereOfInfluenceRadius = double.PositiveInfinity;
                return true;
            }

            int moonIndex = FindMoonIndex(target);
            if (moonIndex < 0 || moonIndex >= moonBodies.Count || moonIndex >= moonRails.Count)
            {
                return false;
            }

            MoonRail rail = moonRails[moonIndex];
            frameName = string.IsNullOrWhiteSpace(rail.Name) ? target.ToString() : rail.Name;
            EvaluateMoonState(rail, sampleTimeSeconds, out framePosition, out frameVelocity);
            frameStandardGravitationalParameter = rail.ResolveStandardGravitationalParameter();
            frameRadius = rail.Radius;
            sphereOfInfluenceRadius = rail.SphereOfInfluenceRadius;
            return true;
        }

        private int FindMoonIndex(ReferenceFrameTarget target)
        {
            string targetName = target.ToString();
            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                if (rail != null && string.Equals(rail.Name, targetName, StringComparison.OrdinalIgnoreCase))
                {
                    return i;
                }
            }

            int fallbackIndex = ((int)target) - 1;
            return fallbackIndex >= 0 && fallbackIndex < moonRails.Count ? fallbackIndex : -1;
        }

        [ContextMenu("Load JPL Galilean Moon Rails")]
        private void LoadJplGalileanMoonRails()
        {
            Dictionary<string, Transform> existingVisuals = new Dictionary<string, Transform>(moonRails.Count, StringComparer.OrdinalIgnoreCase);

            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail existingRail = moonRails[i];
                if (existingRail == null || string.IsNullOrWhiteSpace(existingRail.Name))
                {
                    continue;
                }

                existingVisuals[existingRail.Name] = existingRail.VisualTransform;
            }

            moonRails = GalileanMoonPresets.CreateJplGalileanMoons();

            for (int i = 0; i < moonRails.Count; i++)
            {
                MoonRail rail = moonRails[i];
                if (existingVisuals.TryGetValue(rail.Name, out Transform visualTransform))
                {
                    rail.VisualTransform = visualTransform;
                }
            }

            InitializeBodies();
            SyncAllVisuals();
            RebuildMoonOrbitLines();

            if (shipTrajectoryPredictor != null)
            {
                shipTrajectoryPredictor.ForceRefresh();
            }
        }

        private Vector3d ResolveActiveReferenceVisualPosition()
        {
            // Bodies stay at simulated "now"; maneuver horizon affects only plotted trajectory via ManeuverEvaluator.
            double previewTime = simulationTimeSeconds;
            if (TryGetReferenceStateAtTime(
                ResolveActiveReferenceFrameTarget(),
                previewTime,
                out _,
                out Vector3d framePosition,
                out _,
                out _,
                out _,
                out _))
            {
                return framePosition;
            }

            return jupiterRealPosition;
        }

        private void SyncAllVisuals()
        {
            // Ensure the runtime bodies are initialized and lists are in sync before applying visuals.
            EnsureInitialized();
            // Keep all bodies at simulated "now". Flight-plan time horizon only trims the plotted maneuver trajectory.
            double previewTime = simulationTimeSeconds;

            // Apply visual scale for Jupiter and moons based on real radii
            ApplyVisualScale(jupiterTransform, jupiterRadius);
            ApplyVisualPosition(jupiterTransform, jupiterRealPosition);
            ConfigureCelestialBodyRenderers(jupiterTransform);

            int bodyCount = moonBodies != null ? moonBodies.Count : 0;
            int railCount = moonRails != null ? moonRails.Count : 0;
            int iterateCount = Math.Min(bodyCount, railCount);

            for (int i = 0; i < iterateCount; i++)
            {
                MoonRail rail = moonRails[i];
                if (rail == null)
                {
                    continue;
                }

                Transform visual = rail.VisualTransform;
                if (visual == null)
                {
                    continue;
                }

                ApplyVisualScale(visual, rail.Radius);

                // Evaluate moon state at preview time (does not mutate simulation state)
                EvaluateMoonState(rail, previewTime, out Vector3d moonPos, out Vector3d moonVel);
                ApplyVisualPosition(visual, moonPos);

                // Use a temporary CelestialBody for rotation calculations so we don't alter simulation state
                var tempMoon = new CelestialBody(rail.Mass, moonPos, moonVel, rail.ResolveStandardGravitationalParameter());
                ApplyTidallyLockedMoonVisualRotation(rail, visual, tempMoon);
                ConfigureCelestialBodyRenderers(visual);
            }

            if (shipBody != null && ship != null)
            {
                ApplyVisualScale(ship.VisualTransform, ship.VisualRadiusMeters);

                // Ship visual always follows the running simulation here (not the maneuver preview horizon).
                ApplyVisualPosition(ship.VisualTransform, shipBody.Position);
            }

            if (moonOrbitRoot != null)
            {
                // Position the moon orbit root relative to the active reference at preview time
                if (!TryGetReferenceStateAtTime(ResolveActiveReferenceFrameTarget(), previewTime,
                    out _, out Vector3d framePos, out _, out _, out _, out _))
                {
                    framePos = jupiterRealPosition;
                }
                ApplyVisualPosition(moonOrbitRoot, framePos);
            }
        }

        private void ConfigureCelestialBodyRenderers(Transform target)
        {
            if (target == null)
            {
                return;
            }

            rendererBuffer.Clear();
            target.GetComponentsInChildren(true, rendererBuffer);
            for (int i = 0; i < rendererBuffer.Count; i++)
            {
                Renderer bodyRenderer = rendererBuffer[i];
                if (bodyRenderer != null)
                {
                    bodyRenderer.allowOcclusionWhenDynamic = false;
                }
            }

            rendererBuffer.Clear();
        }

        private void ApplyFloatingOriginIfNeeded()
        {
            if (shipBody == null)
            {
                return;
            }

            Vector3 shipVisualPosition = ToUnityPosition(shipBody.Position);
            double activeThreshold = ResolveFloatingOriginThreshold();

            bool exceedsThreshold =
                Math.Abs(shipVisualPosition.x) > activeThreshold ||
                Math.Abs(shipVisualPosition.y) > activeThreshold ||
                Math.Abs(shipVisualPosition.z) > activeThreshold;

            if (!exceedsThreshold)
            {
                return;
            }

            Vector3 visualShift = shipVisualPosition;
            Vector3d realShift = new Vector3d(visualShift.x, visualShift.y, visualShift.z) * GetMetersPerVisualUnit();
            floatingOriginOffset += realShift;

            ShiftLoadedSceneRoots(visualShift);
        }

        private double ResolveFloatingOriginThreshold()
        {
            double threshold = floatingOriginThreshold <= 0d ? 5000d : floatingOriginThreshold;
            if (cameraMode == SpaceCameraMode.ShipFocus)
            {
                threshold = Math.Min(threshold, 1d);
            }

            return Math.Max(0.001d, threshold);
        }

        private static Vector3d RotateOrbitalToWorld(Vector3d vector, double ascendingNode, double inclination, double periapsis)
        {
            double cosOmega = Math.Cos(ascendingNode);
            double sinOmega = Math.Sin(ascendingNode);
            double cosI = Math.Cos(inclination);
            double sinI = Math.Sin(inclination);
            double cosW = Math.Cos(periapsis);
            double sinW = Math.Sin(periapsis);

            double x =
                ((cosOmega * cosW) - (sinOmega * sinW * cosI)) * vector.X +
                ((-cosOmega * sinW) - (sinOmega * cosW * cosI)) * vector.Y;

            double y =
                ((sinOmega * cosW) + (cosOmega * sinW * cosI)) * vector.X +
                ((-sinOmega * sinW) + (cosOmega * cosW * cosI)) * vector.Y;

            double z =
                (sinW * sinI * vector.X) +
                (cosW * sinI * vector.Y);

            return new Vector3d(x, y, z);
        }

        private static double SolveEccentricAnomaly(double meanAnomaly, double eccentricity)
        {
            double estimate = eccentricity < 0.8d ? meanAnomaly : Math.PI;

            for (int i = 0; i < 8; i++)
            {
                double function = estimate - (eccentricity * Math.Sin(estimate)) - meanAnomaly;
                double derivative = 1d - (eccentricity * Math.Cos(estimate));
                estimate -= function / derivative;
            }

            return estimate;
        }

        private static double DegreesToRadians(double degrees)
        {
            return degrees * (Math.PI / 180d);
        }

        private static double NormalizeAngle(double angle)
        {
            double twoPi = Math.PI * 2d;
            angle %= twoPi;

            if (angle < 0d)
            {
                angle += twoPi;
            }

            return angle;
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;
        }

        private int GetSolverStepCount(double frameDt)
        {
            double normalizedFrameDt = Math.Abs(frameDt);
            return Math.Max(1, (int)Math.Ceiling(normalizedFrameDt / maxSolverStepSeconds));
        }

        private double GetUnityScale()
        {
            return metersPerUnityUnit <= 0d ? 1d : metersPerUnityUnit;
        }

        private double GetMetersPerVisualUnit()
        {
            double distanceMultiplier = visualDistanceMultiplier <= 0d ? 1d : visualDistanceMultiplier;
            return GetUnityScale() / distanceMultiplier;
        }

        private double ResolveJupiterMassForInfluence()
        {
            return jupiterStandardGravitationalParameter > 0d
                ? PhysicsSolver.StandardGravitationalParameterToMass(jupiterStandardGravitationalParameter)
                : jupiterMass;
        }

        private void ShiftLoadedSceneRoots(Vector3 visualShift)
        {
            if (worldContainer != null)
            {
                worldContainer.position -= visualShift;
                return;
            }

            for (int sceneIndex = 0; sceneIndex < SceneManager.sceneCount; sceneIndex++)
            {
                Scene scene = SceneManager.GetSceneAt(sceneIndex);
                if (!scene.isLoaded)
                {
                    continue;
                }

                GameObject[] rootObjects = scene.GetRootGameObjects();
                for (int rootIndex = 0; rootIndex < rootObjects.Length; rootIndex++)
                {
                    rootObjects[rootIndex].transform.position -= visualShift;
                }
            }
        }

        private Vector3d ConvertAstrodynamicToSimulationFrame(Vector3d vector)
        {
            switch (astrodynamicPlaneMapping)
            {
                case AstrodynamicPlaneMapping.UnityXyPlaneZUp:
                    return vector;
                case AstrodynamicPlaneMapping.UnityXzPlaneYUp:
                default:
                    return new Vector3d(vector.X, vector.Z, vector.Y);
            }
        }

        private Vector3d ConvertSimulationToAstrodynamicFrame(Vector3d vector)
        {
            switch (astrodynamicPlaneMapping)
            {
                case AstrodynamicPlaneMapping.UnityXyPlaneZUp:
                    return vector;
                case AstrodynamicPlaneMapping.UnityXzPlaneYUp:
                default:
                    return new Vector3d(vector.X, vector.Z, vector.Y);
            }
        }

        private static string FormatDistance(double meters)
        {
            if (double.IsInfinity(meters))
            {
                return "inf";
            }

            if (double.IsNaN(meters))
            {
                return "n/a";
            }

            double absolute = Math.Abs(meters);
            if (absolute >= 1e9d)
            {
                return $"{meters / 1e9d:0.###} Gm";
            }

            if (absolute >= 1e6d)
            {
                return $"{meters / 1e6d:0.###} Mm";
            }

            if (absolute >= 1e3d)
            {
                return $"{meters / 1e3d:0.###} km";
            }

            return $"{meters:0.###} m";
        }

        private static string FormatSpeed(double metersPerSecond)
        {
            if (double.IsNaN(metersPerSecond))
            {
                return "n/a";
            }

            double absolute = Math.Abs(metersPerSecond);
            if (absolute >= 1e3d)
            {
                return $"{metersPerSecond / 1e3d:0.###} km/s";
            }

            return $"{metersPerSecond:0.###} m/s";
        }

        private static string FormatMu(double standardGravitationalParameter)
        {
            return $"{standardGravitationalParameter:0.###e+0} m3/s2";
        }

        private static string FormatDuration(double seconds)
        {
            if (double.IsInfinity(seconds))
            {
                return "open";
            }

            if (double.IsNaN(seconds) || seconds < 0d)
            {
                return "n/a";
            }

            double days = seconds / 86400d;
            if (days >= 1d)
            {
                return $"{days:0.###} d";
            }

            double hours = seconds / 3600d;
            if (hours >= 1d)
            {
                return $"{hours:0.###} h";
            }

            double minutes = seconds / 60d;
            if (minutes >= 1d)
            {
                return $"{minutes:0.###} min";
            }

            return $"{seconds:0.###} s";
        }

        private void WarnInvalidCoordinates(string targetName, Vector3d coordinates)
        {
            if (!warnOnInvalidCoordinates || hasWarnedAboutInvalidCoordinates)
            {
                return;
            }

            hasWarnedAboutInvalidCoordinates = true;
            Debug.LogError($"UniverseManager detected invalid coordinates for '{targetName}': {coordinates}. Simulation has likely diverged.");
        }

        private static bool IsFinite(Vector3 value)
        {
            return
                !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }

        private static bool Approximately(Vector3 left, Vector3 right)
        {
            return
                Mathf.Approximately(left.x, right.x) &&
                Mathf.Approximately(left.y, right.y) &&
                Mathf.Approximately(left.z, right.z);
        }

        private void EnsureTrajectoryVisuals()
        {
            Transform visualParent = worldContainer != null ? worldContainer : transform;
            EnsureTrajectoryRoots(visualParent);
            EnsureShipTrajectoryVisualizer();
            EnsureMoonOrbitVisualizers();
        }

        private void EnsureTrajectoryRoots(Transform visualParent)
        {
            if (trajectoryVisualRoot == null)
            {
                trajectoryVisualRoot = FindChildByName(visualParent, "Trajectory_Visuals");
                if (trajectoryVisualRoot == null)
                {
                    GameObject rootObject = new GameObject("Trajectory_Visuals");
                    trajectoryVisualRoot = rootObject.transform;
                    trajectoryVisualRoot.SetParent(visualParent, false);
                }
            }
            else if (trajectoryVisualRoot.parent != visualParent)
            {
                trajectoryVisualRoot.SetParent(visualParent, false);
            }

            if (moonOrbitRoot == null)
            {
                moonOrbitRoot = FindChildByName(trajectoryVisualRoot, "Moon_Orbits");
                if (moonOrbitRoot == null)
                {
                    GameObject orbitRootObject = new GameObject("Moon_Orbits");
                    moonOrbitRoot = orbitRootObject.transform;
                    moonOrbitRoot.SetParent(trajectoryVisualRoot, false);
                }
            }
            else if (moonOrbitRoot.parent != trajectoryVisualRoot)
            {
                moonOrbitRoot.SetParent(trajectoryVisualRoot, false);
            }

            ApplyVisualPosition(moonOrbitRoot, ResolveActiveReferenceVisualPosition());
        }

        private void EnsureShipTrajectoryVisualizer()
        {
            if (trajectoryVisualRoot == null)
            {
                return;
            }

            bool hideForManeuverOrbitMap =
                cameraMode == SpaceCameraMode.OrbitMap &&
                FindAnyObjectByType<ManeuverEvaluator>() != null;

            if (!showShipTrajectory || hideForManeuverOrbitMap)
            {
                if (shipTrajectoryPredictor != null)
                {
                    shipTrajectoryPredictor.gameObject.SetActive(false);
                }

                return;
            }

            if (shipTrajectoryPredictor == null)
            {
                Transform existingTransform = FindChildByName(trajectoryVisualRoot, "Ship_Trajectory");
                GameObject predictorObject;

                if (existingTransform != null)
                {
                    predictorObject = existingTransform.gameObject;
                }
                else
                {
                    predictorObject = new GameObject("Ship_Trajectory");
                    predictorObject.transform.SetParent(trajectoryVisualRoot, false);
                }

                predictorObject.layer = ResolveShipTrajectoryLayer();
                LineRenderer lineRenderer = predictorObject.GetComponent<LineRenderer>();
                if (lineRenderer == null)
                {
                    lineRenderer = predictorObject.AddComponent<LineRenderer>();
                }

                ConfigureLineRenderer(
                    lineRenderer,
                    shipTrajectoryColor,
                    shipTrajectoryWidth,
                    false);

                shipTrajectoryPredictor = predictorObject.GetComponent<TrajectoryPredictor>();
                if (shipTrajectoryPredictor == null)
                {
                    shipTrajectoryPredictor = predictorObject.AddComponent<TrajectoryPredictor>();
                }
            }

            shipTrajectoryPredictor.gameObject.SetActive(true);
            shipTrajectoryPredictor.gameObject.layer = ResolveShipTrajectoryLayer();

            LineRenderer shipLineRenderer = shipTrajectoryPredictor.GetComponent<LineRenderer>();
            ConfigureLineRenderer(shipLineRenderer, shipTrajectoryColor, shipTrajectoryWidth, false);

            shipTrajectoryPredictor.Configure(this, shipLineRenderer);
            shipTrajectoryPredictor.ConfigurePrediction(
                shipPredictionSteps,
                shipPredictionStepSeconds,
                RecommendedSolverStepSeconds,
                true,
                shipPredictionRefreshInterval,
                shipPredictionStepsPerBatch);
            shipTrajectoryPredictor.ForceRefresh();
        }

        private void EnsureMoonOrbitVisualizers()
        {
            if (moonOrbitRoot == null)
            {
                return;
            }

            moonOrbitRoot.gameObject.SetActive(ShouldShowMoonOrbitVisuals());
            if (!ShouldShowMoonOrbitVisuals())
            {
                if (jupiterOrbitRenderer != null)
                {
                    jupiterOrbitRenderer.gameObject.SetActive(false);
                }

                return;
            }

            EnsureJupiterOrbitVisualizer();

            while (moonOrbitRenderers.Count < moonRails.Count)
            {
                MoonRail rail = moonRails[moonOrbitRenderers.Count];
                GameObject orbitObject = new GameObject($"{rail.Name}_Orbit");
                orbitObject.transform.SetParent(moonOrbitRoot, false);
                orbitObject.layer = ResolveMoonOrbitLayer(rail);

                LineRenderer orbitRenderer = orbitObject.AddComponent<LineRenderer>();
                ConfigureMoonOrbitRenderer(orbitRenderer, false, ResolveMoonOrbitColor(moonOrbitRenderers.Count));
                moonOrbitRenderers.Add(orbitRenderer);
            }

            for (int i = 0; i < moonOrbitRenderers.Count; i++)
            {
                bool shouldBeActive = i < moonRails.Count;
                if (moonOrbitRenderers[i] != null)
                {
                    moonOrbitRenderers[i].gameObject.SetActive(shouldBeActive);
                }
            }

            RebuildMoonOrbitLines();
        }

        private void RebuildMoonOrbitLines()
        {
            if (moonOrbitRoot == null)
            {
                return;
            }

            if (!ShouldShowMoonOrbitVisuals())
            {
                UpdateMoonOrbitVisibility();
                return;
            }

            ReferenceFrameTarget activeFrame = ResolveActiveReferenceFrameTarget();
            double previewTime = simulationTimeSeconds + Math.Max(0d, previewTimeOffsetSeconds);
            if (!TryGetReferenceStateAtTime(
                activeFrame,
                previewTime,
                out _,
                out Vector3d currentFramePosition,
                out _,
                out _,
                out _,
                out _))
            {
                currentFramePosition = jupiterRealPosition;
            }

            ApplyVisualPosition(moonOrbitRoot, currentFramePosition);

            int sampleCount = Math.Max(16, moonOrbitSamples);
            RebuildJupiterOrbitLine(activeFrame, sampleCount, previewTime);

            for (int railIndex = 0; railIndex < moonRails.Count; railIndex++)
            {
                if (railIndex >= moonOrbitRenderers.Count || moonOrbitRenderers[railIndex] == null)
                {
                    continue;
                }

                MoonRail rail = moonRails[railIndex];
                LineRenderer orbitRenderer = moonOrbitRenderers[railIndex];
                ReferenceFrameTarget orbitTarget = ResolveRailReferenceFrameTarget(railIndex);
                bool isFocusedOrbit = orbitTarget == orbitMapFocusTarget && orbitMapFocusTarget != ReferenceFrameTarget.Jupiter;
                bool shouldRender = orbitTarget != activeFrame || isFocusedOrbit;

                orbitRenderer.gameObject.name = $"{rail.Name}_Orbit";
                orbitRenderer.gameObject.layer = ResolveMoonOrbitLayer(rail);
                orbitRenderer.gameObject.SetActive(shouldRender);

                if (!shouldRender)
                {
                    orbitRenderer.positionCount = 0;
                    continue;
                }

                Color moonColor = ResolveMoonOrbitColor(railIndex);
                ConfigureMoonOrbitRenderer(orbitRenderer, false, moonColor);

                double orbitalPeriod = ResolveOrbitPeriodSeconds(rail);
                int currentSampleIndex = isFocusedOrbit ? ResolveCurrentOrbitSampleIndex(sampleCount, moonOrbitHistoryFraction) : -1;

                orbitRenderer.positionCount = sampleCount;
                for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
                {
                    double orbitFraction = sampleCount <= 1 ? 0d : (double)sampleIndex / (sampleCount - 1);
                    double sampleTime = sampleIndex == currentSampleIndex
                        ? previewTime
                        : ResolveFadedOrbitSampleTime(orbitalPeriod, orbitFraction, previewTime, moonOrbitHistoryFraction);
                    EvaluateMoonState(rail, sampleTime, out Vector3d moonPosition, out _);

                    bool useFixedFocusOrigin = isFocusedOrbit && activeFrame == orbitTarget;
                    if (useFixedFocusOrigin)
                    {
                        orbitRenderer.SetPosition(sampleIndex, ToUnityOffset(moonPosition - currentFramePosition));
                        continue;
                    }

                    if (!TryGetReferenceStateAtTime(
                        activeFrame,
                        sampleTime,
                        out _,
                        out Vector3d framePosition,
                        out _,
                        out _,
                        out _,
                        out _))
                    {
                        framePosition = jupiterRealPosition;
                    }

                    orbitRenderer.SetPosition(sampleIndex, ToUnityOffset(moonPosition - framePosition));
                }
            }
        }

        private void EnsureJupiterOrbitVisualizer()
        {
            if (moonOrbitRoot == null || jupiterOrbitRenderer != null)
            {
                return;
            }

            Transform existingTransform = FindChildByName(moonOrbitRoot, "Jupiter_Orbit");
            GameObject orbitObject = existingTransform != null
                ? existingTransform.gameObject
                : new GameObject("Jupiter_Orbit");

            orbitObject.transform.SetParent(moonOrbitRoot, false);
            orbitObject.layer = jupiterTransform != null ? jupiterTransform.gameObject.layer : gameObject.layer;

            jupiterOrbitRenderer = orbitObject.GetComponent<LineRenderer>();
            if (jupiterOrbitRenderer == null)
            {
                jupiterOrbitRenderer = orbitObject.AddComponent<LineRenderer>();
            }

            ConfigureMoonOrbitRenderer(jupiterOrbitRenderer, false, ResolveMoonOrbitColor(-1));
        }

        private void RebuildJupiterOrbitLine(ReferenceFrameTarget activeFrame, int sampleCount, double previewTime)
        {
            if (jupiterOrbitRenderer == null)
            {
                return;
            }

            MoonRail activeRail = null;
            bool shouldRender = activeFrame != ReferenceFrameTarget.Jupiter && TryGetMoonRail(activeFrame, out activeRail);
            jupiterOrbitRenderer.gameObject.SetActive(shouldRender);

            if (!shouldRender)
            {
                jupiterOrbitRenderer.positionCount = 0;
                return;
            }

            ConfigureMoonOrbitRenderer(jupiterOrbitRenderer, false, ResolveMoonOrbitColor(-1));
            double orbitalPeriod = ResolveOrbitPeriodSeconds(activeRail);

            jupiterOrbitRenderer.positionCount = sampleCount;
            for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
            {
                double orbitFraction = sampleCount <= 1 ? 0d : (double)sampleIndex / (sampleCount - 1);
                double sampleTime = ResolveFadedOrbitSampleTime(orbitalPeriod, orbitFraction, previewTime, moonOrbitHistoryFraction);

                if (!TryGetReferenceStateAtTime(
                    activeFrame,
                    sampleTime,
                    out _,
                    out Vector3d framePosition,
                    out _,
                    out _,
                    out _,
                    out _))
                {
                    framePosition = jupiterRealPosition;
                }

                jupiterOrbitRenderer.SetPosition(sampleIndex, ToUnityOffset(jupiterRealPosition - framePosition));
            }
        }

        private bool TryGetMoonRail(ReferenceFrameTarget target, out MoonRail rail)
        {
            int moonIndex = FindMoonIndex(target);
            if (moonIndex >= 0 && moonIndex < moonRails.Count)
            {
                rail = moonRails[moonIndex];
                return rail != null;
            }

            rail = null;
            return false;
        }

        private ReferenceFrameTarget ResolveRailReferenceFrameTarget(int railIndex)
        {
            if (railIndex >= 0 && railIndex < moonRails.Count)
            {
                MoonRail rail = moonRails[railIndex];
                if (rail != null)
                {
                    foreach (ReferenceFrameTarget target in Enum.GetValues(typeof(ReferenceFrameTarget)))
                    {
                        if (target != ReferenceFrameTarget.Jupiter &&
                            string.Equals(rail.Name, target.ToString(), StringComparison.OrdinalIgnoreCase))
                        {
                            return target;
                        }
                    }
                }
            }

            int enumIndex = railIndex + 1;
            return Enum.IsDefined(typeof(ReferenceFrameTarget), enumIndex)
                ? (ReferenceFrameTarget)enumIndex
                : ReferenceFrameTarget.Jupiter;
        }

        private double ResolveOrbitPeriodSeconds(MoonRail rail)
        {
            double semiMajorAxis = Math.Max(rail.ResolveSemiMajorAxis(), 1d);
            double orbitalMu = jupiterStandardGravitationalParameter + rail.ResolveStandardGravitationalParameter();
            return 2d * Math.PI * Math.Sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / orbitalMu);
        }

        private void RefreshTrajectoryLineStyles()
        {
            if (shipTrajectoryPredictor != null)
            {
                LineRenderer shipLineRenderer = shipTrajectoryPredictor.GetComponent<LineRenderer>();
                ConfigureLineRenderer(shipLineRenderer, shipTrajectoryColor, shipTrajectoryWidth, false);
            }

            for (int i = 0; i < moonOrbitRenderers.Count; i++)
            {
                LineRenderer orbitRenderer = moonOrbitRenderers[i];
                if (orbitRenderer == null)
                {
                    continue;
                }

                ConfigureMoonOrbitRenderer(orbitRenderer, orbitRenderer.loop, ResolveMoonOrbitColor(i));
            }

            if (jupiterOrbitRenderer != null)
            {
                ConfigureMoonOrbitRenderer(jupiterOrbitRenderer, jupiterOrbitRenderer.loop, ResolveMoonOrbitColor(-1));
            }
        }

        private float ResolveMoonOrbitLineWidth()
        {
            float worldWidth = Mathf.Max(0.001f, moonOrbitWidth * 1.5f); // Increase width slightly
            if (celestialCamera == null)
{
                return worldWidth;
            }

            if (!celestialCamera.orthographic && cameraMode != SpaceCameraMode.OrbitMap)
            {
                return worldWidth;
            }

            float pixelHeight = Mathf.Max(1f, celestialCamera.pixelHeight);
            float viewHalfHeight = celestialCamera.orthographic
                ? celestialCamera.orthographicSize
                : Mathf.Max(0.001f, smoothedOrbitMapOrthographicSize);
            float worldUnitsPerPixel = (viewHalfHeight * 2f) / pixelHeight;
            float screenWidth = worldUnitsPerPixel * Mathf.Max(1f, moonOrbitScreenWidth);
            return Mathf.Max(worldWidth, screenWidth);
        }

        public float ResolveWorldLineWidthForPixels(float desiredPixels, float minWorldWidth)
        {
            float worldWidth = Mathf.Max(0.0001f, minWorldWidth);
            if (celestialCamera == null)
            {
                return worldWidth;
            }

            float pixelHeight = Mathf.Max(1f, celestialCamera.pixelHeight);
            float viewHalfHeight;

            if (celestialCamera.orthographic)
            {
                viewHalfHeight = Mathf.Max(0.001f, celestialCamera.orthographicSize);
            }
            else if (cameraMode == SpaceCameraMode.OrbitMap)
            {
                viewHalfHeight = Mathf.Max(0.001f, smoothedOrbitMapOrthographicSize);
            }
            else
            {
                return worldWidth;
            }

            float worldUnitsPerPixel = (viewHalfHeight * 2f) / pixelHeight;
            float screenWidth = worldUnitsPerPixel * Mathf.Max(0.5f, desiredPixels);
            return Mathf.Max(worldWidth, screenWidth);
        }

        private bool ShouldShowMoonOrbitVisuals()
        {
            return showMoonOrbits && cameraMode == SpaceCameraMode.OrbitMap;
        }

        private void UpdateMoonOrbitVisibility()
        {
            bool shouldShow = ShouldShowMoonOrbitVisuals();
            if (moonOrbitRoot != null)
            {
                moonOrbitRoot.gameObject.SetActive(shouldShow);
            }

            if (shouldShow)
            {
                for (int i = 0; i < moonOrbitRenderers.Count; i++)
                {
                    if (moonOrbitRenderers[i] != null)
                    {
                        moonOrbitRenderers[i].gameObject.SetActive(moonOrbitRenderers[i].positionCount > 0);
                    }
                }

                if (jupiterOrbitRenderer != null)
                {
                    jupiterOrbitRenderer.gameObject.SetActive(jupiterOrbitRenderer.positionCount > 0);
                }

                return;
            }

            for (int i = 0; i < moonOrbitRenderers.Count; i++)
            {
                if (moonOrbitRenderers[i] != null)
                {
                    moonOrbitRenderers[i].gameObject.SetActive(false);
                }
            }

            if (jupiterOrbitRenderer != null)
            {
                jupiterOrbitRenderer.gameObject.SetActive(false);
            }
        }

        private double ResolveFadedOrbitSampleTime(double orbitalPeriod, double orbitFraction, double centerTimeSeconds, float historyFraction)
        {
            const double aheadFraction = 0.15d;
            double clampedHistory  = Mathf.Clamp01(historyFraction);
            double totalSpan       = aheadFraction + clampedHistory * (1d - aheadFraction);
            double clampedFraction = Clamp(orbitFraction, 0d, 1d);
            return centerTimeSeconds + (aheadFraction - clampedFraction * totalSpan) * orbitalPeriod;
        }

        private static int ResolveCurrentOrbitSampleIndex(int sampleCount, float historyFraction)
        {
            if (sampleCount <= 1) return 0;
            const double aheadFraction = 0.15d;
            double clampedHistory = Mathf.Clamp01(historyFraction);
            double totalSpan      = aheadFraction + clampedHistory * (1d - aheadFraction);
            double headT          = totalSpan > 0d ? aheadFraction / totalSpan : 0d;
            return Mathf.Clamp(Mathf.RoundToInt((float)(headT * (sampleCount - 1))), 0, sampleCount - 1);
        }

        private void ConfigureMoonOrbitRenderer(LineRenderer lineRenderer, bool loop, Color color)
        {
            ConfigureLineRenderer(lineRenderer, color, ResolveMoonOrbitLineWidth(), loop);
            if (lineRenderer == null)
            {
                return;
            }

            const float aheadFraction = 0.15f;
            float h     = Mathf.Clamp01(moonOrbitHistoryFraction);
            float span  = aheadFraction + h * (1f - aheadFraction);
            float headT = span > 0f ? aheadFraction / span : 0f;
            lineRenderer.colorGradient = BuildMoonOrbitFadeGradient(color, headT);
        }

        private static Gradient BuildMoonOrbitFadeGradient(Color baseColor, float headT)
        {
            headT = Mathf.Clamp01(headT);
            float tailT    = Mathf.Min(headT + 0.08f, 1f);
            float midPastT = Mathf.Min(headT + 0.40f, 1f);

            Color bright    = Color.Lerp(baseColor, Color.white, 0.75f); bright.a = 1f;
            Color mid       = baseColor; mid.a = 1f;
            Color dimFuture = new Color(baseColor.r * 0.50f, baseColor.g * 0.50f, baseColor.b * 0.50f, 1f);
            Color dimPast   = new Color(baseColor.r * 0.18f, baseColor.g * 0.18f, baseColor.b * 0.18f, 1f);

            Gradient gradient = new Gradient();
            gradient.SetKeys(
                new[]
                {
                    new GradientColorKey(dimFuture, 0f),
                    new GradientColorKey(bright,    headT),
                    new GradientColorKey(mid,       tailT),
                    new GradientColorKey(dimPast,   midPastT),
                    new GradientColorKey(dimPast,   1f)
                },
                new[]
                {
                    new GradientAlphaKey(baseColor.a * 0.40f, 0f),
                    new GradientAlphaKey(1.00f,                headT),
                    new GradientAlphaKey(baseColor.a * 0.80f,  tailT),
                    new GradientAlphaKey(baseColor.a * 0.15f,  midPastT),
                    new GradientAlphaKey(0.00f,                1f)
                });
            return gradient;
        }

        private void ConfigureLineRenderer(LineRenderer lineRenderer, Color color, float width, bool loop)
        {
            if (lineRenderer == null)
            {
                return;
            }

            lineRenderer.useWorldSpace = false;
            lineRenderer.loop = loop;
            lineRenderer.startWidth = width;
            lineRenderer.endWidth = width;
            lineRenderer.startColor = color;
            lineRenderer.endColor = color;
            lineRenderer.alignment = LineAlignment.View;
            lineRenderer.numCornerVertices = 6;
            lineRenderer.numCapVertices = 6;
            lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lineRenderer.receiveShadows = false;
            lineRenderer.textureMode = LineTextureMode.Stretch;

            Material lineMaterial = GetOrCreateRuntimeLineMaterial();
            if (lineMaterial != null)
            {
                lineRenderer.sharedMaterial = lineMaterial;
            }

            // Subtle head glow / fade helps readability (Principia-like)
            if (!loop)
            {
                lineRenderer.colorGradient = BuildShipTrajectoryFadeGradient(color);
            }
        }

        private static Gradient BuildShipTrajectoryFadeGradient(Color baseColor)
        {
            Color c = baseColor; c.a = 1f;
            Gradient g = new Gradient();
            g.SetKeys(
                new[]
                {
                    new GradientColorKey(Color.white,  0f),    // нос: белое свечение
                    new GradientColorKey(c,            0.12f), // быстро переходит в цвет
                    new GradientColorKey(c * 0.65f,   0.55f), // середина: тускнее
                    new GradientColorKey(c * 0.20f,   1f)     // хвост: совсем dim
                },
                new[]
                {
                    new GradientAlphaKey(1.00f,                0f),
                    new GradientAlphaKey(baseColor.a * 0.95f,  0.10f),
                    new GradientAlphaKey(baseColor.a * 0.50f,  0.55f),
                    new GradientAlphaKey(0.0f,                 1f)
                });
            return g;
        }

        private Color ResolveMoonOrbitColor(int railIndex)
        {
            switch (railIndex)
            {
                case 0:  return new Color(1.00f, 0.80f, 0.22f, 0.95f); // Io: золотисто-жёлтый
                case 1:  return new Color(0.42f, 0.80f, 1.00f, 0.95f); // Europa: ледяной голубой
                case 2:  return new Color(0.68f, 0.72f, 0.60f, 0.95f); // Ganymede: серо-зелёный
                case 3:  return new Color(0.72f, 0.52f, 0.35f, 0.95f); // Callisto: тёмно-коричневый
                default: return moonOrbitColor;                           // fallback / Jupiter frame
            }
        }

        private Material GetOrCreateRuntimeLineMaterial()
        {
            if (runtimeLineMaterial != null)
            {
                return runtimeLineMaterial;
            }

            Shader selectedShader =
                Shader.Find("Sprites/Default") ??
                Shader.Find("Universal Render Pipeline/Unlit") ??
                Shader.Find("Unlit/Color");

            if (selectedShader == null)
            {
                return null;
            }

            runtimeLineMaterial = new Material(selectedShader)
            {
                hideFlags = HideFlags.DontSave
            };

            if (runtimeLineMaterial.HasProperty("_BaseColor"))
            {
                runtimeLineMaterial.SetColor("_BaseColor", Color.white);
            }

            if (runtimeLineMaterial.HasProperty("_Color"))
            {
                runtimeLineMaterial.SetColor("_Color", Color.white);
            }

            return runtimeLineMaterial;
        }

        private Material GetOrCreateOrbitMapMarkerMaterial()
        {
            if (runtimeOrbitMapMarkerMaterial != null)
            {
                return runtimeOrbitMapMarkerMaterial;
            }

            Shader selectedShader =
                Shader.Find("Universal Render Pipeline/Unlit") ??
                Shader.Find("Unlit/Color") ??
                Shader.Find("Sprites/Default") ??
                Shader.Find("Standard");

            if (selectedShader == null)
            {
                return null;
            }

            runtimeOrbitMapMarkerMaterial = new Material(selectedShader)
            {
                hideFlags = HideFlags.DontSave
            };

            if (runtimeOrbitMapMarkerMaterial.HasProperty("_BaseColor"))
            {
                runtimeOrbitMapMarkerMaterial.SetColor("_BaseColor", Color.white);
            }

            if (runtimeOrbitMapMarkerMaterial.HasProperty("_Color"))
            {
                runtimeOrbitMapMarkerMaterial.SetColor("_Color", Color.white);
            }

            return runtimeOrbitMapMarkerMaterial;
        }

        private int ResolveShipTrajectoryLayer()
        {
            int trajectoryLayer = LayerMask.NameToLayer("Trajectory");
            if (trajectoryLayer >= 0)
            {
                return trajectoryLayer;
            }

            return ship != null && ship.VisualTransform != null
                ? ship.VisualTransform.gameObject.layer
                : gameObject.layer;
        }

        private int ResolveMoonOrbitLayer(MoonRail rail)
        {
            if (rail != null && rail.VisualTransform != null)
            {
                return rail.VisualTransform.gameObject.layer;
            }

            if (jupiterTransform != null)
            {
                return jupiterTransform.gameObject.layer;
            }

            return gameObject.layer;
        }

        private static Transform FindChildByName(Transform parent, string childName)
        {
            if (parent == null)
            {
                return null;
            }

            for (int i = 0; i < parent.childCount; i++)
            {
                Transform child = parent.GetChild(i);
                if (string.Equals(child.name, childName, StringComparison.Ordinal))
                {
                    return child;
                }
            }

            return null;
        }
    }
}

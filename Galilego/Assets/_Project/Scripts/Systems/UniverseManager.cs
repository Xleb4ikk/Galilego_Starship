using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Galilego.Physics
{
    public sealed class UniverseManager : MonoBehaviour
    {
        [Header("Jupiter")]
        [SerializeField] private Transform jupiterTransform;
        [SerializeField] private double jupiterMass = 1.89813e27d;
        [SerializeField] private double jupiterStandardGravitationalParameter = 1.266865319e17d;
        [SerializeField] private double jupiterRadius = 6.9911e7d;
        [SerializeField] private Vector3d jupiterRealPosition = Vector3d.Zero;

        [Header("Ship")]
        [SerializeField] private ShipSettings ship = new ShipSettings();

        [Header("Moon Rails")]
        [SerializeField] private List<MoonRail> moonRails = new List<MoonRail>();

        [Header("Scene")]
        [SerializeField] private Transform worldContainer;

        [Header("Visual Scale")]
        [SerializeField] private double visualDistanceMultiplier = 0.1d;

        [Header("Simulation")]
        [SerializeField] private double simulationTimeSeconds;
        [SerializeField] private double timeScale = 1d;
        [SerializeField] private double maxSolverStepSeconds = 1d;
        [SerializeField] private double metersPerUnityUnit = 100000d;
        [SerializeField] private double floatingOriginThreshold = 5000d;

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
        [SerializeField] private Color shipTrajectoryColor = new Color(0.25f, 0.95f, 1f, 0.95f);
        [SerializeField] private Color moonOrbitColor = new Color(0.65f, 0.85f, 1f, 0.4f);

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
        private TrajectoryPredictor shipTrajectoryPredictor;
        private Material runtimeLineMaterial;
        private bool rebuildRequested = false;

        public IReadOnlyList<CelestialBody> MoonBodies => moonBodies;
        public CelestialBody ShipBody => shipBody;
        public double SimulationTimeSeconds => simulationTimeSeconds;
        public double RecommendedSolverStepSeconds => maxSolverStepSeconds;
        public double MetersPerUnityUnit => GetMetersPerVisualUnit();
        public Vector3d FloatingOriginOffset => floatingOriginOffset;

        private void Awake()
        {
            InitializeBodies();
            SyncAllVisuals();
            EnsureTrajectoryVisuals();
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
            double stepDt = frameDt / stepCount;

            for (int i = 0; i < stepCount; i++)
            {
                StepSimulation(stepDt);
            }

            ApplyFloatingOriginIfNeeded();
            SyncAllVisuals();
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
            const float minScale = 0.0001f;
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

                        Vector3 newLocalScale = meshTransform.localScale * finalMultiplier;
                        meshTransform.localScale = newLocalScale;
                        Debug.Log($"UniverseManager: scaled mesh '{meshTransform.name}' to localScale={newLocalScale} (desiredWorldRadius={desiredWorldRadius})");
                        return;
                    }
                }
            }

            // Fallback: assume model radius 0.5 at localScale == 1
            target.localScale = new Vector3(clamped, clamped, clamped);
            Debug.Log($"UniverseManager: scaled '{target.name}' fallback. uniformScale={clamped}");
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
            }

            // Avoid creating GameObjects or calling SendMessage during OnValidate.
            // Mark that an editor/runtime rebuild is required; actual visual creation will occur in Awake/Start.
            rebuildRequested = true;
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

            position = jupiterRealPosition + RotateOrbitalToWorld(orbitalPosition, ascendingNode, inclination, periapsis);
            velocity = RotateOrbitalToWorld(orbitalVelocity, ascendingNode, inclination, periapsis);
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

        private void SyncAllVisuals()
        {
            // Ensure the runtime bodies are initialized and lists are in sync before applying visuals.
            EnsureInitialized();

            // Apply visual scale for Jupiter and moons based on real radii
            ApplyVisualScale(jupiterTransform, jupiterRadius);
            ApplyVisualPosition(jupiterTransform, jupiterRealPosition);

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
                ApplyVisualPosition(visual, moonBodies[i].Position);
            }

            if (shipBody != null)
            {
                ApplyVisualPosition(ship.VisualTransform, shipBody.Position);
            }

            if (moonOrbitRoot != null)
            {
                ApplyVisualPosition(moonOrbitRoot, jupiterRealPosition);
            }
        }

        private void ApplyFloatingOriginIfNeeded()
        {
            Vector3 shipVisualPosition = ToUnityPosition(shipBody.Position);

            bool exceedsThreshold =
                Math.Abs(shipVisualPosition.x) > floatingOriginThreshold ||
                Math.Abs(shipVisualPosition.y) > floatingOriginThreshold ||
                Math.Abs(shipVisualPosition.z) > floatingOriginThreshold;

            if (!exceedsThreshold)
            {
                return;
            }

            Vector3 visualShift = shipVisualPosition;
            Vector3d realShift = new Vector3d(visualShift.x, visualShift.y, visualShift.z) * GetMetersPerVisualUnit();
            floatingOriginOffset += realShift;

            ShiftLoadedSceneRoots(visualShift);
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

            ApplyVisualPosition(moonOrbitRoot, jupiterRealPosition);
        }

        private void EnsureShipTrajectoryVisualizer()
        {
            if (trajectoryVisualRoot == null)
            {
                return;
            }

            if (!showShipTrajectory)
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

            moonOrbitRoot.gameObject.SetActive(showMoonOrbits);
            if (!showMoonOrbits)
            {
                return;
            }

            while (moonOrbitRenderers.Count < moonRails.Count)
            {
                MoonRail rail = moonRails[moonOrbitRenderers.Count];
                GameObject orbitObject = new GameObject($"{rail.Name}_Orbit");
                orbitObject.transform.SetParent(moonOrbitRoot, false);
                orbitObject.layer = ResolveMoonOrbitLayer(rail);

                LineRenderer orbitRenderer = orbitObject.AddComponent<LineRenderer>();
                ConfigureLineRenderer(orbitRenderer, moonOrbitColor, moonOrbitWidth, true);
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
            if (!showMoonOrbits || moonOrbitRoot == null)
            {
                return;
            }

            ApplyVisualPosition(moonOrbitRoot, jupiterRealPosition);

            int sampleCount = Math.Max(16, moonOrbitSamples);
            for (int railIndex = 0; railIndex < moonRails.Count; railIndex++)
            {
                if (railIndex >= moonOrbitRenderers.Count || moonOrbitRenderers[railIndex] == null)
                {
                    continue;
                }

                MoonRail rail = moonRails[railIndex];
                LineRenderer orbitRenderer = moonOrbitRenderers[railIndex];
                orbitRenderer.gameObject.name = $"{rail.Name}_Orbit";
                orbitRenderer.gameObject.layer = ResolveMoonOrbitLayer(rail);
                ConfigureLineRenderer(orbitRenderer, moonOrbitColor, moonOrbitWidth, true);

                double semiMajorAxis = Math.Max(rail.ResolveSemiMajorAxis(), 1d);
                double orbitalMu = jupiterStandardGravitationalParameter + rail.ResolveStandardGravitationalParameter();
                double orbitalPeriod = 2d * Math.PI * Math.Sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / orbitalMu);

                orbitRenderer.positionCount = sampleCount;
                for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
                {
                    double orbitFraction = (double)sampleIndex / sampleCount;
                    double sampleTime = simulationTimeSeconds + (orbitFraction * orbitalPeriod);
                    EvaluateMoonState(rail, sampleTime, out Vector3d moonPosition, out _);
                    orbitRenderer.SetPosition(sampleIndex, ToUnityOffset(moonPosition - jupiterRealPosition));
                }
            }
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
            lineRenderer.numCornerVertices = 4;
            lineRenderer.numCapVertices = 4;
            lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lineRenderer.receiveShadows = false;
            lineRenderer.textureMode = LineTextureMode.Stretch;

            Material lineMaterial = GetOrCreateRuntimeLineMaterial();
            if (lineMaterial != null)
            {
                lineRenderer.sharedMaterial = lineMaterial;
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

        private int ResolveShipTrajectoryLayer()
        {
            return ship.VisualTransform != null
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

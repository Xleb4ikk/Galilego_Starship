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
        [SerializeField] private Vector3d jupiterRealPosition = Vector3d.Zero;

        [Header("Ship")]
        [SerializeField] private ShipSettings ship = new ShipSettings();

        [Header("Moon Rails")]
        [SerializeField] private List<MoonRail> moonRails = new List<MoonRail>();

        [Header("Scene")]
        [SerializeField] private Transform worldContainer;

        [Header("Simulation")]
        [SerializeField] private double simulationTimeSeconds;
        [SerializeField] private double timeScale = 1d;
        [SerializeField] private double maxSolverStepSeconds = 1d;
        [SerializeField] private double metersPerUnityUnit = 100000d;
        [SerializeField] private double floatingOriginThreshold = 5000d;

        private readonly List<CelestialBody> moonBodies = new List<CelestialBody>();

        private CelestialBody jupiterBody;
        private CelestialBody shipBody;
        private Vector3d floatingOriginOffset = Vector3d.Zero;

        public IReadOnlyList<CelestialBody> MoonBodies => moonBodies;
        public CelestialBody ShipBody => shipBody;
        public double SimulationTimeSeconds => simulationTimeSeconds;
        public double RecommendedSolverStepSeconds => maxSolverStepSeconds;
        public double MetersPerUnityUnit => GetUnityScale();
        public Vector3d FloatingOriginOffset => floatingOriginOffset;

        private void Awake()
        {
            InitializeBodies();
            SyncAllVisuals();
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

            target.position = ToUnityPosition(realPosition);
        }

        public Vector3 ToUnityPosition(Vector3d realPosition)
        {
            Vector3d localPosition = (realPosition - floatingOriginOffset) / GetUnityScale();
            return new Vector3((float)localPosition.X, (float)localPosition.Y, (float)localPosition.Z);
        }

        public Vector3 ToUnityOffset(Vector3d realOffset)
        {
            Vector3d scaledOffset = realOffset / GetUnityScale();
            return new Vector3((float)scaledOffset.X, (float)scaledOffset.Y, (float)scaledOffset.Z);
        }

        public Vector3d EvaluateShipAccelerationAt(Vector3d shipPosition, double sampleTimeSeconds)
        {
            EnsureInitialized();
            return EvaluateShipAcceleration(shipPosition, sampleTimeSeconds);
        }

        private void InitializeBodies()
        {
            jupiterBody = new CelestialBody(jupiterMass, jupiterRealPosition, Vector3d.Zero);
            shipBody = new CelestialBody(ship.Mass, ship.InitialPosition, ship.InitialVelocity);

            moonBodies.Clear();
            for (int i = 0; i < moonRails.Count; i++)
            {
                moonBodies.Add(new CelestialBody(moonRails[i].Mass, Vector3d.Zero, Vector3d.Zero));
            }

            UpdateMoonBodies(simulationTimeSeconds);
        }

        private void OnValidate()
        {
            if (maxSolverStepSeconds <= 0d)
            {
                maxSolverStepSeconds = 1d;
            }

            if (metersPerUnityUnit <= 0d)
            {
                metersPerUnityUnit = 1d;
            }

            if (floatingOriginThreshold <= 0d)
            {
                floatingOriginThreshold = 5000d;
            }
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
            double semiMajorAxis = Math.Max(rail.SemiMajorAxis, 1d);
            double eccentricity = Clamp(rail.Eccentricity, 0d, 0.999d);
            double inclination = DegreesToRadians(rail.InclinationDegrees);
            double ascendingNode = DegreesToRadians(rail.LongitudeOfAscendingNodeDegrees);
            double periapsis = DegreesToRadians(rail.ArgumentOfPeriapsisDegrees);
            double meanAnomalyAtEpoch = DegreesToRadians(rail.MeanAnomalyAtEpochDegrees);

            double gravitationalParameter = PhysicsSolver.GravitationalConstant * (jupiterMass + rail.Mass);
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

            simulationTimeSeconds = stepStartTime + dt;
            shipBody.SetState(shipStep.Position, shipStep.Velocity);
            UpdateMoonBodies(simulationTimeSeconds);
        }

        private Vector3d EvaluateShipAcceleration(Vector3d shipPosition, double sampleTimeSeconds)
        {
            Vector3d totalAcceleration = PhysicsSolver.CalculateAcceleration(shipPosition, jupiterRealPosition, jupiterMass);

            for (int i = 0; i < moonRails.Count; i++)
            {
                EvaluateMoonState(moonRails[i], sampleTimeSeconds, out Vector3d moonPosition, out _);
                totalAcceleration += PhysicsSolver.CalculateAcceleration(shipPosition, moonPosition, moonRails[i].Mass);
            }

            return totalAcceleration;
        }

        private void SyncAllVisuals()
        {
            ApplyVisualPosition(jupiterTransform, jupiterRealPosition);

            for (int i = 0; i < moonRails.Count; i++)
            {
                ApplyVisualPosition(moonRails[i].VisualTransform, moonBodies[i].Position);
            }

            ApplyVisualPosition(ship.VisualTransform, shipBody.Position);
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
            Vector3d realShift = new Vector3d(visualShift.x, visualShift.y, visualShift.z) * GetUnityScale();
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
    }
}

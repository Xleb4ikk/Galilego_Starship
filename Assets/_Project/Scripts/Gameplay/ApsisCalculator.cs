using System;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;
using Unity.Mathematics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Analytical apsis calculation system that replaces trajectory-point-based detection.
    /// Calculates periapsis and apoapsis positions using Keplerian orbital elements,
    /// eliminating race conditions and improving accuracy.
    /// </summary>
    [RequireComponent(typeof(ApsisMarkerSystem))]
    public class ApsisCalculator : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private ManeuverEvaluator maneuverEvaluator;
        [SerializeField] private ApsisMarkerSystem apsisMarkerSystem;

        [Header("Visibility Rules")]
        [Tooltip("Only show apsides that haven't been reached yet")]
        [SerializeField] private bool showOnlyFutureApsides = true;

        [Tooltip("Hide apsides below the surface (negative altitude)")]
        [SerializeField] private bool showBelowSurface = false;

        [Tooltip("Maximum time into the future to predict apsides (seconds). Used as fallback if trajectory time is unavailable.")]
        [SerializeField] private double maxPredictionTime = 10800.0; // ~3 hours fallback

        [Tooltip("Minimum altitude to show apsis markers (meters)")]
        [SerializeField] private double minAltitude = 0.0;

        [Tooltip("Eccentricity threshold below which orbit is considered circular")]
        [SerializeField] private double circularOrbitThreshold = 0.001;

        [Header("Performance")]
        [Tooltip("Position change threshold to trigger recalculation (meters)")]
        [SerializeField] private double positionChangeThreshold = 1.0;

        [Tooltip("Velocity change threshold to trigger recalculation (m/s)")]
        [SerializeField] private double velocityChangeThreshold = 0.1;

        [Tooltip("Time change threshold to trigger recalculation (seconds)")]
        [SerializeField] private double timeChangeThreshold = 0.1;

        // Cache to avoid recalculation when state hasn't changed
        private List<ApsisData> apsisDataCache = new List<ApsisData>();
        private Vector3d cachedShipPosition;
        private Vector3d cachedShipVelocity;
        private double cachedSimulationTime;
        private ReferenceFrameTarget lastReferenceFrame;
        private bool hasCachedData = false;

        private void Awake()
        {
            ResolveReferences();
        }

        private void OnEnable()
        {
            if (universeManager != null)
            {
                universeManager.ActiveReferenceFrameChanged -= OnReferenceFrameChanged;
                universeManager.ActiveReferenceFrameChanged += OnReferenceFrameChanged;
            }
        }

        private void OnDisable()
        {
            if (universeManager != null)
            {
                universeManager.ActiveReferenceFrameChanged -= OnReferenceFrameChanged;
            }
        }

        private void Start()
        {
            // Initialization complete
        }

        private void Update()
        {
            // Removed debug spam
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();

            if (maneuverEvaluator == null)
                maneuverEvaluator = FindAnyObjectByType<ManeuverEvaluator>();

            if (apsisMarkerSystem == null)
                apsisMarkerSystem = GetComponent<ApsisMarkerSystem>();

            // FlightPlan is typically managed by ManeuverEvaluator
            // We'll access it through ManeuverEvaluator if needed
        }

        private void OnReferenceFrameChanged(ReferenceFrameTarget newFrame)
        {
            // Clear cache on SOI transition
            hasCachedData = false;
            apsisDataCache.Clear();
            lastReferenceFrame = newFrame;
        }

        private void LateUpdate()
        {
            if (universeManager == null || apsisMarkerSystem == null)
            {
                ResolveReferences();
                if (universeManager == null || apsisMarkerSystem == null)
                {
                    return;
                }
            }

            // Detect SOI transition
            ReferenceFrameTarget currentFrame = universeManager.ActiveReferenceFrame;
            if (currentFrame != lastReferenceFrame)
            {
                hasCachedData = false;
                apsisDataCache.Clear();
                lastReferenceFrame = currentFrame;
            }

            // Check if recalculation is needed
            if (!ShouldRecalculate())
            {
                // Reuse cached data
                apsisMarkerSystem.UpdateApsisMarkers(apsisDataCache);
                return;
            }

            // Calculate ballistic apsides (current orbit, no maneuvers)
            var ballisticApsides = CalculateBallisticApsides();

            // Calculate maneuver apsides (post-maneuver orbits)
            var maneuverApsides = CalculateManeuverApsides();

            // Combine and cache
            apsisDataCache.Clear();
            apsisDataCache.AddRange(ballisticApsides);
            
            // Filter out maneuver apsides that are too close to ballistic apsides
            FilterDuplicateApsides(ballisticApsides, maneuverApsides);
            
            apsisDataCache.AddRange(maneuverApsides);

            // Update cache state
            cachedShipPosition = universeManager.ShipBody.Position;
            cachedShipVelocity = universeManager.ShipBody.Velocity;
            cachedSimulationTime = universeManager.SimulationTimeSeconds;
            hasCachedData = true;
            
            // Send to marker system
            apsisMarkerSystem.UpdateApsisMarkers(apsisDataCache);
        }

        /// <summary>
        /// Filters out maneuver apsides that are too close to ballistic apsides.
        /// This prevents overlapping markers when maneuvers don't significantly change the orbit.
        /// </summary>
        private void FilterDuplicateApsides(List<ApsisData> ballisticApsides, List<ApsisData> maneuverApsides)
        {
            const double TIME_THRESHOLD = 10.0; // seconds
            const double POSITION_THRESHOLD = 1000.0; // meters

            for (int i = maneuverApsides.Count - 1; i >= 0; i--)
            {
                var maneuverApsis = maneuverApsides[i];
                
                foreach (var ballisticApsis in ballisticApsides)
                {
                    // Only compare same type (Pe with Pe, Ap with Ap)
                    if (maneuverApsis.type != ballisticApsis.type)
                        continue;

                    double timeDiff = Math.Abs(maneuverApsis.timeToReach - ballisticApsis.timeToReach);
                    double positionDiff = (maneuverApsis.worldPosition - ballisticApsis.worldPosition).Magnitude;

                    // If maneuver apsis is very close to ballistic apsis, hide it
                    if (timeDiff < TIME_THRESHOLD && positionDiff < POSITION_THRESHOLD)
                    {
                        maneuverApsides.RemoveAt(i);
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Determines if apsis recalculation is needed based on state changes.
        /// </summary>
        private bool ShouldRecalculate()
        {
            if (!hasCachedData)
                return true;

            Vector3d currentPos = universeManager.ShipBody.Position;
            Vector3d currentVel = universeManager.ShipBody.Velocity;
            double currentTime = universeManager.SimulationTimeSeconds;

            // Check position change
            double positionChange = (currentPos - cachedShipPosition).Magnitude;
            if (positionChange > positionChangeThreshold)
                return true;

            // Check velocity change
            double velocityChange = (currentVel - cachedShipVelocity).Magnitude;
            if (velocityChange > velocityChangeThreshold)
                return true;

            // Check time change
            double timeChange = Math.Abs(currentTime - cachedSimulationTime);
            if (timeChange > timeChangeThreshold)
                return true;

            return false;
        }

        /// <summary>
        /// Calculates apsides for the ballistic orbit (no maneuvers).
        /// </summary>
        private List<ApsisData> CalculateBallisticApsides()
        {
            var result = new List<ApsisData>();

            try
            {
                // Get ship state in simulation frame
                Vector3d simPos = universeManager.ShipBody.Position;
                Vector3d simVel = universeManager.ShipBody.Velocity;

                // Validate state vectors
                if (!simPos.IsFinite || !simVel.IsFinite)
                {
                    Debug.LogWarning("[ApsisCalculator] Invalid ship state vectors (NaN or Infinity), skipping ballistic apsis calculation");
                    return result;
                }

                // Get central body parameters
                double mu = universeManager.GetCurrentCentralBodyMu();
                double radius = universeManager.GetCurrentCentralBodyRadius();
                string bodyName = universeManager.ActiveReferenceFrame.ToString();

                // Transform to astrodynamic frame (Z-up)
                Vector3d astroPos = universeManager.ConvertSimulationToAstrodynamicFrame(simPos);
                Vector3d astroVel = universeManager.ConvertSimulationToAstrodynamicFrame(simVel);

                // Calculate orbital elements
                var elements = OrbitalElements.FromState(astroPos, astroVel, mu);

                if (!elements.IsValid)
                {
                    Debug.LogWarning("[ApsisCalculator] Invalid orbital elements, skipping ballistic apsis calculation");
                    return result;
                }

                // Check for circular orbit (no distinct apsides)
                if (elements.Eccentricity < circularOrbitThreshold)
                {
                    // Circular orbit - no markers
                    return result;
                }

                // Get apsis positions in astrodynamic frame
                if (!elements.TryGetApsisPositions(out Vector3d astroPe, out Vector3d astroAp))
                {
                    Debug.LogWarning("[ApsisCalculator] Failed to calculate apsis positions");
                    return result;
                }

                // Transform back to simulation frame
                Vector3d simPe = universeManager.ConvertAstrodynamicToSimulationFrame(astroPe);
                Vector3d simAp = universeManager.ConvertAstrodynamicToSimulationFrame(astroAp);

                // Calculate time to apsides
                double currentTime = universeManager.SimulationTimeSeconds;
                if (!elements.TryGetTimeToApsides(mu, out double timeToPe, out double timeToAp))
                {
                    Debug.LogWarning("[ApsisCalculator] Failed to calculate time to apsides");
                    return result;
                }

                // Calculate altitudes
                double altPe = simPe.Magnitude - radius;
                double altAp = simAp.Magnitude - radius;

                // Create periapsis data
                var peData = new ApsisData(
                    worldPosition: simPe,
                    altitude: altPe,
                    timeToReach: currentTime + timeToPe,
                    type: ApsisType.Periapsis,
                    orbitType: OrbitType.Ballistic,
                    segmentIndex: -1,
                    isVisible: true,
                    centralBodyName: bodyName
                );

                // Apply visibility rules
                ApplyVisibilityRules(ref peData);
                result.Add(peData);

                // Create apoapsis data (only for elliptical orbits)
                if (elements.Eccentricity < 1.0)
                {
                    var apData = new ApsisData(
                        worldPosition: simAp,
                        altitude: altAp,
                        timeToReach: currentTime + timeToAp,
                        type: ApsisType.Apoapsis,
                        orbitType: OrbitType.Ballistic,
                        segmentIndex: -1,
                        isVisible: true,
                        centralBodyName: bodyName
                    );

                    ApplyVisibilityRules(ref apData);
                    result.Add(apData);
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[ApsisCalculator] Exception in CalculateBallisticApsides: {ex.Message}\n{ex.StackTrace}");
            }

            return result;
        }

        /// <summary>
        /// Calculates apsides for all maneuver segments.
        /// </summary>
        private List<ApsisData> CalculateManeuverApsides()
        {
            var result = new List<ApsisData>();

            if (maneuverEvaluator == null)
                return result;

            try
            {
                // Get node count from ManeuverEvaluator's cached boundaries
                // Each boundary represents a segment after a maneuver node
                int nodeCount = 0;
                
                // Try to get segment count by checking boundaries
                for (int i = 0; i < 100; i++) // Reasonable upper limit
                {
                    if (maneuverEvaluator.TryGetSegmentBoundaryState(i, out _))
                        nodeCount = i + 1;
                    else
                        break;
                }
                
                if (nodeCount == 0)
                    return result;

                // Get central body parameters
                double mu = universeManager.GetCurrentCentralBodyMu();
                double radius = universeManager.GetCurrentCentralBodyRadius();
                string bodyName = universeManager.ActiveReferenceFrame.ToString();

                // Process each segment
                for (int i = 0; i < nodeCount; i++)
                {
                    // Get segment boundary state (post-Δv)
                    if (!maneuverEvaluator.TryGetSegmentBoundaryState(i, out var boundary))
                        continue;

                    // Convert Unity.Mathematics.double3 to Vector3d
                    Vector3d simPos = new Vector3d(boundary.Position.x, boundary.Position.y, boundary.Position.z);
                    Vector3d simVel = new Vector3d(boundary.Velocity.x, boundary.Velocity.y, boundary.Velocity.z);
                    double segmentStartTime = boundary.Time;

                    // Validate state vectors
                    if (!simPos.IsFinite || !simVel.IsFinite)
                        continue;

                    // Transform to astrodynamic frame
                    Vector3d astroPos = universeManager.ConvertSimulationToAstrodynamicFrame(simPos);
                    Vector3d astroVel = universeManager.ConvertSimulationToAstrodynamicFrame(simVel);

                    // Calculate orbital elements
                    var elements = OrbitalElements.FromState(astroPos, astroVel, mu);

                    if (!elements.IsValid)
                        continue;

                    // Check for circular orbit
                    if (elements.Eccentricity < circularOrbitThreshold)
                        continue;

                    // Get apsis positions
                    if (!elements.TryGetApsisPositions(out Vector3d astroPe, out Vector3d astroAp))
                        continue;

                    // Transform back to simulation frame
                    Vector3d simPe = universeManager.ConvertAstrodynamicToSimulationFrame(astroPe);
                    Vector3d simAp = universeManager.ConvertAstrodynamicToSimulationFrame(astroAp);

                    // Calculate time to apsides (relative to segment start)
                    if (!elements.TryGetTimeToApsides(mu, out double timeToPe, out double timeToAp))
                        continue;

                    // Convert to absolute time
                    double absPeTime = segmentStartTime + timeToPe;
                    double absApTime = segmentStartTime + timeToAp;

                    // Calculate altitudes
                    double altPe = simPe.Magnitude - radius;
                    double altAp = simAp.Magnitude - radius;

                    // Create periapsis data
                    var peData = new ApsisData(
                        worldPosition: simPe,
                        altitude: altPe,
                        timeToReach: absPeTime,
                        type: ApsisType.Periapsis,
                        orbitType: OrbitType.Maneuver,
                        segmentIndex: i,
                        isVisible: true,
                        centralBodyName: bodyName
                    );

                    ApplyVisibilityRules(ref peData);
                    result.Add(peData);

                    // Create apoapsis data (only for elliptical orbits)
                    if (elements.Eccentricity < 1.0)
                    {
                        var apData = new ApsisData(
                            worldPosition: simAp,
                            altitude: altAp,
                            timeToReach: absApTime,
                            type: ApsisType.Apoapsis,
                            orbitType: OrbitType.Maneuver,
                            segmentIndex: i,
                            isVisible: true,
                            centralBodyName: bodyName
                        );

                        ApplyVisibilityRules(ref apData);
                        result.Add(apData);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[ApsisCalculator] Exception in CalculateManeuverApsides: {ex.Message}\n{ex.StackTrace}");
            }

            return result;
        }

        /// <summary>
        /// Applies visibility rules to an apsis data structure.
        /// Modifies the isVisible flag based on configured rules.
        /// </summary>
        private void ApplyVisibilityRules(ref ApsisData apsisData)
        {
            double currentTime = universeManager.SimulationTimeSeconds;
            double timeUntilApsis = apsisData.timeToReach - currentTime;

            // Rule 1: Only future apsides
            if (showOnlyFutureApsides && apsisData.timeToReach <= currentTime)
            {
                apsisData.isVisible = false;
                return;
            }

            // Rule 2: Not below surface
            if (!showBelowSurface && apsisData.altitude < minAltitude)
            {
                apsisData.isVisible = false;
                return;
            }

            // Rule 3: Maximum prediction time
            // Use dynamic trajectory time if available, otherwise fall back to configured max
            double effectiveMaxPrediction = maxPredictionTime;
            
            // Use TrajectoryPreviewEndTime from UniverseManager if available
            double trajectoryEndTime = universeManager.TrajectoryPreviewEndTime;
            if (trajectoryEndTime > currentTime)
            {
                // Use actual trajectory end time
                effectiveMaxPrediction = trajectoryEndTime - currentTime;
            }

            if (timeUntilApsis > effectiveMaxPrediction)
            {
                apsisData.isVisible = false;
                return;
            }

            // All rules passed - apsis is visible
            apsisData.isVisible = true;
        }

        #region Public API for Testing

        /// <summary>
        /// Gets the current cached apsis data for testing purposes.
        /// </summary>
        public IReadOnlyList<ApsisData> GetCachedApsisData()
        {
            return apsisDataCache.AsReadOnly();
        }

        /// <summary>
        /// Forces recalculation of apsides, bypassing cache.
        /// </summary>
        public void ForceRecalculation()
        {
            hasCachedData = false;
        }

        #endregion
    }
}

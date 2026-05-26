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
        /// OPTIMIZED: Uses spatial hashing to reduce O(n²) to O(n) complexity.
        /// </summary>
        private void FilterDuplicateApsides(List<ApsisData> ballisticApsides, List<ApsisData> maneuverApsides)
        {
            const double TIME_THRESHOLD = 10.0; // seconds
            const double POSITION_THRESHOLD = 1000.0; // meters
            const double TIME_BUCKET = 5.0; // seconds - bucket size for spatial hash
            const double POS_BUCKET = 500.0; // meters - bucket size for spatial hash

            // Early exit if no ballistic apsides to compare against
            if (ballisticApsides.Count == 0)
                return;

            // Build spatial hash for ballistic apsides
            // Key: spatial hash, Value: list of indices in ballisticApsides
            Dictionary<long, List<int>> spatialHash = new Dictionary<long, List<int>>();
            
            for (int i = 0; i < ballisticApsides.Count; i++)
            {
                long hash = ComputeSpatialHash(ballisticApsides[i], TIME_BUCKET, POS_BUCKET);
                if (!spatialHash.ContainsKey(hash))
                    spatialHash[hash] = new List<int>();
                spatialHash[hash].Add(i);
            }

            // Check maneuver apsides against nearby ballistic apsides only
            for (int i = maneuverApsides.Count - 1; i >= 0; i--)
            {
                var maneuverApsis = maneuverApsides[i];
                long baseHash = ComputeSpatialHash(maneuverApsis, TIME_BUCKET, POS_BUCKET);
                
                bool isDuplicate = false;
                
                // Check current bucket and 26 neighboring buckets (3x3x3 - 1)
                for (int dt = -1; dt <= 1 && !isDuplicate; dt++)
                {
                    for (int dx = -1; dx <= 1 && !isDuplicate; dx++)
                    {
                        for (int dy = -1; dy <= 1 && !isDuplicate; dy++)
                        {
                            // Compute neighbor hash
                            long neighborHash = baseHash + dt + ((long)dx << 20) + ((long)dy << 40);
                            
                            if (spatialHash.TryGetValue(neighborHash, out var indices))
                            {
                                foreach (int idx in indices)
                                {
                                    var ballisticApsis = ballisticApsides[idx];
                                    
                                    // Only compare same type (Pe with Pe, Ap with Ap)
                                    if (maneuverApsis.type != ballisticApsis.type)
                                        continue;

                                    double timeDiff = Math.Abs(maneuverApsis.timeToReach - ballisticApsis.timeToReach);
                                    double posDiff = (maneuverApsis.worldPosition - ballisticApsis.worldPosition).Magnitude;

                                    // If maneuver apsis is very close to ballistic apsis, mark as duplicate
                                    if (timeDiff < TIME_THRESHOLD && posDiff < POSITION_THRESHOLD)
                                    {
                                        isDuplicate = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                
                if (isDuplicate)
                    maneuverApsides.RemoveAt(i);
            }
        }

        /// <summary>
        /// Computes spatial hash for an apsis based on time and position.
        /// Uses 3D bucketing: time, x-position, y-position.
        /// </summary>
        private long ComputeSpatialHash(ApsisData apsis, double timeBucket, double posBucket)
        {
            int timeHash = (int)(apsis.timeToReach / timeBucket);
            int xHash = (int)(apsis.worldPosition.X / posBucket);
            int yHash = (int)(apsis.worldPosition.Y / posBucket);
            
            // Combine hashes using bit shifting to avoid collisions
            // timeHash in lower 20 bits, xHash in middle 20 bits, yHash in upper 20 bits
            return timeHash + ((long)xHash << 20) + ((long)yHash << 40);
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
        /// OPTIMIZED: Uses Burst-compiled parallel jobs for batch processing.
        /// </summary>
        private List<ApsisData> CalculateManeuverApsides()
        {
            var result = new List<ApsisData>();

            if (maneuverEvaluator == null)
                return result;

            try
            {
                // Get node count from ManeuverEvaluator's cached boundaries
                int nodeCount = 0;
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

                // Allocate native arrays for batch processing
                Unity.Collections.NativeArray<Unity.Mathematics.double3> positions = 
                    new Unity.Collections.NativeArray<Unity.Mathematics.double3>(nodeCount, Unity.Collections.Allocator.TempJob);
                Unity.Collections.NativeArray<Unity.Mathematics.double3> velocities = 
                    new Unity.Collections.NativeArray<Unity.Mathematics.double3>(nodeCount, Unity.Collections.Allocator.TempJob);
                Unity.Collections.NativeArray<double> mus = 
                    new Unity.Collections.NativeArray<double>(nodeCount, Unity.Collections.Allocator.TempJob);
                Unity.Collections.NativeArray<Core.OrbitalElementsData> elements = 
                    new Unity.Collections.NativeArray<Core.OrbitalElementsData>(nodeCount, Unity.Collections.Allocator.TempJob);
                Unity.Collections.NativeArray<double> segmentTimes = 
                    new Unity.Collections.NativeArray<double>(nodeCount, Unity.Collections.Allocator.TempJob);
                Unity.Collections.NativeArray<Simulation.ApsisResultPair> apsisResults = 
                    new Unity.Collections.NativeArray<Simulation.ApsisResultPair>(nodeCount, Unity.Collections.Allocator.TempJob);

                // Fill input arrays
                int validCount = 0;
                for (int i = 0; i < nodeCount; i++)
                {
                    if (!maneuverEvaluator.TryGetSegmentBoundaryState(i, out var boundary))
                        continue;

                    Vector3d simPos = new Vector3d(boundary.Position.x, boundary.Position.y, boundary.Position.z);
                    Vector3d simVel = new Vector3d(boundary.Velocity.x, boundary.Velocity.y, boundary.Velocity.z);

                    if (!simPos.IsFinite || !simVel.IsFinite)
                        continue;

                    Vector3d astroPos = universeManager.ConvertSimulationToAstrodynamicFrame(simPos);
                    Vector3d astroVel = universeManager.ConvertSimulationToAstrodynamicFrame(simVel);

                    positions[validCount] = new Unity.Mathematics.double3(astroPos.X, astroPos.Y, astroPos.Z);
                    velocities[validCount] = new Unity.Mathematics.double3(astroVel.X, astroVel.Y, astroVel.Z);
                    mus[validCount] = mu;
                    segmentTimes[validCount] = boundary.Time;
                    validCount++;
                }

                if (validCount > 0)
                {
                    // Step 1: Calculate orbital elements in parallel
                    var elementsJobHandle = Core.OrbitalElements.CalculateBatch(
                        positions, velocities, mus, elements);
                    
                    // Step 2: Calculate apsides in parallel (depends on elements)
                    var apsisJob = new Simulation.ApsisCalculationJob
                    {
                        Elements = elements,
                        SegmentStartTimes = segmentTimes,
                        Mu = mu,
                        CentralBodyRadius = radius,
                        CircularOrbitThreshold = circularOrbitThreshold,
                        Results = apsisResults
                    };
                    
                    int batchSize = Unity.Mathematics.math.max(1, validCount / (UnityEngine.SystemInfo.processorCount - 2));
                    var apsisJobHandle = Unity.Jobs.IJobParallelForExtensions.Schedule(apsisJob, validCount, batchSize, elementsJobHandle);
                    
                    // Wait for completion
                    apsisJobHandle.Complete();

                    // Process results
                    for (int i = 0; i < validCount; i++)
                    {
                        var apsisResult = apsisResults[i];

                        // Add periapsis if valid
                        if (apsisResult.PeValid != 0)
                        {
                            Vector3d simPe = universeManager.ConvertAstrodynamicToSimulationFrame(
                                new Vector3d(apsisResult.PePosition.x, apsisResult.PePosition.y, apsisResult.PePosition.z));

                            var peData = new ApsisData(
                                worldPosition: simPe,
                                altitude: apsisResult.PeAltitude,
                                timeToReach: apsisResult.PeTime,
                                type: ApsisType.Periapsis,
                                orbitType: OrbitType.Maneuver,
                                segmentIndex: i,
                                isVisible: true,
                                centralBodyName: bodyName
                            );

                            ApplyVisibilityRules(ref peData);
                            result.Add(peData);
                        }

                        // Add apoapsis if valid
                        if (apsisResult.ApValid != 0)
                        {
                            Vector3d simAp = universeManager.ConvertAstrodynamicToSimulationFrame(
                                new Vector3d(apsisResult.ApPosition.x, apsisResult.ApPosition.y, apsisResult.ApPosition.z));

                            var apData = new ApsisData(
                                worldPosition: simAp,
                                altitude: apsisResult.ApAltitude,
                                timeToReach: apsisResult.ApTime,
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

                // Cleanup native arrays
                positions.Dispose();
                velocities.Dispose();
                mus.Dispose();
                elements.Dispose();
                segmentTimes.Dispose();
                apsisResults.Dispose();
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

using System;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Type of orbit for the apsis calculation.
    /// </summary>
    public enum OrbitType
    {
        /// <summary>Natural trajectory under gravity with no maneuvers (green markers).</summary>
        Ballistic,
        
        /// <summary>Trajectory after applying a planned maneuver (purple markers).</summary>
        Maneuver
    }

    /// <summary>
    /// Data structure containing all information about an orbital apsis point.
    /// Used by the analytical apsis calculation system to pass data from ApsisCalculator to ApsisMarkerSystem.
    /// </summary>
    [Serializable]
    public struct ApsisData
    {
        /// <summary>
        /// Position of the apsis in real-world coordinates (meters from central body center).
        /// This is in the simulation frame, not Unity coordinates.
        /// Transform to Unity coordinates using UniverseManager.ToUnityPosition().
        /// </summary>
        public Vector3d worldPosition;

        /// <summary>
        /// Altitude above the surface of the central body in meters.
        /// Negative values indicate subsurface collision.
        /// </summary>
        public double altitude;

        /// <summary>
        /// Absolute simulation time when the spacecraft will reach this apsis (seconds).
        /// Compare with UniverseManager.SimulationTimeSeconds to get time remaining.
        /// </summary>
        public double timeToReach;

        /// <summary>
        /// Type of apsis point (Periapsis or Apoapsis).
        /// </summary>
        public ApsisType type;

        /// <summary>
        /// Type of orbit (Ballistic or Maneuver).
        /// Determines marker color: green for ballistic, purple for maneuver.
        /// </summary>
        public OrbitType orbitType;

        /// <summary>
        /// Index of the maneuver segment this apsis belongs to.
        /// -1 for ballistic orbit (no maneuvers).
        /// 0+ for maneuver segments (0 = after first maneuver, 1 = after second maneuver, etc.).
        /// </summary>
        public int segmentIndex;

        /// <summary>
        /// Whether this apsis marker should be visible on screen.
        /// Set to false by visibility rules (e.g., past apsides, subsurface, too far in future).
        /// </summary>
        public bool isVisible;

        /// <summary>
        /// Name of the central body this apsis is relative to.
        /// Examples: "Jupiter", "Io", "Europa", "Ganymede", "Callisto".
        /// </summary>
        public string centralBodyName;

        /// <summary>
        /// Creates a new ApsisData structure with all required fields.
        /// </summary>
        public ApsisData(
            Vector3d worldPosition,
            double altitude,
            double timeToReach,
            ApsisType type,
            OrbitType orbitType,
            int segmentIndex,
            bool isVisible,
            string centralBodyName)
        {
            this.worldPosition = worldPosition;
            this.altitude = altitude;
            this.timeToReach = timeToReach;
            this.type = type;
            this.orbitType = orbitType;
            this.segmentIndex = segmentIndex;
            this.isVisible = isVisible;
            this.centralBodyName = centralBodyName;
        }

        /// <summary>
        /// Converts this ApsisData to the legacy ApsisMarkerData format for backward compatibility.
        /// </summary>
        /// <param name="universeManager">UniverseManager for coordinate conversion.</param>
        /// <param name="currentTime">Current simulation time for calculating time remaining.</param>
        /// <returns>ApsisMarkerData structure for use with existing marker system.</returns>
        public ApsisMarkerData ToMarkerData(UniverseManager universeManager, double currentTime)
        {
            // Convert world position to Unity coordinates
            Vector3 unityPosition = universeManager.ToUnityPosition(worldPosition);

            // Calculate time remaining
            double timeRemaining = timeToReach - currentTime;

            // Format altitude
            string altitudeFormatted = FormatAltitude(altitude);

            // Format time
            string timeFormatted = FormatTime(timeRemaining);

            // Determine color based on orbit type
            Color color = orbitType == OrbitType.Ballistic 
                ? new Color(0.0f, 1.0f, 0.0f, 1.0f)  // Green for ballistic
                : new Color(0.6f, 0.0f, 1.0f, 1.0f); // Purple for maneuver

            // Determine label
            string label = type == ApsisType.Periapsis ? "Pe" : "Ap";

            // Determine edge case
            ApsisEdgeCase edgeCase = ApsisEdgeCase.None;
            if (altitude < 0)
                edgeCase = ApsisEdgeCase.Impact;

            return new ApsisMarkerData
            {
                worldPosition = unityPosition,
                type = type,
                label = label,
                frameName = centralBodyName,
                isManeuver = orbitType == OrbitType.Maneuver,
                isValid = true,
                isVisible = isVisible,
                altitudeMeters = altitude,
                timeToApsisSeconds = timeRemaining,
                altitudeFormatted = altitudeFormatted,
                timeFormatted = timeFormatted,
                edgeCase = edgeCase,
                color = color
            };
        }

        /// <summary>
        /// Formats altitude in human-readable form.
        /// </summary>
        private static string FormatAltitude(double altitudeMeters)
        {
            if (altitudeMeters < 0)
                return $"{altitudeMeters / 1000:F1} km (subsurface)";
            else if (altitudeMeters < 1000)
                return $"{altitudeMeters:F0} m";
            else if (altitudeMeters < 1000000)
                return $"{altitudeMeters / 1000:F1} km";
            else
                return $"{altitudeMeters / 1000000:F2} Mm";
        }

        /// <summary>
        /// Formats time in human-readable form.
        /// </summary>
        private static string FormatTime(double timeSeconds)
        {
            if (timeSeconds < 0)
                return "Past";
            else if (timeSeconds < 60)
                return $"{timeSeconds:F0}s";
            else if (timeSeconds < 3600)
                return $"{timeSeconds / 60:F1}m";
            else if (timeSeconds < 86400)
                return $"{timeSeconds / 3600:F1}h";
            else
                return $"{timeSeconds / 86400:F1}d";
        }

        public override string ToString()
        {
            return $"{type} ({orbitType}): alt={altitude / 1000:F1}km, t={timeToReach:F1}s, visible={isVisible}, body={centralBodyName}";
        }
    }
}

using UnityEngine;

namespace Galilego.Gameplay
{
    public enum ApsisType { Periapsis, Apoapsis }

    public enum ApsisEdgeCase
    {
        None,
        Impact,
        BeyondSOI,
        Now,
        OverOneYear,
        Circular
    }

    [System.Serializable]
    public struct ApsisMarkerData
    {
        public Vector3 worldPosition;
        public ApsisType type;
        public string label;
        public string frameName;
        public bool isManeuver;
        public bool isValid;
        public bool isVisible;
        public double altitudeMeters;
        public double timeToApsisSeconds;
        public string altitudeFormatted;
        public string timeFormatted;
        public ApsisEdgeCase edgeCase;
        public Color color;
    }
}

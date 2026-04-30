using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Galilego.Physics
{
    public sealed class SpeedometerUI : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private TMP_Text frameLabel;
        [SerializeField] private TMP_Text speedLabel;
        [SerializeField] private TMP_Text radialSpeedLabel;
        [SerializeField] private TMP_Text tangentialSpeedLabel;
        [SerializeField] private TMP_Text altitudeLabel;
        [SerializeField] private TMP_Text distanceLabel;
        [SerializeField] private TMP_Text periapsisLabel;
        [SerializeField] private TMP_Text apoapsisLabel;
        [SerializeField] private Image speedFill;
        [SerializeField] private RectTransform speedNeedle;

        [Header("Scale")]
        [SerializeField] private double fullScaleSpeedMetersPerSecond = 20000d;
        [SerializeField] private float needleMinAngle = 130f;
        [SerializeField] private float needleMaxAngle = -130f;

        private void Update()
        {
            ResolveReferences();

            if (universeManager == null)
            {
                return;
            }

            ReferenceFrameTarget activeFrame = universeManager.ActiveReferenceFrame;
            if (!universeManager.TryGetShipRelativeState(
                activeFrame,
                out string frameName,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out _,
                out double frameRadius,
                out _))
            {
                SetUnavailable();
                return;
            }

            double distance = relativePosition.Magnitude;
            double speed = relativeVelocity.Magnitude;
            double radialSpeed = distance > 0d ? Vector3d.Dot(relativePosition, relativeVelocity) / distance : 0d;
            double tangentialSpeed = Math.Sqrt(Math.Max(0d, relativeVelocity.SqrMagnitude - (radialSpeed * radialSpeed)));
            double altitude = distance - frameRadius;
            OrbitalElements orbit = universeManager.GetShipOrbitAround(activeFrame);

            SetText(frameLabel, frameName);
            SetText(speedLabel, FormatSpeed(speed));
            SetText(radialSpeedLabel, FormatSignedSpeed(radialSpeed));
            SetText(tangentialSpeedLabel, FormatSpeed(tangentialSpeed));
            SetText(altitudeLabel, FormatDistance(altitude));
            SetText(distanceLabel, FormatDistance(distance));
            SetText(periapsisLabel, orbit.IsValid ? FormatDistance(orbit.PeriapsisDistance - frameRadius) : "n/a");
            SetText(apoapsisLabel, orbit.IsValid && !double.IsInfinity(orbit.ApoapsisDistance) ? FormatDistance(orbit.ApoapsisDistance - frameRadius) : "open");

            double fullScale = Math.Max(1d, fullScaleSpeedMetersPerSecond);
            float normalizedSpeed = Mathf.Clamp01((float)(speed / fullScale));

            if (speedFill != null)
            {
                speedFill.fillAmount = normalizedSpeed;
            }

            if (speedNeedle != null)
            {
                float angle = Mathf.Lerp(needleMinAngle, needleMaxAngle, normalizedSpeed);
                speedNeedle.localRotation = Quaternion.Euler(0f, 0f, angle);
            }
        }

        private void SetUnavailable()
        {
            SetText(frameLabel, "n/a");
            SetText(speedLabel, "n/a");
            SetText(radialSpeedLabel, "n/a");
            SetText(tangentialSpeedLabel, "n/a");
            SetText(altitudeLabel, "n/a");
            SetText(distanceLabel, "n/a");
            SetText(periapsisLabel, "n/a");
            SetText(apoapsisLabel, "n/a");
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
            {
                universeManager = FindAnyObjectByType<UniverseManager>();
            }
        }

        private static void SetText(TMP_Text text, string value)
        {
            if (text != null)
            {
                text.text = value;
            }
        }

        private static string FormatSignedSpeed(double metersPerSecond)
        {
            return metersPerSecond >= 0d
                ? $"+{FormatSpeed(metersPerSecond)}"
                : FormatSpeed(metersPerSecond);
        }

        private static string FormatSpeed(double metersPerSecond)
        {
            if (double.IsNaN(metersPerSecond))
            {
                return "n/a";
            }

            double absolute = Math.Abs(metersPerSecond);
            if (absolute >= 1000d)
            {
                return $"{metersPerSecond / 1000d:0.###} km/s";
            }

            return $"{metersPerSecond:0.###} m/s";
        }

        private static string FormatDistance(double meters)
        {
            if (double.IsInfinity(meters))
            {
                return "open";
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
    }
}

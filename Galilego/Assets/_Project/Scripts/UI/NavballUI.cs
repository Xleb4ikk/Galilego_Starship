using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Galilego.Physics
{
    public sealed class NavballUI : MonoBehaviour
    {
        public enum NavballDirectionMode
        {
            Prograde = 0,
            Retrograde = 1,
            RadialOut = 2,
            RadialIn = 3,
            Normal = 4,
            AntiNormal = 5,
            North = 6,
            East = 7,
            South = 8,
            West = 9,
            SelectedTarget = 10,
            AntiSelectedTarget = 11,
            Body = 12,
            AntiBody = 13
        }

        [Serializable]
        private sealed class DirectionalMarker
        {
            [SerializeField] private RectTransform marker;
            [SerializeField] private TMP_Text label;
            [SerializeField] private NavballDirectionMode direction = NavballDirectionMode.SelectedTarget;
            [SerializeField] private ReferenceFrameTarget bodyTarget = ReferenceFrameTarget.Jupiter;
            [SerializeField] private string labelOverride;

            public RectTransform Marker => marker != null ? marker : label != null ? label.rectTransform : null;
            public TMP_Text Label => label;
            public NavballDirectionMode Direction => direction;
            public ReferenceFrameTarget BodyTarget => bodyTarget;
            public string LabelOverride => labelOverride;
        }

        private sealed class AutopilotButtonBinding
        {
            public Button Button;
            public Image Background;
            public NavballDirectionMode Direction;
            public bool IsStopButton;
        }

        private const int AutopilotButtonCount = 13;

        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private Transform shipOrientation;
        [SerializeField] private RectTransform ball;
        [SerializeField] private RawImage ballImage;
        [SerializeField] private NavballSphereGraphic ballSphere;
        [SerializeField] private NavballFrameGraphic frameGraphic;
        [SerializeField] private TMP_Text frameLabel;

        [Header("Markers")]
        [SerializeField] private RectTransform progradeMarker;
        [SerializeField] private RectTransform retrogradeMarker;
        [SerializeField] private RectTransform radialOutMarker;
        [SerializeField] private RectTransform radialInMarker;
        [SerializeField] private RectTransform normalMarker;
        [SerializeField] private RectTransform antiNormalMarker;
        [SerializeField] private RectTransform northMarker;
        [SerializeField] private RectTransform eastMarker;
        [SerializeField] private RectTransform southMarker;
        [SerializeField] private RectTransform westMarker;
        [SerializeField] private RectTransform selectedTargetMarker;
        [SerializeField] private RectTransform antiSelectedTargetMarker;
        [SerializeField] private TMP_Text selectedTargetLabel;
        [SerializeField] private TMP_Text antiSelectedTargetLabel;
        [SerializeField] private DirectionalMarker[] dynamicMarkers = Array.Empty<DirectionalMarker>();

        [Header("Autopilot")]
        [SerializeField] private NavballAutopilotController autopilot;
        [SerializeField] private bool createAutopilotButtons = true;
        [SerializeField] private RectTransform autopilotButtonsRoot;
        [SerializeField] private Vector2 autopilotButtonsOffset = new Vector2(0f, -142f);
        [SerializeField] private Vector2 autopilotButtonSize = new Vector2(44f, 24f);
        [SerializeField] private int autopilotButtonColumns = 5;
        [SerializeField] private Color autopilotButtonColor = new Color(0.08f, 0.09f, 0.1f, 0.88f);
        [SerializeField] private Color autopilotActiveButtonColor = new Color(0.1f, 0.58f, 0.72f, 0.95f);
        [SerializeField] private Color autopilotStopButtonColor = new Color(0.28f, 0.08f, 0.08f, 0.9f);

        [Header("Display")]
        [SerializeField] private float markerRadius = 82f;
        [SerializeField] private bool hideBacksideMarkers = true;
        [SerializeField] private float backsideScale = 0.65f;
        [SerializeField] private float minimumDirectionMagnitude = 1e-4f;
        [SerializeField] private bool driveBallTexture = true;
        [SerializeField] private bool alignToOrbitalFrame;
        [SerializeField] private bool stabilizeBallRollAgainstOrbit = true;
        [SerializeField] private Vector3 shipLocalForward = Vector3.up;
        [SerializeField] private Vector3 shipLocalUp = Vector3.forward;
        [SerializeField] private bool flipBallVertical = true;
        [SerializeField] private Vector2 ballUvSize = new Vector2(0.42f, 0.42f);

        private readonly List<AutopilotButtonBinding> autopilotButtonBindings = new List<AutopilotButtonBinding>();
        private bool autopilotButtonsCreated;

        private void LateUpdate()
        {
            ResolveReferences();
            EnsureAutopilotButtons();
            RefreshAutopilotButtons();

            if (universeManager == null || shipOrientation == null)
            {
                HideAllMarkers();
                return;
            }

            ReferenceFrameTarget activeFrame = universeManager.ActiveReferenceFrame;
            if (!universeManager.TryGetShipRelativeState(
                activeFrame,
                out string frameName,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out _,
                out _,
                out _))
            {
                HideAllMarkers();
                return;
            }

            if (frameLabel != null)
            {
                frameLabel.text = frameName;
            }

            Vector3 radialOut = ToUnityDirection(relativePosition);
            Vector3 velocity = ToUnityDirection(relativeVelocity);
            Vector3 normal = ToUnityDirection(Vector3d.Cross(relativePosition, relativeVelocity));

            Vector3 localNorth = ResolveLocalNorth(radialOut);
            Vector3 localEast = ResolveLocalEast(radialOut, localNorth);
            Quaternion displayRotation = ResolveDisplayRotation(radialOut, velocity, localNorth, localEast);

            ResolveBallReferenceBasis(radialOut, localNorth, out Vector3 ballRadialOut, out Vector3 ballLocalNorth, out Vector3 ballLocalEast);
            if (flipBallVertical)
            {
                ballRadialOut = -ballRadialOut;
                ballLocalEast = -ballLocalEast;
            }

            UpdateBall(ballRadialOut, ballLocalNorth, ballLocalEast, displayRotation);

            SetMarker(progradeMarker, velocity, displayRotation);
            SetMarker(retrogradeMarker, -velocity, displayRotation);
            SetMarker(radialOutMarker, radialOut, displayRotation);
            SetMarker(radialInMarker, -radialOut, displayRotation);
            SetMarker(normalMarker, normal, displayRotation);
            SetMarker(antiNormalMarker, -normal, displayRotation);
            SetMarker(northMarker, localNorth, displayRotation);
            SetMarker(southMarker, -localNorth, displayRotation);
            SetMarker(eastMarker, localEast, displayRotation);
            SetMarker(westMarker, -localEast, displayRotation);
            SetMarkerWithLabel(selectedTargetMarker, selectedTargetLabel, -radialOut, displayRotation, frameName);
            SetMarkerWithLabel(antiSelectedTargetMarker, antiSelectedTargetLabel, radialOut, displayRotation, $"Anti {frameName}");
            UpdateDynamicMarkers(
                frameName,
                radialOut,
                velocity,
                normal,
                localNorth,
                localEast,
                displayRotation);

            if (ball != null && ballSphere == null)
            {
                Vector3 localNorthOnBall = Quaternion.Inverse(displayRotation) * localNorth;
                float northAngle = Mathf.Atan2(localNorthOnBall.y, localNorthOnBall.x) * Mathf.Rad2Deg;
                ball.localRotation = Quaternion.Euler(0f, 0f, northAngle - 90f);
            }
        }

        private Vector3 ResolveLocalNorth(Vector3 radialOut)
        {
            if (radialOut.sqrMagnitude <= minimumDirectionMagnitude * minimumDirectionMagnitude)
            {
                return universeManager.AstrodynamicNorthUnityDirection;
            }

            Vector3 referenceNorth = universeManager.AstrodynamicNorthUnityDirection;
            Vector3 localNorth = Vector3.ProjectOnPlane(referenceNorth, radialOut);
            if (localNorth.sqrMagnitude <= minimumDirectionMagnitude * minimumDirectionMagnitude)
            {
                localNorth = Vector3.ProjectOnPlane(universeManager.AstrodynamicEastUnityDirection, radialOut);
            }

            if (localNorth.sqrMagnitude <= minimumDirectionMagnitude * minimumDirectionMagnitude)
            {
                localNorth = Vector3.ProjectOnPlane(Vector3.forward, radialOut);
            }

            return localNorth.sqrMagnitude > 0f ? localNorth.normalized : Vector3.up;
        }

        private Vector3 ResolveLocalEast(Vector3 radialOut, Vector3 localNorth)
        {
            if (!IsUsableDirection(radialOut) || !IsUsableDirection(localNorth))
            {
                return universeManager.AstrodynamicEastUnityDirection;
            }

            Vector3 localEast = Vector3.Cross(localNorth, radialOut);
            if (!IsUsableDirection(localEast))
            {
                localEast = Vector3.ProjectOnPlane(universeManager.AstrodynamicEastUnityDirection, radialOut);
            }

            return IsUsableDirection(localEast) ? localEast.normalized : Vector3.right;
        }

        private void ResolveBallReferenceBasis(
            Vector3 orbitRadialOut,
            Vector3 orbitLocalNorth,
            out Vector3 ballRadialOut,
            out Vector3 ballLocalNorth,
            out Vector3 ballLocalEast)
        {
            if (stabilizeBallRollAgainstOrbit && TryResolveFixedBallReferenceBasis(out ballRadialOut, out ballLocalNorth, out ballLocalEast))
            {
                return;
            }

            ballRadialOut = orbitRadialOut;
            ballLocalNorth = orbitLocalNorth;
            ballLocalEast = ResolveLocalEast(orbitRadialOut, orbitLocalNorth);
        }

        private bool TryResolveFixedBallReferenceBasis(out Vector3 ballRadialOut, out Vector3 ballLocalNorth, out Vector3 ballLocalEast)
        {
            ballRadialOut = universeManager != null
                ? universeManager.AstrodynamicNorthUnityDirection
                : Vector3.up;

            if (!IsUsableDirection(ballRadialOut))
            {
                ballRadialOut = Vector3.up;
            }

            ballRadialOut = ballRadialOut.normalized;

            Vector3 preferredNorth = universeManager != null
                ? universeManager.AstrodynamicEastUnityDirection
                : Vector3.forward;

            ballLocalNorth = Vector3.ProjectOnPlane(preferredNorth, ballRadialOut);
            if (!IsUsableDirection(ballLocalNorth))
            {
                ballLocalNorth = Vector3.ProjectOnPlane(Vector3.forward, ballRadialOut);
            }

            if (!IsUsableDirection(ballLocalNorth))
            {
                ballLocalNorth = Vector3.ProjectOnPlane(Vector3.right, ballRadialOut);
            }

            if (!IsUsableDirection(ballLocalNorth))
            {
                ballLocalEast = Vector3.zero;
                return false;
            }

            ballLocalNorth = ballLocalNorth.normalized;
            ballLocalEast = Vector3.Cross(ballLocalNorth, ballRadialOut);
            if (!IsUsableDirection(ballLocalEast))
            {
                return false;
            }

            ballLocalEast = ballLocalEast.normalized;
            return true;
        }

        private Quaternion ResolveDisplayRotation(Vector3 radialOut, Vector3 velocity, Vector3 localNorth, Vector3 localEast)
        {
            if (alignToOrbitalFrame)
            {
                Vector3 forward = IsUsableDirection(velocity) ? velocity.normalized : shipOrientation.forward;
                Vector3 up = IsUsableDirection(radialOut) ? radialOut.normalized : shipOrientation.up;

                up = Vector3.ProjectOnPlane(up, forward);
                if (!IsUsableDirection(up))
                {
                    up = Vector3.ProjectOnPlane(localNorth, forward);
                }

                if (!IsUsableDirection(up))
                {
                    up = Vector3.ProjectOnPlane(localEast, forward);
                }

                if (!IsUsableDirection(forward) || !IsUsableDirection(up))
                {
                    return shipOrientation.rotation;
                }

                return Quaternion.LookRotation(forward.normalized, up.normalized);
            }

            Vector3 shipForward = TransformShipAxis(shipLocalForward, shipOrientation.forward);
            if (!IsUsableDirection(shipForward))
            {
                return shipOrientation.rotation;
            }

            Vector3 shipUp = TransformShipAxis(shipLocalUp, shipOrientation.up);
            Vector3 displayUp = ProjectDisplayUp(shipUp, shipForward);
            if (!IsUsableDirection(displayUp))
            {
                displayUp = ProjectDisplayUp(shipOrientation.up, shipForward);
            }

            if (!IsUsableDirection(displayUp))
            {
                displayUp = ProjectDisplayUp(radialOut, shipForward);
            }

            return IsUsableDirection(displayUp)
                ? Quaternion.LookRotation(shipForward.normalized, displayUp.normalized)
                : shipOrientation.rotation;
        }

        private void UpdateDynamicMarkers(
            string frameName,
            Vector3 radialOut,
            Vector3 velocity,
            Vector3 normal,
            Vector3 localNorth,
            Vector3 localEast,
            Quaternion displayRotation)
        {
            if (dynamicMarkers == null)
            {
                return;
            }

            for (int i = 0; i < dynamicMarkers.Length; i++)
            {
                DirectionalMarker marker = dynamicMarkers[i];
                if (marker == null)
                {
                    continue;
                }

                if (!TryResolveMarkerDirection(
                    marker.Direction,
                    marker.BodyTarget,
                    radialOut,
                    velocity,
                    normal,
                    localNorth,
                    localEast,
                    out Vector3 direction,
                    out string bodyName))
                {
                    SetMarkerWithLabelActive(marker.Marker, marker.Label, false);
                    continue;
                }

                string label = ResolveMarkerLabel(marker, frameName, bodyName);
                SetMarkerWithLabel(marker.Marker, marker.Label, direction, displayRotation, label);
            }
        }

        private bool TryResolveMarkerDirection(
            NavballDirectionMode markerDirection,
            ReferenceFrameTarget bodyTarget,
            Vector3 radialOut,
            Vector3 velocity,
            Vector3 normal,
            Vector3 localNorth,
            Vector3 localEast,
            out Vector3 direction,
            out string bodyName)
        {
            bodyName = bodyTarget.ToString();

            switch (markerDirection)
            {
                case NavballDirectionMode.Prograde:
                    direction = velocity;
                    return true;
                case NavballDirectionMode.Retrograde:
                    direction = -velocity;
                    return true;
                case NavballDirectionMode.RadialOut:
                    direction = radialOut;
                    return true;
                case NavballDirectionMode.RadialIn:
                case NavballDirectionMode.SelectedTarget:
                    direction = -radialOut;
                    return true;
                case NavballDirectionMode.Normal:
                    direction = normal;
                    return true;
                case NavballDirectionMode.AntiNormal:
                    direction = -normal;
                    return true;
                case NavballDirectionMode.North:
                    direction = localNorth;
                    return true;
                case NavballDirectionMode.East:
                    direction = localEast;
                    return true;
                case NavballDirectionMode.South:
                    direction = -localNorth;
                    return true;
                case NavballDirectionMode.West:
                    direction = -localEast;
                    return true;
                case NavballDirectionMode.AntiSelectedTarget:
                    direction = radialOut;
                    return true;
                case NavballDirectionMode.Body:
                    return TryGetBodyDirection(bodyTarget, false, out direction, out bodyName);
                case NavballDirectionMode.AntiBody:
                    return TryGetBodyDirection(bodyTarget, true, out direction, out bodyName);
                default:
                    direction = Vector3.zero;
                    return false;
            }
        }

        private bool TryGetBodyDirection(
            ReferenceFrameTarget bodyTarget,
            bool invert,
            out Vector3 direction,
            out string bodyName)
        {
            direction = Vector3.zero;
            bodyName = bodyTarget.ToString();

            if (universeManager == null || universeManager.ShipBody == null)
            {
                return false;
            }

            if (!universeManager.TryGetReferenceState(
                bodyTarget,
                out bodyName,
                out Vector3d bodyPosition,
                out _,
                out _,
                out _,
                out _))
            {
                return false;
            }

            Vector3d realDirection = bodyPosition - universeManager.ShipBody.Position;
            direction = ToUnityDirection(invert ? -realDirection : realDirection);
            return IsUsableDirection(direction);
        }

        private string ResolveMarkerLabel(DirectionalMarker marker, string frameName, string bodyName)
        {
            if (marker == null)
            {
                return string.Empty;
            }

            if (!string.IsNullOrWhiteSpace(marker.LabelOverride))
            {
                return marker.LabelOverride;
            }

            switch (marker.Direction)
            {
                case NavballDirectionMode.Prograde:
                    return "Prograde";
                case NavballDirectionMode.Retrograde:
                    return "Retrograde";
                case NavballDirectionMode.RadialOut:
                    return "Out";
                case NavballDirectionMode.RadialIn:
                    return "In";
                case NavballDirectionMode.Normal:
                    return "Normal";
                case NavballDirectionMode.AntiNormal:
                    return "Anti-normal";
                case NavballDirectionMode.North:
                    return "N";
                case NavballDirectionMode.East:
                    return "E";
                case NavballDirectionMode.South:
                    return "S";
                case NavballDirectionMode.West:
                    return "W";
                case NavballDirectionMode.SelectedTarget:
                    return frameName;
                case NavballDirectionMode.AntiSelectedTarget:
                    return $"Anti {frameName}";
                case NavballDirectionMode.Body:
                    return bodyName;
                case NavballDirectionMode.AntiBody:
                    return $"Anti {bodyName}";
                default:
                    return string.Empty;
            }
        }

        private Vector3 TransformShipAxis(Vector3 localAxis, Vector3 fallbackWorldAxis)
        {
            return IsUsableDirection(localAxis)
                ? shipOrientation.TransformDirection(localAxis.normalized).normalized
                : fallbackWorldAxis.normalized;
        }

        private Vector3 ProjectDisplayUp(Vector3 candidate, Vector3 forward)
        {
            if (!IsUsableDirection(candidate) || !IsUsableDirection(forward))
            {
                return Vector3.zero;
            }

            Vector3 projected = Vector3.ProjectOnPlane(candidate, forward.normalized);
            return IsUsableDirection(projected) ? projected.normalized : Vector3.zero;
        }

        private void UpdateBall(Vector3 radialOut, Vector3 localNorth, Vector3 localEast, Quaternion displayRotation)
        {
            if (ballSphere != null)
            {
                UpdateBallSphere(radialOut, localNorth, localEast, displayRotation);
                if (ball != null)
                {
                    ball.localRotation = Quaternion.identity;
                }

                return;
            }

            UpdateBallTexture(radialOut, localNorth, localEast, displayRotation);
        }

        private void UpdateBallSphere(Vector3 radialOut, Vector3 localNorth, Vector3 localEast, Quaternion displayRotation)
        {
            if (!driveBallTexture || ballSphere == null || shipOrientation == null)
            {
                return;
            }

            if (!IsUsableDirection(radialOut) || !IsUsableDirection(localNorth) || !IsUsableDirection(localEast))
            {
                return;
            }

            Quaternion worldToDisplay = Quaternion.Inverse(displayRotation);
            ballSphere.SetReferenceBasis(
                worldToDisplay * radialOut.normalized,
                worldToDisplay * localNorth.normalized,
                worldToDisplay * localEast.normalized);
        }

        private void UpdateBallTexture(Vector3 radialOut, Vector3 localNorth, Vector3 localEast, Quaternion displayRotation)
        {
            if (!driveBallTexture || ballImage == null || shipOrientation == null)
            {
                return;
            }

            if (!IsUsableDirection(radialOut) || !IsUsableDirection(localNorth) || !IsUsableDirection(localEast))
            {
                return;
            }

            Vector3 forward = displayRotation * Vector3.forward;
            float headingRadians = Mathf.Atan2(Vector3.Dot(forward, localEast), Vector3.Dot(forward, localNorth));
            float pitchRadians = Mathf.Asin(Mathf.Clamp(Vector3.Dot(forward, radialOut), -1f, 1f));

            float width = Mathf.Clamp(ballUvSize.x, 0.05f, 1f);
            float height = Mathf.Clamp(ballUvSize.y, 0.05f, 1f);
            float centerU = Mathf.Repeat(0.5f + (headingRadians / (Mathf.PI * 2f)), 1f);
            float centerV = Mathf.Clamp01(0.5f + (pitchRadians / Mathf.PI));

            ballImage.uvRect = new Rect(centerU - (width * 0.5f), centerV - (height * 0.5f), width, height);
        }

        private bool IsUsableDirection(Vector3 direction)
        {
            return direction.sqrMagnitude > minimumDirectionMagnitude * minimumDirectionMagnitude;
        }

        private Vector3 ToUnityDirection(Vector3d direction)
        {
            Vector3 unityDirection = universeManager.ToUnityDirection(direction);
            return unityDirection.sqrMagnitude > minimumDirectionMagnitude * minimumDirectionMagnitude
                ? unityDirection.normalized
                : Vector3.zero;
        }

        private void SetMarkerWithLabel(RectTransform marker, TMP_Text label, Vector3 worldDirection, Quaternion displayRotation, string text)
        {
            if (label != null)
            {
                label.text = text;
            }

            RectTransform labelTransform = label != null ? label.rectTransform : null;
            RectTransform markerTransform = marker != null ? marker : labelTransform;

            SetMarker(markerTransform, worldDirection, displayRotation);

            if (marker != null && labelTransform != null && labelTransform != marker && !labelTransform.IsChildOf(marker))
            {
                SetMarker(labelTransform, worldDirection, displayRotation);
            }
        }

        private void SetMarker(RectTransform marker, Vector3 worldDirection, Quaternion displayRotation)
        {
            if (marker == null)
            {
                return;
            }

            if (worldDirection.sqrMagnitude <= minimumDirectionMagnitude * minimumDirectionMagnitude)
            {
                marker.gameObject.SetActive(false);
                return;
            }

            Vector3 localDirection = Quaternion.Inverse(displayRotation) * worldDirection.normalized;
            bool isFrontSide = localDirection.z >= 0f;

            if (hideBacksideMarkers && !isFrontSide)
            {
                marker.gameObject.SetActive(false);
                return;
            }

            marker.gameObject.SetActive(true);
            marker.anchoredPosition = new Vector2(localDirection.x, localDirection.y) * markerRadius;
            marker.localScale = Vector3.one * (isFrontSide ? 1f : Mathf.Max(0.1f, backsideScale));

            CanvasGroup canvasGroup = marker.GetComponent<CanvasGroup>();
            if (canvasGroup != null)
            {
                canvasGroup.alpha = isFrontSide ? 1f : 0.35f;
            }
        }

        private void HideAllMarkers()
        {
            SetMarkerActive(progradeMarker, false);
            SetMarkerActive(retrogradeMarker, false);
            SetMarkerActive(radialOutMarker, false);
            SetMarkerActive(radialInMarker, false);
            SetMarkerActive(normalMarker, false);
            SetMarkerActive(antiNormalMarker, false);
            SetMarkerActive(northMarker, false);
            SetMarkerActive(eastMarker, false);
            SetMarkerActive(southMarker, false);
            SetMarkerActive(westMarker, false);
            SetMarkerWithLabelActive(selectedTargetMarker, selectedTargetLabel, false);
            SetMarkerWithLabelActive(antiSelectedTargetMarker, antiSelectedTargetLabel, false);
            HideDynamicMarkers();
        }

        private static void SetMarkerActive(RectTransform marker, bool active)
        {
            if (marker != null)
            {
                marker.gameObject.SetActive(active);
            }
        }

        private static void SetMarkerWithLabelActive(RectTransform marker, TMP_Text label, bool active)
        {
            RectTransform labelTransform = label != null ? label.rectTransform : null;
            RectTransform markerTransform = marker != null ? marker : labelTransform;

            SetMarkerActive(markerTransform, active);

            if (marker != null && labelTransform != null && labelTransform != marker && !labelTransform.IsChildOf(marker))
            {
                SetMarkerActive(labelTransform, active);
            }
        }

        private void HideDynamicMarkers()
        {
            if (dynamicMarkers == null)
            {
                return;
            }

            for (int i = 0; i < dynamicMarkers.Length; i++)
            {
                DirectionalMarker marker = dynamicMarkers[i];
                if (marker != null)
                {
                    SetMarkerWithLabelActive(marker.Marker, marker.Label, false);
                }
            }
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
            {
                universeManager = FindAnyObjectByType<UniverseManager>();
            }

            if (shipOrientation == null && universeManager != null)
            {
                shipOrientation = universeManager.ShipVisualTransform;
            }

            if (ballImage == null)
            {
                ballImage = ballSphere != null ? ballSphere : GetComponentInChildren<RawImage>(true);
            }

            if (ball == null && ballImage != null)
            {
                ball = ballImage.rectTransform;
            }

            if (ballSphere == null)
            {
                ballSphere = GetComponentInChildren<NavballSphereGraphic>(true);
            }

            if (frameGraphic == null)
            {
                frameGraphic = GetComponentInChildren<NavballFrameGraphic>(true);
            }

            if (autopilot == null)
            {
                autopilot = GetComponent<NavballAutopilotController>();
                if (autopilot == null)
                {
                    autopilot = FindAnyObjectByType<NavballAutopilotController>();
                }
            }

            ResolveMarkerReferences();
        }

        private void EnsureAutopilotButtons()
        {
            if (!createAutopilotButtons || autopilotButtonsCreated)
            {
                return;
            }

            if (autopilot == null)
            {
                autopilot = gameObject.AddComponent<NavballAutopilotController>();
            }

            if (autopilotButtonsRoot == null)
            {
                autopilotButtonsRoot = CreateAutopilotButtonsRoot();
            }

            if (autopilotButtonsRoot == null)
            {
                return;
            }

            autopilotButtonsCreated = true;
            autopilotButtonBindings.Clear();

            CreateAutopilotButton("PRO", NavballDirectionMode.Prograde);
            CreateAutopilotButton("RET", NavballDirectionMode.Retrograde);
            CreateAutopilotButton("IN", NavballDirectionMode.RadialIn);
            CreateAutopilotButton("OUT", NavballDirectionMode.RadialOut);
            CreateAutopilotButton("TGT", NavballDirectionMode.SelectedTarget);
            CreateAutopilotButton("ATG", NavballDirectionMode.AntiSelectedTarget);
            CreateAutopilotButton("NRM", NavballDirectionMode.Normal);
            CreateAutopilotButton("AN", NavballDirectionMode.AntiNormal);
            CreateAutopilotButton("N", NavballDirectionMode.North);
            CreateAutopilotButton("E", NavballDirectionMode.East);
            CreateAutopilotButton("S", NavballDirectionMode.South);
            CreateAutopilotButton("W", NavballDirectionMode.West);
            CreateAutopilotStopButton("OFF");
        }

        private RectTransform CreateAutopilotButtonsRoot()
        {
            GameObject rootObject = new GameObject("AutopilotButtons", typeof(RectTransform), typeof(GridLayoutGroup));
            rootObject.transform.SetParent(transform, false);

            RectTransform root = rootObject.GetComponent<RectTransform>();
            root.anchorMin = new Vector2(0.5f, 0.5f);
            root.anchorMax = new Vector2(0.5f, 0.5f);
            root.pivot = new Vector2(0.5f, 0.5f);
            root.anchoredPosition = autopilotButtonsOffset;

            GridLayoutGroup grid = rootObject.GetComponent<GridLayoutGroup>();
            grid.cellSize = autopilotButtonSize;
            grid.spacing = new Vector2(4f, 4f);
            grid.constraint = GridLayoutGroup.Constraint.FixedColumnCount;
            grid.constraintCount = Mathf.Max(1, autopilotButtonColumns);
            grid.childAlignment = TextAnchor.MiddleCenter;

            int columns = Mathf.Max(1, autopilotButtonColumns);
            int rows = Mathf.CeilToInt(AutopilotButtonCount / (float)columns);
            root.sizeDelta = new Vector2(
                (autopilotButtonSize.x * columns) + (grid.spacing.x * (columns - 1)),
                (autopilotButtonSize.y * rows) + (grid.spacing.y * (rows - 1)));

            return root;
        }

        private void CreateAutopilotButton(string label, NavballDirectionMode direction)
        {
            Button button = CreateAutopilotButtonObject(label, autopilotButtonColor, out Image background);
            NavballDirectionMode capturedDirection = direction;
            button.onClick.AddListener(() =>
            {
                if (autopilot != null)
                {
                    autopilot.SetHoldDirection(capturedDirection);
                    RefreshAutopilotButtons();
                }
            });

            autopilotButtonBindings.Add(new AutopilotButtonBinding
            {
                Button = button,
                Background = background,
                Direction = direction,
                IsStopButton = false
            });
        }

        private void CreateAutopilotStopButton(string label)
        {
            Button button = CreateAutopilotButtonObject(label, autopilotStopButtonColor, out Image background);
            button.onClick.AddListener(() =>
            {
                if (autopilot != null)
                {
                    autopilot.StopHold();
                    RefreshAutopilotButtons();
                }
            });

            autopilotButtonBindings.Add(new AutopilotButtonBinding
            {
                Button = button,
                Background = background,
                IsStopButton = true
            });
        }

        private Button CreateAutopilotButtonObject(string label, Color backgroundColor, out Image background)
        {
            GameObject buttonObject = new GameObject(label, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image), typeof(Button));
            buttonObject.transform.SetParent(autopilotButtonsRoot, false);

            RectTransform rectTransform = buttonObject.GetComponent<RectTransform>();
            rectTransform.sizeDelta = autopilotButtonSize;

            background = buttonObject.GetComponent<Image>();
            background.color = backgroundColor;
            background.raycastTarget = true;

            Button button = buttonObject.GetComponent<Button>();
            ColorBlock colors = button.colors;
            colors.normalColor = Color.white;
            colors.highlightedColor = new Color(1.15f, 1.15f, 1.15f, 1f);
            colors.pressedColor = new Color(0.78f, 0.78f, 0.78f, 1f);
            colors.selectedColor = Color.white;
            colors.disabledColor = new Color(0.45f, 0.45f, 0.45f, 0.6f);
            colors.colorMultiplier = 1f;
            button.colors = colors;

            GameObject textObject = new GameObject("Label", typeof(RectTransform), typeof(CanvasRenderer), typeof(TextMeshProUGUI));
            textObject.transform.SetParent(buttonObject.transform, false);

            RectTransform textTransform = textObject.GetComponent<RectTransform>();
            textTransform.anchorMin = Vector2.zero;
            textTransform.anchorMax = Vector2.one;
            textTransform.offsetMin = Vector2.zero;
            textTransform.offsetMax = Vector2.zero;

            TextMeshProUGUI text = textObject.GetComponent<TextMeshProUGUI>();
            text.text = label;
            text.fontSize = 11f;
            text.enableAutoSizing = true;
            text.fontSizeMin = 7f;
            text.fontSizeMax = 11f;
            text.alignment = TextAlignmentOptions.Center;
            text.color = Color.white;
            text.raycastTarget = false;

            return button;
        }

        private void RefreshAutopilotButtons()
        {
            if (autopilotButtonBindings.Count == 0)
            {
                return;
            }

            bool isHolding = autopilot != null && autopilot.IsHolding;
            NavballDirectionMode activeDirection = autopilot != null ? autopilot.HoldDirection : NavballDirectionMode.Prograde;

            for (int i = 0; i < autopilotButtonBindings.Count; i++)
            {
                AutopilotButtonBinding binding = autopilotButtonBindings[i];
                if (binding == null || binding.Background == null)
                {
                    continue;
                }

                if (binding.IsStopButton)
                {
                    binding.Background.color = isHolding ? autopilotStopButtonColor : autopilotButtonColor;
                    continue;
                }

                binding.Background.color = isHolding && binding.Direction == activeDirection
                    ? autopilotActiveButtonColor
                    : autopilotButtonColor;
            }
        }

        private void ResolveMarkerReferences()
        {
            if (northMarker == null)
            {
                northMarker = FindChildRectTransformByName("North", "N", "NorthMarker", "NavballNorth");
            }

            if (eastMarker == null)
            {
                eastMarker = FindChildRectTransformByName("East", "E", "EastMarker", "NavballEast");
            }

            if (southMarker == null)
            {
                southMarker = FindChildRectTransformByName("South", "S", "SouthMarker", "NavballSouth");
            }

            if (westMarker == null)
            {
                westMarker = FindChildRectTransformByName("West", "W", "WestMarker", "NavballWest");
            }

            if (selectedTargetMarker == null)
            {
                selectedTargetMarker = FindChildRectTransformByName(
                    "Target",
                    "SelectedTarget",
                    "ReferenceTarget",
                    "FrameTarget",
                    "BodyTarget");
            }

            if (antiSelectedTargetMarker == null)
            {
                antiSelectedTargetMarker = FindChildRectTransformByName(
                    "AntiTarget",
                    "AntiSelectedTarget",
                    "AntiReferenceTarget",
                    "AntiFrameTarget",
                    "AntiBodyTarget");
            }

            if (selectedTargetLabel == null)
            {
                selectedTargetLabel = FindTextOnMarker(selectedTargetMarker);
            }

            if (antiSelectedTargetLabel == null)
            {
                antiSelectedTargetLabel = FindTextOnMarker(antiSelectedTargetMarker);
            }
        }

        private RectTransform FindChildRectTransformByName(params string[] names)
        {
            if (names == null || names.Length == 0)
            {
                return null;
            }

            RectTransform[] rectTransforms = GetComponentsInChildren<RectTransform>(true);
            for (int i = 0; i < rectTransforms.Length; i++)
            {
                RectTransform rectTransform = rectTransforms[i];
                if (rectTransform == null || rectTransform == transform)
                {
                    continue;
                }

                for (int nameIndex = 0; nameIndex < names.Length; nameIndex++)
                {
                    if (string.Equals(rectTransform.name, names[nameIndex], StringComparison.OrdinalIgnoreCase))
                    {
                        return rectTransform;
                    }
                }
            }

            return null;
        }

        private static TMP_Text FindTextOnMarker(RectTransform marker)
        {
            return marker != null ? marker.GetComponentInChildren<TMP_Text>(true) : null;
        }
    }
}

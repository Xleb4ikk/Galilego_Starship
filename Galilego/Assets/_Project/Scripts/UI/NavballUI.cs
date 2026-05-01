using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Galilego.Physics
{
    public sealed class NavballUI : MonoBehaviour
    {
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

        [Header("Display")]
        [SerializeField] private float markerRadius = 82f;
        [SerializeField] private bool hideBacksideMarkers = true;
        [SerializeField] private float backsideScale = 0.65f;
        [SerializeField] private float minimumDirectionMagnitude = 1e-4f;
        [SerializeField] private bool driveBallTexture = true;
        [SerializeField] private bool alignToOrbitalFrame;
        [SerializeField] private Vector3 shipLocalForward = Vector3.up;
        [SerializeField] private Vector3 shipLocalUp = Vector3.forward;
        [SerializeField] private bool flipBallVertical = true;
        [SerializeField] private Vector2 ballUvSize = new Vector2(0.42f, 0.42f);

        private void LateUpdate()
        {
            ResolveReferences();

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

            Vector3 ballRadialOut = radialOut;
            Vector3 ballLocalEast = localEast;
            if (flipBallVertical)
            {
                ballRadialOut = -ballRadialOut;
                ballLocalEast = -ballLocalEast;
            }

            UpdateBall(ballRadialOut, localNorth, ballLocalEast, displayRotation);

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
        }

        private static void SetMarkerActive(RectTransform marker, bool active)
        {
            if (marker != null)
            {
                marker.gameObject.SetActive(active);
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
        }
    }
}

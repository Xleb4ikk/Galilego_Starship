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
        [SerializeField] private Vector2 ballUvSize = new Vector2(0.42f, 0.42f);

        private void Update()
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
            Vector3 localEast = Vector3.Cross(radialOut, localNorth).normalized;

            UpdateBall(radialOut, localNorth, localEast);

            SetMarker(progradeMarker, velocity);
            SetMarker(retrogradeMarker, -velocity);
            SetMarker(radialOutMarker, radialOut);
            SetMarker(radialInMarker, -radialOut);
            SetMarker(normalMarker, normal);
            SetMarker(antiNormalMarker, -normal);
            SetMarker(northMarker, localNorth);
            SetMarker(southMarker, -localNorth);
            SetMarker(eastMarker, localEast);
            SetMarker(westMarker, -localEast);

            if (ball != null && ballSphere == null)
            {
                Vector3 localNorthOnBall = shipOrientation.InverseTransformDirection(localNorth);
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

        private void UpdateBall(Vector3 radialOut, Vector3 localNorth, Vector3 localEast)
        {
            if (ballSphere != null)
            {
                UpdateBallSphere(radialOut, localNorth, localEast);
                if (ball != null)
                {
                    ball.localRotation = Quaternion.identity;
                }

                return;
            }

            UpdateBallTexture(radialOut, localNorth, localEast);
        }

        private void UpdateBallSphere(Vector3 radialOut, Vector3 localNorth, Vector3 localEast)
        {
            if (!driveBallTexture || ballSphere == null || shipOrientation == null)
            {
                return;
            }

            if (!IsUsableDirection(radialOut) || !IsUsableDirection(localNorth) || !IsUsableDirection(localEast))
            {
                return;
            }

            ballSphere.SetReferenceBasis(
                shipOrientation.InverseTransformDirection(radialOut.normalized),
                shipOrientation.InverseTransformDirection(localNorth.normalized),
                shipOrientation.InverseTransformDirection(localEast.normalized));
        }

        private void UpdateBallTexture(Vector3 radialOut, Vector3 localNorth, Vector3 localEast)
        {
            if (!driveBallTexture || ballImage == null || shipOrientation == null)
            {
                return;
            }

            if (!IsUsableDirection(radialOut) || !IsUsableDirection(localNorth) || !IsUsableDirection(localEast))
            {
                return;
            }

            Vector3 forward = shipOrientation.forward.normalized;
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

        private void SetMarker(RectTransform marker, Vector3 worldDirection)
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

            Vector3 localDirection = shipOrientation.InverseTransformDirection(worldDirection.normalized);
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

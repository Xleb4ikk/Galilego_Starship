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

            if (ball != null)
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
        }
    }
}

using UnityEngine;
using Galilego.Core;
using Galilego.UI;
using Galilego.Universe;

namespace Galilego.Gameplay
{
    public sealed class NavballAutopilotController : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private Transform shipOrientation;

        [Header("Hold")]
        [SerializeField] private bool holdEnabled;
        [SerializeField] private NavballUI.NavballDirectionMode holdDirection = NavballUI.NavballDirectionMode.Prograde;
        [SerializeField] private ReferenceFrameTarget holdBodyTarget = ReferenceFrameTarget.Jupiter;
        [SerializeField] private float turnRateDegreesPerSecond = 90f;
        [SerializeField] private float minimumDirectionMagnitude = 1e-4f;
        [SerializeField] private Vector3 shipLocalForward = Vector3.up;
        [SerializeField] private Vector3 shipLocalUp = Vector3.forward;
        [SerializeField] private bool stopManualRotationWhileHolding = true;

        private SpaceRotation manualRotation;

        public bool IsHolding => holdEnabled;
        public NavballUI.NavballDirectionMode HoldDirection => holdDirection;

        private void LateUpdate()
        {
            ResolveReferences();

            if (!holdEnabled || universeManager == null || shipOrientation == null)
            {
                return;
            }

            if (!TryResolveHoldDirection(out Vector3 targetDirection, out Vector3 referenceUp))
            {
                return;
            }

            if (stopManualRotationWhileHolding && manualRotation != null)
            {
                manualRotation.Stop();
            }

            Quaternion desiredRotation = ResolveDesiredRotation(targetDirection, referenceUp);
            float maxStep = Mathf.Max(1f, turnRateDegreesPerSecond) * Time.deltaTime;
            shipOrientation.rotation = Quaternion.RotateTowards(shipOrientation.rotation, desiredRotation, maxStep);
        }

        public void SetHoldDirection(NavballUI.NavballDirectionMode direction)
        {
            holdDirection = direction;
            holdEnabled = true;

            if (stopManualRotationWhileHolding)
            {
                ResolveReferences();
                if (manualRotation != null)
                {
                    manualRotation.Stop();
                }
            }
        }

        public void SetHoldBody(ReferenceFrameTarget target, bool antiDirection = false)
        {
            holdBodyTarget = target;
            SetHoldDirection(antiDirection ? NavballUI.NavballDirectionMode.AntiBody : NavballUI.NavballDirectionMode.Body);
        }

        public void StopHold()
        {
            holdEnabled = false;
        }

        private bool TryResolveHoldDirection(out Vector3 direction, out Vector3 referenceUp)
        {
            direction = Vector3.zero;
            referenceUp = Vector3.zero;

            ReferenceFrameTarget activeFrame = universeManager.ActiveReferenceFrame;
            if (!universeManager.TryGetShipRelativeState(
                activeFrame,
                out _,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out _,
                out _,
                out _))
            {
                return false;
            }

            Vector3 radialOut = ToUnityDirection(relativePosition);
            Vector3 velocity = ToUnityDirection(relativeVelocity);
            Vector3 normal = ToUnityDirection(Vector3d.Cross(relativePosition, relativeVelocity));
            Vector3 localNorth = ResolveLocalNorth(radialOut);
            Vector3 localEast = ResolveLocalEast(radialOut, localNorth);

            switch (holdDirection)
            {
                case NavballUI.NavballDirectionMode.Prograde:
                    direction = velocity;
                    break;
                case NavballUI.NavballDirectionMode.Retrograde:
                    direction = -velocity;
                    break;
                case NavballUI.NavballDirectionMode.RadialOut:
                    direction = radialOut;
                    break;
                case NavballUI.NavballDirectionMode.RadialIn:
                case NavballUI.NavballDirectionMode.SelectedTarget:
                    direction = -radialOut;
                    break;
                case NavballUI.NavballDirectionMode.Normal:
                    direction = normal;
                    break;
                case NavballUI.NavballDirectionMode.AntiNormal:
                    direction = -normal;
                    break;
                case NavballUI.NavballDirectionMode.North:
                    direction = localNorth;
                    break;
                case NavballUI.NavballDirectionMode.East:
                    direction = localEast;
                    break;
                case NavballUI.NavballDirectionMode.South:
                    direction = -localNorth;
                    break;
                case NavballUI.NavballDirectionMode.West:
                    direction = -localEast;
                    break;
                case NavballUI.NavballDirectionMode.AntiSelectedTarget:
                    direction = radialOut;
                    break;
                case NavballUI.NavballDirectionMode.Body:
                    if (!TryGetBodyDirection(holdBodyTarget, false, out direction))
                    {
                        return false;
                    }
                    break;
                case NavballUI.NavballDirectionMode.AntiBody:
                    if (!TryGetBodyDirection(holdBodyTarget, true, out direction))
                    {
                        return false;
                    }
                    break;
                default:
                    return false;
            }

            if (!IsUsableDirection(direction))
            {
                return false;
            }

            direction = direction.normalized;
            referenceUp = ResolveReferenceUp(direction, radialOut, localNorth, localEast);
            return true;
        }

        private Quaternion ResolveDesiredRotation(Vector3 targetDirection, Vector3 referenceUp)
        {
            Vector3 localForward = IsUsableDirection(shipLocalForward) ? shipLocalForward.normalized : Vector3.forward;
            Vector3 localUp = ProjectDisplayUp(shipLocalUp, localForward);
            if (!IsUsableDirection(localUp))
            {
                localUp = Mathf.Abs(Vector3.Dot(localForward, Vector3.up)) > 0.999f
                    ? Vector3.forward
                    : Vector3.up;
            }

            Vector3 worldUp = ProjectDisplayUp(referenceUp, targetDirection);
            if (!IsUsableDirection(worldUp))
            {
                worldUp = ProjectDisplayUp(shipOrientation.TransformDirection(localUp), targetDirection);
            }

            if (!IsUsableDirection(worldUp))
            {
                worldUp = Mathf.Abs(Vector3.Dot(targetDirection.normalized, Vector3.up)) > 0.999f
                    ? Vector3.forward
                    : Vector3.up;
            }

            Quaternion localLook = Quaternion.LookRotation(localForward, localUp);
            Quaternion worldLook = Quaternion.LookRotation(targetDirection.normalized, worldUp.normalized);
            return worldLook * Quaternion.Inverse(localLook);
        }

        private Vector3 ResolveReferenceUp(Vector3 targetDirection, Vector3 radialOut, Vector3 localNorth, Vector3 localEast)
        {
            Vector3 currentUp = IsUsableDirection(shipLocalUp)
                ? shipOrientation.TransformDirection(shipLocalUp.normalized)
                : shipOrientation.up;

            Vector3 referenceUp = ProjectDisplayUp(currentUp, targetDirection);
            if (IsUsableDirection(referenceUp))
            {
                return referenceUp;
            }

            referenceUp = ProjectDisplayUp(radialOut, targetDirection);
            if (IsUsableDirection(referenceUp))
            {
                return referenceUp;
            }

            referenceUp = ProjectDisplayUp(localNorth, targetDirection);
            if (IsUsableDirection(referenceUp))
            {
                return referenceUp;
            }

            referenceUp = ProjectDisplayUp(localEast, targetDirection);
            return IsUsableDirection(referenceUp) ? referenceUp : Vector3.up;
        }

        private Vector3 ResolveLocalNorth(Vector3 radialOut)
        {
            if (!IsUsableDirection(radialOut))
            {
                return universeManager.AstrodynamicNorthUnityDirection;
            }

            Vector3 referenceNorth = universeManager.AstrodynamicNorthUnityDirection;
            Vector3 localNorth = Vector3.ProjectOnPlane(referenceNorth, radialOut);
            if (!IsUsableDirection(localNorth))
            {
                localNorth = Vector3.ProjectOnPlane(universeManager.AstrodynamicEastUnityDirection, radialOut);
            }

            if (!IsUsableDirection(localNorth))
            {
                localNorth = Vector3.ProjectOnPlane(Vector3.forward, radialOut);
            }

            return IsUsableDirection(localNorth) ? localNorth.normalized : Vector3.up;
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

        private bool TryGetBodyDirection(ReferenceFrameTarget bodyTarget, bool invert, out Vector3 direction)
        {
            direction = Vector3.zero;

            if (universeManager == null || universeManager.ShipBody == null)
            {
                return false;
            }

            if (!universeManager.TryGetReferenceState(
                bodyTarget,
                out _,
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

        private Vector3 ToUnityDirection(Vector3d direction)
        {
            Vector3 unityDirection = universeManager.ToUnityDirection(direction);
            return IsUsableDirection(unityDirection) ? unityDirection.normalized : Vector3.zero;
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

        private bool IsUsableDirection(Vector3 direction)
        {
            return direction.sqrMagnitude > minimumDirectionMagnitude * minimumDirectionMagnitude;
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

            if (manualRotation == null && shipOrientation != null)
            {
                manualRotation = shipOrientation.GetComponent<SpaceRotation>();
            }
        }
    }
}

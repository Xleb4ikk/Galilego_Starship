using UnityEngine;

namespace Galilego.Physics
{
    /// <summary>
    /// Lightweight Navball autopilot controller used by the UI.
    /// Provides a minimal API expected by <see cref="NavballUI"/>: hold/stop and current hold direction.
    /// This is intentionally small — actual flight-control logic can be implemented later.
    /// </summary>
    [DisallowMultipleComponent]
    public class NavballAutopilotController : MonoBehaviour
    {
        [SerializeField] private bool isHolding = false;
        [SerializeField] private NavballUI.NavballDirectionMode holdDirection = NavballUI.NavballDirectionMode.Prograde;

        public bool IsHolding => isHolding;
        public NavballUI.NavballDirectionMode HoldDirection => holdDirection;

        public void SetHoldDirection(NavballUI.NavballDirectionMode direction)
        {
            holdDirection = direction;
            isHolding = true;
        }

        public void StopHold()
        {
            isHolding = false;
        }
    }
}

using UnityEngine;
using UnityEngine.InputSystem;

public class SpaceRotation : MonoBehaviour
{
    [Header("Сила вращения")]
    public float rotationForce = 150f;

    [Header("Инерция")]
    [Range(0f, 1f)]
    public float damping = 0.02f;

    private Vector3 _angularVelocity;

    // ─── Клавиши (новый Input System) ────────────────────────────────────────
    // Ось X: W / S
    // Ось Y: A / D
    // Ось Z: Q / E
    // Стоп:  Space

    void Start()
    {
        // Silent start — avoid log spam during performance-sensitive runs
    }

    void Update()
    {
        HandleInput();
        ApplyInertia();
    }

    void HandleInput()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        float force = rotationForce * Time.deltaTime;

        // Ось X
        if (kb.wKey.isPressed)
        {
            _angularVelocity.x -= force;
        }
        if (kb.sKey.isPressed)
        {
            _angularVelocity.x += force;
        }

        // Ось Y
        if (kb.aKey.isPressed)
        {
            _angularVelocity.y -= force;
        }
        if (kb.dKey.isPressed)
        {
            _angularVelocity.y += force;
        }

        // Ось Z
        if (kb.qKey.isPressed)
        {
            _angularVelocity.z += force;
        }
        if (kb.eKey.isPressed)
        {
            _angularVelocity.z -= force;
        }

        // Стоп
        if (kb.spaceKey.wasPressedThisFrame)
        {
            Stop();
        }
    }

    void ApplyInertia()
    {
        // If angular velocity components became invalid, reset to safe state
        if (float.IsNaN(_angularVelocity.x) || float.IsNaN(_angularVelocity.y) || float.IsNaN(_angularVelocity.z) ||
            float.IsInfinity(_angularVelocity.x) || float.IsInfinity(_angularVelocity.y) || float.IsInfinity(_angularVelocity.z))
        {
            _angularVelocity = Vector3.zero;
            return;
        }

        // Use squared magnitude check to avoid an expensive sqrt call
        if (_angularVelocity.sqrMagnitude > 0.0001f)
        {
            transform.Rotate(
                _angularVelocity.x * Time.deltaTime,
                _angularVelocity.y * Time.deltaTime,
                _angularVelocity.z * Time.deltaTime,
                Space.World
            );
        }

        _angularVelocity = Vector3.Lerp(_angularVelocity, Vector3.zero, damping);
    }

    public void AddTorque(Vector3 torque)
    {
        _angularVelocity += torque;
    }

    public void Stop()
    {
        _angularVelocity = Vector3.zero;
    }
}

using UnityEngine;
using UnityEngine.InputSystem;

public class SpaceRotation : MonoBehaviour
{
    [Header("Rotation thrust")]
    public float rotationForce = 150f;

    [Header("Optional angular drag")]
    [Min(0f)]
    public float damping = 0f;

    private Vector3 _angularVelocity;

    private void Start()
    {
        Debug.Log($"[SpaceRotation] Active on '{gameObject.name}' | force={rotationForce} angularDrag={damping}");
    }

    private void Update()
    {
        HandleInput();
        ApplyInertia();
    }

    private void HandleInput()
    {
        Keyboard kb = Keyboard.current;
        if (kb == null)
        {
            return;
        }

        float force = rotationForce * Time.deltaTime;

        if (kb.wKey.isPressed)
        {
            _angularVelocity.x -= force;
        }

        if (kb.sKey.isPressed)
        {
            _angularVelocity.x += force;
        }

        if (kb.aKey.isPressed)
        {
            _angularVelocity.y -= force;
        }

        if (kb.dKey.isPressed)
        {
            _angularVelocity.y += force;
        }

        if (kb.qKey.isPressed)
        {
            _angularVelocity.z += force;
        }

        if (kb.eKey.isPressed)
        {
            _angularVelocity.z -= force;
        }

        if (kb.spaceKey.wasPressedThisFrame)
        {
            Stop();
        }
    }

    private void ApplyInertia()
    {
        if (_angularVelocity.sqrMagnitude > 0f)
        {
            transform.Rotate(
                _angularVelocity.x * Time.deltaTime,
                _angularVelocity.y * Time.deltaTime,
                _angularVelocity.z * Time.deltaTime,
                Space.World);
        }

        if (damping > 0f)
        {
            float drag = Mathf.Clamp01(damping * Time.deltaTime);
            _angularVelocity = Vector3.Lerp(_angularVelocity, Vector3.zero, drag);
        }
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

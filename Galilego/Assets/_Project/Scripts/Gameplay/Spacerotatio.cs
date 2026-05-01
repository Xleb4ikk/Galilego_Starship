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
        Debug.Log($"[SpaceRotation] Запущен на '{gameObject.name}' | force={rotationForce} damping={damping}");
        Debug.Log("[SpaceRotation] Управление: W/S=ОсьX  A/D=ОсьY  Q/E=ОсьZ  Space=Стоп");
    }

    void Update()
    {
        HandleInput();
        ApplyInertia();
    }

    void HandleInput()
    {
        var kb = Keyboard.current;
        if (kb == null)
        {
            Debug.LogWarning("[SpaceRotation] Клавиатура не найдена!");
            return;
        }

        float force = rotationForce * Time.deltaTime;

        // Ось X
        if (kb.wKey.isPressed)
        {
            _angularVelocity.x -= force;
            Debug.Log($"[SpaceRotation] W нажата → X velocity: {_angularVelocity.x:F2}");
        }
        if (kb.sKey.isPressed)
        {
            _angularVelocity.x += force;
            Debug.Log($"[SpaceRotation] S нажата → X velocity: {_angularVelocity.x:F2}");
        }

        // Ось Y
        if (kb.aKey.isPressed)
        {
            _angularVelocity.y -= force;
            Debug.Log($"[SpaceRotation] A нажата → Y velocity: {_angularVelocity.y:F2}");
        }
        if (kb.dKey.isPressed)
        {
            _angularVelocity.y += force;
            Debug.Log($"[SpaceRotation] D нажата → Y velocity: {_angularVelocity.y:F2}");
        }

        // Ось Z
        if (kb.qKey.isPressed)
        {
            _angularVelocity.z += force;
            Debug.Log($"[SpaceRotation] Q нажата → Z velocity: {_angularVelocity.z:F2}");
        }
        if (kb.eKey.isPressed)
        {
            _angularVelocity.z -= force;
            Debug.Log($"[SpaceRotation] E нажата → Z velocity: {_angularVelocity.z:F2}");
        }

        // Стоп
        if (kb.spaceKey.wasPressedThisFrame)
        {
            Debug.Log("[SpaceRotation] СТОП — угловая скорость сброшена");
            Stop();
        }
    }

    void ApplyInertia()
    {
        if (_angularVelocity.magnitude > 0.01f)
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
        Debug.Log($"[SpaceRotation] AddTorque: {torque} → velocity: {_angularVelocity}");
    }

    public void Stop()
    {
        _angularVelocity = Vector3.zero;
    }
}

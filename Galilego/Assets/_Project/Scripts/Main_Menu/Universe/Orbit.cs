using UnityEngine;

public class Orbit : MonoBehaviour
{
    public Transform center; // :contentReference[oaicite:1]{index=1}

    [Header("Orbit Settings")]
    public float distance = 10f;
    public float orbitalPeriodSeconds = 10000f;
    public float inclination = 0f;

    [Header("Time Mode")]
    public TimeMode mode = TimeMode.Real;

    [Header("Fast Mode")]
    public float fastSpeed = 30f;

    [Header("Time Scale (Real mode)")]
    public float timeScale = 1f;

    private float angle;

    public enum TimeMode
    {
        Fast,
        Real,
        UltraReal
    }

    void Update()
    {
        float speed;

        switch (mode)
        {
            case TimeMode.Fast:
                speed = fastSpeed;
                break;

            case TimeMode.Real:
                speed = (360f / orbitalPeriodSeconds) * timeScale;
                break;

            case TimeMode.UltraReal:
                speed = 360f / orbitalPeriodSeconds;
                break;

            default:
                speed = fastSpeed;
                break;
        }

        angle += speed * Time.deltaTime;

        float rad = angle * Mathf.Deg2Rad;

        Vector3 offset = new Vector3(
            Mathf.Cos(rad) * distance,
            0f,
            Mathf.Sin(rad) * distance
        );

        Quaternion tilt = Quaternion.Euler(inclination, 0f, 0f);

        transform.position = center.position + tilt * offset;
    }

    // удобные переключатели (можно вызвать из UI)
    public void SetFast() => mode = TimeMode.Fast;
    public void SetReal() => mode = TimeMode.Real;
    public void SetUltraReal() => mode = TimeMode.UltraReal;
}
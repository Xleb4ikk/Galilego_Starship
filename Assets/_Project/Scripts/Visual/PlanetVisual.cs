using Galilego.MoonVisualSetting;
using Galilego.Core;
using System;
using System.Collections.Generic;
using UnityEngine;

public class PlanetVisual : MonoBehaviour
{
    [Header("Jupiter")]
    [SerializeField] private Transform jupiterTransform;
    [SerializeField] private double jupiterRadius = 6.9911e7d;

    [SerializeField] private double metersPerUnityUnit = 100000d;

    [Header("Visual Scale")]
    [SerializeField] private double visualDistanceMultiplier = 0.1d;

    [SerializeField] private List<MoonSettingVisual> moonVisualListSetting = new List<MoonSettingVisual>();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ApplyVisualScale(jupiterTransform, jupiterRadius);

        for (int i = 0; i < moonVisualListSetting.Count; i++)
        {
            MoonSettingVisual moon = moonVisualListSetting[i];

            if (moon == null)
                continue;

            Transform visual = moon.VisualTransform;
            if (visual == null)
                continue;

            ApplyVisualScale(visual, moon.Radius);
        }
    }

    private void ApplyVisualScale(Transform target, double realRadiusMeters)
    {
        if (target == null)
            return;

        if (realRadiusMeters <= 0d)
            return;

        double metersPerVisual = GetMetersPerVisualUnit();
        if (metersPerVisual <= 0d)
            return;

        double desiredDiameterInUnits = (realRadiusMeters / metersPerVisual) * 2d;

        if (double.IsNaN(desiredDiameterInUnits) || double.IsInfinity(desiredDiameterInUnits))
        {
            Debug.LogWarning($"UniverseManager: invalid scale computed for '{target.name}': {desiredDiameterInUnits}");
            return;
        }

        const float minScale = 0.000000001f;
        const float maxScale = 10000f;

        float uniformScale = (float)desiredDiameterInUnits;
        float clamped = Mathf.Clamp(uniformScale, minScale, maxScale);

        if (!Mathf.Approximately(clamped, uniformScale))
        {
            Debug.LogWarning($"UniverseManager: clamped visual scale for '{target.name}' from {uniformScale} to {clamped}");
        }

        bool hasRenderer = target.GetComponentInChildren<Renderer>(true) != null;
        if (!hasRenderer)
        {
            Debug.LogWarning($"UniverseManager: skipped scaling '{target.name}' because no Renderer found in children.");
            return;
        }

        double desiredWorldRadius = realRadiusMeters / metersPerVisual;

        MeshFilter[] meshFilters = target.GetComponentsInChildren<MeshFilter>(true);

        bool appliedMeshScaling = false;

        foreach (var meshFilter in meshFilters)
        {
            if (meshFilter == null || meshFilter.sharedMesh == null)
                continue;

            Transform t = meshFilter.transform;

            Bounds meshBounds = meshFilter.sharedMesh.bounds;

            float meshRadiusLocal = Mathf.Max(
                meshBounds.extents.x,
                Mathf.Max(meshBounds.extents.y, meshBounds.extents.z)
            );

            if (meshRadiusLocal <= 0f)
                continue;

            Vector3 lossy = t.lossyScale;
            float currentWorldScale = Mathf.Max(lossy.x, Mathf.Max(lossy.y, lossy.z));
            float currentWorldRadius = meshRadiusLocal * currentWorldScale;

            if (currentWorldRadius <= 0f)
                continue;

            float scaleMultiplier = (float)(desiredWorldRadius / currentWorldRadius);
            float finalMultiplier = Mathf.Clamp(scaleMultiplier, minScale, maxScale);

            if (!Mathf.Approximately(finalMultiplier, 1f))
            {
                t.localScale *= finalMultiplier;

                Debug.Log($"UniverseManager: scaled mesh '{t.name}' by {finalMultiplier}, desiredWorldRadius={desiredWorldRadius}");
            }

            appliedMeshScaling = true;
        }

        if (appliedMeshScaling)
            return;

        Vector3 fallbackScale = Vector3.one * clamped;

        if (!Approximately(target.localScale, fallbackScale))
        {
            target.localScale = fallbackScale;
            Debug.Log($"UniverseManager: fallback scaled '{target.name}' to {fallbackScale}");
        }
    }

    private static bool Approximately(Vector3 left, Vector3 right)
    {
        return
            Mathf.Approximately(left.x, right.x) &&
            Mathf.Approximately(left.y, right.y) &&
            Mathf.Approximately(left.z, right.z);
    }

    private double GetMetersPerVisualUnit()
    {
        double distanceMultiplier = visualDistanceMultiplier <= 0d ? 1d : visualDistanceMultiplier;
        return GetUnityScale() / distanceMultiplier;
    }

    private double GetUnityScale()
    {
        return metersPerUnityUnit <= 0d ? 1d : metersPerUnityUnit;
    }
}

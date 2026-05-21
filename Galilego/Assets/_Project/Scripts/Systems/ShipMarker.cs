using UnityEngine;

public class ShipMarker : MonoBehaviour
{
    private LineRenderer bodyLine;
    private LineRenderer panelLine;
    private LineRenderer antLine;

    public void Initialize(Material material, int layer)
    {
        float s = 0.4f;
        float bw = s * 0.4f;
        float bh = s * 0.7f;
        float ps = s * 2.2f;
        float ah = s * 0.5f;
        Color gray = new Color(0.55f, 0.55f, 0.6f, 0.9f);

        gameObject.layer = layer;

        GameObject bodyObj = CreateLineChild("Body", layer);
        bodyLine = bodyObj.AddComponent<LineRenderer>();
        bodyLine.positionCount = 4;
        bodyLine.SetPositions(new Vector3[] {
            new Vector3(-bw, 0, bh),
            new Vector3(bw, 0, bh),
            new Vector3(bw, 0, -bh),
            new Vector3(-bw, 0, -bh)
        });
        ConfigureLine(bodyLine, material, gray, 0.06f, true);

        GameObject panelObj = CreateLineChild("Panels", layer);
        panelLine = panelObj.AddComponent<LineRenderer>();
        panelLine.positionCount = 2;
        panelLine.SetPositions(new Vector3[] {
            new Vector3(-ps, 0, 0),
            new Vector3(ps, 0, 0)
        });
        ConfigureLine(panelLine, material, gray, 0.03f, false);

        GameObject antObj = CreateLineChild("Antenna", layer);
        antLine = antObj.AddComponent<LineRenderer>();
        antLine.positionCount = 2;
        antLine.SetPositions(new Vector3[] {
            new Vector3(0, 0, bh),
            new Vector3(0, 0, bh + ah)
        });
        ConfigureLine(antLine, material, gray, 0.02f, false);
    }

    public void SetPositionAndVisibility(Vector3 localPosition, bool visible)
    {
        if (gameObject.activeSelf != visible)
            gameObject.SetActive(visible);
        if (visible)
            transform.localPosition = localPosition;
    }

    private GameObject CreateLineChild(string name, int layer)
    {
        GameObject child = new GameObject(name);
        child.transform.SetParent(transform, false);
        child.layer = layer;
        return child;
    }

    private static void ConfigureLine(LineRenderer lr, Material material, Color color, float width, bool loop)
    {
        lr.useWorldSpace = false;
        lr.loop = loop;
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = color;
        lr.endColor = color;
        lr.alignment = LineAlignment.View;
        lr.numCornerVertices = 6;
        lr.numCapVertices = 6;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
        lr.textureMode = LineTextureMode.Stretch;
        lr.sharedMaterial = material;
    }
}

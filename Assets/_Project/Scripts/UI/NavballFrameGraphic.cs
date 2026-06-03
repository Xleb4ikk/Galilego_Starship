using UnityEngine;
using UnityEngine.UI;

namespace Galilego.UI
{
    public sealed class NavballFrameGraphic : Graphic
    {
        [SerializeField, Range(48, 256)] private int segments = 160;
        [SerializeField, Range(0.35f, 0.49f)] private float innerRadius = 0.425f;
        [SerializeField, Range(0.45f, 0.55f)] private float outerRadius = 0.5f;
        [SerializeField] private Color outerShadow = new Color(0.13f, 0.13f, 0.13f, 0.95f);
        [SerializeField] private Color outerHighlight = new Color(0.72f, 0.72f, 0.68f, 1f);
        [SerializeField] private Color innerBezel = new Color(0.42f, 0.42f, 0.40f, 1f);
        [SerializeField] private Color innerShadow = new Color(0.06f, 0.06f, 0.06f, 0.95f);
        [SerializeField] private Color markerColor = new Color(0.88f, 0.88f, 0.82f, 1f);

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();

            Rect rect = GetPixelAdjustedRect();
            float radiusScale = Mathf.Min(rect.width, rect.height);
            if (radiusScale <= 0f)
            {
                return;
            }

            Vector2 center = rect.center;
            float inner = radiusScale * innerRadius;
            float outer = radiusScale * outerRadius;

            AddRing(vh, center, inner, outer, innerBezel, outerHighlight);
            AddRing(vh, center, inner * 0.965f, inner * 1.03f, innerShadow, innerBezel);
            AddRing(vh, center, outer * 0.985f, outer * 1.015f, outerHighlight, outerShadow);
            AddTopIndex(vh, center, radiusScale);
            AddSideGrip(vh, center, radiusScale, -1f);
            AddSideGrip(vh, center, radiusScale, 1f);
        }

        private void AddRing(VertexHelper vh, Vector2 center, float inner, float outer, Color innerColor, Color outerColor)
        {
            int segmentCount = Mathf.Clamp(segments, 48, 256);

            for (int i = 0; i < segmentCount; i++)
            {
                float angle0 = (i / (float)segmentCount) * Mathf.PI * 2f;
                float angle1 = ((i + 1) / (float)segmentCount) * Mathf.PI * 2f;
                Vector2 inner0 = center + FromPolar(inner, angle0);
                Vector2 outer0 = center + FromPolar(outer, angle0);
                Vector2 outer1 = center + FromPolar(outer, angle1);
                Vector2 inner1 = center + FromPolar(inner, angle1);

                int start = vh.currentVertCount;
                vh.AddVert(inner0, innerColor, Vector2.zero);
                vh.AddVert(outer0, outerColor, Vector2.zero);
                vh.AddVert(outer1, outerColor, Vector2.zero);
                vh.AddVert(inner1, innerColor, Vector2.zero);
                vh.AddTriangle(start, start + 1, start + 2);
                vh.AddTriangle(start + 2, start + 3, start);
            }
        }

        private void AddTopIndex(VertexHelper vh, Vector2 center, float radius)
        {
            float width = radius * 0.085f;
            float top = radius * 0.505f;
            float bottom = radius * 0.385f;

            AddQuad(
                vh,
                center + new Vector2(-width, bottom),
                center + new Vector2(width, bottom),
                center + new Vector2(width * 0.55f, top),
                center + new Vector2(-width * 0.55f, top),
                markerColor);
        }

        private void AddSideGrip(VertexHelper vh, Vector2 center, float radius, float side)
        {
            float x0 = side * radius * 0.485f;
            float x1 = side * radius * 0.545f;
            float y0 = -radius * 0.19f;
            float y1 = radius * 0.19f;

            AddQuad(
                vh,
                center + new Vector2(x0, y0),
                center + new Vector2(x1, y0),
                center + new Vector2(x1, y1),
                center + new Vector2(x0, y1),
                outerHighlight);
        }

        private static void AddQuad(VertexHelper vh, Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color quadColor)
        {
            int start = vh.currentVertCount;
            vh.AddVert(a, quadColor, Vector2.zero);
            vh.AddVert(b, quadColor, Vector2.zero);
            vh.AddVert(c, quadColor, Vector2.zero);
            vh.AddVert(d, quadColor, Vector2.zero);
            vh.AddTriangle(start, start + 1, start + 2);
            vh.AddTriangle(start + 2, start + 3, start);
        }

        private static Vector2 FromPolar(float radius, float angle)
        {
            return new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
        }

#if UNITY_EDITOR
        protected override void OnValidate()
        {
            base.OnValidate();
            SetVerticesDirty();
        }
#endif
    }
}

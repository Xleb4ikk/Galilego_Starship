using UnityEngine;
using UnityEngine.UI;

namespace Galilego.Physics
{
    public sealed class CircularRawImage : RawImage
    {
        [SerializeField, Min(12)] private int segments = 96;

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();

            Rect rect = GetPixelAdjustedRect();
            float radius = Mathf.Min(rect.width, rect.height) * 0.5f;
            Vector2 center = rect.center;
            Color32 color32 = color;
            Rect uv = uvRect;

            vh.AddVert(center, color32, uv.center);

            int segmentCount = Mathf.Max(12, segments);
            for (int i = 0; i <= segmentCount; i++)
            {
                float angle = (i / (float)segmentCount) * Mathf.PI * 2f;
                Vector2 direction = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
                Vector2 position = center + direction * radius;
                Vector2 uvPosition = new Vector2(
                    uv.x + ((direction.x + 1f) * 0.5f * uv.width),
                    uv.y + ((direction.y + 1f) * 0.5f * uv.height));

                vh.AddVert(position, color32, uvPosition);
            }

            for (int i = 1; i <= segmentCount; i++)
            {
                vh.AddTriangle(0, i, i + 1);
            }
        }

#if UNITY_EDITOR
        protected override void OnValidate()
        {
            base.OnValidate();
            segments = Mathf.Max(12, segments);
            SetVerticesDirty();
        }
#endif
    }
}

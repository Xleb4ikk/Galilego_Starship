using UnityEngine;
using UnityEngine.UI;

namespace Galilego.UI
{
    public sealed class NavballSphereGraphic : RawImage
    {
        private const float BasisChangeEpsilon = 1e-10f;

        [SerializeField, Range(12, 96)] private int rings = 56;
        [SerializeField, Range(48, 192)] private int segments = 144;
        [SerializeField, Range(0.75f, 1f)] private float sphereFill = 0.93f;
        [SerializeField, Range(0.25f, 1f)] private float edgeBrightness = 0.56f;
        [SerializeField, Range(0.75f, 1.3f)] private float centerBrightness = 1.08f;
        [SerializeField, Range(0.25f, 1.5f)] private float limbPower = 0.55f;
        [SerializeField] private Vector3 lightDirection = new Vector3(-0.35f, 0.45f, 0.82f);
        [SerializeField, Range(0f, 0.35f)] private float lightStrength = 0.12f;

        [SerializeField] private Vector3 radialOutInShip = Vector3.up;
        [SerializeField] private Vector3 northInShip = Vector3.forward;
        [SerializeField] private Vector3 eastInShip = Vector3.right;

        public void SetReferenceBasis(Vector3 radialOut, Vector3 north, Vector3 east)
        {
            if (!TryNormalize(radialOut, out radialOut) ||
                !TryNormalize(north, out north) ||
                !TryNormalize(east, out east))
            {
                return;
            }

            if ((radialOutInShip - radialOut).sqrMagnitude < BasisChangeEpsilon &&
                (northInShip - north).sqrMagnitude < BasisChangeEpsilon &&
                (eastInShip - east).sqrMagnitude < BasisChangeEpsilon)
            {
                return;
            }

            radialOutInShip = radialOut;
            northInShip = north;
            eastInShip = east;
            SetVerticesDirty();
        }

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();

            Rect rect = GetPixelAdjustedRect();
            float radius = Mathf.Min(rect.width, rect.height) * 0.5f * sphereFill;
            if (radius <= 0f)
            {
                return;
            }

            int ringCount = Mathf.Clamp(rings, 12, 96);
            int segmentCount = Mathf.Clamp(segments, 48, 192);
            Vector2 center = rect.center;
            Vector3 light = lightDirection.sqrMagnitude > 0.0001f ? lightDirection.normalized : Vector3.forward;

            for (int ring = 0; ring < ringCount; ring++)
            {
                float innerRadius = ring / (float)ringCount;
                float outerRadius = (ring + 1) / (float)ringCount;

                for (int segment = 0; segment < segmentCount; segment++)
                {
                    float angle0 = (segment / (float)segmentCount) * Mathf.PI * 2f;
                    float angle1 = ((segment + 1) / (float)segmentCount) * Mathf.PI * 2f;

                    AddSphereQuad(vh, center, radius, innerRadius, outerRadius, angle0, angle1, light);
                }
            }
        }

        private void AddSphereQuad(
            VertexHelper vh,
            Vector2 center,
            float radius,
            float innerRadius,
            float outerRadius,
            float angle0,
            float angle1,
            Vector3 light)
        {
            Vector2 p0 = FromPolar(innerRadius, angle0);
            Vector2 p1 = FromPolar(outerRadius, angle0);
            Vector2 p2 = FromPolar(outerRadius, angle1);
            Vector2 p3 = FromPolar(innerRadius, angle1);

            Vector2 uv0 = SampleUv(p0);
            Vector2 uv1 = SampleUv(p1);
            Vector2 uv2 = SampleUv(p2);
            Vector2 uv3 = SampleUv(p3);
            NormalizeSeam(ref uv0, ref uv1, ref uv2, ref uv3);

            int startIndex = vh.currentVertCount;
            vh.AddVert(center + (p0 * radius), VertexColor(p0, light), uv0);
            vh.AddVert(center + (p1 * radius), VertexColor(p1, light), uv1);
            vh.AddVert(center + (p2 * radius), VertexColor(p2, light), uv2);
            vh.AddVert(center + (p3 * radius), VertexColor(p3, light), uv3);

            vh.AddTriangle(startIndex, startIndex + 1, startIndex + 2);
            vh.AddTriangle(startIndex + 2, startIndex + 3, startIndex);
        }

        private Vector2 SampleUv(Vector2 projectedPoint)
        {
            Vector3 direction = ProjectToSphere(projectedPoint);

            float heading = Mathf.Atan2(Vector3.Dot(direction, eastInShip), Vector3.Dot(direction, northInShip));
            float pitch = Mathf.Asin(Mathf.Clamp(Vector3.Dot(direction, radialOutInShip), -1f, 1f));

            float u = Mathf.Repeat(0.5f + (heading / (Mathf.PI * 2f)), 1f);
            float v = Mathf.Clamp01(0.5f + (pitch / Mathf.PI));

            Rect uv = uvRect;
            return new Vector2(uv.x + (u * uv.width), uv.y + (v * uv.height));
        }

        private Color32 VertexColor(Vector2 projectedPoint, Vector3 light)
        {
            Vector3 direction = ProjectToSphere(projectedPoint);
            float limb = Mathf.Clamp01(direction.z);
            float shade = Mathf.Lerp(edgeBrightness, centerBrightness, Mathf.Pow(limb, limbPower));
            shade += Mathf.Clamp01(Vector3.Dot(direction, light)) * lightStrength;
            shade = Mathf.Clamp(shade, 0f, 1.35f);

            Color baseColor = color;
            return new Color(
                baseColor.r * shade,
                baseColor.g * shade,
                baseColor.b * shade,
                baseColor.a);
        }

        private static Vector2 FromPolar(float radius, float angle)
        {
            return new Vector2(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
        }

        private static Vector3 ProjectToSphere(Vector2 projectedPoint)
        {
            float x = Mathf.Clamp(projectedPoint.x, -1f, 1f);
            float y = Mathf.Clamp(projectedPoint.y, -1f, 1f);
            float z = Mathf.Sqrt(Mathf.Max(0f, 1f - (x * x) - (y * y)));
            return new Vector3(x, y, z).normalized;
        }

        private static void NormalizeSeam(ref Vector2 uv0, ref Vector2 uv1, ref Vector2 uv2, ref Vector2 uv3)
        {
            float reference = uv0.x;
            uv1.x = ClosestWrappedU(uv1.x, reference);
            uv2.x = ClosestWrappedU(uv2.x, reference);
            uv3.x = ClosestWrappedU(uv3.x, reference);
        }

        private static float ClosestWrappedU(float u, float reference)
        {
            while (u - reference > 0.5f)
            {
                u -= 1f;
            }

            while (reference - u > 0.5f)
            {
                u += 1f;
            }

            return u;
        }

        private static bool TryNormalize(Vector3 value, out Vector3 normalized)
        {
            if (value.sqrMagnitude <= 0.000001f)
            {
                normalized = Vector3.zero;
                return false;
            }

            normalized = value.normalized;
            return true;
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

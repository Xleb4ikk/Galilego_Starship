using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Galilego.Core;
using Galilego.Simulation;
using Galilego.Universe;
using Galilego.UI;

namespace Galilego.Gameplay
{
    public sealed class ApsisMarkerSystem : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private Camera referenceCamera;
        [SerializeField] private OrbitTooltip tooltip;

        [Header("Sprites")]
        [SerializeField] private Sprite periapsisSprite;
        [SerializeField] private Sprite apoapsisSprite;
        [SerializeField] private Sprite maneuverPeriapsisSprite;
        [SerializeField] private Sprite maneuverApoapsisSprite;

        [Header("Sizes")]
        [SerializeField] private float markerSizePixels = 32f;
        [SerializeField] private float hoverThresholdPixels = 40f;  // Increased to cover full marker area

        [Header("Analytical System")]
        [Tooltip("Use analytical apsis calculation system instead of trajectory-point search")]
        [SerializeField] private bool useAnalyticalSystem = true;

        private SpriteRenderer periapsisMarker;
        private SpriteRenderer apoapsisMarker;
        private SpriteRenderer maneuverPeMarker;
        private SpriteRenderer maneuverApMarker;

        // Object pool for analytical system markers
        private List<GameObject> markerPool = new List<GameObject>();
        private const int INITIAL_POOL_SIZE = 8;

        private bool isOrbitMapActive;

        private ApsisMarkerData periapsisData;
        private ApsisMarkerData apoapsisData;

        private readonly List<ApsisMarkerData> markerDataList = new List<ApsisMarkerData>();
        public IReadOnlyList<ApsisMarkerData> MarkerData => markerDataList;

        // ─── Trajectory-point-based apsis data ────────────────────────────────
        private Vector3[] lastPositions;
        private double[] lastTimes;
        private int lastCount;
        private double lastBodyRadius;
        private Transform lastTrajectoryRoot;
        private bool hasValidTrajectoryData;

        private int peIndex = -1;
        private int apIndex = -1;
        private Vector3 peWorldPosition;
        private Vector3 apWorldPosition;
        private double peAltitude;
        private double apAltitude;
        private double peTime;
        private double apTime;

        private bool markersCreated;

        private void Awake()
        {
            ResolveReferences();
            EnsureMarkers();
            InitializeMarkerPool();
            SubscribeToEvents();
        }

        private void OnDestroy()
        {
            UnsubscribeFromEvents();
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();
            if (referenceCamera == null)
                referenceCamera = Camera.main;
            if (tooltip == null)
                tooltip = FindAnyObjectByType<OrbitTooltip>();
        }

        private void SubscribeToEvents()
        {
            if (universeManager != null)
                universeManager.ActiveReferenceFrameChanged += OnReferenceFrameChanged;
        }

        private void UnsubscribeFromEvents()
        {
            if (universeManager != null)
                universeManager.ActiveReferenceFrameChanged -= OnReferenceFrameChanged;
        }

        private void OnReferenceFrameChanged(ReferenceFrameTarget target)
        {
            hasValidTrajectoryData = false;
        }

        private void LateUpdate()
        {
            if (universeManager == null) { return; }
            if (tooltip == null) tooltip = FindAnyObjectByType<OrbitTooltip>();

            bool shouldBeActive = universeManager.CameraMode == SpaceCameraMode.OrbitMap;
            if (shouldBeActive != isOrbitMapActive)
            {
                isOrbitMapActive = shouldBeActive;
                if (!useAnalyticalSystem)
                {
                    SetMarkersActive(isOrbitMapActive);
                }
            }

            if (!isOrbitMapActive) return;

            // ── Analytical system: только hover, маркеры управляются через UpdateApsisMarkers() ──
            if (useAnalyticalSystem)
            {
                CheckHover();
                return;
            }

            // ── Legacy system: позиции маркеров применяются в OnTrajectoryUpdated() ──
            if (hasValidTrajectoryData && periapsisMarker != null && apoapsisMarker != null)
            {
                UpdateMarkerScale();
            }
            else
            {
                SetMarkersActive(false);
                markerDataList.Clear();
            }

            CheckHover();
        }

        public void OnTrajectoryUpdated(Vector3[] positions, double[] times, int count,
            double bodyRadius, Transform trajectoryRoot, int startIndex = 0, int visibleCount = -1)
        {
            // If analytical system is enabled, ignore legacy trajectory updates
            if (useAnalyticalSystem)
            {
                return;
            }
            
            lastPositions = positions;
            lastTimes = times;
            lastCount = count;
            lastBodyRadius = bodyRadius;
            lastTrajectoryRoot = trajectoryRoot;
            peIndex = -1;
            apIndex = -1;

            // Минимальная проверка целостности данных (нужны хотя бы 2 точки во всём массиве)
            if (count < 2 || startIndex >= count)
            {
                hasValidTrajectoryData = false;
                SetMarkersActive(false);
                return;
            }

            int visCount = visibleCount >= 0 ? visibleCount : count - startIndex;

            // ── Поиск Pe/Ap по ВСЕМУ массиву (игнорируем clipping) ──────────
            float peDistSq = float.MaxValue;
            float apDistSq = float.MinValue;

            // Основной поиск: локальные экстремумы
            for (int i = 1; i < count - 1; i++)
            {
                float prev = positions[i - 1].sqrMagnitude;
                float cur = positions[i].sqrMagnitude;
                float next = positions[i + 1].sqrMagnitude;

                if (cur < prev && cur < next && cur < peDistSq)
                {
                    peDistSq = cur;
                    peIndex = i;
                }

                if (cur > prev && cur > next && cur > apDistSq)
                {
                    apDistSq = cur;
                    apIndex = i;
                }
            }

            // Edge-случаи: Pe/Ap на первой точке массива
            if (peIndex < 0 && count >= 2 && positions[0].sqrMagnitude < positions[1].sqrMagnitude)
                { peIndex = 0; peDistSq = positions[0].sqrMagnitude; }
            if (apIndex < 0 && count >= 2 && positions[0].sqrMagnitude > positions[1].sqrMagnitude)
                { apIndex = 0; apDistSq = positions[0].sqrMagnitude; }

            // Edge-случай: Pe на последней точке массива (траектория нисходящая к Pe)
            int lastIdx = count - 1;
            if (peIndex < 0 && lastIdx >= 1 && positions[lastIdx].sqrMagnitude < positions[lastIdx - 1].sqrMagnitude)
                { peIndex = lastIdx; peDistSq = positions[lastIdx].sqrMagnitude; }

            // ── Применяем clipping к отображению ─────────────────────────
            // Если видимых точек меньше 2 — линия траектории не рисуется,
            // но маркеры всё равно показываем (данные по полному массиву есть).
            int visibleEnd = startIndex + visCount - 1;

            // Если Pe/Ap найден, но ДО startIndex → мы прошли эту точку,
            // показываем маркер на первой видимой точке (текущая позиция)
            if (peIndex >= 0 && peIndex < startIndex && startIndex < count)
                { peIndex = startIndex; peDistSq = positions[startIndex].sqrMagnitude; }
            if (apIndex >= 0 && apIndex < startIndex && startIndex < count)
                { apIndex = startIndex; apDistSq = positions[startIndex].sqrMagnitude; }

            // Если Pe/Ap найден, но ПОСЛЕ visibleEnd → скрываем
            if (peIndex >= 0 && peIndex > visibleEnd) peIndex = -1;
            if (apIndex >= 0 && apIndex > visibleEnd) apIndex = -1;

            bool showPe = peIndex >= 0;
            bool showAp = apIndex >= 0;

            if (showPe)
            {
                peWorldPosition = trajectoryRoot.TransformPoint(positions[peIndex]);
                peAltitude = Math.Sqrt(peDistSq) - bodyRadius;
                peTime = times != null && peIndex < times.Length ? times[peIndex] : double.NaN;
            }

            if (showAp)
            {
                apWorldPosition = trajectoryRoot.TransformPoint(positions[apIndex]);
                apAltitude = Math.Sqrt(apDistSq) - bodyRadius;
                apTime = times != null && apIndex < times.Length ? times[apIndex] : double.NaN;
            }

            hasValidTrajectoryData = true;

            // НЕМЕДЛЕННО применить позиции (без ожидания LateUpdate)
            ApplyPositionsFromData();
            UpdateMarkerScale();
        }

        private void ApplyPositionsFromData()
        {
            bool showPe = peIndex >= 0;
            bool showAp = apIndex >= 0;

            if (periapsisMarker != null)
            {
                if (showPe)
                {
                    Vector3 localPos = transform.InverseTransformPoint(peWorldPosition);
                    periapsisMarker.transform.localPosition = localPos;

                    Vector3 worldAfter = periapsisMarker.transform.position;
                    float scale = ComputeConstantScreenScale(referenceCamera, worldAfter, markerSizePixels);
                    Vector3 offsetWorld = ComputeCameraFacingOffset(periapsisMarker.gameObject, scale);
                    Vector3 offsetLocal = transform.InverseTransformDirection(offsetWorld);
                    periapsisMarker.transform.localPosition = localPos + offsetLocal;

                    periapsisMarker.gameObject.SetActive(true);
                }
                else
                {
                    periapsisMarker.gameObject.SetActive(false);
                }
            }

            if (apoapsisMarker != null)
            {
                if (showAp)
                {
                    Vector3 localPos = transform.InverseTransformPoint(apWorldPosition);
                    apoapsisMarker.transform.localPosition = localPos;

                    Vector3 worldAfter = apoapsisMarker.transform.position;
                    float scale = ComputeConstantScreenScale(referenceCamera, worldAfter, markerSizePixels);
                    Vector3 offsetWorld = ComputeCameraFacingOffset(apoapsisMarker.gameObject, scale);
                    Vector3 offsetLocal = transform.InverseTransformDirection(offsetWorld);
                    apoapsisMarker.transform.localPosition = localPos + offsetLocal;

                    apoapsisMarker.gameObject.SetActive(true);
                }
                else
                {
                    apoapsisMarker.gameObject.SetActive(false);
                }
            }

            markerDataList.Clear();

            if (showPe)
            {
                ApsisEdgeCase peEdge = peAltitude < 0 ? ApsisEdgeCase.Impact : ApsisEdgeCase.None;
                periapsisData = new ApsisMarkerData
                {
                    worldPosition = peWorldPosition,
                    type = ApsisType.Periapsis,
                    label = "Пе",
                    frameName = universeManager.ActiveReferenceFrame.ToString(),
                    isValid = true,
                    isVisible = true,
                    isManeuver = false,
                    altitudeMeters = peAltitude,
                    timeToApsisSeconds = peTime,
                    altitudeFormatted = FormatAltitude(peAltitude),
                    timeFormatted = FormatTime(peTime),
                    edgeCase = peEdge,
                    color = GetPeriapsisColor()
                };
                markerDataList.Add(periapsisData);
            }

            if (showAp)
            {
                ApsisEdgeCase apEdge = apAltitude < 0 ? ApsisEdgeCase.Impact : ApsisEdgeCase.None;
                apoapsisData = new ApsisMarkerData
                {
                    worldPosition = apWorldPosition,
                    type = ApsisType.Apoapsis,
                    label = "Ап",
                    frameName = universeManager.ActiveReferenceFrame.ToString(),
                    isValid = true,
                    isVisible = true,
                    isManeuver = false,
                    altitudeMeters = apAltitude,
                    timeToApsisSeconds = apTime,
                    altitudeFormatted = FormatAltitude(apAltitude),
                    timeFormatted = FormatTime(apTime),
                    edgeCase = apEdge,
                    color = GetApoapsisColor()
                };
                markerDataList.Add(apoapsisData);
            }
        }

        private void UpdateMarkerScale()
        {
            if (referenceCamera == null) return;

            ApplySpriteScale(periapsisMarker, markerSizePixels);
            ApplySpriteScale(apoapsisMarker, markerSizePixels);

            if (maneuverPeMarker != null && maneuverPeMarker.gameObject.activeSelf)
                ApplySpriteScale(maneuverPeMarker, markerSizePixels);
            if (maneuverApMarker != null && maneuverApMarker.gameObject.activeSelf)
                ApplySpriteScale(maneuverApMarker, markerSizePixels);
        }

        private void ApplySpriteScale(SpriteRenderer sr, float targetPixels)
        {
            if (sr == null || !sr.gameObject.activeSelf || sr.sprite == null) return;

            float nativeHeight = Mathf.Max(0.001f, sr.sprite.bounds.size.y);
            float baseScale = ComputeConstantScreenScale(referenceCamera, sr.transform.position, targetPixels);
            sr.transform.localScale = Vector3.one * (baseScale / nativeHeight);
        }

        private void CheckHover()
        {
            if (referenceCamera == null || tooltip == null)
            {
                if (referenceCamera == null)
                    Debug.LogWarning("[ApsisMarkerSystem.CheckHover] referenceCamera is null");
                if (tooltip == null)
                    Debug.LogWarning("[ApsisMarkerSystem.CheckHover] tooltip is null");
                return;
            }

            Vector2 mousePos = Mouse.current != null
                ? Mouse.current.position.ReadValue()
                : Vector2.zero;

            float closestDist = float.MaxValue;
            ApsisMarkerData closestData = default;
            bool found = false;

            for (int i = 0; i < markerDataList.Count; i++)
            {
                ApsisMarkerData data = markerDataList[i];
                if (!data.isVisible || !data.isValid) continue;

                Vector3 screenPos = referenceCamera.WorldToScreenPoint(data.worldPosition);
                if (screenPos.z < 0f) continue;

                float dist = Vector2.Distance(new Vector2(screenPos.x, screenPos.y), mousePos);
                
                if (dist < closestDist)
                {
                    closestDist = dist;
                    closestData = data;
                    found = true;
                }
            }

            if (found && closestDist < hoverThresholdPixels)
            {
                // Показываем tooltip рядом с курсором мыши
                Debug.Log($"[CheckHover] Showing tooltip: mousePos={mousePos}, markerWorldPos={closestData.worldPosition}, closestDist={closestDist:F1}px");
                tooltip.Show(closestData, mousePos);
            }
            else
            {
                tooltip.Hide();
            }
        }

        private Color GetPeriapsisColor() => new Color(0.2f, 1.0f, 0.3f, 1f);
        private Color GetApoapsisColor() => new Color(1.0f, 0.3f, 0.2f, 1f);
        private Color GetManeuverColor() => new Color(1.0f, 0.6f, 0f, 0.5f);
        private Color GetImpactColor() => new Color(1.0f, 0.2f, 0.2f, 1f);

        private void SetMarkersActive(bool active)
        {
            if (periapsisMarker != null) periapsisMarker.gameObject.SetActive(active);
            if (apoapsisMarker != null) apoapsisMarker.gameObject.SetActive(active);
            if (!active)
            {
                if (maneuverPeMarker != null) maneuverPeMarker.gameObject.SetActive(false);
                if (maneuverApMarker != null) maneuverApMarker.gameObject.SetActive(false);
            }
        }

        public void ForceUpdate()
        {
            hasValidTrajectoryData = false;
        }

        private void EnsureMarkers()
        {
            // Guard: маркеры уже созданы
            if (periapsisMarker != null && apoapsisMarker != null)
            {
                Debug.Log("[ApsisMarkerSystem.EnsureMarkers] Маркеры уже существуют, пропускаем создание");
                return;
            }

            if (markersCreated) return;
            markersCreated = true;

            Debug.Log("[ApsisMarkerSystem.EnsureMarkers] Создаем маркеры...");

            // Загружаем PNG-спрайты из Resources
            Sprite peSprite = LoadApsisSprite("ПеЗел");
            Sprite apSprite = LoadApsisSprite("АпЗел");
            Sprite maneuverPeSprite = LoadApsisSprite("ПеФиол");
            Sprite maneuverApSprite = LoadApsisSprite("АпФиол");

            periapsisMarker = CreateMarker("Periapsis_Marker", peSprite ?? periapsisSprite);
            apoapsisMarker = CreateMarker("Apoapsis_Marker", apSprite ?? apoapsisSprite);
            maneuverPeMarker = CreateMarker("Maneuver_Pe_Marker", maneuverPeSprite ?? maneuverPeriapsisSprite);
            maneuverApMarker = CreateMarker("Maneuver_Ap_Marker", maneuverApSprite ?? maneuverApoapsisSprite);

            // НЕ создаем текстовые метки - используем только PNG-спрайты

            maneuverPeMarker.gameObject.SetActive(false);
            maneuverApMarker.gameObject.SetActive(false);

            Debug.Log("[ApsisMarkerSystem.EnsureMarkers] Маркеры созданы успешно");
        }

        private Sprite LoadApsisSprite(string spriteName)
        {
            // Попытка 1: Загрузка из Prefabs/UI/ApsisMarker
            string path = $"Prefabs/UI/ApsisMarker/{spriteName}";
            Texture2D texture = Resources.Load<Texture2D>(path);
            
            if (texture != null)
            {
                Debug.Log($"[ApsisMarkerSystem.LoadApsisSprite] Загружен спрайт '{spriteName}' из Resources: {path}");
                // Создаем спрайт из текстуры с pivot в центре для корректного hover detection
                Sprite sprite = Sprite.Create(
                    texture,
                    new Rect(0, 0, texture.width, texture.height),
                    new Vector2(0.5f, 0.5f), // pivot в центре спрайта
                    100f // pixels per unit
                );
                return sprite;
            }

            // Попытка 2: Прямая загрузка спрайта
            Sprite directSprite = Resources.Load<Sprite>(path);
            if (directSprite != null)
            {
                Debug.Log($"[ApsisMarkerSystem.LoadApsisSprite] Загружен спрайт '{spriteName}' напрямую из Resources");
                return directSprite;
            }

            Debug.LogWarning($"[ApsisMarkerSystem.LoadApsisSprite] ⚠️ Не удалось загрузить спрайт '{spriteName}' из Resources/{path}");
            return null;
        }

        private SpriteRenderer CreateMarker(string name, Sprite sprite)
        {
            GameObject obj = new GameObject(name);
            obj.transform.SetParent(transform, false);
            obj.transform.localPosition = Vector3.zero;

            SpriteRenderer sr = obj.AddComponent<SpriteRenderer>();
            sr.sprite = sprite != null ? sprite : GenerateFallbackSprite(name);
            sr.sortingOrder = 100;

            obj.AddComponent<BillboardBehaviour>();

            return sr;
        }

        private static Sprite GenerateFallbackSprite(string name)
        {
            int size = 64;
            Texture2D tex = new Texture2D(size, size, TextureFormat.RGBA32, false);
            Color color = name.Contains("Periapsis") ? new Color(0.2f, 1f, 0.3f) :
                          name.Contains("Apoapsis") ? new Color(1f, 0.3f, 0.2f) :
                          new Color(1f, 0.6f, 0f);

            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    float dx = x - size * 0.5f, dy = y - size * 0.5f;
                    bool inCircle = (dx * dx + dy * dy) < (size * 0.25f * size * 0.25f);
                    bool inArrow = x > size * 0.4f && x < size * 0.6f && y < size * 0.3f;
                    tex.SetPixel(x, y, (inCircle || inArrow) ? color : Color.clear);
                }
            }
            tex.Apply();
            return Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 64f);
        }

        private float ComputeConstantScreenScale(Camera cam, Vector3 worldPosition, float targetPixels)
        {
            float distance = Vector3.Distance(cam.transform.position, worldPosition);
            distance = Mathf.Max(distance, cam.nearClipPlane + 0.01f);
            float frustumHeight = 2f * distance * Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad);
            return frustumHeight * targetPixels / Mathf.Max(1f, cam.pixelHeight);
        }

        /// <summary>
        /// Computes a camera-facing offset to position the marker sprite above the orbit line.
        /// The offset is proportional to the sprite scale to maintain consistent visual appearance.
        /// Since pivot is at center (0.5, 0.5), we need to offset by half sprite height plus additional spacing.
        /// </summary>
        /// <param name="markerRoot">GameObject with BillboardBehaviour that provides camera-facing rotation</param>
        /// <param name="currentScale">Current scale factor from ComputeConstantScreenScale</param>
        /// <returns>Vector3 offset in world space, pointing "up" in camera-facing direction</returns>
        private Vector3 ComputeCameraFacingOffset(GameObject markerRoot, float currentScale)
        {
            if (markerRoot == null || referenceCamera == null) return Vector3.zero;

            // Get billboard's current rotation (facing camera)
            Transform markerTransform = markerRoot.transform;

            // Extract camera-facing "up" direction: the billboard's local Y-axis in world space
            Vector3 cameraFacingUp = markerTransform.up;

            // Calculate offset distance proportional to sprite size
            // spriteHeight = 0.45f (approximate sprite bounds height)
            // offsetFactor controls how far above the orbit line the marker appears
            float spriteHeight = 0.45f;
            float offsetFactor = 15f; // Position marker above orbit line (1.0 = full sprite height)
            float totalOffset = currentScale * spriteHeight * offsetFactor;

            // Return offset vector: cameraFacingUp * offsetDistance
            return cameraFacingUp * totalOffset;
        }

        public static string FormatAltitude(double meters)
        {
            if (double.IsNaN(meters) || double.IsInfinity(meters))
                return "∞";

            double abs = Math.Abs(meters);
            if (abs >= 1e9)
                return $"{meters / 1e9:F2} Gm";
            if (abs >= 1e6)
                return $"{meters / 1e6:F2} Mm";
            if (abs >= 1e3)
                return $"{meters / 1e3:F2} km";
            return $"{meters:F0} m";
        }

        public static string FormatTime(double seconds)
        {
            if (double.IsNaN(seconds) || double.IsInfinity(seconds))
                return "∞";
            if (seconds < 0.1)
                return "Now";
            if (seconds > 365.0 * 86400.0)
                return "> 1 year";

            if (seconds >= 86400.0)
            {
                int days = (int)(seconds / 86400.0);
                int hours = (int)((seconds % 86400.0) / 3600.0);
                int minutes = (int)((seconds % 3600.0) / 60.0);
                return $"{days}d {hours}h {minutes}m";
            }
            if (seconds >= 3600.0)
            {
                int hours = (int)(seconds / 3600.0);
                int minutes = (int)((seconds % 3600.0) / 60.0);
                int secs = (int)(seconds % 60.0);
                return $"{hours}h {minutes}m {secs}s";
            }
            if (seconds >= 60.0)
            {
                int minutes = (int)(seconds / 60.0);
                int secs = (int)(seconds % 60.0);
                return $"{minutes}m {secs:D2}s";
            }
            return $"{(int)seconds}s";
        }

        #region Analytical System API

        /// <summary>
        /// Initialize the marker object pool for the analytical system.
        /// Pre-allocates markers to avoid runtime instantiation.
        /// </summary>
        private void InitializeMarkerPool()
        {
            // Safety check: ensure we're in a valid state
            if (transform == null)
            {
                Debug.LogError("[ApsisMarkerSystem] Cannot initialize marker pool: transform is null");
                return;
            }

            Debug.Log($"[ApsisMarkerSystem] Initializing marker pool with {INITIAL_POOL_SIZE} markers");
            
            for (int i = 0; i < INITIAL_POOL_SIZE; i++)
            {
                GameObject marker = CreatePooledMarker();
                if (marker == null)
                {
                    Debug.LogWarning($"[ApsisMarkerSystem] Failed to create initial marker {i}/{INITIAL_POOL_SIZE}");
                    continue;
                }
                marker.SetActive(false);
                markerPool.Add(marker);
            }
            
            Debug.Log($"[ApsisMarkerSystem] Marker pool initialized with {markerPool.Count} markers");
        }

        /// <summary>
        /// Creates a new marker GameObject for the pool.
        /// </summary>
        private GameObject CreatePooledMarker()
        {
            // Safety check: ensure parent transform is valid
            if (transform == null)
            {
                Debug.LogError("[ApsisMarkerSystem] Cannot create pooled marker: transform is null");
                return null;
            }

            GameObject markerObj = new GameObject($"ApsisMarker_Pooled_{markerPool.Count}");
            
            // Set parent with worldPositionStays=false to avoid transform issues
            markerObj.transform.SetParent(transform, false);
            markerObj.transform.localPosition = Vector3.zero;
            markerObj.transform.localRotation = Quaternion.identity;
            markerObj.transform.localScale = Vector3.one;
            markerObj.layer = gameObject.layer;

            // Add SpriteRenderer
            var spriteRenderer = markerObj.AddComponent<SpriteRenderer>();
            spriteRenderer.sprite = periapsisSprite; // Default sprite
            spriteRenderer.sortingOrder = 100;

            // Add BillboardBehaviour for camera-facing
            markerObj.AddComponent<BillboardBehaviour>();
            // BillboardBehaviour will auto-find Camera.main in its Start()

            return markerObj;
        }

        /// <summary>
        /// Expands the marker pool if more markers are needed.
        /// </summary>
        private void ExpandMarkerPool(int targetSize)
        {
            // Safety check: don't expand during scene transitions or if transform is invalid
            if (transform == null || !gameObject.activeInHierarchy)
                return;

            int initialCount = markerPool.Count;
            while (markerPool.Count < targetSize)
            {
                GameObject marker = CreatePooledMarker();
                if (marker == null)
                {
                    Debug.LogWarning($"[ApsisMarkerSystem] Failed to create pooled marker at index {markerPool.Count}");
                    break;
                }
                marker.SetActive(false);
                markerPool.Add(marker);
            }
            
            if (markerPool.Count > initialCount)
            {
                Debug.Log($"[ApsisMarkerSystem] Expanded marker pool from {initialCount} to {markerPool.Count}");
            }
        }

        /// <summary>
        /// Updates apsis markers using the analytical calculation system.
        /// This is the new API that replaces trajectory-point-based detection.
        /// </summary>
        /// <param name="apsisDataList">List of apsis data from ApsisCalculator.</param>
        public void UpdateApsisMarkers(List<ApsisData> apsisDataList)
        {
            if (!useAnalyticalSystem)
            {
                // Analytical system disabled - do nothing
                return;
            }

            // Safety check: ensure we're in a valid state
            if (transform == null || !gameObject.activeInHierarchy)
            {
                return;
            }

            if (apsisDataList == null)
            {
                // Clear all markers
                DeactivateAllPooledMarkers();
                markerDataList.Clear();
                return;
            }

            // Count visible apsides
            int visibleCount = 0;
            foreach (var apsisData in apsisDataList)
            {
                if (apsisData.isVisible)
                    visibleCount++;
            }

            // Ensure pool has enough markers (with safety limit)
            if (visibleCount > markerPool.Count)
            {
                // Limit maximum pool size to prevent excessive memory usage
                int maxPoolSize = 50;
                int targetSize = Mathf.Min(visibleCount, maxPoolSize);
                ExpandMarkerPool(targetSize);
            }

            // Update visible markers
            int markerIndex = 0;
            markerDataList.Clear();

            foreach (var apsisData in apsisDataList)
            {
                if (!apsisData.isVisible)
                    continue;

                if (markerIndex >= markerPool.Count)
                    break;

                GameObject markerObj = markerPool[markerIndex];
                if (markerObj == null)
                {
                    markerIndex++;
                    continue;
                }

                var spriteRenderer = markerObj.GetComponent<SpriteRenderer>();
                if (spriteRenderer == null)
                {
                    markerIndex++;
                    continue;
                }

                // Transform world position to Unity coordinates
                Vector3 unityPosition = universeManager.ToUnityPosition(apsisData.worldPosition);
                Vector3 localPosition = transform.InverseTransformPoint(unityPosition);

                // Update sprite based on type and orbit
                Sprite sprite = GetSpriteForApsis(apsisData.type, apsisData.orbitType);
                if (sprite != null)
                    spriteRenderer.sprite = sprite;

                // Compute constant screen-space scale
                float scale = ComputeConstantScreenScale(referenceCamera, unityPosition, markerSizePixels);
                markerObj.transform.localScale = Vector3.one * scale;

                // Apply camera-facing offset
                Vector3 offsetWorld = ComputeCameraFacingOffset(markerObj, scale);
                Vector3 offsetLocal = transform.InverseTransformDirection(offsetWorld);
                markerObj.transform.localPosition = localPosition + offsetLocal;

                // Activate marker
                markerObj.SetActive(true);

                // Convert to ApsisMarkerData for tooltip system
                double currentTime = universeManager != null ? universeManager.SimulationTimeSeconds : 0;
                ApsisMarkerData markerData = apsisData.ToMarkerData(universeManager, currentTime);
                
                // Update worldPosition to actual marker position (after offset) for accurate hover detection
                Vector3 actualMarkerWorldPos = markerObj.transform.position;
                Vector3 orbitPos = universeManager.ToUnityPosition(apsisData.worldPosition);
                float offsetDistance = Vector3.Distance(actualMarkerWorldPos, orbitPos);
                
                markerDataList.Add(markerData);

                markerIndex++;
            }

            // Deactivate unused markers
            for (int i = markerIndex; i < markerPool.Count; i++)
            {
                if (markerPool[i] != null)
                    markerPool[i].SetActive(false);
            }
        }

        /// <summary>
        /// Gets the appropriate sprite for an apsis based on type and orbit.
        /// </summary>
        private Sprite GetSpriteForApsis(ApsisType type, OrbitType orbitType)
        {
            if (orbitType == OrbitType.Ballistic)
            {
                return type == ApsisType.Periapsis ? periapsisSprite : apoapsisSprite;
            }
            else // Maneuver
            {
                return type == ApsisType.Periapsis ? maneuverPeriapsisSprite : maneuverApoapsisSprite;
            }
        }

        /// <summary>
        /// Deactivates all pooled markers.
        /// </summary>
        private void DeactivateAllPooledMarkers()
        {
            foreach (var marker in markerPool)
            {
                if (marker != null)
                    marker.SetActive(false);
            }
        }

        #endregion
    }
}

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;
using Galilego.Gameplay;

namespace Galilego.UI
{
    public sealed class OrbitTooltip : MonoBehaviour
    {
        [Header("Tooltip Style")]
        [SerializeField] private Color backgroundColor = new Color(0.1f, 0.1f, 0.15f, 0.9f);
        [SerializeField] private Color textColor = Color.white;
        [SerializeField] private Color warningColor = new Color(1f, 0.9f, 0f);
        [SerializeField] private int fontSize = 12;

        [Header("Behavior")]
        [SerializeField] private Vector2 mouseOffset = new Vector2(15f, -15f);

        private Canvas tooltipCanvas;
        private RectTransform tooltipRect;
        private Image backgroundImage;
        private TMP_Text headerText;
        private TMP_Text altitudeText;
        private TMP_Text timeText;
        private TMP_Text warningText;

        private Camera referenceCamera;

        private static readonly Color ImpactColor = new Color(1f, 0.9f, 0f);

        private void Awake()
        {
            EnsureCanvas();
            EnsureTooltipVisuals();
            Hide();
        }

        private void EnsureCanvas()
        {
            string canvasName = "OrbitTooltipCanvas";
            GameObject existing = GameObject.Find(canvasName);
            if (existing != null)
            {
                tooltipCanvas = existing.GetComponent<Canvas>();
                return;
            }

            GameObject canvasObj = new GameObject(canvasName, typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            tooltipCanvas = canvasObj.GetComponent<Canvas>();
            tooltipCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
            tooltipCanvas.sortingOrder = 1000;

            CanvasScaler scaler = canvasObj.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;
        }

        private void EnsureTooltipVisuals()
        {
            if (tooltipCanvas == null) return;

            Transform canvasTransform = tooltipCanvas.transform;

            GameObject panelObj = new GameObject("Tooltip_Panel", typeof(RectTransform), typeof(Image));
            panelObj.transform.SetParent(canvasTransform, false);
            tooltipRect = panelObj.GetComponent<RectTransform>();

            tooltipRect.anchorMin = new Vector2(0.5f, 0.5f);
            tooltipRect.anchorMax = new Vector2(0.5f, 0.5f);
            tooltipRect.pivot = new Vector2(0f, 1f);
            tooltipRect.sizeDelta = new Vector2(220f, 85f);

            backgroundImage = panelObj.GetComponent<Image>();
            backgroundImage.color = backgroundColor;
            backgroundImage.raycastTarget = false;

            GameObject headerObj = CreateTextChild(panelObj.transform, "Header_Text", "Periapsis", fontSize + 2, FontStyles.Bold);
            headerText = headerObj.GetComponent<TMP_Text>();
            RectTransform headerRt = headerObj.GetComponent<RectTransform>();
            headerRt.anchorMin = new Vector2(0f, 1f);
            headerRt.anchorMax = new Vector2(1f, 1f);
            headerRt.pivot = new Vector2(0.5f, 1f);
            headerRt.anchoredPosition = new Vector2(0f, -5f);
            headerRt.sizeDelta = new Vector2(-10f, 20f);

            GameObject altObj = CreateTextChild(panelObj.transform, "Altitude_Text", "Altitude: ---", fontSize);
            altitudeText = altObj.GetComponent<TMP_Text>();
            RectTransform altRt = altObj.GetComponent<RectTransform>();
            altRt.anchorMin = new Vector2(0f, 0.5f);
            altRt.anchorMax = new Vector2(1f, 0.5f);
            altRt.pivot = new Vector2(0.5f, 0.5f);
            altRt.anchoredPosition = new Vector2(0f, -2f);
            altRt.sizeDelta = new Vector2(-10f, 18f);

            GameObject timeObj = CreateTextChild(panelObj.transform, "Time_Text", "Time: ---", fontSize);
            timeText = timeObj.GetComponent<TMP_Text>();
            RectTransform timeRt = timeObj.GetComponent<RectTransform>();
            timeRt.anchorMin = new Vector2(0f, 0f);
            timeRt.anchorMax = new Vector2(1f, 0f);
            timeRt.pivot = new Vector2(0.5f, 0f);
            timeRt.anchoredPosition = new Vector2(0f, 5f);
            timeRt.sizeDelta = new Vector2(-10f, 18f);

            GameObject warnObj = CreateTextChild(panelObj.transform, "Warning_Text", "", fontSize, FontStyles.Bold);
            warningText = warnObj.GetComponent<TMP_Text>();
            RectTransform warnRt = warnObj.GetComponent<RectTransform>();
            warnRt.anchorMin = new Vector2(0f, 0f);
            warnRt.anchorMax = new Vector2(1f, 0f);
            warnRt.pivot = new Vector2(0.5f, 0f);
            warnRt.anchoredPosition = new Vector2(0f, -15f);
            warnRt.sizeDelta = new Vector2(-10f, 18f);

            Hide();
        }

        private GameObject CreateTextChild(Transform parent, string name, string defaultText, int size, FontStyles style = FontStyles.Normal)
        {
            GameObject obj = new GameObject(name, typeof(RectTransform), typeof(TextMeshProUGUI));
            obj.transform.SetParent(parent, false);

            TextMeshProUGUI tmp = obj.GetComponent<TextMeshProUGUI>();
            AssignFontAsset(tmp);
            tmp.text = defaultText;
            tmp.fontSize = size;
            tmp.color = textColor;
            tmp.fontStyle = style;
            tmp.alignment = TextAlignmentOptions.Left;
            tmp.enableWordWrapping = false;
            tmp.raycastTarget = false;

            return obj;
        }

        private static void AssignFontAsset(TextMeshProUGUI tmp)
        {
            if (TMP_Settings.instance != null && TMP_Settings.defaultFontAsset != null)
            {
                tmp.font = TMP_Settings.defaultFontAsset;
                return;
            }

            TMP_FontAsset font = Resources.Load<TMP_FontAsset>("Fonts & Materials/LiberationSans SDF");
            if (font != null)
            {
                tmp.font = font;
                return;
            }

            TMP_FontAsset[] fonts = Resources.FindObjectsOfTypeAll<TMP_FontAsset>();
            if (fonts.Length > 0)
            {
                tmp.font = fonts[0];
            }
        }

        private void OnDestroy()
        {
            if (headerText != null) headerText.enabled = false;
            if (altitudeText != null) altitudeText.enabled = false;
            if (timeText != null) timeText.enabled = false;
            if (warningText != null) warningText.enabled = false;

            if (tooltipCanvas != null)
                Destroy(tooltipCanvas.gameObject);
        }

        public void Show(ApsisMarkerData data, Vector2 mousePos)
        {
            if (tooltipCanvas == null || tooltipRect == null) return;

            referenceCamera = Camera.main;

            tooltipCanvas.gameObject.SetActive(true);
            tooltipRect.gameObject.SetActive(true);

            UpdateTooltipContent(data);
            UpdateTooltipPosition(mousePos);
        }

        public void Hide()
        {
            if (tooltipRect != null)
                tooltipRect.gameObject.SetActive(false);
        }

        private void UpdateTooltipContent(ApsisMarkerData data)
        {
            if (this == null) return;

            try
            {
                if (headerText != null && headerText.font != null)
                    headerText.text = FormatManeuverHeader(data);

                if (altitudeText != null && altitudeText.font != null)
                {
                    if (data.edgeCase == ApsisEdgeCase.BeyondSOI)
                    {
                        altitudeText.text = "Altitude: Beyond SOI";
                        altitudeText.color = warningColor;
                    }
                    else if (data.edgeCase == ApsisEdgeCase.Impact)
                    {
                        altitudeText.text = $"Altitude: Impact: {data.altitudeFormatted}";
                        altitudeText.color = ImpactColor;
                    }
                    else if (data.edgeCase == ApsisEdgeCase.Circular)
                    {
                        altitudeText.text = $"Altitude: {data.altitudeFormatted} (Circular orbit)";
                        altitudeText.color = textColor;
                    }
                    else
                    {
                        altitudeText.text = $"Altitude: {data.altitudeFormatted}";
                        altitudeText.color = textColor;
                    }
                }

                if (timeText != null && timeText.font != null)
                {
                    if (data.edgeCase == ApsisEdgeCase.Now)
                    {
                        timeText.text = "Time: Now";
                        timeText.color = warningColor;
                    }
                    else if (data.edgeCase == ApsisEdgeCase.OverOneYear)
                    {
                        timeText.text = "Time: > 1 year";
                        timeText.color = textColor;
                    }
                    else if (data.edgeCase == ApsisEdgeCase.Circular)
                    {
                        timeText.text = "Time: Circular orbit";
                        timeText.color = textColor;
                    }
                    else
                    {
                        timeText.text = $"Time: {data.timeFormatted}";
                        timeText.color = textColor;
                    }
                }

                if (warningText != null && warningText.font != null)
                {
                    if (data.edgeCase == ApsisEdgeCase.Impact)
                    {
                        warningText.text = "WARNING: Surface impact trajectory";
                        warningText.color = ImpactColor;
                        warningText.gameObject.SetActive(true);
                    }
                    else
                    {
                        warningText.gameObject.SetActive(false);
                    }
                }

                UpdateTooltipSize();
            }
            catch (System.Exception e)
            {
                Debug.LogError("[OrbitTooltip] Error updating content: " + e.Message);
            }
        }

        private void UpdateTooltipSize()
        {
            if (tooltipRect == null) return;

            float width = 220f;
            float height = 75f;

            if (warningText != null && warningText.gameObject.activeSelf)
                height += 22f;

            tooltipRect.sizeDelta = new Vector2(width, height);
        }

        private void UpdateTooltipPosition(Vector2 mouseScreenPos)
        {
            if (tooltipRect == null || tooltipCanvas == null) return;

            float tooltipWidth = tooltipRect.rect.width;
            float tooltipHeight = tooltipRect.rect.height;

            if (tooltipWidth <= 0) tooltipWidth = 220f;
            if (tooltipHeight <= 0) tooltipHeight = 85f;

            // Конвертируем экранные координаты мыши в координаты canvas
            RectTransform canvasRect = tooltipCanvas.GetComponent<RectTransform>();
            Vector2 canvasPos;
            RectTransformUtility.ScreenPointToLocalPointInRectangle(
                canvasRect, 
                mouseScreenPos, 
                tooltipCanvas.renderMode == RenderMode.ScreenSpaceOverlay ? null : tooltipCanvas.worldCamera, 
                out canvasPos
            );

            // Применяем смещение от курсора
            float offsetX = mouseOffset.x;
            float offsetY = mouseOffset.y;

            Vector2 targetPos = new Vector2(canvasPos.x + offsetX, canvasPos.y + offsetY);

            // Получаем размеры canvas
            Vector2 canvasSize = canvasRect.sizeDelta;

            // Если tooltip выходит за правый край canvas, показываем слева от курсора
            if (targetPos.x + tooltipWidth > canvasSize.x / 2f)
                targetPos.x = canvasPos.x - tooltipWidth - offsetX;

            // Если tooltip выходит за нижний край canvas, показываем сверху от курсора
            if (targetPos.y - tooltipHeight < -canvasSize.y / 2f)
                targetPos.y = canvasPos.y + tooltipHeight - offsetY;

            // Если tooltip выходит за верхний край canvas
            if (targetPos.y > canvasSize.y / 2f)
                targetPos.y = canvasSize.y / 2f - tooltipHeight;

            // Если tooltip выходит за левый край canvas
            if (targetPos.x < -canvasSize.x / 2f)
                targetPos.x = -canvasSize.x / 2f;

            tooltipRect.anchoredPosition = targetPos;
        }

        private static string FormatManeuverHeader(ApsisMarkerData data)
        {
            string type = data.type == ApsisType.Periapsis ? "Перицентр" : "Апоцентр";
            string prefix = data.isManeuver ? "После манёвра: " : "";
            string bodyName = string.IsNullOrEmpty(data.frameName) ? "Unknown" : data.frameName;
            return $"{prefix}{type} ({bodyName})";
        }
    }
}

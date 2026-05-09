using System;
using UnityEngine;
using Galilego.Physics;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace Galilego.Gameplay
{
    /// <summary>
    /// Планировщик манёвров в стиле MechJeb / Principia.
    ///
    /// Каждая ось ΔV имеет три способа редактирования:
    ///   1. Velocity-drag слайдер — тянешь от центра, значение меняется
    ///      со скоростью, пропорциональной смещению. Отпустил — фиксируется.
    ///   2. Кнопки инкрементов ±0.01 … ±1000 m/s.
    ///   3. Прямой ввод числа кликом по полю значения.
    ///
    /// Подключение: добавить компонент на любой GameObject в сцене.
    /// ManeuverEvaluator найдётся автоматически, если поле не заполнено.
    /// </summary>
    public class FlightPlanUI : MonoBehaviour
    {
        // ─── Inspector ──────────────────────────────────────────────────────────
        [Header("References")]
        [SerializeField] private ManeuverEvaluator evaluator;

        [Header("Window")]
        [SerializeField] private Rect windowRect = new Rect(16f, 16f, 410f, 640f);
        [SerializeField] private bool showWindow  = true;
        [SerializeField] private KeyCode toggleKey = KeyCode.N;

        [Header("Velocity Slider")]
        [Tooltip("Максимальная скорость изменения ΔV (m/s·s⁻¹) при полном отклонении слайдера")]
        [SerializeField] private float maxDragRate = 100f;

        // ─── Runtime ────────────────────────────────────────────────────────────
        private FlightPlan      flightPlan;
        private UniverseManager universeManager;
        private int             selectedNodeIndex;
        private Vector2         nodeListScroll;
        private Vector2         mainScroll;
        private bool            showPrediction = true;
        private bool            showSettings;

        // Direct-input state (per axis)
        private string inputStartTime = "";
        private string inputDvT = "", inputDvN = "", inputDvB = "";
        private bool   editingTime, editingT, editingN, editingB;

        // ── Velocity-drag state ─────────────────────────────────────────────────
        // Three independent channels: T=0 N=1 B=2.
        // normalizedOffset: -1 (full-left) … 0 (centre) … +1 (full-right).
        // isDragging: set when mouse pressed inside slider rect, cleared on MouseUp.
        private bool  dragActiveT, dragActiveN, dragActiveB;
        private float dragOffsetT, dragOffsetN, dragOffsetB;
        private int   sliderHotId = -1;   // which channel owns the drag: 0/1/2, -1=none

        // Last computed slider rects (needed to detect release outside rect)
        private Rect sliderRectT, sliderRectN, sliderRectB;

        // Prediction slider state
        private bool  dragActivePrediction;
        private float dragOffsetPrediction;
        private Rect  sliderRectPrediction;
        // ── Prediction cache ────────────────────────────────────────────────────
        private OrbitalElements orbitBefore = OrbitalElements.Invalid;
        private OrbitalElements orbitAfter  = OrbitalElements.Invalid;
        private bool   predictionDirty = true;
        private double lastPredTime    = -1;

        // ── GUIStyle cache ──────────────────────────────────────────────────────
        private GUIStyle styleWindow, styleHeader, styleLabel, styleLabelDim;
        private GUIStyle styleLabelGreen, styleLabelYellow, styleLabelRed;
        private GUIStyle styleButton, styleButtonSmall, styleButtonActive;
        private GUIStyle styleInputField, styleSection, styleSliderBg;
        private bool     stylesBuilt;

        private static readonly double[] DV_STEPS = { 0.01, 0.1, 1, 10, 100, 1000 };

        // ─── Unity lifecycle ─────────────────────────────────────────────────────
        private void Awake()
        {
            if (evaluator == null) evaluator = FindAnyObjectByType<ManeuverEvaluator>();
            universeManager = FindAnyObjectByType<UniverseManager>();
        }

        private void Start()
        {
            if (evaluator != null)
            {
                flightPlan = evaluator.GetFlightPlan();
                EnsureAtLeastOneNode();
            }
        }

        private void Update()
        {
            // Global mouse-up detection (covers releasing outside the window)
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            bool mousePressed = Mouse.current != null && Mouse.current.leftButton != null && Mouse.current.leftButton.isPressed;
#else
            bool mousePressed = Input.GetMouseButton(0);
#endif
            if (!mousePressed && sliderHotId >= 0)
            {
                dragActiveT = dragActiveN = dragActiveB = false;
                dragOffsetT = dragOffsetN = dragOffsetB = 0f;
                dragActivePrediction = false;
                dragOffsetPrediction = 0f;
                sliderHotId = -1;
            }

            // Toggle planner window visibility with the configured key
#if ENABLE_INPUT_SYSTEM && !ENABLE_LEGACY_INPUT_MANAGER
            if (Keyboard.current != null)
            {
                // Try direct enum name match (works for letter keys like 'N')
                if (Enum.TryParse<UnityEngine.InputSystem.Key>(toggleKey.ToString(), out var parsedKey))
                {
                    if (Keyboard.current[parsedKey].wasPressedThisFrame)
                    {
                        showWindow = !showWindow;
                    }
                }
                else
                {
                    // Handle numeric row keys (KeyCode.Alpha0..Alpha9 -> Key.Digit0..Digit9)
                    if (toggleKey >= KeyCode.Alpha0 && toggleKey <= KeyCode.Alpha9)
                    {
                        int digit = toggleKey - KeyCode.Alpha0;
                        string keyName = "Digit" + digit;
                        if (Enum.TryParse<UnityEngine.InputSystem.Key>(keyName, out var digitKey) &&
                            Keyboard.current[digitKey].wasPressedThisFrame)
                        {
                            showWindow = !showWindow;
                        }
                    }
                }
            }
#else
            if (Input.GetKeyDown(toggleKey))
            {
                showWindow = !showWindow;
            }
#endif

            // Apply velocity-drag to the active node each frame
            if (flightPlan != null && flightPlan.Nodes.Count > 0)
            {
                int idx  = Mathf.Clamp(selectedNodeIndex, 0, flightPlan.Nodes.Count - 1);
                var node = flightPlan.Nodes[idx];
                float dt = Time.unscaledDeltaTime;
                bool changed = false;

                if (dragActiveT && sliderHotId == 0)
                { node.DvTangent  += CalcRate(dragOffsetT) * dt; changed = true; }
                if (dragActiveN && sliderHotId == 1)
                { node.DvNormal   += CalcRate(dragOffsetN) * dt; changed = true; }
                if (dragActiveB && sliderHotId == 2)
                { node.DvBinormal += CalcRate(dragOffsetB) * dt; changed = true; }

                if (changed) MarkDirty();
            }

            // Live prediction slider update
            if (dragActivePrediction && flightPlan != null)
            {
                float dt = Time.unscaledDeltaTime;

                // Exponential speed: gentle near center, fast at extremes
                float speed = Mathf.Pow(Mathf.Abs(dragOffsetPrediction), 2.2f);

                // Scale: from ~10 seconds to 30 days per second of full drag
                double delta = dragOffsetPrediction * speed * 86400.0 * dt;

                flightPlan.PredictionLengthSeconds += delta;

                flightPlan.PredictionLengthSeconds = Math.Max(
                    10.0,
                    Math.Min(86400.0 * 30.0, flightPlan.PredictionLengthSeconds)
                );

                MarkDirty();
            }

            // Prediction refresh
            if (universeManager != null)
            {
                double now = universeManager.SimulationTimeSeconds;
                if (predictionDirty || Math.Abs(now - lastPredTime) > 0.5)
                {
                    RefreshPredictionCache();
                    lastPredTime    = now;
                    predictionDirty = false;
                }
                // Publish preview time to UniverseManager for live visuals
                if (flightPlan != null) universeManager.PreviewTimeOffsetSeconds = flightPlan.PredictionLengthSeconds;
            }
        }

        // Cubic response curve: fine near centre, fast at extremes
        private float CalcRate(float normalizedOffset)
        {
            float s = Mathf.Sign(normalizedOffset);
            return s * Mathf.Pow(Mathf.Abs(normalizedOffset), 2.2f) * maxDragRate;
        }

        private void OnGUI()
        {
            if (!showWindow) return;
            BuildStylesOnce();
            windowRect = GUILayout.Window(unchecked((int)0xF1A9_0001), windowRect, DrawWindow, GUIContent.none, styleWindow);
            windowRect = ClampToScreen(windowRect);
        }

        // ─── Main window ─────────────────────────────────────────────────────────
        private void DrawWindow(int id)
        {
            // Title bar
            GUILayout.BeginHorizontal(styleSection);
            GUILayout.Label("✦  MANEUVER PLANNER", styleHeader);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("⚙", styleButtonSmall)) showSettings = !showSettings;
            if (GUILayout.Button("✕", styleButtonSmall)) showWindow   = false;
            GUILayout.EndHorizontal();

            if (showSettings) DrawSettings();

            if (flightPlan == null)
            { GUILayout.Label("No FlightPlan attached.", styleLabel); GUI.DragWindow(); return; }

            DrawNodeBar();

            mainScroll = GUILayout.BeginScrollView(mainScroll, false, true);
            DrawCurrentNodeEditor();
            DrawDivider();
            DrawPredictionPanel();
            DrawDivider();
            DrawFlightSummary();
            GUILayout.EndScrollView();

            GUI.DragWindow(new Rect(0, 0, windowRect.width, 20));
        }

        // ─── Node selector ───────────────────────────────────────────────────────
        private void DrawNodeBar()
        {
            GUILayout.BeginHorizontal();
            nodeListScroll = GUILayout.BeginScrollView(nodeListScroll, false, false,
                GUILayout.Height(28), GUILayout.MaxWidth(windowRect.width - 84));
            GUILayout.BeginHorizontal();
            for (int i = 0; i < flightPlan.Nodes.Count; i++)
            {
                bool active = i == selectedNodeIndex;
                if (GUILayout.Button($"#{i + 1}", active ? styleButtonActive : styleButtonSmall,
                    GUILayout.Width(34)))
                    SelectNode(i);
            }
            GUILayout.EndHorizontal();
            GUILayout.EndScrollView();
            if (GUILayout.Button("+", styleButtonSmall, GUILayout.Width(28))) AddNode();
            if (GUILayout.Button("−", styleButtonSmall, GUILayout.Width(28))) RemoveSelectedNode();
            GUILayout.EndHorizontal();
        }

        // ─── Node editor ─────────────────────────────────────────────────────────
        private void DrawCurrentNodeEditor()
        {
            if (flightPlan.Nodes.Count == 0) return;
            selectedNodeIndex = Mathf.Clamp(selectedNodeIndex, 0, flightPlan.Nodes.Count - 1);
            ManeuverNode node = flightPlan.Nodes[selectedNodeIndex];

            // Header row
            GUILayout.BeginHorizontal(styleSection);
            GUILayout.Label($"NODE #{selectedNodeIndex + 1}", styleHeader);
            GUILayout.FlexibleSpace();
            GUILayout.Label(FormatDV(node.TotalDeltaV),
                node.TotalDeltaV > 0.001 ? styleLabelYellow : styleLabel);
            GUILayout.EndHorizontal();

            DrawTimeRow(node);

            // T− countdown
            if (universeManager != null)
            {
                double tte    = node.StartTime - universeManager.SimulationTimeSeconds;
                string tteStr = tte >= 0 ? $"T−  {FormatDuration(tte)}"
                                         : $"T+  {FormatDuration(-tte)}  (PAST)";
                GUILayout.Label(tteStr,
                    tte < 0 ? styleLabelRed : (tte < 60 ? styleLabelYellow : styleLabelGreen));
            }

            GUILayout.Space(4);

            // Three ΔV axes
            DrawDVAxis(node, "Prograde / Retrograde", ref node.DvTangent,
                ref inputDvT, ref editingT, new Color(0.25f, 1.00f, 0.35f), 0,
                ref dragActiveT, ref dragOffsetT, ref sliderRectT);

            DrawDVAxis(node, "Normal / Anti-Normal", ref node.DvNormal,
                ref inputDvN, ref editingN, new Color(0.30f, 0.80f, 1.00f), 1,
                ref dragActiveN, ref dragOffsetN, ref sliderRectN);

            DrawDVAxis(node, "Radial / Anti-Radial", ref node.DvBinormal,
                ref inputDvB, ref editingB, new Color(1.00f, 0.60f, 0.25f), 2,
                ref dragActiveB, ref dragOffsetB, ref sliderRectB);

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Zero ΔV",     styleButton)) { node.DvTangent = node.DvNormal = node.DvBinormal = 0; MarkDirty(); }
            if (GUILayout.Button("T = Now+60s", styleButton)) { node.StartTime = NowPlus(60);  MarkDirty(); }
            if (GUILayout.Button("T = Now+10m", styleButton)) { node.StartTime = NowPlus(600); MarkDirty(); }
            GUILayout.EndHorizontal();
        }

        // ─── Time row ────────────────────────────────────────────────────────────
        private void DrawTimeRow(ManeuverNode node)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Start (s):", styleLabel, GUILayout.Width(68));
            if (editingTime)
            {
                inputStartTime = GUILayout.TextField(inputStartTime, styleInputField, GUILayout.Width(110));
                if (GUILayout.Button("✓", styleButtonSmall, GUILayout.Width(24)))
                {
                    if (double.TryParse(inputStartTime, out double v)) { node.StartTime = v; MarkDirty(); }
                    editingTime = false;
                }
                if (GUILayout.Button("✕", styleButtonSmall, GUILayout.Width(24))) editingTime = false;
            }
            else
            {
                if (GUILayout.Button(node.StartTime.ToString("F1"), styleInputField, GUILayout.Width(110)))
                { inputStartTime = node.StartTime.ToString("F1"); editingTime = true; }
            }
            if (GUILayout.Button("−1h", styleButtonSmall)) { node.StartTime -= 3600; MarkDirty(); }
            if (GUILayout.Button("−1m", styleButtonSmall)) { node.StartTime -= 60;   MarkDirty(); }
            if (GUILayout.Button("+1m", styleButtonSmall)) { node.StartTime += 60;   MarkDirty(); }
            if (GUILayout.Button("+1h", styleButtonSmall)) { node.StartTime += 3600; MarkDirty(); }
            GUILayout.EndHorizontal();
        }

        // ─── ΔV axis block ───────────────────────────────────────────────────────
        private void DrawDVAxis(ManeuverNode node, string axisLabel, ref double dv,
            ref string inputField, ref bool editing,
            Color axisColor, int axisId,
            ref bool dragActive, ref float dragOffset, ref Rect storedSliderRect)
        {
            Color prev = GUI.color;
            GUILayout.BeginVertical(styleSection);

            // — Label + value field ———————————————————————————————————————
            GUILayout.BeginHorizontal();
            GUI.color = axisColor;
            GUILayout.Label("▌", styleLabel, GUILayout.Width(10));
            GUI.color = prev;
            GUILayout.Label(axisLabel, styleLabel, GUILayout.Width(155));
            GUILayout.FlexibleSpace();

            if (editing)
            {
                inputField = GUILayout.TextField(inputField, styleInputField, GUILayout.Width(84));
                if (GUILayout.Button("✓", styleButtonSmall, GUILayout.Width(22)))
                {
                    if (double.TryParse(inputField.Replace(',', '.'),
                        System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out double v))
                    { dv = v; MarkDirty(); }
                    editing = false;
                }
                if (GUILayout.Button("✕", styleButtonSmall, GUILayout.Width(22))) editing = false;
            }
            else
            {
                GUI.color = dv != 0 ? axisColor : prev;
                if (GUILayout.Button(FormatDV(dv), styleInputField, GUILayout.Width(95)))
                {
                    inputField = dv.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
                    editing    = true;
                }
                GUI.color = prev;
            }
            GUILayout.EndHorizontal();

            // — Velocity-drag slider ——————————————————————————————————————
            DrawVelocitySlider(ref dragActive, ref dragOffset, ref storedSliderRect,
                axisColor, axisId, CalcRate(dragActive && sliderHotId == axisId ? dragOffset : 0f));

            // — Increment buttons —————————————————————————————————————————
            GUILayout.BeginHorizontal();
            foreach (double step in DV_STEPS)
            {
                if (GUILayout.Button($"−{ShortStep(step)}", styleButtonSmall))
                { dv -= step; MarkDirty(); }
            }
            GUILayout.FlexibleSpace();
            foreach (double step in DV_STEPS)
            {
                if (GUILayout.Button($"+{ShortStep(step)}", styleButtonSmall))
                { dv += step; MarkDirty(); }
            }
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }

        // ─── Velocity-drag slider ────────────────────────────────────────────────
        //
        //   Visual layout (18 px tall, full width):
        //
        //   |  ·  ·  ·  │  ·  ·  ·  |    ← tick marks at ±25 / ±50 / ±75 / 100 %
        //   |            ║            |    ← bright centre line
        //   |        [===▐===]        |    ← coloured thumb following mouse
        //   |   ← drag to adjust →   |    ← hint text (idle) or rate label (active)
        //
        //   While dragging: tinted half-bar shows direction & magnitude.
        //
        private void DrawVelocitySlider(
            ref bool  dragActive, ref float dragOffset,
            ref Rect  storedRect, Color axisColor, int axisId,
            float     currentRate)
        {
            const float H      = 18f;
            const float THUMB_W = 8f;

            Rect r = GUILayoutUtility.GetRect(
                GUIContent.none, styleSliderBg,
                GUILayout.Height(H), GUILayout.ExpandWidth(true));

            Event e = Event.current;

            // ── Input ─────────────────────────────────────────────────────────
            if (e.type == EventType.MouseDown && e.button == 0 && r.Contains(e.mousePosition))
            {
                dragActive  = true;
                sliderHotId = axisId;
                dragOffset  = PosToOffset(e.mousePosition.x, r);
                GUIUtility.hotControl = GUIUtility.GetControlID(FocusType.Passive);
                e.Use();
            }
            else if (e.type == EventType.MouseUp && e.button == 0 && dragActive && sliderHotId == axisId)
            {
                dragActive  = false;
                dragOffset  = 0f;
                sliderHotId = -1;
                GUIUtility.hotControl = 0;
                e.Use();
            }
            else if (e.type == EventType.MouseDrag && dragActive && sliderHotId == axisId)
            {
                dragOffset = PosToOffset(e.mousePosition.x, r);
                e.Use();
            }

            if (e.type == EventType.Repaint) storedRect = r;

            // ── Drawing ───────────────────────────────────────────────────────
            if (e.type != EventType.Repaint) return;

            bool isHot = dragActive && sliderHotId == axisId;

            // Background
            DrawRect(r, new Color(0.05f, 0.05f, 0.07f, 1f));

            // Direction tint (filled half-bar)
            if (isHot && Mathf.Abs(dragOffset) > 0.01f)
            {
                float abs   = Mathf.Abs(dragOffset);
                Color tint  = new Color(axisColor.r, axisColor.g, axisColor.b,
                    Mathf.Lerp(0.05f, 0.25f, abs));
                float halfW = r.width * 0.5f;
                float cx    = r.x + halfW;
                Rect  tintR = dragOffset > 0
                    ? new Rect(cx, r.y, halfW * abs, r.height)
                    : new Rect(cx - halfW * abs, r.y, halfW * abs, r.height);
                DrawRect(tintR, tint);
            }

            // Tick marks at ±25 / ±50 / ±75 / ±100 %
            Color tickCol = new Color(0.25f, 0.25f, 0.30f, 1f);
            foreach (float frac in new[] { 0.25f, 0.50f, 0.75f, 1.00f })
            {
                float xR = r.x + r.width * (0.5f + frac * 0.5f);
                float xL = r.x + r.width * (0.5f - frac * 0.5f);
                DrawRect(new Rect(xR - 0.5f, r.y + 4f, 1f, r.height - 8f), tickCol);
                DrawRect(new Rect(xL - 0.5f, r.y + 4f, 1f, r.height - 8f), tickCol);
            }

            // Centre line (bright)
            float centrePx = r.x + r.width * 0.5f;
            DrawRect(new Rect(centrePx - 1f, r.y + 1f, 2f, r.height - 2f),
                new Color(0.60f, 0.60f, 0.65f, 1f));

            // Thumb
            float thumbCx   = isHot
                ? Mathf.Clamp(centrePx + dragOffset * r.width * 0.5f, r.x + THUMB_W, r.xMax - THUMB_W)
                : centrePx;
            Color thumbColor = isHot
                ? new Color(axisColor.r, axisColor.g, axisColor.b, 0.95f)
                : new Color(0.42f, 0.42f, 0.48f, 0.90f);
            DrawRect(new Rect(thumbCx - THUMB_W * 0.5f, r.y + 1f, THUMB_W, r.height - 2f), thumbColor);

            // Label overlay
            GUIStyle lblStyle = new GUIStyle(GUI.skin.label)
            { fontSize = 9, alignment = TextAnchor.MiddleCenter };

            if (isHot && Mathf.Abs(dragOffset) > 0.02f)
            {
                string sign  = currentRate >= 0 ? "+" : "";
                string label = $"{sign}{currentRate:F1} m/s·s⁻¹";
                lblStyle.normal.textColor = new Color(axisColor.r, axisColor.g, axisColor.b, 0.95f);
                GUI.Label(new Rect(r.x + 4f, r.y, r.width - 8f, r.height), label, lblStyle);
            }
            else if (!isHot)
            {
                lblStyle.normal.textColor = new Color(0.36f, 0.36f, 0.40f, 1f);
                GUI.Label(new Rect(r.x, r.y, r.width, r.height), "← drag to adjust →", lblStyle);
            }
        }

        // Converts mouse x → normalised offset [-1, +1] relative to rect centre
        private static float PosToOffset(float mouseX, Rect r)
            => Mathf.Clamp((mouseX - (r.x + r.width * 0.5f)) / (r.width * 0.5f), -1f, 1f);

        // Thin wrapper for immediate-mode coloured rect
        private static void DrawRect(Rect rect, Color color)
        {
            Color prev = GUI.color;
            GUI.color  = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color  = prev;
        }

        // ─── Orbital prediction ──────────────────────────────────────────────────
        private void DrawPredictionPanel()
        {
            GUILayout.BeginHorizontal(styleSection);
            GUILayout.Label("ORBIT PREDICTION", styleHeader);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button(showPrediction ? "▾" : "▸", styleButtonSmall))
                showPrediction = !showPrediction;
            GUILayout.EndHorizontal();

            if (!showPrediction) return;

            if (!orbitBefore.IsValid && !orbitAfter.IsValid)
            { GUILayout.Label("Unavailable — need UniverseManager.", styleLabelDim); return; }

            GUILayout.BeginHorizontal();
            GUILayout.Label("",       styleLabel,    GUILayout.Width(90));
            GUILayout.Label("BEFORE", styleLabelDim, GUILayout.Width(110));
            GUILayout.Label("AFTER",  styleLabelDim, GUILayout.Width(110));
            GUILayout.EndHorizontal();

            OrbitRow("Periapsis",    o => FormatDist(o.PeriapsisDistance));
            OrbitRow("Apoapsis",     o => FormatDist(o.ApoapsisDistance));
            OrbitRow("Period",       o => FormatDuration(o.OrbitalPeriodSeconds));
            OrbitRow("Inclination",  o => $"{o.InclinationDegrees:F2}°");
            OrbitRow("Eccentricity", o => $"{o.Eccentricity:F4}");
            OrbitRow("Energy",       o => $"{o.SpecificOrbitalEnergy:F2}");
        }

        private void OrbitRow(string lbl, Func<OrbitalElements, string> fmt)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(lbl + ":", styleLabel,      GUILayout.Width(90));
            GUILayout.Label(orbitBefore.IsValid ? fmt(orbitBefore) : "—", styleLabelDim,    GUILayout.Width(110));
            GUILayout.Label(orbitAfter.IsValid  ? fmt(orbitAfter)  : "—", styleLabelYellow, GUILayout.Width(110));
            GUILayout.EndHorizontal();
        }

        // ─── Flight summary ──────────────────────────────────────────────────────
        private void DrawFlightSummary()
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label("FLIGHT PLAN SUMMARY", styleHeader);

            for (int i = 0; i < flightPlan.Nodes.Count; i++)
            {
                ManeuverNode n   = flightPlan.Nodes[i];
                double tte       = universeManager != null
                    ? n.StartTime - universeManager.SimulationTimeSeconds : 0;
                string tteStr    = $"T{(tte >= 0 ? "−" : "+")}{FormatDuration(Math.Abs(tte))}";
                GUILayout.BeginHorizontal();
                GUILayout.Label($"#{i + 1}",        styleLabel,      GUILayout.Width(28));
                GUILayout.Label(tteStr,             styleLabelDim,   GUILayout.Width(90));
                GUILayout.Label(FormatDV(n.TotalDeltaV), styleLabelYellow, GUILayout.Width(90));
                GUILayout.Label(
                    $"T:{n.DvTangent:+0.##;-0.##;0}  " +
                    $"N:{n.DvNormal:+0.##;-0.##;0}  " +
                    $"R:{n.DvBinormal:+0.##;-0.##;0}",
                    styleLabelDim);
                GUILayout.EndHorizontal();
            }

            DrawDivider();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Total Δv:", styleLabel, GUILayout.Width(90));
            GUILayout.Label(FormatDV(flightPlan.GetTotalDeltaV()), styleLabelGreen);
            GUILayout.EndHorizontal();

            GUILayout.Space(4);
            if (GUILayout.Button("Clear All Nodes", styleButton))
            {
                flightPlan.Nodes.Clear();
                EnsureAtLeastOneNode();
                selectedNodeIndex = 0;
                MarkDirty();
            }
            GUILayout.EndVertical();
        }

        // ─── Settings panel ──────────────────────────────────────────────────────
        private void DrawSettings()
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label("PREDICTION SETTINGS", styleHeader);

            // Custom prediction slider with velocity-drag behavior
            DrawPredictionSlider();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Max rate:", styleLabel, GUILayout.Width(90));
            maxDragRate = GUILayout.HorizontalSlider(maxDragRate, 1f, 1000f);
            GUILayout.Label($"{maxDragRate:F0} m/s/s", styleLabel, GUILayout.Width(74));
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }

        private void DrawPredictionSlider()
        {
            if (flightPlan == null) return;

            GUILayout.BeginVertical(styleSection);

            GUILayout.Label("PREDICTION TIME", styleHeader);

            Rect r = GUILayoutUtility.GetRect(320f, 28f, GUILayout.ExpandWidth(true));

            sliderRectPrediction = r;

            Event e = Event.current;

            bool isHot = sliderHotId == 10;

            if (e.type == EventType.MouseDown && r.Contains(e.mousePosition))
            {
                sliderHotId = 10;
                dragActivePrediction = true;
                dragOffsetPrediction = PosToOffset(e.mousePosition.x, r);
                e.Use();
            }

            if (isHot && e.type == EventType.MouseDrag)
            {
                dragOffsetPrediction = PosToOffset(e.mousePosition.x, r);
                e.Use();
            }

            if (isHot && e.type == EventType.MouseUp)
            {
                dragActivePrediction = false;
                dragOffsetPrediction = 0f;
                sliderHotId = -1;
                e.Use();
            }

            DrawRect(r, new Color(0.12f, 0.12f, 0.14f));

            Rect center = new Rect(r.center.x - 1f, r.y, 2f, r.height);
            DrawRect(center, new Color(0.35f, 0.35f, 0.4f));

            // Compute display offset: if dragging use current drag offset, otherwise derive from flightPlan value
            float minSec = 10f;
            float maxSec = 86400f * 30f;
            float valueFrac = Mathf.InverseLerp(minSec, maxSec, (float)flightPlan.PredictionLengthSeconds);
            float displayOffset = dragActivePrediction ? dragOffsetPrediction : (valueFrac * 2f - 1f);

            float knobX = Mathf.Lerp(r.xMin, r.xMax, (displayOffset + 1f) * 0.5f);

            Rect knob = new Rect(knobX - 8f, r.y + 2f, 16f, r.height - 4f);

            DrawRect(knob, new Color(0.2f, 0.75f, 1f));

            string text = FormatPredictionTime(flightPlan.PredictionLengthSeconds);

            GUI.Label(r, text, styleLabel);

            GUILayout.EndVertical();
        }

        private string FormatPredictionTime(double seconds)
        {
            if (seconds < 60)
                return $"{seconds:F0} sec";

            if (seconds < 3600)
                return $"{seconds / 60:F1} min";

            if (seconds < 86400)
                return $"{seconds / 3600:F1} h";

            return $"{seconds / 86400:F1} d";
        }

        // ─── Internal helpers ────────────────────────────────────────────────────
        private void SelectNode(int i)
        {
            selectedNodeIndex = Mathf.Clamp(i, 0, flightPlan.Nodes.Count - 1);
            predictionDirty   = true;
            dragActiveT = dragActiveN = dragActiveB = false;
            dragOffsetT = dragOffsetN = dragOffsetB = 0f;
            sliderHotId = -1;
            var n = flightPlan.Nodes[selectedNodeIndex];
            inputStartTime = n.StartTime.ToString("F1");
            inputDvT = n.DvTangent.ToString( "F3", System.Globalization.CultureInfo.InvariantCulture);
            inputDvN = n.DvNormal.ToString(  "F3", System.Globalization.CultureInfo.InvariantCulture);
            inputDvB = n.DvBinormal.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
            editingTime = editingT = editingN = editingB = false;
        }

        private void AddNode()
        {
            double t = flightPlan.Nodes.Count > 0
                ? flightPlan.Nodes[flightPlan.Nodes.Count - 1].StartTime + 3600
                : NowPlus(600);
            flightPlan.Nodes.Add(new ManeuverNode(t));
            SelectNode(flightPlan.Nodes.Count - 1);
            MarkDirty();
        }

        private void RemoveSelectedNode()
        {
            if (flightPlan.Nodes.Count == 0) return;
            flightPlan.Nodes.RemoveAt(Mathf.Clamp(selectedNodeIndex, 0, flightPlan.Nodes.Count - 1));
            EnsureAtLeastOneNode();
            SelectNode(Mathf.Clamp(selectedNodeIndex, 0, flightPlan.Nodes.Count - 1));
            MarkDirty();
        }

        private void EnsureAtLeastOneNode()
        {
            if (flightPlan == null || flightPlan.Nodes.Count > 0) return;
            flightPlan.Nodes.Add(new ManeuverNode(NowPlus(600)));
        }

        private void MarkDirty()
        {
            predictionDirty = true;
            evaluator?.MarkAsDirty();
        }

        private void RefreshPredictionCache()
        {
            orbitBefore = orbitAfter = OrbitalElements.Invalid;
            if (universeManager == null || flightPlan == null || flightPlan.Nodes.Count == 0) return;
            int idx  = Mathf.Clamp(selectedNodeIndex, 0, flightPlan.Nodes.Count - 1);
            var node = flightPlan.Nodes[idx];
            var frame = universeManager.ActiveReferenceFrame;
            orbitBefore = universeManager.GetShipOrbitAround(frame);
            if (universeManager.ShipBody == null) return;
            if (!universeManager.TryGetReferenceState(frame, out _, out Vector3d framePos,
                out Vector3d frameVel, out double mu, out _, out _)) return;
            Vector3d pos    = universeManager.ShipBody.Position;
            Vector3d vel    = universeManager.ShipBody.Velocity;
            Vector3d dv     = FlightPlan.CalculateWorldDeltaV(pos, vel, node);
            orbitAfter = OrbitalElements.FromState(pos - framePos, (vel + dv) - frameVel, mu);
        }

        private double NowPlus(double sec)
            => (universeManager?.SimulationTimeSeconds ?? 0) + sec;

        private static Rect ClampToScreen(Rect r)
        {
            r.x = Mathf.Clamp(r.x, 0f, Mathf.Max(0f, Screen.width  - r.width));
            r.y = Mathf.Clamp(r.y, 0f, Mathf.Max(0f, Screen.height - r.height));
            return r;
        }

        private static void DrawDivider()
            => GUILayout.Box("", GUILayout.ExpandWidth(true), GUILayout.Height(1));

        // ─── Formatters ──────────────────────────────────────────────────────────
        private static string FormatDV(double v)
            => Math.Abs(v) >= 1000 ? $"{v / 1000:F3} km/s" : $"{v:F3} m/s";

        private static string ShortStep(double v)
            => v >= 1000 ? $"{v / 1000:0}k" : (v >= 1 ? $"{v:0}" : $"{v:G2}");

        private static string FormatDist(double m)
        {
            if (double.IsInfinity(m)) return "∞";
            if (m >= 1e9) return $"{m / 1e9:F3} Gm";
            if (m >= 1e6) return $"{m / 1e6:F3} Mm";
            if (m >= 1e3) return $"{m / 1e3:F3} km";
            return $"{m:F1} m";
        }

        private static string FormatDuration(double s)
        {
            if (double.IsInfinity(s) || double.IsNaN(s)) return "∞";
            if (s < 0) s = 0;
            int d = (int)(s / 86400), h = (int)((s % 86400) / 3600);
            int m = (int)((s % 3600) / 60), sec = (int)(s % 60);
            if (d > 0) return $"{d}d {h:00}h {m:00}m";
            if (h > 0) return $"{h}h {m:00}m {sec:00}s";
            if (m > 0) return $"{m}m {sec:00}s";
            return $"{sec}s";
        }

        // ─── Style builder ────────────────────────────────────────────────────────
        private void BuildStylesOnce()
        {
            if (stylesBuilt) return;
            stylesBuilt = true;

            Texture2D Tex(Color c) { var t = new Texture2D(1, 1); t.SetPixel(0, 0, c); t.Apply(); return t; }

            Color bgDark    = new Color(0.06f, 0.06f, 0.08f, 0.94f);
            Color bgSection = new Color(0.09f, 0.09f, 0.12f, 0.98f);
            Color bgField   = new Color(0.03f, 0.03f, 0.05f, 1.00f);
            Color bgBtn     = new Color(0.14f, 0.14f, 0.18f, 1.00f);
            Color bgBtnH    = new Color(0.22f, 0.22f, 0.28f, 1.00f);
            Color bgActive  = new Color(0.12f, 0.26f, 0.12f, 1.00f);
            Color textMain  = new Color(0.90f, 0.90f, 0.88f, 1.00f);
            Color textDim   = new Color(0.55f, 0.55f, 0.55f, 1.00f);
            Color textGreen = new Color(0.35f, 1.00f, 0.50f, 1.00f);
            Color textYellow= new Color(1.00f, 0.85f, 0.20f, 1.00f);
            Color textRed   = new Color(1.00f, 0.35f, 0.25f, 1.00f);

            styleWindow = new GUIStyle(GUI.skin.window)
            { padding = new RectOffset(4, 4, 4, 6), margin = new RectOffset(0, 0, 0, 0), fontSize = 11 };
            styleWindow.normal.background   = Tex(bgDark);
            styleWindow.normal.textColor    = textMain;
            styleWindow.onNormal.background = styleWindow.normal.background;

            styleHeader = new GUIStyle(GUI.skin.label)
            { fontStyle = FontStyle.Bold, fontSize = 11, padding = new RectOffset(4, 4, 2, 2) };
            styleHeader.normal.textColor = textMain;

            styleLabel = new GUIStyle(GUI.skin.label)
            { fontSize = 11, padding = new RectOffset(2, 2, 1, 1) };
            styleLabel.normal.textColor = textMain;

            styleLabelDim    = Cloned(styleLabel, textDim);
            styleLabelGreen  = Cloned(styleLabel, textGreen);
            styleLabelYellow = Cloned(styleLabel, textYellow);
            styleLabelRed    = Cloned(styleLabel, textRed);

            styleButton = new GUIStyle(GUI.skin.button)
            { fontSize = 11, padding = new RectOffset(6, 6, 3, 3), margin = new RectOffset(2, 2, 1, 1) };
            styleButton.normal.background  = Tex(bgBtn);
            styleButton.hover.background   = Tex(bgBtnH);
            styleButton.active.background  = Tex(bgActive);
            styleButton.normal.textColor   = textMain;
            styleButton.hover.textColor    = Color.white;
            styleButton.active.textColor   = textGreen;

            styleButtonSmall = new GUIStyle(styleButton)
            { fontSize = 10, padding = new RectOffset(3, 3, 2, 2), margin = new RectOffset(1, 1, 1, 1) };

            styleButtonActive = new GUIStyle(styleButton);
            styleButtonActive.normal.background = Tex(bgActive);
            styleButtonActive.normal.textColor  = textGreen;

            styleInputField = new GUIStyle(GUI.skin.textField)
            { fontSize = 11, alignment = TextAnchor.MiddleRight, padding = new RectOffset(4, 4, 2, 2), margin = new RectOffset(1, 1, 1, 1) };
            styleInputField.normal.background  = Tex(bgField);
            styleInputField.normal.textColor   = textYellow;
            styleInputField.hover.background   = Tex(bgField);
            styleInputField.hover.textColor    = Color.white;
            styleInputField.focused.background = Tex(bgField);
            styleInputField.focused.textColor  = Color.white;

            styleSection = new GUIStyle(GUI.skin.box)
            { padding = new RectOffset(4, 4, 3, 3), margin = new RectOffset(0, 0, 2, 2) };
            styleSection.normal.background = Tex(bgSection);

            styleSliderBg = new GUIStyle(styleSection)
            { padding = new RectOffset(0, 0, 0, 0), margin = new RectOffset(0, 0, 2, 2) };
        }

        private static GUIStyle Cloned(GUIStyle src, Color color)
        {
            var s = new GUIStyle(src);
            s.normal.textColor = color;
            return s;
        }
    }
}
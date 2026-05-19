// ============================================================================
// ПЛАНИРОВЩИК МАНЁВРОВ (FlightPlanUI) — Principia-style
// ============================================================================
// Полностью новый интерфейс планировщика манёвров на основе документации Principia.
//
// Структура:
//   - Селектор плана полёта
//   - Слайдер конечного времени
//   - Параметры интегрирования
//   - Суммарный Δv
//   - Статус ошибок
//   - Участки свободного полёта + редакторы манёвров
//   - Кнопки управления
// ============================================================================

using System;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Physics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Главное окно планировщика манёвров в стиле Principia.
    /// 
    /// Отображает:
    /// - Список манёвров с редакторами
    /// - Участки свободного полёта с анализом орбиты
    /// - Параметры интегрирования
    /// - Статус ошибок
    /// </summary>
    public class FlightPlanUI : MonoBehaviour
    {
        // ─── Inspector ──────────────────────────────────────────────────────────
        [Header("References")]
        [SerializeField] private ManeuverEvaluator evaluator;
        [SerializeField] private OrbitAnalyzer orbitAnalyzer;

        [Header("Window")]
        [SerializeField] private Rect windowRect = new Rect(16f, 16f, 460f, 700f);
        [SerializeField] private bool showWindow = true;
        [SerializeField] private KeyCode toggleKey = KeyCode.N;

        [Header("Settings")]
        [SerializeField] private bool showSettings = false;
        [SerializeField] private bool showGuidance = false;

        // ─── Runtime ────────────────────────────────────────────────────────────
        private FlightPlan flightPlan;
        private UniverseManager universeManager;

        // Состояние
        private int? firstFutureManeuver;
        private int numberOfAnomalousManeuvers;
        private bool reachedDeadline;
        private bool mustTickle;
        private bool messageWasDisplayed;
        private float warningHeight = 1f;

        // Text field editing state
        private int editingTimeIndex = -1;
        private string editingTimeText;
        private int editingDvIndex = -1;
        private int editingDvAxis = -1; // 0=prograde, 1=normal, 2=radial
        private string editingDvText;

        // Scrub state
        private float predictionScrubValue = 0f;

        // Статус ошибки
        private ManeuverStatus currentStatus = ManeuverStatus.OK;
        private string statusMessage = "";
        private int? firstErrorManeuver;

        // Параметры интегрирования
        private int lengthToleranceIndex = 6;  // 1 м
        private int maxStepsIndex = 4;          // 1 << 14

        // Scroll
        private Vector2 mainScroll;

        // ─── Constants ──────────────────────────────────────────────────────────
        private static readonly double[] IntegrationTolerances = {
            1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6
        };
        private static readonly string[] ToleranceNames = {
            "1 мкм", "10 мкм", "100 мкм", "1 мм", "1 см", "10 см", "1 м", "10 м",
            "100 м", "1 км", "10 км", "100 км", "1000 км"
        };
        private static readonly int[] MaxSteps = {
            64, 256, 1024, 4096, 16384, 65536, 262144, 1048576
        };

        // ─── GUIStyle cache ─────────────────────────────────────────────────────
        private GUIStyle styleWindow, styleHeader, styleLabel, styleLabelDim;
        private GUIStyle styleLabelGreen, styleLabelYellow, styleLabelRed;
        private GUIStyle styleButton, styleButtonSmall, styleButtonActive;
        private GUIStyle styleInputField, styleSection, styleSliderBg, styleWarning;
        private bool stylesBuilt;

        private static readonly double[] DV_STEPS = { 0.01, 0.1, 1, 10, 100, 1000 };

        // ─── Unity lifecycle ─────────────────────────────────────────────────────
        private void Awake()
        {
            if (evaluator == null) evaluator = FindAnyObjectByType<ManeuverEvaluator>();
            universeManager = FindAnyObjectByType<UniverseManager>();
            if (orbitAnalyzer == null) orbitAnalyzer = FindAnyObjectByType<OrbitAnalyzer>();
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
            // Toggle planner window
#if ENABLE_INPUT_SYSTEM
            if (UnityEngine.InputSystem.Keyboard.current != null)
            {
                var kb = UnityEngine.InputSystem.Keyboard.current;
                string keyName = toggleKey.ToString();
                // KeyCode.N -> Key.N, KeyCode.Alpha0 -> Key.Digit0
                if (toggleKey >= KeyCode.Alpha0 && toggleKey <= KeyCode.Alpha9)
                {
                    int digit = toggleKey - KeyCode.Alpha0;
                    keyName = "Digit" + digit;
                }
                if (Enum.TryParse<UnityEngine.InputSystem.Key>(keyName, out var parsedKey))
                {
                    if (kb[parsedKey].wasPressedThisFrame)
                    {
                        showWindow = !showWindow;
                    }
                }
            }
#else
            if (Input.GetKeyDown(toggleKey))
            {
                showWindow = !showWindow;
            }
#endif

            // Apply scrub rate (centered spring-loaded slider)
            if (Mathf.Abs(predictionScrubValue) > 0.01f && flightPlan != null)
            {
                double length = flightPlan.PredictionLengthSeconds;
                double mag = predictionScrubValue * predictionScrubValue;
                double effectivePos = predictionScrubValue > 0 ? mag : -mag;
                double rate = length * effectivePos * 2.0;
                double newLength = Math.Max(60.0, Math.Min(315360000.0, length + rate * Time.unscaledDeltaTime));
                var result = flightPlan.SetDesiredFinalTime(newLength);
                if (result.IsOk)
                {
                    evaluator?.MarkAsDirtyLightweight();
                }
            }

            // Update prediction offset
            if (universeManager != null && flightPlan != null && flightPlan.Nodes.Count > 0)
            {
                universeManager.PreviewTimeOffsetSeconds = flightPlan.PredictionLengthSeconds;
            }
        }

        private void OnGUI()
        {
            if (!showWindow) return;
            BuildStylesOnce();
            windowRect = GUILayout.Window(unchecked((int)0xF1A9_0001), windowRect, DrawWindow, "✦ MANEUVER PLANNER", styleWindow);
            windowRect = ClampToScreen(windowRect);
        }

        // ─── Main window ─────────────────────────────────────────────────────────
        private void DrawWindow(int id)
        {
            if (flightPlan == null)
            {
                GUILayout.Label("No FlightPlan attached.", styleLabel);
                GUI.DragWindow();
                return;
            }

            // Settings panel
            if (showSettings)
                DrawSettings();

            // Flight plan controls
            DrawFlightPlanControls();

            // Integration parameters
            DrawIntegrationParameters();

            // Total Δv
            double totalDv = 0;
            foreach (var node in flightPlan.Nodes) totalDv += node.TotalDeltaV;
            GUILayout.Label($"Total Δv: {FormatDV(totalDv)}", styleLabelGreen);

            // Status message
            DrawStatusMessage();

            // Control buttons
            DrawControlButtons();

            // Coasts and maneuvers
            DrawCoastsAndManeuvers();

            GUI.DragWindow(new Rect(0, 0, windowRect.width, 20));
        }

        // ─── Flight plan controls ───────────────────────────────────────────────
        private void DrawFlightPlanControls()
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label("FLIGHT PLAN", styleHeader);

            double currentLength = flightPlan.PredictionLengthSeconds;

            // Display current end time
            GUILayout.BeginHorizontal();
            GUILayout.Label("End time:", styleLabel, GUILayout.Width(70));
            GUILayout.Label(FormatDuration(currentLength), styleLabelGreen, GUILayout.Width(80));
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            // Preset time buttons
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("1h", styleButtonSmall)) ApplyPredictionLength(3600);
            if (GUILayout.Button("6h", styleButtonSmall)) ApplyPredictionLength(21600);
            if (GUILayout.Button("1d", styleButtonSmall)) ApplyPredictionLength(86400);
            if (GUILayout.Button("7d", styleButtonSmall)) ApplyPredictionLength(604800);
            if (GUILayout.Button("30d", styleButtonSmall)) ApplyPredictionLength(2592000);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("1y", styleButtonSmall)) ApplyPredictionLength(31557600);
            if (GUILayout.Button("10y", styleButtonSmall)) ApplyPredictionLength(315360000);
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            // Centered scrub slider (spring-loaded)
            GUILayout.BeginHorizontal();
            GUILayout.Label("◀", styleLabelDim, GUILayout.Width(14));
            predictionScrubValue = GUILayout.HorizontalSlider(predictionScrubValue, -1f, 1f);
            GUILayout.Label("▶", styleLabelDim, GUILayout.Width(14));
            GUILayout.EndHorizontal();

            if (Event.current.type == EventType.MouseUp)
            {
                predictionScrubValue = 0f;
            }

            GUILayout.EndVertical();
        }

        private void ApplyPredictionLength(double seconds)
        {
            var result = flightPlan.SetDesiredFinalTime(seconds);
            if (result.IsOk)
            {
                evaluator?.MarkAsDirty();
                ResetStatus();
            }
            else
            {
                UpdateStatus(result.Status, result.Message);
            }
        }

        // ─── Integration parameters ─────────────────────────────────────────────
        private void DrawIntegrationParameters()
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label("INTEGRATION", styleHeader);

            GUILayout.BeginHorizontal();
            
            // Max steps
            GUILayout.BeginHorizontal();
            GUILayout.Label("Max steps:", styleLabel, GUILayout.Width(70));
            if (maxStepsIndex > 0 && GUILayout.Button("−", styleButtonSmall, GUILayout.Width(24)))
            {
                maxStepsIndex--;
            }
            GUILayout.Label($"{MaxSteps[maxStepsIndex]}", styleLabel, GUILayout.Width(60));
            if (maxStepsIndex < MaxSteps.Length - 1 && GUILayout.Button("+", styleButtonSmall, GUILayout.Width(24)))
            {
                maxStepsIndex++;
            }
            GUILayout.EndHorizontal();

            GUILayout.FlexibleSpace();

            // Tolerance
            GUILayout.BeginHorizontal();
            GUILayout.Label("Tolerance:", styleLabel, GUILayout.Width(70));
            if (lengthToleranceIndex > 0 && GUILayout.Button("−", styleButtonSmall, GUILayout.Width(24)))
            {
                lengthToleranceIndex--;
            }
            GUILayout.Label(ToleranceNames[lengthToleranceIndex], styleLabel, GUILayout.Width(60));
            if (lengthToleranceIndex < ToleranceNames.Length - 1 && GUILayout.Button("+", styleButtonSmall, GUILayout.Width(24)))
            {
                lengthToleranceIndex++;
            }
            GUILayout.EndHorizontal();

            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        // ─── Status message ─────────────────────────────────────────────────────
        private void DrawStatusMessage()
        {
            if (currentStatus == ManeuverStatus.OK) return;

            string message = statusMessage;
            if (numberOfAnomalousManeuvers > 0)
            {
                message += $"\n{numberOfAnomalousManeuvers} anomalous maneuvers.";
            }

            if (styleWarning == null)
            {
                styleWarning = new GUIStyle(styleLabel)
                {
                    normal = { textColor = new Color(1f, 0.35f, 0.25f) },
                    wordWrap = true
                };
            }

            warningHeight = Mathf.Max(warningHeight, styleWarning.CalcHeight(new GUIContent(message), windowRect.width - 20));
            GUILayout.Label(message, styleWarning, GUILayout.Height(Mathf.Min(warningHeight, 60)));
        }

        private void UpdateStatus(ManeuverStatus status, string message)
        {
            if (currentStatus == ManeuverStatus.OK && status != ManeuverStatus.OK)
            {
                currentStatus = status;
                statusMessage = message;
                messageWasDisplayed = false;
            }
        }

        private void ResetStatus()
        {
            currentStatus = ManeuverStatus.OK;
            statusMessage = "";
            firstErrorManeuver = null;
            messageWasDisplayed = true;
        }

        // ─── Control buttons ────────────────────────────────────────────────────
        private void DrawControlButtons()
        {
            GUILayout.BeginHorizontal();

            if (flightPlan.Nodes.Count == 0)
            {
                if (GUILayout.Button("Delete Plan", styleButton))
                {
                    flightPlan.Nodes.Clear();
                    ResetStatus();
                }
            }
            else
            {
                if (GUILayout.Button("Rebuild", styleButton))
                {
                    evaluator?.MarkAsDirty();
                    ResetStatus();
                }

                if (GUILayout.Button("Clear All", styleButton))
                {
                    flightPlan.Nodes.Clear();
                    ResetStatus();
                }
            }

            if (GUILayout.Button(showSettings ? "Hide Settings" : "Settings", styleButton))
            {
                showSettings = !showSettings;
            }

            GUILayout.EndHorizontal();
        }

        // ─── Coasts and maneuvers ───────────────────────────────────────────────
        private void DrawCoastsAndManeuvers()
        {
            int nodeCount = flightPlan.Nodes.Count;
            double currentTime = universeManager != null ? universeManager.SimulationTimeSeconds : 0;

            for (int i = 0; i <= nodeCount; i++)
            {
                // Coast segment before maneuver i (or final coast after last maneuver)
                DrawCoastSegment(i, currentTime);

                // Maneuver i (if exists)
                if (i < nodeCount)
                {
                    DrawManeuverEditor(i, currentTime);
                }
            }
        }

        // ─── Coast segment ──────────────────────────────────────────────────────
        private void DrawCoastSegment(int index, double currentTime)
        {
            GUILayout.BeginHorizontal(styleSection);

            double startTime, endTime;
            string coastDescription;

            if (index == 0)
            {
                // Coast from current time to first maneuver
                startTime = currentTime;
                endTime = flightPlan.Nodes.Count > 0 ? flightPlan.Nodes[0].StartTime : flightPlan.PredictionLengthSeconds;
            }
            else if (index < flightPlan.Nodes.Count)
            {
                // Coast between maneuvers
                var prevNode = flightPlan.Nodes[index - 1];
                startTime = prevNode.FinalTime;
                endTime = flightPlan.Nodes[index].StartTime;
            }
            else
            {
                // Final coast after last maneuver
                var lastNode = flightPlan.Nodes[flightPlan.Nodes.Count - 1];
                startTime = lastNode.FinalTime;
                endTime = flightPlan.PredictionLengthSeconds;
            }

            double duration = endTime - startTime;

            // Get orbit analysis for this coast
            if (universeManager != null && orbitAnalyzer != null)
            {
                var frame = universeManager.ActiveReferenceFrame;
                var orbit = universeManager.GetShipOrbitAround(frame);
                
                if (orbit.IsValid)
                {
                    double periapsis = orbit.PeriapsisDistance / 1000;
                    double apoapsis = orbit.ApoapsisDistance / 1000;
                    double inclination = orbit.InclinationDegrees;
                    double eccentricity = orbit.Eccentricity;

                    coastDescription = $"a={(periapsis + apoapsis) / 2:F0} km, e={eccentricity:F3}, " +
                                      $"P={periapsis:F0} km, A={apoapsis:F0} km, i={inclination:F1}°";
                }
                else
                {
                    coastDescription = "Coasting";
                }
            }
            else
            {
                coastDescription = "Coasting";
            }

            string durationStr = FormatDuration(duration);
            GUILayout.Label($"{coastDescription} ({durationStr})", styleLabelDim);

            // Add maneuver button
            if (GUILayout.Button("+", styleButtonSmall, GUILayout.Width(24)))
            {
                double newTime = startTime + Math.Max(60, duration * 0.5);
                var newNode = new ManeuverNode(newTime);
                
                var result = flightPlan.Insert(newNode, index);
                if (result.IsOk)
                {
                    evaluator?.MarkAsDirty();
                    ResetStatus();
                }
                else
                {
                    UpdateStatus(result.Status, result.Message);
                }
            }

            GUILayout.EndHorizontal();
        }

        // ─── Maneuver editor ────────────────────────────────────────────────────
        private void DrawManeuverEditor(int index, double currentTime)
        {
            if (index < 0 || index >= flightPlan.Nodes.Count) return;
            var node = flightPlan.Nodes[index];
            bool isPast = node.FinalTime < currentTime;
            bool isFuture = node.StartTime > currentTime;
            bool isActive = !isPast && !isFuture;

            if (isFuture && firstFutureManeuver == null)
            {
                firstFutureManeuver = index;
            }

            GUILayout.BeginVertical(isPast ? styleSection : styleSection);

            // Header
            GUILayout.BeginHorizontal();
            string status = isPast ? "✓ PAST" : (isActive ? "▶ ACTIVE" : "○ PLANNED");
            Color statusColor = isPast ? new Color(0.5f, 0.5f, 0.5f) : (isActive ? new Color(0.35f, 1f, 0.5f) : new Color(1f, 0.85f, 0.2f));
            
            Color prevColor = GUI.color;
            GUI.color = statusColor;
            GUILayout.Label($"Maneuver #{index + 1}  [{status}]", styleHeader);
            GUI.color = prevColor;

            GUILayout.FlexibleSpace();
            GUILayout.Label(FormatDV(node.TotalDeltaV), node.TotalDeltaV > 0.001 ? styleLabelYellow : styleLabelDim);

            // Delete button
            if (GUILayout.Button("✕", styleButtonSmall, GUILayout.Width(24)))
            {
                var result = flightPlan.Remove(index);
                if (result.IsOk)
                {
                    evaluator?.MarkAsDirty();
                    ResetStatus();
                }
                else
                {
                    UpdateStatus(result.Status, result.Message);
                }
                GUILayout.EndHorizontal();
                GUILayout.EndVertical();
                return;
            }

            GUILayout.EndHorizontal();

            // T− countdown
            if (universeManager != null)
            {
                double tte = node.StartTime - currentTime;
                string tteStr = tte >= 0 ? $"T−  {FormatDuration(tte)}" : $"T+  {FormatDuration(-tte)}  (PAST)";
                GUILayout.Label(tteStr, tte < 0 ? styleLabelRed : (tte < 60 ? styleLabelYellow : styleLabelGreen));
            }

            // Time controls
            DrawTimeControls(node, index);

            GUILayout.Space(4);

            // Three ΔV axes
            DrawDVAxis(node, "Prograde / Retrograde", ref node.DvPrograde,
                new Color(0.25f, 1.00f, 0.35f), index, 0);

            DrawDVAxis(node, "Normal / Anti-Normal", ref node.DvNormal,
                new Color(0.30f, 0.80f, 1.00f), index, 1);

            DrawDVAxis(node, "Radial / Anti-Radial", ref node.DvRadial,
                new Color(1.00f, 0.60f, 0.25f), index, 2);

            GUILayout.Space(4);

            // Quick actions
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Zero ΔV", styleButton))
            {
                node.DvPrograde = node.DvNormal = node.DvRadial = 0;
                evaluator?.MarkAsDirty();
            }
            if (GUILayout.Button("T = Now+60s", styleButton))
            {
                node.StartTime = NowPlus(60);
                evaluator?.MarkAsDirty();
            }
            if (GUILayout.Button("T = Now+10m", styleButton))
            {
                node.StartTime = NowPlus(600);
                evaluator?.MarkAsDirty();
            }
            GUILayout.EndHorizontal();

            // Mode toggle: Instant vs Engine
            GUILayout.BeginHorizontal();
            bool newInstant = GUILayout.Toggle(node.IsInstant, "⚡ Instant", styleButton, GUILayout.Width(90));
            bool newEngine = GUILayout.Toggle(!node.IsInstant, "🔥 Engine", !node.IsInstant ? styleButtonActive : styleButton, GUILayout.Width(90));
                if (newInstant != node.IsInstant)
            {
                node.IsInstant = newInstant;
                node.InvalidateCalculation();
                if (newInstant)
                {
                    node.Duration = 0;
                    node.Engine = null;
                }
                else
                {
                    // Setup default engine params if none set
                    if (!node.Engine.HasValue)
                    {
                        node.Engine = new EngineParameters
                        {
                            SpecificImpulseSeconds = 300f,
                            ThrustNewtons = 500000f,
                            InitialMassKg = 50000f
                        };
                    }
                }
                evaluator?.MarkAsDirty();
            }
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();

            // Engine info (if set)
            if (node.Engine.HasValue)
            {
                var eng = node.Engine.Value;
                var calc = node.GetCalculation();
                if (!calc.IsSingular)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"Burn time: {FormatDuration(calc.DurationSeconds)}", styleLabelDim);
                    GUILayout.Label($"Fuel: {eng.InitialMassKg - calc.FinalMassKg:F1} kg", styleLabelDim);
                    GUILayout.EndHorizontal();
                }
            }

            GUILayout.EndVertical();
        }

        // ─── Time controls ──────────────────────────────────────────────────────
        private void DrawTimeControls(ManeuverNode node, int nodeIndex)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Start (s):", styleLabel, GUILayout.Width(68));

            if (editingTimeIndex == nodeIndex)
            {
                // Editing mode — show user's typed text
                editingTimeText = GUILayout.TextField(editingTimeText, styleInputField, GUILayout.Width(110));
                if (GUILayout.Button("✓", styleButtonSmall, GUILayout.Width(22)))
                {
                    if (double.TryParse(editingTimeText.Replace(',', '.'),
                        System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out double v))
                    {
                        node.StartTime = v;
                        evaluator?.MarkAsDirty();
                    }
                    editingTimeIndex = -1;
                    editingTimeText = null;
                }
                if (GUILayout.Button("✕", styleButtonSmall, GUILayout.Width(22)))
                {
                    editingTimeIndex = -1;
                    editingTimeText = null;
                }
            }
            else
            {
                // Display mode — show formatted value, click to edit
                if (GUILayout.Button(node.StartTime.ToString("F1"), styleInputField, GUILayout.Width(110)))
                {
                    editingTimeIndex = nodeIndex;
                    editingTimeText = node.StartTime.ToString("F1", System.Globalization.CultureInfo.InvariantCulture);
                }

                if (GUILayout.Button("−1h", styleButtonSmall)) { node.StartTime -= 3600; evaluator?.MarkAsDirty(); }
                if (GUILayout.Button("−1m", styleButtonSmall)) { node.StartTime -= 60;   evaluator?.MarkAsDirty(); }
                if (GUILayout.Button("+1m", styleButtonSmall)) { node.StartTime += 60;   evaluator?.MarkAsDirty(); }
                if (GUILayout.Button("+1h", styleButtonSmall)) { node.StartTime += 3600; evaluator?.MarkAsDirty(); }
            }

            GUILayout.EndHorizontal();
        }

        // ─── ΔV axis block ───────────────────────────────────────────────────────
        private void DrawDVAxis(ManeuverNode node, string axisLabel, ref double dv,
            Color axisColor, int nodeIndex, int axisIndex)
        {
            GUILayout.BeginVertical(styleSection);

            GUILayout.BeginHorizontal();
            Color prev = GUI.color;
            GUI.color = axisColor;
            GUILayout.Label("▌", styleLabel, GUILayout.Width(10));
            GUI.color = prev;
            GUILayout.Label(axisLabel, styleLabel, GUILayout.Width(155));
            GUILayout.FlexibleSpace();

            bool isEditingDv = editingDvIndex == nodeIndex && editingDvAxis == axisIndex;

            if (isEditingDv)
            {
                editingDvText = GUILayout.TextField(editingDvText, styleInputField, GUILayout.Width(84));
                if (GUILayout.Button("✓", styleButtonSmall, GUILayout.Width(22)))
                {
                    if (double.TryParse(editingDvText.Replace(',', '.'),
                        System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out double v))
                    {
                        dv = v;
                        evaluator?.MarkAsDirty();
                    }
                    editingDvIndex = -1;
                    editingDvText = null;
                }
                if (GUILayout.Button("✕", styleButtonSmall, GUILayout.Width(22)))
                {
                    editingDvIndex = -1;
                    editingDvText = null;
                }
            }
            else
            {
                GUI.color = dv != 0 ? axisColor : prev;
                if (GUILayout.Button(FormatDV(dv), styleInputField, GUILayout.Width(95)))
                {
                    editingDvIndex = nodeIndex;
                    editingDvAxis = axisIndex;
                    editingDvText = dv.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
                }
                GUI.color = prev;
            }

            GUILayout.EndHorizontal();

            // Increment buttons
            GUILayout.BeginHorizontal();
            foreach (double step in DV_STEPS)
            {
                if (GUILayout.Button($"−{ShortStep(step)}", styleButtonSmall))
                { dv -= step; evaluator?.MarkAsDirty(); }
            }
            GUILayout.FlexibleSpace();
            foreach (double step in DV_STEPS)
            {
                if (GUILayout.Button($"+{ShortStep(step)}", styleButtonSmall))
                { dv += step; evaluator?.MarkAsDirty(); }
            }
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }

        // ─── Settings panel ─────────────────────────────────────────────────────
        private void DrawSettings()
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label("SETTINGS", styleHeader);

            if (universeManager != null)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Orbit history:", styleLabel, GUILayout.Width(90));
                float newHist = GUILayout.HorizontalSlider(universeManager.MoonOrbitHistoryFraction, 0f, 1f);
                if (!Mathf.Approximately(newHist, universeManager.MoonOrbitHistoryFraction))
                    universeManager.MoonOrbitHistoryFraction = newHist;
                string histLabel = universeManager.MoonOrbitHistoryFraction < 0.01f ? "OFF" : $"{universeManager.MoonOrbitHistoryFraction * 100f:F0}%";
                GUILayout.Label(histLabel, universeManager.MoonOrbitHistoryFraction < 0.01f ? styleLabelDim : styleLabelGreen, GUILayout.Width(40));
                GUILayout.EndHorizontal();
            }

            if (GUILayout.Button("Close Settings", styleButton))
            {
                showSettings = false;
            }

            GUILayout.EndVertical();
        }

        // ─── Internal helpers ────────────────────────────────────────────────────
        private void EnsureAtLeastOneNode()
        {
            if (flightPlan == null || flightPlan.Nodes.Count > 0) return;
            flightPlan.Nodes.Add(new ManeuverNode(NowPlus(600)));
        }

        private double NowPlus(double sec)
            => (universeManager?.SimulationTimeSeconds ?? 0) + sec;

        private static Rect ClampToScreen(Rect r)
        {
            r.x = Mathf.Clamp(r.x, 0f, Mathf.Max(0f, Screen.width - r.width));
            r.y = Mathf.Clamp(r.y, 0f, Mathf.Max(0f, Screen.height - r.height));
            return r;
        }

        // ─── Formatters ──────────────────────────────────────────────────────────
        private static string FormatDV(double v)
            => Math.Abs(v) >= 1000 ? $"{v / 1000:F3} km/s" : $"{v:F3} m/s";

        private static string ShortStep(double v)
            => v >= 1000 ? $"{v / 1000:0}k" : (v >= 1 ? $"{v:0}" : $"{v:G2}");

        private static string FormatDuration(double s)
        {
            if (double.IsInfinity(s) || double.IsNaN(s)) return "∞";
            if (s < 0) s = 0;
            const double day = 86400;
            const double year = 31557600; // 365.25 days
            int y = (int)(s / year);
            int d = (int)((s % year) / day);
            int h = (int)((s % day) / 3600);
            int m = (int)((s % 3600) / 60);
            int sec = (int)(s % 60);
            if (y > 0) return $"{y}y {d}d";
            if (d > 0) return $"{d}d {h:00}h";
            if (h > 0) return $"{h}h {m:00}m";
            if (m > 0) return $"{m}m {sec:00}s";
            return $"{sec}s";
        }

        // ─── Style builder ────────────────────────────────────────────────────────
        private void BuildStylesOnce()
        {
            if (stylesBuilt) return;
            stylesBuilt = true;

            Texture2D Tex(Color c) { var t = new Texture2D(1, 1); t.SetPixel(0, 0, c); t.Apply(); return t; }

            Color bgDark = new Color(0.06f, 0.06f, 0.08f, 0.94f);
            Color bgSection = new Color(0.09f, 0.09f, 0.12f, 0.98f);
            Color bgField = new Color(0.03f, 0.03f, 0.05f, 1.00f);
            Color bgBtn = new Color(0.14f, 0.14f, 0.18f, 1.00f);
            Color bgBtnH = new Color(0.22f, 0.22f, 0.28f, 1.00f);
            Color bgActive = new Color(0.12f, 0.26f, 0.12f, 1.00f);
            Color textMain = new Color(0.90f, 0.90f, 0.88f, 1.00f);
            Color textDim = new Color(0.55f, 0.55f, 0.55f, 1.00f);
            Color textGreen = new Color(0.35f, 1.00f, 0.50f, 1.00f);
            Color textYellow = new Color(1.00f, 0.85f, 0.20f, 1.00f);
            Color textRed = new Color(1.00f, 0.35f, 0.25f, 1.00f);

            styleWindow = new GUIStyle(GUI.skin.window)
            { padding = new RectOffset(4, 4, 4, 6), margin = new RectOffset(0, 0, 0, 0), fontSize = 11 };
            styleWindow.normal.background = Tex(bgDark);
            styleWindow.normal.textColor = textMain;
            styleWindow.onNormal.background = styleWindow.normal.background;

            styleHeader = new GUIStyle(GUI.skin.label)
            { fontStyle = FontStyle.Bold, fontSize = 11, padding = new RectOffset(4, 4, 2, 2) };
            styleHeader.normal.textColor = textMain;

            styleLabel = new GUIStyle(GUI.skin.label)
            { fontSize = 11, padding = new RectOffset(2, 2, 1, 1) };
            styleLabel.normal.textColor = textMain;

            styleLabelDim = Cloned(styleLabel, textDim);
            styleLabelGreen = Cloned(styleLabel, textGreen);
            styleLabelYellow = Cloned(styleLabel, textYellow);
            styleLabelRed = Cloned(styleLabel, textRed);

            styleButton = new GUIStyle(GUI.skin.button)
            { fontSize = 11, padding = new RectOffset(6, 6, 3, 3), margin = new RectOffset(2, 2, 1, 1) };
            styleButton.normal.background = Tex(bgBtn);
            styleButton.hover.background = Tex(bgBtnH);
            styleButton.active.background = Tex(bgActive);
            styleButton.normal.textColor = textMain;
            styleButton.hover.textColor = Color.white;
            styleButton.active.textColor = textGreen;

            styleButtonSmall = new GUIStyle(styleButton)
            { fontSize = 10, padding = new RectOffset(3, 3, 2, 2), margin = new RectOffset(1, 1, 1, 1) };

            styleButtonActive = new GUIStyle(styleButton);
            styleButtonActive.normal.background = Tex(bgActive);
            styleButtonActive.normal.textColor = textGreen;

            styleInputField = new GUIStyle(GUI.skin.textField)
            { fontSize = 11, alignment = TextAnchor.MiddleRight, padding = new RectOffset(4, 4, 2, 2), margin = new RectOffset(1, 1, 1, 1) };
            styleInputField.normal.background = Tex(bgField);
            styleInputField.normal.textColor = textYellow;
            styleInputField.hover.background = Tex(bgField);
            styleInputField.hover.textColor = Color.white;
            styleInputField.focused.background = Tex(bgField);
            styleInputField.focused.textColor = Color.white;

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

// ============================================================================
// РАСШИРЕНИЕ UI ДЛЯ АНАЛИЗА ОРБИТ
// ============================================================================
// Компонент для отображения анализа орбит в планировщике манёвров

using System;
using UnityEngine;
using Galilego.Physics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// UI компонент для отображения анализа орбит.
    /// Работает совместно с FlightPlanUI и OrbitAnalyzer.
    /// </summary>
    public class OrbitAnalysisUI : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private OrbitAnalyzer orbitAnalyzer;
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private ManeuverEvaluator evaluator;
        
        [Header("Window Settings")]
        [SerializeField] private Rect windowRect = new Rect(440f, 16f, 360f, 480f);
        [SerializeField] private bool showWindow = false;
        [SerializeField] private KeyCode toggleKey = KeyCode.O;
        
        [Header("Analysis Settings")]
        [SerializeField] private ReferenceFrameTarget analysisTarget = ReferenceFrameTarget.Jupiter;
        [SerializeField] private bool showBeforeManeuver = true;
        [SerializeField] private bool showAfterManeuver = true;
        [SerializeField] private bool autoUpdate = true;
        [SerializeField] private float updateInterval = 0.5f;
        
        private OrbitAnalysisResult currentOrbit;
        private OrbitAnalysisResult orbitAfterManeuver;
        private float lastUpdateTime;
        private Vector2 scrollPosition;
        
        // GUIStyle cache
        private GUIStyle styleWindow, styleHeader, styleLabel, styleLabelDim;
        private GUIStyle styleLabelGreen, styleLabelYellow, styleLabelRed;
        private GUIStyle styleButton, styleButtonSmall, styleSection;
        private bool stylesBuilt;
        
        private void Awake()
        {
            if (orbitAnalyzer == null)
                orbitAnalyzer = FindAnyObjectByType<OrbitAnalyzer>();
            
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();
            
            if (evaluator == null)
                evaluator = FindAnyObjectByType<ManeuverEvaluator>();
        }
        
        private void Update()
        {
            // Toggle window
            if (Input.GetKeyDown(toggleKey))
            {
                showWindow = !showWindow;
            }
            
            // Auto update
            if (autoUpdate && showWindow && Time.time - lastUpdateTime >= updateInterval)
            {
                UpdateAnalysis();
                lastUpdateTime = Time.time;
            }
        }
        
        private void OnGUI()
        {
            if (!showWindow) return;
            
            BuildStylesOnce();
            windowRect = GUILayout.Window(
                unchecked((int)0xF1A9_0002), 
                windowRect, 
                DrawWindow, 
                GUIContent.none, 
                styleWindow);
            windowRect = ClampToScreen(windowRect);
        }
        
        private void DrawWindow(int id)
        {
            // Title bar
            GUILayout.BeginHorizontal(styleSection);
            GUILayout.Label("✦  ORBIT ANALYSIS", styleHeader);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("↻", styleButtonSmall))
            {
                UpdateAnalysis();
            }
            if (GUILayout.Button("✕", styleButtonSmall))
            {
                showWindow = false;
            }
            GUILayout.EndHorizontal();
            
            // Target selection
            DrawTargetSelector();
            
            // Analysis content
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, true);
            
            if (showBeforeManeuver)
            {
                DrawOrbitSection("CURRENT ORBIT", currentOrbit);
            }
            
            if (showAfterManeuver)
            {
                GUILayout.Space(8);
                DrawOrbitSection("AFTER MANEUVER", orbitAfterManeuver);
            }
            
            GUILayout.EndScrollView();
            
            // Settings
            DrawSettings();
            
            GUI.DragWindow(new Rect(0, 0, windowRect.width, 20));
        }
        
        private void DrawTargetSelector()
        {
            GUILayout.BeginHorizontal(styleSection);
            GUILayout.Label("Target:", styleLabel, GUILayout.Width(50));
            
            string[] targetNames = Enum.GetNames(typeof(ReferenceFrameTarget));
            int currentIndex = (int)analysisTarget;
            
            for (int i = 0; i < targetNames.Length; i++)
            {
                bool isSelected = i == currentIndex;
                if (GUILayout.Toggle(isSelected, targetNames[i], 
                    isSelected ? styleButton : styleButtonSmall))
                {
                    if (i != currentIndex)
                    {
                        analysisTarget = (ReferenceFrameTarget)i;
                        UpdateAnalysis();
                    }
                }
            }
            
            GUILayout.EndHorizontal();
        }
        
        private void DrawOrbitSection(string title, OrbitAnalysisResult result)
        {
            GUILayout.BeginVertical(styleSection);
            GUILayout.Label(title, styleHeader);
            
            if (!result.IsValid)
            {
                GUILayout.Label("No valid orbit data", styleLabelDim);
                GUILayout.EndVertical();
                return;
            }
            
            var elements = result.Elements;
            
            // Orbital parameters
            DrawParameter("Periapsis", FormatDistance(elements.PeriapsisDistance), 
                elements.PeriapsisDistance < result.BodyRadius ? styleLabelRed : styleLabel);
            DrawParameter("Apoapsis", FormatDistance(elements.ApoapsisDistance), 
                elements.ApoapsisDistance > result.SphereOfInfluence ? styleLabelYellow : styleLabel);
            DrawParameter("Semi-major axis", FormatDistance(elements.SemiMajorAxis), styleLabel);
            DrawParameter("Eccentricity", $"{elements.Eccentricity:F4}", styleLabel);
            DrawParameter("Inclination", $"{elements.InclinationDegrees:F2}°", styleLabel);
            DrawParameter("Period", FormatDuration(elements.OrbitalPeriodSeconds), styleLabel);
            
            GUILayout.Space(4);
            
            // Advanced parameters
            DrawParameter("LAN", $"{elements.LongitudeOfAscendingNodeDegrees:F2}°", styleLabelDim);
            DrawParameter("Arg. of Pe", $"{elements.ArgumentOfPeriapsisDegrees:F2}°", styleLabelDim);
            DrawParameter("True Anomaly", $"{elements.TrueAnomalyDegrees:F2}°", styleLabelDim);
            DrawParameter("Energy", $"{elements.SpecificOrbitalEnergy:F2} J/kg", styleLabelDim);
            
            GUILayout.Space(4);
            
            // Time to apsides
            if (!double.IsInfinity(result.TimeToPeriapsis))
            {
                DrawParameter("Time to Pe", FormatDuration(result.TimeToPeriapsis), styleLabelGreen);
            }
            
            if (!double.IsInfinity(result.TimeToApoapsis))
            {
                DrawParameter("Time to Ap", FormatDuration(result.TimeToApoapsis), styleLabelGreen);
            }
            
            GUILayout.Space(4);
            
            // Warnings
            if (result.WillImpact)
            {
                GUILayout.Label($"⚠ IMPACT in {FormatDuration(result.ImpactTime)}", styleLabelRed);
            }
            else if (result.WillEscape)
            {
                GUILayout.Label($"⚠ ESCAPE in {FormatDuration(result.EscapeTime)}", styleLabelYellow);
            }
            else if (result.IsStable)
            {
                GUILayout.Label("✓ Stable orbit", styleLabelGreen);
            }
            
            GUILayout.EndVertical();
        }
        
        private void DrawParameter(string label, string value, GUIStyle valueStyle)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label + ":", styleLabel, GUILayout.Width(120));
            GUILayout.Label(value, valueStyle);
            GUILayout.EndHorizontal();
        }
        
        private void DrawSettings()
        {
            GUILayout.BeginVertical(styleSection);
            
            GUILayout.BeginHorizontal();
            showBeforeManeuver = GUILayout.Toggle(showBeforeManeuver, "Show Current", styleButtonSmall);
            showAfterManeuver = GUILayout.Toggle(showAfterManeuver, "Show After", styleButtonSmall);
            autoUpdate = GUILayout.Toggle(autoUpdate, "Auto Update", styleButtonSmall);
            GUILayout.EndHorizontal();
            
            GUILayout.EndVertical();
        }
        
        private void UpdateAnalysis()
        {
            if (orbitAnalyzer == null || universeManager == null)
                return;
            
            // Analyze current orbit
            currentOrbit = orbitAnalyzer.AnalyzeShipOrbit(analysisTarget);
            
            // Analyze orbit after maneuver (if flight plan exists)
            var flightPlan = evaluator != null ? evaluator.GetFlightPlan() : null;
            if (flightPlan != null && flightPlan.Nodes.Count > 0)
            {
                // Get first maneuver
                var node = flightPlan.Nodes[0];
                
                // Get current state
                if (universeManager.TryGetShipRelativeState(
                    analysisTarget,
                    out _,
                    out Vector3d relativePos,
                    out Vector3d relativeVel,
                    out _,
                    out _,
                    out _))
                {
                    // Calculate delta-v in world space
                    Vector3d deltaV = FlightPlan.CalculateWorldDeltaV(relativePos, relativeVel, node);
                    
                    // Analyze orbit after maneuver
                    orbitAfterManeuver = orbitAnalyzer.AnalyzeOrbitAfterManeuver(
                        analysisTarget,
                        universeManager.ShipBody.Position,
                        universeManager.ShipBody.Velocity,
                        deltaV);
                }
            }
            else
            {
                orbitAfterManeuver = OrbitAnalysisResult.Invalid;
            }
        }
        
        private FlightPlan GetFlightPlan()
        {
            return evaluator != null ? evaluator.GetFlightPlan() : null;
        }
        
        // Formatting helpers
        private string FormatDistance(double meters)
        {
            if (double.IsInfinity(meters)) return "∞";
            if (double.IsNaN(meters)) return "N/A";
            if (meters >= 1e9) return $"{meters / 1e9:F2} Gm";
            if (meters >= 1e6) return $"{meters / 1e6:F2} Mm";
            if (meters >= 1e3) return $"{meters / 1e3:F2} km";
            return $"{meters:F0} m";
        }
        
        private string FormatDuration(double seconds)
        {
            if (double.IsInfinity(seconds) || double.IsNaN(seconds)) return "∞";
            if (seconds < 0) return "N/A";
            
            int days = (int)(seconds / 86400);
            int hours = (int)((seconds % 86400) / 3600);
            int minutes = (int)((seconds % 3600) / 60);
            int secs = (int)(seconds % 60);
            
            if (days > 0) return $"{days}d {hours:00}h {minutes:00}m";
            if (hours > 0) return $"{hours}h {minutes:00}m {secs:00}s";
            if (minutes > 0) return $"{minutes}m {secs:00}s";
            return $"{secs}s";
        }
        
        private static Rect ClampToScreen(Rect r)
        {
            r.x = Mathf.Clamp(r.x, 0f, Mathf.Max(0f, Screen.width - r.width));
            r.y = Mathf.Clamp(r.y, 0f, Mathf.Max(0f, Screen.height - r.height));
            return r;
        }
        
        // Style builder
        private void BuildStylesOnce()
        {
            if (stylesBuilt) return;
            stylesBuilt = true;
            
            Texture2D Tex(Color c) { var t = new Texture2D(1, 1); t.SetPixel(0, 0, c); t.Apply(); return t; }
            
            Color bgDark = new Color(0.06f, 0.06f, 0.08f, 0.94f);
            Color bgSection = new Color(0.09f, 0.09f, 0.12f, 0.98f);
            Color bgBtn = new Color(0.14f, 0.14f, 0.18f, 1.00f);
            Color bgBtnH = new Color(0.22f, 0.22f, 0.28f, 1.00f);
            Color textMain = new Color(0.90f, 0.90f, 0.88f, 1.00f);
            Color textDim = new Color(0.55f, 0.55f, 0.55f, 1.00f);
            Color textGreen = new Color(0.35f, 1.00f, 0.50f, 1.00f);
            Color textYellow = new Color(1.00f, 0.85f, 0.20f, 1.00f);
            Color textRed = new Color(1.00f, 0.35f, 0.25f, 1.00f);
            
            styleWindow = new GUIStyle(GUI.skin.window)
            { padding = new RectOffset(4, 4, 4, 6), margin = new RectOffset(0, 0, 0, 0), fontSize = 11 };
            styleWindow.normal.background = Tex(bgDark);
            styleWindow.normal.textColor = textMain;
            
            styleHeader = new GUIStyle(GUI.skin.label)
            { fontStyle = FontStyle.Bold, fontSize = 11, padding = new RectOffset(4, 4, 2, 2) };
            styleHeader.normal.textColor = textMain;
            
            styleLabel = new GUIStyle(GUI.skin.label)
            { fontSize = 11, padding = new RectOffset(2, 2, 1, 1) };
            styleLabel.normal.textColor = textMain;
            
            styleLabelDim = new GUIStyle(styleLabel);
            styleLabelDim.normal.textColor = textDim;
            
            styleLabelGreen = new GUIStyle(styleLabel);
            styleLabelGreen.normal.textColor = textGreen;
            
            styleLabelYellow = new GUIStyle(styleLabel);
            styleLabelYellow.normal.textColor = textYellow;
            
            styleLabelRed = new GUIStyle(styleLabel);
            styleLabelRed.normal.textColor = textRed;
            
            styleButton = new GUIStyle(GUI.skin.button)
            { fontSize = 11, padding = new RectOffset(6, 6, 3, 3), margin = new RectOffset(2, 2, 1, 1) };
            styleButton.normal.background = Tex(bgBtn);
            styleButton.hover.background = Tex(bgBtnH);
            styleButton.normal.textColor = textMain;
            
            styleButtonSmall = new GUIStyle(styleButton)
            { fontSize = 10, padding = new RectOffset(3, 3, 2, 2), margin = new RectOffset(1, 1, 1, 1) };
            
            styleSection = new GUIStyle(GUI.skin.box)
            { padding = new RectOffset(4, 4, 3, 3), margin = new RectOffset(0, 0, 2, 2) };
            styleSection.normal.background = Tex(bgSection);
        }
    }
}

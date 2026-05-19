using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Galilego.Physics
{
    public sealed class SpeedometerUI : MonoBehaviour
    {
        private enum OrbitEvent
        {
            Periapsis,
            Apoapsis
        }

        private readonly struct MetricReadout
        {
            public readonly string Digits;
            public readonly string Suffix;

            public MetricReadout(string digits, string suffix)
            {
                Digits = digits;
                Suffix = suffix;
            }
        }

        private readonly struct VerticalSpeedReadout
        {
            public readonly string Digits;
            public readonly bool UsesKilometersPerSecond;

            public VerticalSpeedReadout(string digits, bool usesKilometersPerSecond)
            {
                Digits = digits;
                UsesKilometersPerSecond = usesKilometersPerSecond;
            }
        }

        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        [SerializeField] private TMP_Text frameLabel;
        [SerializeField] private TMP_Text speedLabel;
        [SerializeField] private TMP_Text radialSpeedLabel;
        [SerializeField] private TMP_Text tangentialSpeedLabel;
        [SerializeField] private TMP_Text altitudeLabel;
        [SerializeField] private TMP_Text distanceLabel;
        [SerializeField] private TMP_Text periapsisLabel;
        [SerializeField] private TMP_Text apoapsisLabel;
        [SerializeField] private Image speedFill;
        [SerializeField] private RectTransform speedNeedle;

        [Header("Digital Readouts")]
        [SerializeField] private PixelReadoutGraphic altitudeDigitsReadout;
        [SerializeField] private PixelReadoutGraphic altitudeSuffixReadout;
        [SerializeField] private TMP_Text altitudeSuffixLabel;
        [SerializeField] private PixelReadoutGraphic verticalSpeedDigitsReadout;
        [SerializeField] private TMP_Text metersPerSecondUnitLabel;
        [SerializeField] private TMP_Text kilometersPerSecondUnitLabel;
        [SerializeField] private Graphic metersPerSecondUnitIndicator;
        [SerializeField] private Graphic kilometersPerSecondUnitIndicator;
        [SerializeField] private PixelReadoutGraphic timeToPeriapsisReadout;
        [SerializeField] private TMP_Text timeToPeriapsisLabel;
        [SerializeField] private PixelReadoutGraphic timeToApoapsisReadout;
        [SerializeField] private TMP_Text timeToApoapsisLabel;

        [Header("Digital Style")]
        [SerializeField] private Color readoutColor = new Color(0.72f, 0.92f, 1f, 1f);
        [SerializeField] private Color activeUnitColor = new Color(0.32f, 0.84f, 1f, 1f);
        [SerializeField] private Color inactiveUnitColor = new Color(0.28f, 0.32f, 0.35f, 0.7f);
        [SerializeField] private bool wrapTmpReadoutsInMonospace = true;
        [SerializeField] private string tmpMonospaceWidth = "0.62em";
        [SerializeField] private double verticalSpeedKilometersThresholdMetersPerSecond = 9999d;

        [Header("Scale")]
        [SerializeField] private double fullScaleSpeedMetersPerSecond = 20000d;
        [SerializeField] private float needleMinAngle = 130f;
        [SerializeField] private float needleMaxAngle = -130f;

        private static readonly string[] MetricSuffixes = { string.Empty, "K", "M", "G", "T", "P" };

        private void Update()
        {
            ResolveReferences();

            if (universeManager == null)
            {
                return;
            }

            ReferenceFrameTarget activeFrame = universeManager.ActiveReferenceFrame;
            if (!universeManager.TryGetShipRelativeState(
                activeFrame,
                out string frameName,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out _,
                out double frameRadius,
                out _))
            {
                SetUnavailable();
                return;
            }

            double distance = relativePosition.Magnitude;
            double speed = relativeVelocity.Magnitude;
            double radialSpeed = distance > 0d ? Vector3d.Dot(relativePosition, relativeVelocity) / distance : 0d;
            double tangentialSpeed = Math.Sqrt(Math.Max(0d, relativeVelocity.SqrMagnitude - (radialSpeed * radialSpeed)));
            double altitude = distance - frameRadius;
            OrbitalElements orbit = universeManager.GetShipOrbitAround(activeFrame);
            MetricReadout altitudeReadout = FormatMetricReadout(altitude);
            VerticalSpeedReadout verticalSpeedReadout = FormatVerticalSpeedReadout(radialSpeed);
            string periapsisTime = FormatOrbitEventTime(orbit, OrbitEvent.Periapsis);
            string apoapsisTime = FormatOrbitEventTime(orbit, OrbitEvent.Apoapsis);

            SetText(frameLabel, frameName);
            SetMetricReadout(altitudeDigitsReadout, altitudeLabel, altitudeSuffixReadout, altitudeSuffixLabel, altitudeReadout);
            SetDigitalReadout(verticalSpeedDigitsReadout, speedLabel, verticalSpeedReadout.Digits);
            SetTextIfDistinct(radialSpeedLabel, speedLabel, FormatSignedSpeed(radialSpeed));
            SetDigitalReadout(timeToPeriapsisReadout, ResolveTimeToPeriapsisLabel(), periapsisTime);
            SetDigitalReadout(timeToApoapsisReadout, ResolveTimeToApoapsisLabel(), apoapsisTime);
            SetVerticalSpeedUnitState(verticalSpeedReadout.UsesKilometersPerSecond);

            SetText(tangentialSpeedLabel, FormatSpeed(tangentialSpeed));
            SetText(distanceLabel, FormatDistance(distance));

            double fullScale = Math.Max(1d, fullScaleSpeedMetersPerSecond);
            float normalizedSpeed = Mathf.Clamp01((float)(speed / fullScale));

            if (speedFill != null)
            {
                speedFill.fillAmount = normalizedSpeed;
            }

            if (speedNeedle != null)
            {
                float angle = Mathf.Lerp(needleMinAngle, needleMaxAngle, normalizedSpeed);
                speedNeedle.localRotation = Quaternion.Euler(0f, 0f, angle);
            }
        }

        private void SetUnavailable()
        {
            SetText(frameLabel, "n/a");
            SetMetricReadout(
                altitudeDigitsReadout,
                altitudeLabel,
                altitudeSuffixReadout,
                altitudeSuffixLabel,
                new MetricReadout("------", string.Empty));
            SetDigitalReadout(verticalSpeedDigitsReadout, speedLabel, "------");
            SetTextIfDistinct(radialSpeedLabel, speedLabel, "n/a");
            SetText(tangentialSpeedLabel, "n/a");
            SetText(distanceLabel, "n/a");
            SetDigitalReadout(timeToPeriapsisReadout, ResolveTimeToPeriapsisLabel(), "--:--:--");
            SetDigitalReadout(timeToApoapsisReadout, ResolveTimeToApoapsisLabel(), "--:--:--");
            SetVerticalSpeedUnitState(false);
        }

        private void ResolveReferences()
        {
            if (universeManager == null)
            {
                universeManager = FindAnyObjectByType<UniverseManager>();
            }
        }

        private TMP_Text ResolveTimeToPeriapsisLabel()
        {
            return timeToPeriapsisLabel != null ? timeToPeriapsisLabel : periapsisLabel;
        }

        private TMP_Text ResolveTimeToApoapsisLabel()
        {
            return timeToApoapsisLabel != null ? timeToApoapsisLabel : apoapsisLabel;
        }

        private static void SetText(TMP_Text text, string value)
        {
            if (text != null)
            {
                text.text = value;
            }
        }

        private static void SetTextIfDistinct(TMP_Text text, TMP_Text otherText, string value)
        {
            if (text != null && text != otherText)
            {
                text.text = value;
            }
        }

        private void SetMetricReadout(
            PixelReadoutGraphic digitsReadout,
            TMP_Text digitsLabel,
            PixelReadoutGraphic suffixReadout,
            TMP_Text suffixLabel,
            MetricReadout readout)
        {
            bool hasSeparateSuffix = suffixReadout != null || suffixLabel != null;
            SetDigitalReadout(digitsReadout, digitsLabel, hasSeparateSuffix ? readout.Digits : readout.Digits + readout.Suffix);
            SetDigitalReadout(suffixReadout, suffixLabel, readout.Suffix);
        }

        private void SetDigitalReadout(PixelReadoutGraphic readout, TMP_Text text, string value)
        {
            if (readout != null)
            {
                readout.SetReadoutColor(readoutColor);
                readout.SetText(value);
            }

            if (text != null)
            {
                text.color = readoutColor;
                text.text = ShouldWrapTmpReadout(value) ? WrapTmpMonospace(value) : value;
            }
        }

        private bool ShouldWrapTmpReadout(string value)
        {
            return wrapTmpReadoutsInMonospace && !string.IsNullOrWhiteSpace(tmpMonospaceWidth) && !string.IsNullOrEmpty(value);
        }

        private string WrapTmpMonospace(string value)
        {
            return $"<mspace={tmpMonospaceWidth}>{value}</mspace>";
        }

        private void SetVerticalSpeedUnitState(bool useKilometersPerSecond)
        {
            SetGraphicColor(metersPerSecondUnitLabel, useKilometersPerSecond ? inactiveUnitColor : activeUnitColor);
            SetGraphicColor(kilometersPerSecondUnitLabel, useKilometersPerSecond ? activeUnitColor : inactiveUnitColor);
            SetGraphicColor(metersPerSecondUnitIndicator, useKilometersPerSecond ? inactiveUnitColor : activeUnitColor);
            SetGraphicColor(kilometersPerSecondUnitIndicator, useKilometersPerSecond ? activeUnitColor : inactiveUnitColor);
        }

        private static void SetGraphicColor(Graphic graphic, Color value)
        {
            if (graphic != null)
            {
                graphic.color = value;
            }
        }

        private static string FormatSignedSpeed(double metersPerSecond)
        {
            return metersPerSecond >= 0d
                ? $"+{FormatSpeed(metersPerSecond)}"
                : FormatSpeed(metersPerSecond);
        }

        private static string FormatSpeed(double metersPerSecond)
        {
            if (double.IsNaN(metersPerSecond))
            {
                return "n/a";
            }

            double absolute = Math.Abs(metersPerSecond);
            if (absolute >= 1000d)
            {
                return $"{metersPerSecond / 1000d:0.###} km/s";
            }

            return $"{metersPerSecond:0.###} m/s";
        }

        private static string FormatDistance(double meters)
        {
            if (double.IsInfinity(meters))
            {
                return "open";
            }

            if (double.IsNaN(meters))
            {
                return "n/a";
            }

            double absolute = Math.Abs(meters);
            if (absolute >= 1e9d)
            {
                return $"{meters / 1e9d:0.###} Gm";
            }

            if (absolute >= 1e6d)
            {
                return $"{meters / 1e6d:0.###} Mm";
            }

            if (absolute >= 1e3d)
            {
                return $"{meters / 1e3d:0.###} km";
            }

            return $"{meters:0.###} m";
        }

        private static MetricReadout FormatMetricReadout(double meters)
        {
            if (double.IsNaN(meters) || double.IsInfinity(meters))
            {
                return new MetricReadout("------", string.Empty);
            }

            double absolute = Math.Abs(meters);
            int suffixIndex = 0;
            double scale = 1d;
            while (suffixIndex < MetricSuffixes.Length - 1 && absolute / scale > 999999.5d)
            {
                suffixIndex++;
                scale *= 1000d;
            }

            double scaled = meters / scale;
            double rounded = Math.Round(Math.Abs(scaled), MidpointRounding.AwayFromZero);
            if (rounded > 999999d && suffixIndex < MetricSuffixes.Length - 1)
            {
                suffixIndex++;
                scale *= 1000d;
                scaled = meters / scale;
            }

            return new MetricReadout(FormatSignedSixDigits(scaled), MetricSuffixes[suffixIndex]);
        }

        private VerticalSpeedReadout FormatVerticalSpeedReadout(double metersPerSecond)
        {
            if (double.IsNaN(metersPerSecond) || double.IsInfinity(metersPerSecond))
            {
                return new VerticalSpeedReadout("------", false);
            }

            double threshold = Math.Max(1d, verticalSpeedKilometersThresholdMetersPerSecond);
            bool useKilometersPerSecond = Math.Abs(metersPerSecond) > threshold;
            double displayValue = useKilometersPerSecond ? metersPerSecond / 1000d : metersPerSecond;
            return new VerticalSpeedReadout(FormatSignedSixDigits(displayValue), useKilometersPerSecond);
        }

        private static string FormatSignedSixDigits(double value)
        {
            long rounded = RoundAbsoluteToLong(value);
            if (value < 0d)
            {
                return "-" + Math.Min(rounded, 99999L).ToString("D5");
            }

            return Math.Min(rounded, 999999L).ToString("D6");
        }

        private static long RoundAbsoluteToLong(double value)
        {
            double absolute = Math.Abs(value);
            if (absolute >= 999999d)
            {
                return 999999L;
            }

            return (long)Math.Round(absolute, MidpointRounding.AwayFromZero);
        }

        private static string FormatOrbitEventTime(OrbitalElements orbit, OrbitEvent orbitEvent)
        {
            if (!TryGetTimeToOrbitEvent(orbit, orbitEvent, out double seconds))
            {
                return "--:--:--";
            }

            return FormatDigitalDuration(seconds);
        }

        private static bool TryGetTimeToOrbitEvent(OrbitalElements orbit, OrbitEvent orbitEvent, out double seconds)
        {
            seconds = 0d;
            if (!orbit.IsValid ||
                !orbit.IsBound ||
                orbit.OrbitalPeriodSeconds <= 0d ||
                double.IsNaN(orbit.OrbitalPeriodSeconds) ||
                double.IsInfinity(orbit.OrbitalPeriodSeconds) ||
                double.IsNaN(orbit.MeanAnomalyDegrees) ||
                double.IsInfinity(orbit.MeanAnomalyDegrees))
            {
                return false;
            }

            double currentMeanAnomalyFraction = NormalizeDegrees01(orbit.MeanAnomalyDegrees);
            double targetFraction = orbitEvent == OrbitEvent.Apoapsis ? 0.5d : 0d;
            double deltaFraction = targetFraction - currentMeanAnomalyFraction;
            if (deltaFraction < 0d)
            {
                deltaFraction += 1d;
            }

            if (orbitEvent == OrbitEvent.Periapsis && deltaFraction > 0.999999d)
            {
                deltaFraction = 0d;
            }

            seconds = deltaFraction * orbit.OrbitalPeriodSeconds;
            return !double.IsNaN(seconds) && !double.IsInfinity(seconds) && seconds >= 0d;
        }

        private static double NormalizeDegrees01(double degrees)
        {
            degrees %= 360d;
            if (degrees < 0d)
            {
                degrees += 360d;
            }

            return degrees / 360d;
        }

        private static string FormatDigitalDuration(double seconds)
        {
            if (seconds < 0d || double.IsNaN(seconds) || double.IsInfinity(seconds))
            {
                return "--:--:--";
            }

            long totalSeconds = (long)Math.Round(seconds, MidpointRounding.AwayFromZero);
            long days = totalSeconds / 86400L;
            long hours = (totalSeconds / 3600L) % 24L;
            long minutes = (totalSeconds / 60L) % 60L;
            long remainingSeconds = totalSeconds % 60L;

            if (days > 0L)
            {
                return days <= 99L
                    ? $"{days:00}:{hours:00}:{minutes:00}"
                    : $"{Math.Min(days, 9999L):0000}D";
            }

            return $"{hours:00}:{minutes:00}:{remainingSeconds:00}";
        }
    }

    [ExecuteAlways]
    public sealed partial class PixelReadoutGraphic : MaskableGraphic
    {
        [SerializeField] private string text = "000000";
        [SerializeField] private bool fitToRect = true;
        [SerializeField] private float pixelSize = 4f;
        [SerializeField] private float pixelSpacing = 0.25f;
        [SerializeField] private float characterSpacing = 1f;
        [SerializeField] private TextAnchor alignment = TextAnchor.MiddleRight;

        private const int GlyphWidth = 5;
        private const int GlyphHeight = 7;

        private static readonly string[] SpaceGlyph =
        {
            "     ",
            "     ",
            "     ",
            "     ",
            "     ",
            "     ",
            "     "
        };

        private static readonly Dictionary<char, string[]> Glyphs = new Dictionary<char, string[]>
        {
            ['0'] = new[] { " ### ", "#   #", "#  ##", "# # #", "##  #", "#   #", " ### " },
            ['1'] = new[] { "  #  ", " ##  ", "# #  ", "  #  ", "  #  ", "  #  ", "#####" },
            ['2'] = new[] { " ### ", "#   #", "    #", "   # ", "  #  ", " #   ", "#####" },
            ['3'] = new[] { "#### ", "    #", "    #", " ### ", "    #", "    #", "#### " },
            ['4'] = new[] { "#   #", "#   #", "#   #", "#####", "    #", "    #", "    #" },
            ['5'] = new[] { "#####", "#    ", "#    ", "#### ", "    #", "    #", "#### " },
            ['6'] = new[] { " ### ", "#    ", "#    ", "#### ", "#   #", "#   #", " ### " },
            ['7'] = new[] { "#####", "    #", "   # ", "  #  ", " #   ", " #   ", " #   " },
            ['8'] = new[] { " ### ", "#   #", "#   #", " ### ", "#   #", "#   #", " ### " },
            ['9'] = new[] { " ### ", "#   #", "#   #", " ####", "    #", "    #", " ### " },
            ['A'] = new[] { " ### ", "#   #", "#   #", "#####", "#   #", "#   #", "#   #" },
            ['D'] = new[] { "#### ", "#   #", "#   #", "#   #", "#   #", "#   #", "#### " },
            ['E'] = new[] { "#####", "#    ", "#    ", "#### ", "#    ", "#    ", "#####" },
            ['G'] = new[] { " ### ", "#   #", "#    ", "# ###", "#   #", "#   #", " ### " },
            ['K'] = new[] { "#   #", "#  # ", "# #  ", "##   ", "# #  ", "#  # ", "#   #" },
            ['M'] = new[] { "#   #", "## ##", "# # #", "#   #", "#   #", "#   #", "#   #" },
            ['N'] = new[] { "#   #", "##  #", "# # #", "#  ##", "#   #", "#   #", "#   #" },
            ['O'] = new[] { " ### ", "#   #", "#   #", "#   #", "#   #", "#   #", " ### " },
            ['P'] = new[] { "#### ", "#   #", "#   #", "#### ", "#    ", "#    ", "#    " },
            ['T'] = new[] { "#####", "  #  ", "  #  ", "  #  ", "  #  ", "  #  ", "  #  " },
            ['-'] = new[] { "     ", "     ", "     ", "#####", "     ", "     ", "     " },
            ['+'] = new[] { "     ", "  #  ", "  #  ", "#####", "  #  ", "  #  ", "     " },
            [':'] = new[] { "     ", "  #  ", "  #  ", "     ", "  #  ", "  #  ", "     " },
            ['.'] = new[] { "     ", "     ", "     ", "     ", "     ", " ##  ", " ##  " },
            ['/'] = new[] { "    #", "   # ", "   # ", "  #  ", " #   ", " #   ", "#    " },
            [' '] = SpaceGlyph
        };

        public string Text
        {
            get => text;
            set
            {
                string nextText = value ?? string.Empty;
                if (text == nextText)
                {
                    return;
                }

                text = nextText;
                SetVerticesDirty();
            }
        }

        public void SetText(string value)
        {
            Text = value;
        }

        public void SetReadoutColor(Color value)
        {
            color = value;
            SetVerticesDirty();
        }

        protected override void OnPopulateMesh(VertexHelper vertexHelper)
        {
            vertexHelper.Clear();

            string displayText = string.IsNullOrEmpty(text) ? string.Empty : text.ToUpperInvariant();
            if (displayText.Length == 0)
            {
                return;
            }

            Rect rect = rectTransform.rect;
            float blockSize = ResolveBlockSize(displayText, rect);
            float gap = fitToRect ? blockSize * Mathf.Max(0f, pixelSpacing) : Mathf.Max(0f, pixelSpacing);
            float characterGap = fitToRect ? blockSize * Mathf.Max(0f, characterSpacing) : Mathf.Max(0f, characterSpacing);
            Vector2 readoutSize = CalculateTextSize(displayText, blockSize, gap, characterGap);
            Vector2 start = ResolveStartPosition(rect, readoutSize);
            float penX = start.x;

            for (int i = 0; i < displayText.Length; i++)
            {
                string[] glyph = ResolveGlyph(displayText[i]);
                AddGlyph(vertexHelper, glyph, penX, start.y + readoutSize.y, blockSize, gap);
                penX += CalculateGlyphWidth(blockSize, gap);

                if (i < displayText.Length - 1)
                {
                    penX += characterGap;
                }
            }
        }

        private float ResolveBlockSize(string displayText, Rect rect)
        {
            if (!fitToRect)
            {
                return Mathf.Max(0.1f, pixelSize);
            }

            float normalizedGap = Mathf.Max(0f, pixelSpacing);
            float normalizedCharacterGap = Mathf.Max(0f, characterSpacing);
            float widthUnits =
                (displayText.Length * GlyphWidth) +
                (displayText.Length * (GlyphWidth - 1) * normalizedGap) +
                (Mathf.Max(0, displayText.Length - 1) * normalizedCharacterGap);
            float heightUnits = GlyphHeight + ((GlyphHeight - 1) * normalizedGap);

            if (widthUnits <= 0f || heightUnits <= 0f)
            {
                return Mathf.Max(0.1f, pixelSize);
            }

            float widthSize = rect.width / widthUnits;
            float heightSize = rect.height / heightUnits;
            return Mathf.Max(0.1f, Mathf.Min(widthSize, heightSize));
        }

        private static Vector2 CalculateTextSize(string displayText, float blockSize, float gap, float characterGap)
        {
            float glyphWidth = CalculateGlyphWidth(blockSize, gap);
            float width = (displayText.Length * glyphWidth) + (Mathf.Max(0, displayText.Length - 1) * characterGap);
            float height = (GlyphHeight * blockSize) + ((GlyphHeight - 1) * gap);
            return new Vector2(width, height);
        }

        private static float CalculateGlyphWidth(float blockSize, float gap)
        {
            return (GlyphWidth * blockSize) + ((GlyphWidth - 1) * gap);
        }

        private Vector2 ResolveStartPosition(Rect rect, Vector2 readoutSize)
        {
            float x = rect.xMin;
            if (alignment == TextAnchor.UpperCenter ||
                alignment == TextAnchor.MiddleCenter ||
                alignment == TextAnchor.LowerCenter)
            {
                x = rect.xMin + ((rect.width - readoutSize.x) * 0.5f);
            }
            else if (alignment == TextAnchor.UpperRight ||
                alignment == TextAnchor.MiddleRight ||
                alignment == TextAnchor.LowerRight)
            {
                x = rect.xMax - readoutSize.x;
            }

            float y = rect.yMin;
            if (alignment == TextAnchor.MiddleLeft ||
                alignment == TextAnchor.MiddleCenter ||
                alignment == TextAnchor.MiddleRight)
            {
                y = rect.yMin + ((rect.height - readoutSize.y) * 0.5f);
            }
            else if (alignment == TextAnchor.UpperLeft ||
                alignment == TextAnchor.UpperCenter ||
                alignment == TextAnchor.UpperRight)
            {
                y = rect.yMax - readoutSize.y;
            }

            return new Vector2(x, y);
        }

        private static string[] ResolveGlyph(char character)
        {
            return Glyphs.TryGetValue(character, out string[] glyph) ? glyph : SpaceGlyph;
        }

        private void AddGlyph(
            VertexHelper vertexHelper,
            string[] glyph,
            float x,
            float topY,
            float blockSize,
            float gap)
        {
            Color32 vertexColor = color;
            for (int row = 0; row < GlyphHeight; row++)
            {
                string pattern = glyph[row];
                for (int column = 0; column < GlyphWidth; column++)
                {
                    if (pattern[column] == ' ')
                    {
                        continue;
                    }

                    float blockX = x + (column * (blockSize + gap));
                    float blockY = topY - ((row + 1) * blockSize) - (row * gap);
                    AddQuad(vertexHelper, blockX, blockY, blockSize, vertexColor);
                }
            }
        }

        private static void AddQuad(VertexHelper vertexHelper, float x, float y, float size, Color32 vertexColor)
        {
            int startIndex = vertexHelper.currentVertCount;
            vertexHelper.AddVert(new Vector3(x, y), vertexColor, Vector2.zero);
            vertexHelper.AddVert(new Vector3(x, y + size), vertexColor, Vector2.zero);
            vertexHelper.AddVert(new Vector3(x + size, y + size), vertexColor, Vector2.zero);
            vertexHelper.AddVert(new Vector3(x + size, y), vertexColor, Vector2.zero);
            vertexHelper.AddTriangle(startIndex, startIndex + 1, startIndex + 2);
            vertexHelper.AddTriangle(startIndex + 2, startIndex + 3, startIndex);
        }
    }
}

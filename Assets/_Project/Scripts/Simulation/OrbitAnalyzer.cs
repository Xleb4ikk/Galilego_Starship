// ============================================================================
// АНАЛИЗАТОР ОРБИТ
// ============================================================================
// Полноценный анализатор орбит для планировщика манёвров
// Интегрирован с существующей системой UniverseManager

using System;
using System.Collections.Generic;
using UnityEngine;
using Galilego.Core;
using Galilego.Universe;

namespace Galilego.Simulation
{
    /// <summary>
    /// Анализатор орбит для участков свободного полёта.
    /// 
    /// Вычисляет:
    /// - Элементы орбиты (большая полуось, эксцентриситет, наклонение и т.д.)
    /// - Апсиды (перицентр, апоцентр)
    /// - Период обращения
    /// - Стабильность орбиты (внутри SOI)
    /// - Пересечения с поверхностью
    /// </summary>
    public class OrbitAnalyzer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private UniverseManager universeManager;
        
        [Header("Analysis Settings")]
        [SerializeField] private bool autoAnalyze = true;
        [SerializeField] private float analysisInterval = 0.5f;
        
        // Кэш результатов анализа
        private Dictionary<int, OrbitAnalysisResult> cachedAnalysis = new Dictionary<int, OrbitAnalysisResult>();
        private float lastAnalysisTime;
        
        private void Awake()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();
        }
        
        private void Update()
        {
            if (autoAnalyze && Time.time - lastAnalysisTime >= analysisInterval)
            {
                lastAnalysisTime = Time.time;
                // Автоматический анализ можно добавить здесь
            }
        }
        
        /// <summary>
        /// Анализ орбиты корабля вокруг указанного тела.
        /// </summary>
        public OrbitAnalysisResult AnalyzeShipOrbit(ReferenceFrameTarget target)
        {
            if (universeManager == null)
                return OrbitAnalysisResult.Invalid;
            
            // Получаем орбитальные элементы (с корректным преобразованием системы координат)
            OrbitalElements elements = universeManager.GetShipOrbitAround(target);
            
            if (!elements.IsValid)
                return OrbitAnalysisResult.Invalid;
            
            return BuildAnalysisResult(target, elements);
        }
        
        /// <summary>
        /// Анализ орбиты после манёвра.
        /// </summary>
        public OrbitAnalysisResult AnalyzeOrbitAfterManeuver(
            ReferenceFrameTarget target,
            Vector3d currentPosition,
            Vector3d currentVelocity,
            Vector3d deltaV)
        {
            if (universeManager == null)
                return OrbitAnalysisResult.Invalid;
            
            // Получаем параметры тела
            if (!universeManager.TryGetReferenceState(
                target,
                out string frameName,
                out Vector3d framePosition,
                out Vector3d frameVelocity,
                out double mu,
                out double bodyRadius,
                out double soi))
            {
                return OrbitAnalysisResult.Invalid;
            }
            
            // Вычисляем относительные координаты (в симуляционной системе)
            Vector3d relativePos = currentPosition - framePosition;
            Vector3d relativeVel = currentVelocity - frameVelocity;
            
            // Применяем манёвр
            Vector3d newVelocity = relativeVel + deltaV;
            
            // 🔴 КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Преобразуем в астродинамическую систему (Z-up)
            // перед вызовом OrbitalElements.FromState, который ожидает Z-up контракт
            OrbitalElements elements = OrbitalElements.FromState(
                universeManager.ConvertSimulationToAstrodynamicFrame(relativePos),
                universeManager.ConvertSimulationToAstrodynamicFrame(newVelocity),
                mu);
            
            if (!elements.IsValid)
                return OrbitAnalysisResult.Invalid;
            
            return BuildAnalysisResult(target, elements, bodyRadius, soi, mu);
        }
        
        /// <summary>
        /// Проверка стабильности орбиты.
        /// </summary>
        private bool IsOrbitStable(OrbitalElements elements, double bodyRadius, double soi)
        {
            if (!elements.IsValid || !elements.IsBound)
                return false;
            
            // Орбита стабильна если:
            // 1. Перицентр выше поверхности
            // 2. Апоцентр внутри SOI
            return elements.PeriapsisDistance > bodyRadius && 
                   elements.ApoapsisDistance < soi;
        }
        
        /// <summary>
        /// Оценка времени до столкновения с поверхностью.
        /// </summary>
        private double EstimateTimeToImpact(
            Vector3d position, 
            Vector3d velocity, 
            double mu, 
            double bodyRadius)
        {
            // Упрощённая оценка: время до перицентра
            double r = position.Magnitude;
            double v = velocity.Magnitude;
            
            if (r <= bodyRadius)
                return 0;
            
            // Используем vis-viva уравнение для оценки
            double energy = 0.5 * v * v - mu / r;
            double a = -mu / (2.0 * energy);
            
            if (a <= 0 || double.IsInfinity(a))
                return double.PositiveInfinity;
            
            double period = 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
            
            // Грубая оценка: четверть периода
            return period * 0.25;
        }
        
        /// <summary>
        /// Оценка времени до выхода из SOI.
        /// </summary>
        private double EstimateTimeToEscape(
            Vector3d position, 
            Vector3d velocity, 
            double mu, 
            double soi)
        {
            double r = position.Magnitude;
            
            if (r >= soi)
                return 0;
            
            // Радиальная скорость
            double vr = Vector3d.Dot(position.Normalized, velocity);
            
            if (vr <= 0)
                return double.PositiveInfinity;
            
            // Простая оценка: расстояние / скорость
            return (soi - r) / vr;
        }
        
        /// <summary>
        /// Вычисление времени до перицентра.
        /// </summary>
        private double ComputeTimeToPeriapsis(
            OrbitalElements elements,
            Vector3d position,
            Vector3d velocity,
            double mu)
        {
            if (!elements.IsValid || !elements.IsBound)
                return double.PositiveInfinity;
            
            // Используем истинную аномалию
            double trueAnomaly = elements.TrueAnomalyDegrees * Math.PI / 180.0;
            
            // Время до перицентра через аномалию
            double period = elements.OrbitalPeriodSeconds;
            
            if (double.IsInfinity(period) || period <= 0)
                return double.PositiveInfinity;
            
            // Нормализуем аномалию
            double normalizedAnomaly = trueAnomaly % (2.0 * Math.PI);
            if (normalizedAnomaly < 0)
                normalizedAnomaly += 2.0 * Math.PI;
            
            // Время от перицентра
            double timeFromPeriapsis = (normalizedAnomaly / (2.0 * Math.PI)) * period;
            
            // Время до следующего перицентра
            return period - timeFromPeriapsis;
        }
        
        /// <summary>
        /// Вычисление времени до апоцентра.
        /// </summary>
        private double ComputeTimeToApoapsis(
            OrbitalElements elements,
            Vector3d position,
            Vector3d velocity,
            double mu)
        {
            if (!elements.IsValid || !elements.IsBound)
                return double.PositiveInfinity;
            
            double timeToPeriapsis = ComputeTimeToPeriapsis(elements, position, velocity, mu);
            
            if (double.IsInfinity(timeToPeriapsis))
                return double.PositiveInfinity;
            
            double period = elements.OrbitalPeriodSeconds;
            double halfPeriod = period * 0.5;
            
            // Апоцентр находится через полпериода от перицентра
            double timeToApoapsis = timeToPeriapsis + halfPeriod;
            
            // Если больше периода, вычитаем период
            if (timeToApoapsis > period)
                timeToApoapsis -= period;
            
            return timeToApoapsis;
        }
        
        /// <summary>
        /// Получение кэшированного анализа.
        /// </summary>
        public OrbitAnalysisResult GetCachedAnalysis(int segmentIndex)
        {
            if (cachedAnalysis.TryGetValue(segmentIndex, out var result))
                return result;
            
            return OrbitAnalysisResult.Invalid;
        }
        
        /// <summary>
        /// Сохранение результата анализа в кэш.
        /// </summary>
        public void CacheAnalysis(int segmentIndex, OrbitAnalysisResult result)
        {
            cachedAnalysis[segmentIndex] = result;
        }
        
        /// <summary>
        /// Очистка кэша.
        /// </summary>
        public void ClearCache()
        {
            cachedAnalysis.Clear();
        }
        
        // ─── Вспомогательные методы ─────────────────────────────────────────
        
        /// <summary>
        /// Построение результата анализа с получением параметров тела.
        /// </summary>
        private OrbitAnalysisResult BuildAnalysisResult(ReferenceFrameTarget target, OrbitalElements elements)
        {
            if (!universeManager.TryGetShipRelativeState(
                target,
                out string frameName,
                out Vector3d relativePosition,
                out Vector3d relativeVelocity,
                out double mu,
                out double bodyRadius,
                out double soi))
            {
                return OrbitAnalysisResult.Invalid;
            }
            
            return BuildAnalysisResult(target, elements, bodyRadius, soi, mu, relativePosition, relativeVelocity);
        }
        
        /// <summary>
        /// Построение результата анализа с переданными параметрами тела.
        /// </summary>
        private OrbitAnalysisResult BuildAnalysisResult(
            ReferenceFrameTarget target, 
            OrbitalElements elements,
            double bodyRadius,
            double soi,
            double mu,
            Vector3d? relativePosition = null,
            Vector3d? relativeVelocity = null)
        {
            var result = new OrbitAnalysisResult
            {
                IsValid = true,
                TargetName = target.ToString(),
                Elements = elements,
                BodyRadius = bodyRadius,
                SphereOfInfluence = soi,
                GravitationalParameter = mu
            };
            
            // Анализ стабильности
            result.IsStable = IsOrbitStable(elements, bodyRadius, soi);
            
            // Проверка столкновения с поверхностью
            result.WillImpact = elements.PeriapsisDistance < bodyRadius;
            
            if (result.WillImpact && relativePosition.HasValue && relativeVelocity.HasValue)
            {
                result.ImpactTime = EstimateTimeToImpact(
                    relativePosition.Value, relativeVelocity.Value, mu, bodyRadius);
            }
            
            // Проверка выхода из SOI
            result.WillEscape = !elements.IsBound || elements.ApoapsisDistance > soi;
            
            if (result.WillEscape && elements.IsBound && relativePosition.HasValue && relativeVelocity.HasValue)
            {
                result.EscapeTime = EstimateTimeToEscape(
                    relativePosition.Value, relativeVelocity.Value, mu, soi);
            }
            
            // Вычисление времени до апсид
            if (elements.IsBound && relativePosition.HasValue && relativeVelocity.HasValue)
            {
                result.TimeToPeriapsis = ComputeTimeToPeriapsis(
                    elements, relativePosition.Value, relativeVelocity.Value, mu);
                result.TimeToApoapsis = ComputeTimeToApoapsis(
                    elements, relativePosition.Value, relativeVelocity.Value, mu);
            }
            
            return result;
        }
    }
    
    /// <summary>
    /// Результат анализа орбиты.
    /// </summary>
    [Serializable]
    public struct OrbitAnalysisResult
    {
        public bool IsValid;
        public string TargetName;
        public OrbitalElements Elements;
        
        // Параметры тела
        public double BodyRadius;
        public double SphereOfInfluence;
        public double GravitationalParameter;
        
        // Анализ стабильности
        public bool IsStable;
        public bool WillImpact;
        public bool WillEscape;
        
        // Временные оценки
        public double ImpactTime;
        public double EscapeTime;
        public double TimeToPeriapsis;
        public double TimeToApoapsis;
        
        public static OrbitAnalysisResult Invalid => new OrbitAnalysisResult
        {
            IsValid = false,
            TargetName = "",
            Elements = OrbitalElements.Invalid,
            BodyRadius = 0,
            SphereOfInfluence = 0,
            GravitationalParameter = 0,
            IsStable = false,
            WillImpact = false,
            WillEscape = false,
            ImpactTime = double.PositiveInfinity,
            EscapeTime = double.PositiveInfinity,
            TimeToPeriapsis = double.PositiveInfinity,
            TimeToApoapsis = double.PositiveInfinity
        };
        
        /// <summary>
        /// Форматированное описание орбиты.
        /// </summary>
        public string GetDescription()
        {
            if (!IsValid)
                return "Invalid orbit";
            
            string desc = $"Orbit around {TargetName}\n";
            desc += $"Periapsis: {FormatDistance(Elements.PeriapsisDistance)}\n";
            desc += $"Apoapsis: {FormatDistance(Elements.ApoapsisDistance)}\n";
            desc += $"Period: {FormatDuration(Elements.OrbitalPeriodSeconds)}\n";
            desc += $"Eccentricity: {Elements.Eccentricity:F4}\n";
            desc += $"Inclination: {Elements.InclinationDegrees:F2}°\n";
            
            if (WillImpact)
                desc += $"\n⚠ IMPACT in {FormatDuration(ImpactTime)}";
            else if (WillEscape)
                desc += $"\n⚠ ESCAPE in {FormatDuration(EscapeTime)}";
            else if (IsStable)
                desc += "\n✓ Stable orbit";
            
            return desc;
        }
        
        private string FormatDistance(double meters)
        {
            if (double.IsInfinity(meters)) return "∞";
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
    }
}

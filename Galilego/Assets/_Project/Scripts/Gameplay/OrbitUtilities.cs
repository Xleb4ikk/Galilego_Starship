// ============================================================================
// УТИЛИТЫ ДЛЯ РАБОТЫ С ОРБИТАМИ
// ============================================================================
// Вспомогательные методы для анализа и расчёта орбит
// На основе документации Principia (Part5)

using System;
using Galilego.Physics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Утилиты для работы с орбитами.
    /// </summary>
    public static class OrbitUtilities
    {
        /// <summary>
        /// Вычисление Δv для перехода между двумя орбитами (манёвр Гоманна).
        /// 
        /// Формулы:
        /// - v1 = √(μ / r1) — скорость на начальной орбите
        /// - v_peri = √(μ * (2/r1 - 2/(r1+r2))) — скорость на переходной орбите в перицентре
        /// - Δv1 = v_peri - v1 — первый импульс
        /// - v2 = √(μ / r2) — скорость на конечной орбите
        /// - v_apo = √(μ * (2/r2 - 2/(r1+r2))) — скорость на переходной орбите в апоцентре
        /// - Δv2 = v2 - v_apo — второй импульс
        /// </summary>
        public static double ComputeHohmannTransferDeltaV(
            double r1,  // Радиус начальной орбиты (м)
            double r2,  // Радиус конечной орбиты (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            // Скорость на начальной орбите
            double v1 = Math.Sqrt(mu / r1);
            
            // Скорость на переходной орбите в перицентре
            double v_peri = Math.Sqrt(mu * (2.0 / r1 - 2.0 / (r1 + r2)));
            
            // Первый импульс
            double delta_v1 = v_peri - v1;
            
            // Скорость на конечной орбите
            double v2 = Math.Sqrt(mu / r2);
            
            // Скорость на переходной орбите в апоцентре
            double v_apo = Math.Sqrt(mu * (2.0 / r2 - 2.0 / (r1 + r2)));
            
            // Второй импульс
            double delta_v2 = v2 - v_apo;
            
            return Math.Abs(delta_v1) + Math.Abs(delta_v2);
        }
        
        /// <summary>
        /// Вычисление Δv для изменения наклонения.
        /// 
        /// Формула: Δv = 2 * v * sin(Δi / 2)
        /// 
        /// где v — орбитальная скорость, Δi — изменение наклонения.
        /// </summary>
        public static double ComputePlaneChangeDeltaV(
            double v,       // Орбитальная скорость (м/с)
            double delta_i) // Изменение наклонения (радианы)
        {
            return 2.0 * v * Math.Sin(delta_i / 2.0);
        }
        
        /// <summary>
        /// Вычисление Δv для круглизации орбиты.
        /// 
        /// Формулы:
        /// - v_current = √(μ * (2/r - 1/a)) — текущая скорость
        /// - v_circular = √(μ / r) — скорость на круговой орбите
        /// - Δv = |v_circular - v_current|
        /// </summary>
        public static double ComputeCircularizationDeltaV(
            double r,   // Текущее расстояние (м)
            double a,   // Большая полуось текущей орбиты (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            // Текущая скорость
            double v_current = Math.Sqrt(mu * (2.0 / r - 1.0 / a));
            
            // Скорость на круговой орбите
            double v_circular = Math.Sqrt(mu / r);
            
            return Math.Abs(v_circular - v_current);
        }
        
        /// <summary>
        /// Вычисление орбитальной скорости на заданном расстоянии.
        /// 
        /// Формула: v = √(μ * (2/r - 1/a))
        /// 
        /// Для круговой орбиты: v = √(μ / r)
        /// </summary>
        public static double ComputeOrbitalVelocity(
            double r,   // Расстояние от центра (м)
            double a,   // Большая полуось (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            return Math.Sqrt(mu * (2.0 / r - 1.0 / a));
        }
        
        /// <summary>
        /// Вычисление орбитального периода.
        /// 
        /// Формула: T = 2π * √(a³ / μ)
        /// </summary>
        public static double ComputeOrbitalPeriod(
            double a,   // Большая полуось (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
        }
        
        /// <summary>
        /// Вычисление большой полуоси из периода.
        /// 
        /// Формула: a = ∛(μ * T² / (4π²))
        /// </summary>
        public static double ComputeSemiMajorAxisFromPeriod(
            double period, // Период (с)
            double mu)     // Гравитационный параметр (м³/с²)
        {
            double t_squared = period * period;
            return Math.Pow(mu * t_squared / (4.0 * Math.PI * Math.PI), 1.0 / 3.0);
        }
        
        /// <summary>
        /// Вычисление удельной орбитальной энергии.
        /// 
        /// Формула: ε = v²/2 - μ/r = -μ/(2a)
        /// </summary>
        public static double ComputeSpecificOrbitalEnergy(
            double a,   // Большая полуось (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            return -mu / (2.0 * a);
        }
        
        /// <summary>
        /// Вычисление первой космической скорости (круговая орбита).
        /// 
        /// Формула: v₁ = √(μ / r)
        /// </summary>
        public static double ComputeCircularVelocity(
            double r,   // Радиус орбиты (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            return Math.Sqrt(mu / r);
        }
        
        /// <summary>
        /// Вычисление второй космической скорости (параболическая траектория).
        /// 
        /// Формула: v₂ = √(2μ / r) = v₁ * √2
        /// </summary>
        public static double ComputeEscapeVelocity(
            double r,   // Расстояние от центра (м)
            double mu)  // Гравитационный параметр (м³/с²)
        {
            return Math.Sqrt(2.0 * mu / r);
        }
        
        /// <summary>
        /// Вычисление радиуса сферы влияния (Sphere of Influence).
        /// 
        /// Формула: r_SOI = a * (m/M)^(2/5)
        /// 
        /// где a — большая полуось орбиты тела вокруг родителя,
        /// m — масса тела, M — масса родителя.
        /// </summary>
        public static double ComputeSphereOfInfluence(
            double semiMajorAxis, // Большая полуось орбиты тела (м)
            double bodyMass,      // Масса тела (кг)
            double parentMass)    // Масса родителя (кг)
        {
            return semiMajorAxis * Math.Pow(bodyMass / parentMass, 2.0 / 5.0);
        }
        
        /// <summary>
        /// Вычисление радиуса Хилла (Hill sphere).
        /// 
        /// Формула: r_H = a * ∛(m / (3M))
        /// 
        /// Радиус Хилла — это область, в которой спутник может стабильно вращаться вокруг тела.
        /// </summary>
        public static double ComputeHillSphere(
            double semiMajorAxis, // Большая полуось орбиты тела (м)
            double bodyMass,      // Масса тела (кг)
            double parentMass)    // Масса родителя (кг)
        {
            return semiMajorAxis * Math.Pow(bodyMass / (3.0 * parentMass), 1.0 / 3.0);
        }
        
        /// <summary>
        /// Вычисление синодического периода (период повторения конфигурации двух тел).
        /// 
        /// Формула: T_syn = |T₁ * T₂ / (T₁ - T₂)|
        /// 
        /// где T₁ и T₂ — орбитальные периоды двух тел.
        /// </summary>
        public static double ComputeSynodicPeriod(
            double period1, // Период первого тела (с)
            double period2) // Период второго тела (с)
        {
            if (Math.Abs(period1 - period2) < 1e-10)
                return double.PositiveInfinity;
            
            return Math.Abs(period1 * period2 / (period1 - period2));
        }
        
        /// <summary>
        /// Вычисление фазового угла для перехода Гоманна.
        /// 
        /// Формула: φ = π * (1 - 1/(2√2) * √((r₁/r₂ + 1)³ / (r₁/r₂)))
        /// 
        /// Упрощённо: φ ≈ π - π * √((r₁ + r₂)³ / (8 * r₂³))
        /// </summary>
        public static double ComputeHohmannPhaseAngle(
            double r1, // Радиус начальной орбиты (м)
            double r2, // Радиус конечной орбиты (м)
            double mu) // Гравитационный параметр (м³/с²)
        {
            // Период переходной орбиты
            double a_transfer = (r1 + r2) / 2.0;
            double t_transfer = Math.PI * Math.Sqrt(a_transfer * a_transfer * a_transfer / mu);
            
            // Угловая скорость целевой орбиты
            double omega2 = Math.Sqrt(mu / (r2 * r2 * r2));
            
            // Фазовый угол
            return Math.PI - omega2 * t_transfer;
        }
        
        /// <summary>
        /// Проверка, является ли орбита стабильной (внутри сферы влияния).
        /// </summary>
        public static bool IsOrbitStable(
            double periapsis,     // Перицентр (м)
            double apoapsis,      // Апоцентр (м)
            double bodyRadius,    // Радиус тела (м)
            double sphereOfInfluence) // Радиус сферы влияния (м)
        {
            // Орбита должна быть выше поверхности и внутри SOI
            return periapsis > bodyRadius && apoapsis < sphereOfInfluence;
        }
        
        /// <summary>
        /// Вычисление времени до следующего перицентра.
        /// 
        /// Использует аномалию и период орбиты.
        /// </summary>
        public static double ComputeTimeToPeriapsis(
            double trueAnomaly, // Истинная аномалия (радианы)
            double period)      // Орбитальный период (с)
        {
            // Нормализуем аномалию в диапазон [0, 2π)
            double normalizedAnomaly = trueAnomaly % (2.0 * Math.PI);
            if (normalizedAnomaly < 0)
                normalizedAnomaly += 2.0 * Math.PI;
            
            // Время до перицентра
            double timeFromPeriapsis = (normalizedAnomaly / (2.0 * Math.PI)) * period;
            return period - timeFromPeriapsis;
        }
    }
}

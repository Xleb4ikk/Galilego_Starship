// ============================================================================
// УТИЛИТЫ ДЛЯ РАБОТЫ С МАНЁВРАМИ
// ============================================================================
// Вспомогательные методы для расчёта параметров манёвров
// На основе документации Principia (Part2)

using System;
using System.Collections.Generic;
using Galilego.Simulation;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Утилиты для работы с манёврами.
    /// Реализует формулы из уравнения Циолковского.
    /// </summary>
    public static class ManeuverUtilities
    {
        /// <summary>
        /// Стандартное ускорение свободного падения (м/с²).
        /// </summary>
        public const double g0 = 9.80665;
        
        /// <summary>
        /// Вычисление параметров манёвра по Δv и характеристикам двигателя.
        /// 
        /// Формулы:
        /// - Массовый расход: ṁ = F / (Isp * g0)
        /// - Конечная масса: m1 = m0 * exp(-Δv / (Isp * g0))
        /// - Длительность: Δt = (m0 - m1) / ṁ
        /// - Время половинного Δv: t_½ = Isp * m0 * (1 - √(m1/m0)) / F
        /// </summary>
        public static ManeuverCalculation CalculateManeuver(
            double deltaVMagnitude,
            EngineParameters engine)
        {
            // Проверка на сингулярность
            if (double.IsNaN(deltaVMagnitude) || double.IsInfinity(deltaVMagnitude))
            {
                return ManeuverCalculation.Invalid;
            }
            
            if (engine.ThrustNewtons <= 0 || engine.SpecificImpulseSeconds <= 0 || 
                engine.InitialMassKg <= 0)
            {
                return ManeuverCalculation.Invalid;
            }
            
            var result = new ManeuverCalculation { IsSingular = false };
            
            // 1. Скорость истечения (м/с)
            double exhaustVelocity = engine.SpecificImpulseSeconds * g0;
            
            // 2. Массовый расход: ṁ = F / (Isp * g0)
            result.MassFlowRate = engine.ThrustNewtons / exhaustVelocity;
            
            // 3. Конечная масса: m1 = m0 * exp(-Δv / (Isp * g0))
            double massRatio = Math.Exp(-deltaVMagnitude / exhaustVelocity);
            result.FinalMassKg = engine.InitialMassKg * massRatio;
            
            // 4. Длительность: Δt = (m0 - m1) / ṁ
            if (result.MassFlowRate > 0)
            {
                result.DurationSeconds = (engine.InitialMassKg - result.FinalMassKg) / result.MassFlowRate;
            }
            else
            {
                result.DurationSeconds = 0; // Мгновенный импульс
            }
            
            // 5. Время половинного Δv: t_½ = Isp * m0 * (1 - √(m1/m0)) / F
            if (engine.ThrustNewtons > 0 && result.FinalMassKg > 0)
            {
                double sqrtMassRatio = Math.Sqrt(result.FinalMassKg / engine.InitialMassKg);
                result.TimeToHalfDeltaV = engine.SpecificImpulseSeconds * engine.InitialMassKg * 
                                         (1.0 - sqrtMassRatio) / engine.ThrustNewtons;
            }
            else
            {
                result.TimeToHalfDeltaV = 0;
            }
            
            return result;
        }
        
        /// <summary>
        /// Вычисление конечной массы по уравнению Циолковского.
        /// 
        /// Формула: m1 = m0 * exp(-Δv / (Isp * g0))
        /// </summary>
        public static double ComputeFinalMass(
            double initialMassKg,
            double deltaVMps,
            double specificImpulseSeconds)
        {
            double exhaustVelocity = specificImpulseSeconds * g0;
            return initialMassKg * Math.Exp(-deltaVMps / exhaustVelocity);
        }
        
        /// <summary>
        /// Вычисление Δv по массам (обратное уравнение Циолковского).
        /// 
        /// Формула: Δv = Isp * g0 * ln(m0/m1)
        /// </summary>
        public static double ComputeDeltaV(
            double initialMassKg,
            double finalMassKg,
            double specificImpulseSeconds)
        {
            if (finalMassKg <= 0 || initialMassKg <= 0)
                return double.PositiveInfinity;
            
            double exhaustVelocity = specificImpulseSeconds * g0;
            return exhaustVelocity * Math.Log(initialMassKg / finalMassKg);
        }
        
        /// <summary>
        /// Вычисление массового расхода.
        /// 
        /// Формула: ṁ = F / (Isp * g0)
        /// </summary>
        public static double ComputeMassFlow(
            double thrustNewtons,
            double specificImpulseSeconds)
        {
            double exhaustVelocity = specificImpulseSeconds * g0;
            return thrustNewtons / exhaustVelocity;
        }
        
        /// <summary>
        /// Вычисление длительности манёвра.
        /// 
        /// Формула: Δt = (m0 - m1) / ṁ
        /// </summary>
        public static double ComputeDuration(
            double initialMassKg,
            double finalMassKg,
            double massFlowKgs)
        {
            if (massFlowKgs <= 0)
                return 0;
            
            return (initialMassKg - finalMassKg) / massFlowKgs;
        }
        
        /// <summary>
        /// Вычисление времени половинного Δv.
        /// 
        /// Формула: t_½ = Isp * m0 * (1 - √(m1/m0)) / F
        /// </summary>
        public static double ComputeTimeToHalfDeltaV(
            double initialMassKg,
            double finalMassKg,
            double specificImpulseSeconds,
            double thrustNewtons)
        {
            if (thrustNewtons <= 0 || finalMassKg <= 0)
                return 0;
            
            double sqrtMassRatio = Math.Sqrt(finalMassKg / initialMassKg);
            return specificImpulseSeconds * initialMassKg * 
                   (1.0 - sqrtMassRatio) / thrustNewtons;
        }
        
        /// <summary>
        /// Вычисление удельного импульса из тяги и массового расхода.
        /// 
        /// Формула: Isp = F / (ṁ * g0)
        /// </summary>
        public static double ComputeSpecificImpulse(
            double thrustNewtons,
            double massFlowKgs)
        {
            if (massFlowKgs <= 0)
                return 0;
            
            return thrustNewtons / (massFlowKgs * g0);
        }
        
        /// <summary>
        /// Вычисление средневзвешенного удельного импульса для нескольких двигателей.
        /// 
        /// Формула: Isp_avg = ΣFᵢ / Σ(Fᵢ / Ispᵢ)
        /// </summary>
        public static double ComputeAverageSpecificImpulse(
            List<(double thrustNewtons, double specificImpulseSeconds)> engines)
        {
            double totalThrust = 0;
            double sumFOverIsp = 0;
            
            foreach (var (thrust, isp) in engines)
            {
                totalThrust += thrust;
                if (isp > 0)
                {
                    sumFOverIsp += thrust / isp;
                }
            }
            
            if (sumFOverIsp <= 0)
                return 0;
            
            return totalThrust / sumFOverIsp;
        }
        
        /// <summary>
        /// Проверка, достаточно ли топлива для манёвра.
        /// </summary>
        public static bool HasEnoughFuel(
            double initialMassKg,
            double dryMassKg,
            double deltaVMps,
            double specificImpulseSeconds)
        {
            double finalMass = ComputeFinalMass(
                initialMassKg, deltaVMps, specificImpulseSeconds);
            
            return finalMass >= dryMassKg;
        }
        
        /// <summary>
        /// Вычисление максимального доступного Δv для данного запаса топлива.
        /// </summary>
        public static double ComputeMaxDeltaV(
            double initialMassKg,
            double dryMassKg,
            double specificImpulseSeconds)
        {
            return ComputeDeltaV(initialMassKg, dryMassKg, specificImpulseSeconds);
        }
        
        /// <summary>
        /// Вычисление ускорения в момент времени t во время манёвра.
        /// 
        /// Формула: a(t) = F / m(t), где m(t) = m0 - ṁ * t
        /// </summary>
        public static double ComputeAccelerationAtTime(
            double thrustNewtons,
            double initialMassKg,
            double massFlowKgs,
            double timeFromStart)
        {
            double currentMass = initialMassKg - massFlowKgs * timeFromStart;
            
            if (currentMass <= 0)
                return 0;
            
            return thrustNewtons / currentMass;
        }
        
        /// <summary>
        /// Вычисление массы в момент времени t во время манёвра.
        /// 
        /// Формула: m(t) = m0 - ṁ * t
        /// </summary>
        public static double ComputeMassAtTime(
            double initialMassKg,
            double massFlowKgs,
            double timeFromStart)
        {
            return Math.Max(0, initialMassKg - massFlowKgs * timeFromStart);
        }
    }
}

using Galilego.Core;
using Galilego.Simulation;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Maneuver node data.
    ///
    /// Δv axes (orbital frame):
    ///   DvPrograde: along velocity direction (prograde/retrograde)
    ///   DvNormal: along orbit normal (normal/anti-normal)
    ///   DvRadial: along radial direction (radial out/in)
    /// </summary>
    [Serializable]
    public class ManeuverNode
    {
        public string Name = "Maneuver";
        public double StartTime;
        public double DvPrograde;   // Δv along prograde direction
        public double DvNormal;     // Δv along orbit normal
        public double DvRadial;     // Δv along radial direction (out/in)

        public double Duration = 0d;
        public bool IsInstant = true;

        // Параметры двигателя (опционально)
        public EngineParameters? Engine = null;
        
        // Кэшированный расчёт манёвра
        private ManeuverCalculation? cachedCalculation = null;

        [Obsolete("Use DvPrograde instead")]
        public double DvTangent
        {
            get => DvPrograde;
            set => DvPrograde = value;
        }

        [Obsolete("Use DvRadial instead")]
        public double DvBinormal
        {
            get => DvRadial;
            set => DvRadial = value;
        }

        public ManeuverNode(double time, double prograde = 0, double normal = 0, double radial = 0)
        {
            StartTime = time;
            DvPrograde = prograde;
            DvNormal = normal;
            DvRadial = radial;
        }

        public double TotalDeltaV => Math.Sqrt(
            DvPrograde * DvPrograde +
            DvNormal * DvNormal +
            DvRadial * DvRadial);
        
        /// <summary>
        /// Создание копии манёвра с изменённым Δv.
        /// </summary>
        public ManeuverNode WithDeltaV(double prograde, double normal, double radial)
        {
            return new ManeuverNode(StartTime, prograde, normal, radial)
            {
                Name = Name,
                Duration = Duration,
                IsInstant = IsInstant,
                Engine = Engine
            };
        }
        
        /// <summary>
        /// Создание копии манёвра с изменённым временем начала.
        /// </summary>
        public ManeuverNode WithInitialTime(double newStartTime)
        {
            return new ManeuverNode(newStartTime, DvPrograde, DvNormal, DvRadial)
            {
                Name = Name,
                Duration = Duration,
                IsInstant = IsInstant,
                Engine = Engine
            };
        }
        
        /// <summary>
        /// Создание копии манёвра с изменённой начальной массой двигателя.
        /// </summary>
        public ManeuverNode WithInitialMass(double newInitialMassKg)
        {
            var node = new ManeuverNode(StartTime, DvPrograde, DvNormal, DvRadial)
            {
                Name = Name,
                Duration = Duration,
                IsInstant = IsInstant
            };
            
            if (Engine.HasValue)
            {
                var eng = Engine.Value;
                eng.InitialMassKg = newInitialMassKg;
                node.Engine = eng;
            }
            
            return node;
        }
        
        /// <summary>
        /// Получение расчёта манёвра с учётом расхода топлива.
        /// Результат кэшируется.
        /// </summary>
        public ManeuverCalculation GetCalculation()
        {
            if (cachedCalculation.HasValue)
                return cachedCalculation.Value;
            
            if (!Engine.HasValue)
                return ManeuverCalculation.Invalid;
            
            cachedCalculation = ManeuverUtilities.CalculateManeuver(TotalDeltaV, Engine.Value);
            return cachedCalculation.Value;
        }
        
        /// <summary>
        /// Сброс кэша расчёта (вызывать при изменении параметров).
        /// </summary>
        public void InvalidateCalculation()
        {
            cachedCalculation = null;
        }
        
        /// <summary>
        /// Конечная масса после манёвра (если заданы параметры двигателя).
        /// </summary>
        public double? FinalMass
        {
            get
            {
                if (!Engine.HasValue) return null;
                var calc = GetCalculation();
                return calc.IsSingular ? null : (double?)calc.FinalMassKg;
            }
        }
        
        /// <summary>
        /// Конечное время манёвра.
        /// </summary>
        public double FinalTime
        {
            get
            {
                if (!Engine.HasValue || IsInstant)
                    return StartTime;
                
                var calc = GetCalculation();
                return StartTime + calc.DurationSeconds;
            }
        }
    }

    /// <summary>
    /// Класс, управляющий списком маневров.
    /// </summary>
    [Serializable]
    public class FlightPlan
    {
        public List<ManeuverNode> Nodes = new List<ManeuverNode>();

        // Глобальные настройки планирования (из скриншота)
        public double PredictionLengthSeconds = 7200d; // По умолчанию 2 часа
        public double DisplayTimeSeconds = 0d; // Текущее время для отображения позиции на орбите
        public int MaxStepsPerSegment = 4096;
        public double Tolerance = 1.0d; // м

        public double GetTotalDeltaV()
        {
            double total = 0;
            foreach (var node in Nodes) total += node.TotalDeltaV;
            return total;
        }
        
        /// <summary>
        /// Вставка манёвра в указанную позицию.
        /// </summary>
        public OperationResult Insert(ManeuverNode node, int index)
        {
            if (index < 0 || index > Nodes.Count)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    $"Index {index} out of range [0, {Nodes.Count}]");
            }
            
            // Проверка на сингулярность
            if (double.IsNaN(node.TotalDeltaV) || double.IsInfinity(node.TotalDeltaV))
            {
                return OperationResult.Error(ManeuverStatus.FailedPrecondition, 
                    "Maneuver has singular delta-v (NaN or Infinity)");
            }
            
            // Проверка, помещается ли манёвр между соседними
            if (index > 0 && node.StartTime <= Nodes[index - 1].StartTime)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    "Maneuver time conflicts with previous maneuver");
            }
            
            if (index < Nodes.Count && node.StartTime >= Nodes[index].StartTime)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    "Maneuver time conflicts with next maneuver");
            }
            
            Nodes.Insert(index, node);
            UpdateInitialMassesAfter(index);
            
            return OperationResult.Ok;
        }
        
        /// <summary>
        /// Удаление манёвра.
        /// </summary>
        public OperationResult Remove(int index)
        {
            if (index < 0 || index >= Nodes.Count)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    $"Index {index} out of range [0, {Nodes.Count})");
            }
            
            Nodes.RemoveAt(index);
            UpdateInitialMassesAfter(index);
            
            return OperationResult.Ok;
        }
        
        /// <summary>
        /// Замена манёвра.
        /// </summary>
        public OperationResult Replace(ManeuverNode node, int index)
        {
            if (index < 0 || index >= Nodes.Count)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    $"Index {index} out of range [0, {Nodes.Count})");
            }
            
            // Проверка на сингулярность
            if (double.IsNaN(node.TotalDeltaV) || double.IsInfinity(node.TotalDeltaV))
            {
                return OperationResult.Error(ManeuverStatus.FailedPrecondition, 
                    "Maneuver has singular delta-v (NaN or Infinity)");
            }
            
            // Проверка времени
            if (index > 0 && node.StartTime <= Nodes[index - 1].StartTime)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    "Maneuver time conflicts with previous maneuver");
            }
            
            if (index < Nodes.Count - 1 && node.StartTime >= Nodes[index + 1].StartTime)
            {
                return OperationResult.Error(ManeuverStatus.OutOfRange, 
                    "Maneuver time conflicts with next maneuver");
            }
            
            // Сохраняем начальную массу
            if (Nodes[index].Engine.HasValue && node.Engine.HasValue)
            {
                var eng = node.Engine.Value;
                eng.InitialMassKg = Nodes[index].Engine.Value.InitialMassKg;
                node.Engine = eng;
            }
            
            Nodes[index] = node;
            UpdateInitialMassesAfter(index);
            
            return OperationResult.Ok;
        }
        
        /// <summary>
        /// Установка желаемого конечного времени плана.
        /// </summary>
        public OperationResult SetDesiredFinalTime(double desiredFinalTime)
        {
            if (Nodes.Count > 0)
            {
                double lastManeuverEnd = Nodes[Nodes.Count - 1].FinalTime;
                if (desiredFinalTime < lastManeuverEnd)
                {
                    return OperationResult.Error(ManeuverStatus.OutOfRange, 
                        "Desired final time is before last maneuver end");
                }
            }
            
            PredictionLengthSeconds = desiredFinalTime;
            return OperationResult.Ok;
        }
        
        /// <summary>
        /// Обновление начальных масс манёвров после указанного индекса.
        /// 
        /// При изменении одного манёвра нужно пересчитать начальные массы
        /// всех последующих, так как они зависят от конечной массы предыдущего.
        /// </summary>
        private void UpdateInitialMassesAfter(int index)
        {
            if (index >= Nodes.Count)
                return;
            
            // Начальная масса следующего манёвра = конечная масса текущего
            for (int i = index + 1; i < Nodes.Count; i++)
            {
                if (!Nodes[i - 1].Engine.HasValue || !Nodes[i].Engine.HasValue)
                    continue;
                
                double finalMass = Nodes[i - 1].FinalMass ?? Nodes[i - 1].Engine.Value.InitialMassKg;
                Nodes[i] = Nodes[i].WithInitialMass(finalMass);
            }
        }
        
        /// <summary>
        /// Получение манёвра по индексу.
        /// </summary>
        public ManeuverNode GetManeuver(int index)
        {
            if (index < 0 || index >= Nodes.Count)
                throw new IndexOutOfRangeException($"Index {index} out of range [0, {Nodes.Count})");
            
            return Nodes[index];
        }
        
        /// <summary>
        /// Количество манёвров.
        /// </summary>
        public int ManeuverCount => Nodes.Count;

        /// <summary>
        /// Calculate world-space Δv vector for a maneuver node.
        ///
        /// Uses canonical orbital basis:
        ///   - Radial: direction from central body to spacecraft
        ///   - Normal: orbit angular momentum (R × V)
        ///   - Prograde: perpendicular to radial in velocity direction (N × R)
        ///
        /// All inputs must be relative to the reference frame.
        /// </summary>
        public static Vector3d CalculateWorldDeltaV(
            Vector3d relativePosition,
            Vector3d relativeVelocity,
            ManeuverNode node)
        {
            if (node == null) return Vector3d.Zero;

            if (!relativePosition.IsFinite || !relativeVelocity.IsFinite)
                return Vector3d.Zero;

            if (relativeVelocity.SqrMagnitude < 0.001d)
                return Vector3d.Zero;

            // Compute canonical orbital basis
            bool basisValid = OrbitalBasis.TryComputeBasis(
                relativePosition,
                relativeVelocity,
                out Vector3d radial,
                out Vector3d normal,
                out Vector3d prograde);

            if (!basisValid)
            {
                UnityEngine.Debug.LogWarning($"[FlightPlan] Orbital basis invalid at pos={relativePosition}, vel={relativeVelocity}. Using velocity direction as fallback.");
                radial = relativePosition.Normalized;
                Vector3d velDir = relativeVelocity.Normalized;
                // Compute normal from cross product, fallback to arbitrary perpendicular
                normal = Vector3d.Cross(radial, velDir).Normalized;
                if (!normal.IsFinite || normal.SqrMagnitude < 1e-12d)
                {
                    normal = new Vector3d(0d, 1d, 0d);
                }
                prograde = Vector3d.Cross(normal, radial).Normalized;
            }

            UnityEngine.Debug.Log($"[FlightPlan] Orbital basis: radial={radial}, normal={normal}, prograde={prograde}, valid={basisValid}");
            UnityEngine.Debug.Log($"[FlightPlan] Node deltaV: prograde={node.DvPrograde}, normal={node.DvNormal}, radial={node.DvRadial}");

            // Apply Δv in orbital frame
            Vector3d worldDv = (prograde * node.DvPrograde) +
                   (normal * node.DvNormal) +
                   (radial * node.DvRadial);
                    
            UnityEngine.Debug.Log($"[FlightPlan] World deltaV: {worldDv}, magnitude={worldDv.Magnitude:F2}");
            
            return worldDv;
        }
    }
}

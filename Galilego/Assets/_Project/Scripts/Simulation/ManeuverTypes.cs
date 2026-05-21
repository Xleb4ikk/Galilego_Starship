// ============================================================================
// ТИПЫ ДАННЫХ ДЛЯ ПЛАНИРОВЩИКА МАНЁВРОВ
// ============================================================================
// Дополнительные типы данных из документации Principia
// Интегрированы в существующую систему Galilego

using System;
using Galilego.Core;

namespace Galilego.Simulation
{
    /// <summary>
    /// Параметры двигателя для манёвра.
    /// Используется для расчёта расхода топлива и длительности.
    /// </summary>
    [Serializable]
    public struct EngineParameters
    {
        /// <summary>
        /// Тяга в Ньютонах.
        /// </summary>
        public double ThrustNewtons;
        
        /// <summary>
        /// Удельный импульс в секундах.
        /// </summary>
        public double SpecificImpulseSeconds;
        
        /// <summary>
        /// Начальная масса корабля в кг.
        /// </summary>
        public double InitialMassKg;
        
        public static EngineParameters Default => new EngineParameters
        {
            ThrustNewtons = 1000.0,
            SpecificImpulseSeconds = 300.0,
            InitialMassKg = 1000.0
        };
    }
    
    /// <summary>
    /// Результат расчёта манёвра с учётом расхода топлива.
    /// </summary>
    [Serializable]
    public struct ManeuverCalculation
    {
        /// <summary>
        /// Конечная масса после манёвра (кг).
        /// </summary>
        public double FinalMassKg;
        
        /// <summary>
        /// Массовый расход (кг/с).
        /// </summary>
        public double MassFlowRate;
        
        /// <summary>
        /// Длительность манёвра (секунды).
        /// </summary>
        public double DurationSeconds;
        
        /// <summary>
        /// Время половинного Δv (секунды от начала).
        /// </summary>
        public double TimeToHalfDeltaV;
        
        /// <summary>
        /// Является ли манёвр сингулярным (NaN или бесконечность).
        /// </summary>
        public bool IsSingular;
        
        public static ManeuverCalculation Invalid => new ManeuverCalculation
        {
            FinalMassKg = 0,
            MassFlowRate = 0,
            DurationSeconds = 0,
            TimeToHalfDeltaV = 0,
            IsSingular = true
        };
    }
    
    /// <summary>
    /// Статус операции с планом полёта.
    /// </summary>
    public enum ManeuverStatus
    {
        OK = 0,
        InvalidArgument = 3,
        DeadlineExceeded = 4,
        ResourceExhausted = 8,
        FailedPrecondition = 9,
        Aborted = 10,
        OutOfRange = 11,
        Unavailable = 14
    }
    
    /// <summary>
    /// Результат операции с планом полёта.
    /// </summary>
    [Serializable]
    public struct OperationResult
    {
        public ManeuverStatus Status;
        public string Message;
        
        public bool IsOk => Status == ManeuverStatus.OK;
        
        public static OperationResult Ok => new OperationResult
        {
            Status = ManeuverStatus.OK,
            Message = ""
        };
        
        public static OperationResult Error(ManeuverStatus status, string message)
        {
            return new OperationResult
            {
                Status = status,
                Message = message
            };
        }
    }
}

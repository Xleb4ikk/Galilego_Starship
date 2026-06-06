using Galilego.Core;
using Galilego.Universe;
using Galilego.Simulation;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Mathematics;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Автоматическое выполнение запланированных манёвров.
    /// 
    /// Позволяет пометить манёвр для выполнения через UI, после чего:
    /// 1. Отслеживает время симуляции
    /// 2. Когда достигнуто StartTime манёвра — мгновенно применяет ΔV
    /// 3. Удаляет выполненный манёвр из FlightPlan
    /// 4. Триггерит пересчёт траектории
    /// </summary>
    public class ManeuverExecutor : MonoBehaviour
    {
    [Header("References")]
    [SerializeField] private UniverseManager universeManager;
    [SerializeField] private ManeuverEvaluator evaluator;
    
    [Header("Integration Settings")]
    /// <summary>
    /// Feature flag для включения унифицированного OrbitIntegrator.StepToTime для точного timing применения delta-V.
    /// Когда false, использует оригинальный метод RK4 backward integration.
    /// 
    /// Цель: Позволяет безопасный rollback, если унифицированный интегратор вызовет проблемы.
    /// 
    /// Rollback процедура:
    /// 1. Установить этот флаг в false в Unity Inspector или через код
    /// 2. ManeuverExecutor вернётся к использованию PhysicsSolver.RK4
    /// 3. Перезапустить симуляцию или выполнить манёвр заново
    /// 
    /// Примечание: Этот флаг является частью Phase 5 исправления trajectory prediction mismatch бага.
    /// После завершения миграции и валидации, этот флаг будет удалён и унифицированный
    /// интегратор станет единственной опцией.
    /// </summary>
    [SerializeField]
    [Tooltip("Включить унифицированный OrbitIntegrator.StepToTime для точного timing delta-V. Отключить для rollback к RK4.")]
    private bool useExecutorUnifiedIntegrator = true;
    
    // Индексы манёвров, помеченных для выполнения
    private HashSet<int> queuedManeuvers = new HashSet<int>();
    
    // Последнее проверенное время (для детекции пропущенных манёвров)
    private double lastCheckedTime = 0d;

        private void Awake()
        {
            if (universeManager == null)
                universeManager = FindAnyObjectByType<UniverseManager>();
            
            if (evaluator == null)
                evaluator = FindAnyObjectByType<ManeuverEvaluator>();
        }

        private void Update()
        {
            if (universeManager == null || evaluator == null) return;
            
            FlightPlan plan = evaluator.GetFlightPlan();
            if (plan == null || plan.Nodes.Count == 0) return;
            
            double currentTime = universeManager.SimulationTimeSeconds;
            
            // Проверяем каждый манёвр в очереди
            List<int> toExecute = new List<int>();
            foreach (int index in queuedManeuvers)
            {
                if (index >= plan.Nodes.Count) continue;
                
                ManeuverNode node = plan.Nodes[index];
                
                // Время выполнения достигнуто?
                if (currentTime >= node.StartTime && lastCheckedTime < node.StartTime)
                {
                    toExecute.Add(index);
                }
            }
            
            // Выполняем манёвры (в порядке возрастания индекса, чтобы удаление было корректным)
            toExecute.Sort();
            toExecute.Reverse(); // Удаляем с конца, чтобы индексы не сбивались
            
            foreach (int index in toExecute)
            {
                if (index < plan.Nodes.Count)
                {
                    ManeuverNode node = plan.Nodes[index];
                    ExecuteManeuver(node, index);
                    queuedManeuvers.Remove(index);
                }
            }
            
            lastCheckedTime = currentTime;
        }
        
        /// <summary>
        /// Пометить манёвр для автоматического выполнения в StartTime
        /// </summary>
        public void QueueManeuver(int index)
        {
            queuedManeuvers.Add(index);
            Debug.Log($"[ManeuverExecutor] Maneuver #{index} queued for execution");
        }
        
        /// <summary>
        /// Отменить автоматическое выполнение манёвра
        /// </summary>
        public void CancelManeuver(int index)
        {
            queuedManeuvers.Remove(index);
            Debug.Log($"[ManeuverExecutor] Maneuver #{index} execution cancelled");
        }
        
        /// <summary>
        /// Проверить, помечен ли манёвр для выполнения
        /// </summary>
        public bool IsQueued(int index) => queuedManeuvers.Contains(index);
        
        /// <summary>
        /// Выполнить манёвр немедленно (импульсное применение ΔV)
        /// </summary>
        public void ExecuteManeuverImmediate(int index)
        {
            FlightPlan plan = evaluator.GetFlightPlan();
            if (plan == null || index < 0 || index >= plan.Nodes.Count) return;
            
            ManeuverNode node = plan.Nodes[index];
            ExecuteManeuver(node, index);
            queuedManeuvers.Remove(index);
        }
        
        /// <summary>
        /// Внутренний метод выполнения манёвра с высокой точностью
        /// </summary>
        private void ExecuteManeuver(ManeuverNode node, int index)
        {
            if (universeManager.ShipBody == null)
            {
                Debug.LogError("[ManeuverExecutor] ShipBody is null, cannot execute maneuver");
                return;
            }
            
            double currentTime = universeManager.SimulationTimeSeconds;
            double targetTime = node.StartTime;
            double timeDelta = currentTime - targetTime;
            
            // Получить текущее состояние корабля
            Vector3d currentShipPos = universeManager.ShipBody.Position;
            Vector3d currentShipVel = universeManager.ShipBody.Velocity;
            
            // ═══════════════════════════════════════════════════════════════════
            // HIGH-PRECISION INTERPOLATION: Интегрировать назад к точному времени
            // ═══════════════════════════════════════════════════════════════════
            
            Vector3d shipPosAtTarget;
            Vector3d shipVelAtTarget;
            
            if (Math.Abs(timeDelta) > 0.001) // Если есть разница во времени > 1 мс
            {
                if (useExecutorUnifiedIntegrator)
                {
                    // ═══ НОВЫЙ КОД: Унифицированный интегратор (Phase 5) ═══
                    // Используем OrbitIntegrator.StepToTime для точной backward/forward интеграции
                    // StepToTime автоматически обрабатывает направление (signed dt)
                    var stateAtTarget = OrbitIntegrator.StepToTime(
                        currentShipPos,
                        currentShipVel,
                        currentTime,
                        targetTime,  // StepToTime автоматически определяет направление
                        universeManager.EvaluateShipAccelerationAt,
                        absoluteTolerance: 1e-6,  // 1 mm position error
                        relativeTolerance: 1e-9); // 1 ppb relative error
                    
                    shipPosAtTarget = stateAtTarget.Position;
                    shipVelAtTarget = stateAtTarget.Velocity;
                    
                    if (!shipPosAtTarget.IsFinite || !shipVelAtTarget.IsFinite)
                    {
                        Debug.LogWarning($"[ManeuverExecutor] OrbitIntegrator.StepToTime failed, using current state");
                        shipPosAtTarget = currentShipPos;
                        shipVelAtTarget = currentShipVel;
                    }
                    else
                    {
                        Debug.Log($"[ManeuverExecutor] OrbitIntegrator.StepToTime: Δt={timeDelta:F6}s");
                        Debug.Log($"[ManeuverExecutor]   Current pos: {currentShipPos}");
                        Debug.Log($"[ManeuverExecutor]   Target pos:  {shipPosAtTarget}");
                        Debug.Log($"[ManeuverExecutor]   Position shift: {(currentShipPos - shipPosAtTarget).Magnitude:F2} m");
                    }
                }
                else
                {
                    // ═══ LEGACY CODE: RK4 backward integration (для rollback) ═══
                    shipPosAtTarget = currentShipPos;
                    shipVelAtTarget = currentShipVel;
                    
                    double integrationStep = -timeDelta; // Отрицательный шаг = интеграция назад
                    int substeps = Math.Max(1, (int)Math.Ceiling(Math.Abs(timeDelta) / 0.1)); // Подшаги по 0.1 сек
                    double substepDt = integrationStep / substeps;
                    
                    for (int i = 0; i < substeps; i++)
                    {
                        double t = currentTime + substepDt * i;
                        
                        // RK4 интегрирование
                        var result = PhysicsSolver.RK4(
                            shipPosAtTarget,
                            shipVelAtTarget,
                            t,
                            substepDt,
                            universeManager.EvaluateShipAccelerationAt);
                        
                        shipPosAtTarget = result.Position;
                        shipVelAtTarget = result.Velocity;
                        
                        if (!shipPosAtTarget.IsFinite || !shipVelAtTarget.IsFinite)
                        {
                            Debug.LogWarning($"[ManeuverExecutor] Interpolation failed, using current state");
                            shipPosAtTarget = currentShipPos;
                            shipVelAtTarget = currentShipVel;
                            break;
                        }
                    }
                    
                    Debug.Log($"[ManeuverExecutor] RK4 time interpolation: Δt={timeDelta:F6}s, substeps={substeps}");
                    Debug.Log($"[ManeuverExecutor]   Current pos: {currentShipPos}");
                    Debug.Log($"[ManeuverExecutor]   Target pos:  {shipPosAtTarget}");
                    Debug.Log($"[ManeuverExecutor]   Position shift: {(currentShipPos - shipPosAtTarget).Magnitude:F2} m");
                }
            }
            else
            {
                // Время совпадает с точностью до миллисекунды
                shipPosAtTarget = currentShipPos;
                shipVelAtTarget = currentShipVel;
                Debug.Log($"[ManeuverExecutor] Exact timing: Δt={timeDelta:F6}s (no interpolation needed)");
            }
            
            // Получить reference frame state В ЦЕЛЕВОЕ ВРЕМЯ
            Vector3d framePos, frameVel;
            if (!universeManager.TryGetReferenceStateAtTime(
                universeManager.ActiveReferenceFrame,
                targetTime,
                out _,
                out framePos,
                out frameVel,
                out _, out _, out _))
            {
                Debug.LogError("[ManeuverExecutor] Cannot get reference frame state at target time");
                return;
            }
            
            // Вычислить relative state В ЦЕЛЕВОЕ ВРЕМЯ
            Vector3d relativePos = shipPosAtTarget - framePos;
            Vector3d relativeVel = shipVelAtTarget - frameVel;
            
            Debug.Log($"[ManeuverExecutor] Reference frame at target time:");
            Debug.Log($"[ManeuverExecutor]   Frame: {universeManager.ActiveReferenceFrame}");
            Debug.Log($"[ManeuverExecutor]   Frame pos: {framePos}");
            Debug.Log($"[ManeuverExecutor]   Frame vel: {frameVel.Magnitude:F2} m/s");
            
            // ═══════════════════════════════════════════════════════════════════
            // CRITICAL FIX: Использовать ТОТ ЖЕ метод, что и планировщик!
            // FullTrajectoryJob использует OrbitalBasisJob.ComputeBasis
            // ═══════════════════════════════════════════════════════════════════
            
            // Конвертировать в double3 для совместимости с OrbitalBasisJob
            double3 pos3 = new double3(relativePos.X, relativePos.Y, relativePos.Z);
            double3 vel3 = new double3(relativeVel.X, relativeVel.Y, relativeVel.Z);
            
            // Использовать ТОЧНО такой же метод, как FullTrajectoryJob
            OrbitalBasisJob.ComputeBasis(pos3, vel3, out double3 radial3, out double3 normal3, out double3 prograde3);
            
            // Вычислить world ΔV напрямую (как в FullTrajectoryJob.CalculateWorldDeltaV)
            double3 worldDv3 = prograde3 * node.DvPrograde + normal3 * node.DvNormal + radial3 * node.DvRadial;
            
            // Конвертировать обратно в Vector3d
            Vector3d worldDeltaV = new Vector3d(worldDv3.x, worldDv3.y, worldDv3.z);
            
            Debug.Log($"[ManeuverExecutor] Orbital basis (using OrbitalBasisJob.ComputeBasis):");
            Debug.Log($"[ManeuverExecutor]   Radial:   ({radial3.x:F6}, {radial3.y:F6}, {radial3.z:F6})");
            Debug.Log($"[ManeuverExecutor]   Normal:   ({normal3.x:F6}, {normal3.y:F6}, {normal3.z:F6})");
            Debug.Log($"[ManeuverExecutor]   Prograde: ({prograde3.x:F6}, {prograde3.y:F6}, {prograde3.z:F6})");
            
            if (!worldDeltaV.IsFinite)
            {
                Debug.LogError("[ManeuverExecutor] Invalid ΔV computed, aborting execution");
                return;
            }
            
            // ═══════════════════════════════════════════════════════════════════
            // CRITICAL FIX: Применить delta-V к состоянию В ЦЕЛЕВОЕ ВРЕМЯ!
            // ═══════════════════════════════════════════════════════════════════
            // Проблема: Если применить delta-V к currentShipPos/currentShipVel,
            // то манёвр выполняется в неправильной точке орбиты!
            // 
            // Правильно: Применить к shipPosAtTarget/shipVelAtTarget (состояние в targetTime)
            // 
            // Примечание: simulationTimeSeconds приватное и не может быть установлено извне.
            // Но это OK - время продолжит идти естественно, а состояние корабля будет правильным.
            // Корабль будет в позиции targetTime, и симуляция продолжится оттуда.
            Vector3d newVelocity = shipVelAtTarget + worldDeltaV;
            universeManager.ShipBody.SetState(shipPosAtTarget, newVelocity);
            
            Debug.Log($"[ManeuverExecutor] ✓ Executed '{node.Name}'");
            Debug.Log($"[ManeuverExecutor]   Planned time:  {targetTime:F6}s");
            Debug.Log($"[ManeuverExecutor]   Actual time:   {currentTime:F6}s");
            Debug.Log($"[ManeuverExecutor]   Time error:    {timeDelta:F6}s");
            Debug.Log($"[ManeuverExecutor]   ΔV: prograde={node.DvPrograde:F1}, normal={node.DvNormal:F1}, radial={node.DvRadial:F1} m/s");
            Debug.Log($"[ManeuverExecutor]   Total ΔV magnitude: {worldDeltaV.Magnitude:F2} m/s");
            Debug.Log($"[ManeuverExecutor]   World ΔV vector: {worldDeltaV}");
            Debug.Log($"[ManeuverExecutor]   Position at target: {shipPosAtTarget}");
            Debug.Log($"[ManeuverExecutor]   Velocity at target (before): {shipVelAtTarget.Magnitude:F2} m/s");
            Debug.Log($"[ManeuverExecutor]   Velocity at target (after):  {newVelocity.Magnitude:F2} m/s");
            Debug.Log($"[ManeuverExecutor]   Applied at correct time: {targetTime:F6}s");
            
            // Удалить манёвр из плана
            FlightPlan plan = evaluator.GetFlightPlan();
            plan.Remove(index);
            
            // Принудительно пересчитать траекторию
            evaluator.InvalidateEphemerisRevision();
            evaluator.MarkAsDirty();
            
            // Обновить индексы оставшихся манёвров в очереди
            HashSet<int> updatedQueue = new HashSet<int>();
            foreach (int queuedIndex in queuedManeuvers)
            {
                if (queuedIndex > index)
                    updatedQueue.Add(queuedIndex - 1);
                else if (queuedIndex < index)
                    updatedQueue.Add(queuedIndex);
                // queuedIndex == index уже не добавляем (удалён)
            }
            queuedManeuvers = updatedQueue;
        }
        
        /// <summary>
        /// Очистить очередь выполнения (например, при загрузке нового сценария)
        /// </summary>
        public void ClearQueue()
        {
            queuedManeuvers.Clear();
            Debug.Log("[ManeuverExecutor] Execution queue cleared");
        }
        
        /// <summary>
        /// Получить количество манёвров в очереди
        /// </summary>
        public int QueuedCount => queuedManeuvers.Count;
    }
}

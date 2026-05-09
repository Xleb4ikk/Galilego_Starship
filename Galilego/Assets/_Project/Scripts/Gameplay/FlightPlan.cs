// Добавляем using Galilego.Physics, так как Vector3d и PhysicsSolver находятся там
using Galilego.Physics;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Galilego.Gameplay
{
    /// <summary>
    /// Модель данных для одного узла маневра (Maneuver Node).
    /// </summary>
    [Serializable]
    public class ManeuverNode
    {
        public string Name = "Maneuver";
        public double StartTime; // Время начала маневра (секунды симуляции)
        public double DvTangent; // Δv вдоль касательной (Prograde/Retrograde)
        public double DvNormal;  // Δv вдоль нормали (Normal/Anti-Normal)
        public double DvBinormal;// Δv вдоль бинормали (Radial In/Out)

        // Дополнительные параметры для реализма (из скриншота)
        public double Duration = 0d; // Длительность прожига (если не мгновенный)
        public bool IsInstant = true; // Мгновенный ли импульс

        public ManeuverNode(double time, double tangent = 0, double normal = 0, double binormal = 0)
        {
            StartTime = time;
            DvTangent = tangent;
            DvNormal = normal;
            DvBinormal = binormal;
        }

        public double TotalDeltaV => Math.Sqrt(DvTangent * DvTangent + DvNormal * DvNormal + DvBinormal * DvBinormal);
    }

    /// <summary>
    /// Класс, управляющий списком маневров.
    /// </summary>
    [Serializable]
    public class FlightPlan
    {
        public List<ManeuverNode> Nodes = new List<ManeuverNode>();

        // Глобальные настройки планирования (из скриншота)
        public double PredictionLengthSeconds = 3600d; // По умолчанию 1 час
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
        /// Рассчитывает мировой вектор Δv для конкретного узла.
        /// </summary>
        public static Vector3d CalculateWorldDeltaV(Vector3d position, Vector3d velocity, ManeuverNode node)
        {
            if (node == null) return Vector3d.Zero;

            // Guard against invalid inputs
            if (!position.IsFinite || !velocity.IsFinite) return Vector3d.Zero;

            // Avoid normalizing an almost-zero velocity vector which can lead to NaN propagation
            if (velocity.SqrMagnitude < 0.001d) return Vector3d.Zero;

            // Frenet-Serret frame (Prograde, Normal, Radial)
            Vector3d tangentDir = velocity.Normalized;
            if (!tangentDir.IsFinite) tangentDir = Vector3d.Zero;

            // Normal is perpendicular to the orbital plane
            // N = (V x R) / |V x R|
            Vector3d radialDir = position.Normalized;
            Vector3d normalDir = Vector3d.Cross(velocity, radialDir).Normalized;
            if (!normalDir.IsFinite) normalDir = Vector3d.Zero;

            // Binormal (Radial) completes the right-handed set: B = T x N
            Vector3d binormalDir = Vector3d.Cross(tangentDir, normalDir).Normalized;
            if (!binormalDir.IsFinite) binormalDir = radialDir;

            return (tangentDir * node.DvTangent) +
                   (normalDir * node.DvNormal) +
                   (binormalDir * node.DvBinormal);
        }
    }
}
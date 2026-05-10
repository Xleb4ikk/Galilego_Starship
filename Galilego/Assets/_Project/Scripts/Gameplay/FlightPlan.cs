// Добавляем using Galilego.Physics, так как Vector3d и PhysicsSolver находятся там
using Galilego.Physics;
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
            OrbitalBasis.TryComputeBasis(
                relativePosition,
                relativeVelocity,
                out Vector3d radial,
                out Vector3d normal,
                out Vector3d prograde);

            // Apply Δv in orbital frame
            return (prograde * node.DvPrograde) +
                   (normal * node.DvNormal) +
                   (radial * node.DvRadial);
        }
    }
}

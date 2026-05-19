using System;
using UnityEngine;

namespace Galilego.Physics
{
    /// <summary>
    /// Canonical orbital basis computation.
    /// 
    /// Orbital frame (right-handed):
    ///   Radial (R): direction from central body to spacecraft (position normalized)
    ///   Normal (N): orbit angular momentum direction (R × V normalized)
    ///   Prograde (P): velocity direction perpendicular to radial (N × R normalized)
    /// 
    /// This is the standard orbital mechanics convention:
    ///   R points away from central body
    ///   N is orbit normal (angular momentum direction)
    ///   P completes right-handed triad (approximately velocity direction)
    /// 
    /// Cross product order: R × V gives angular momentum direction (right-hand rule)
    /// </summary>
    public static class OrbitalBasis
    {
        /// <summary>
        /// Compute canonical orbital basis from relative position and velocity.
        /// </summary>
        /// <param name="relativePosition">Position relative to central body (spacecraft - body)</param>
        /// <param name="relativeVelocity">Velocity relative to central body (spacecraftVel - bodyVel)</param>
        /// <param name="radial">Unit vector pointing away from central body</param>
        /// <param name="normal">Unit vector in angular momentum direction (R × V)</param>
        /// <param name="prograde">Unit vector perpendicular to radial in velocity direction (N × R)</param>
        public static void ComputeBasis(
            Vector3d relativePosition,
            Vector3d relativeVelocity,
            out Vector3d radial,
            out Vector3d normal,
            out Vector3d prograde)
        {
            // Radial: direction from central body to spacecraft
            radial = relativePosition.Normalized;
            if (!radial.IsFinite || radial.SqrMagnitude < 1e-12d)
            {
                radial = new Vector3d(1d, 0d, 0d);
            }

            // Normal: angular momentum direction (R × V)
            // Right-hand rule: thumb points along angular momentum
            normal = Vector3d.Cross(relativePosition, relativeVelocity).Normalized;
            if (!normal.IsFinite || normal.SqrMagnitude < 1e-12d)
            {
                // Degenerate case: position and velocity are parallel
                // Choose arbitrary normal perpendicular to radial
                normal = ComputePerpendicular(radial);
            }

            // Prograde: completes right-handed triad (N × R)
            // This is perpendicular to radial, in the direction of motion
            prograde = Vector3d.Cross(normal, radial).Normalized;
            if (!prograde.IsFinite || prograde.SqrMagnitude < 1e-12d)
            {
                prograde = ComputePerpendicular(normal);
            }
        }

        /// <summary>
        /// Compute orbital basis with validation.
        /// Returns true if basis is valid (non-degenerate).
        /// </summary>
        public static bool TryComputeBasis(
            Vector3d relativePosition,
            Vector3d relativeVelocity,
            out Vector3d radial,
            out Vector3d normal,
            out Vector3d prograde)
        {
            radial = Vector3d.Zero;
            normal = Vector3d.Zero;
            prograde = Vector3d.Zero;

            if (!relativePosition.IsFinite || !relativeVelocity.IsFinite)
                return false;

            double posMag = relativePosition.Magnitude;
            double velMag = relativeVelocity.Magnitude;

            if (posMag < 1e-6d || velMag < 1e-6d)
                return false;

            ComputeBasis(relativePosition, relativeVelocity, out radial, out normal, out prograde);

            // Validate orthogonality
            double rn = Math.Abs(Vector3d.Dot(radial, normal));
            double rp = Math.Abs(Vector3d.Dot(radial, prograde));
            double np = Math.Abs(Vector3d.Dot(normal, prograde));

            const double orthoTol = 0.01d;
            if (rn > orthoTol || rp > orthoTol || np > orthoTol)
                return false;

            return true;
        }

        /// <summary>
        /// Compute a unit vector perpendicular to the given vector.
        /// </summary>
        private static Vector3d ComputePerpendicular(Vector3d v)
        {
            // Find the smallest component and cross with that axis
            Vector3d absV = new Vector3d(Math.Abs(v.X), Math.Abs(v.Y), Math.Abs(v.Z));
            
            Vector3d candidate;
            if (absV.X <= absV.Y && absV.X <= absV.Z)
                candidate = new Vector3d(1d, 0d, 0d);
            else if (absV.Y <= absV.X && absV.Y <= absV.Z)
                candidate = new Vector3d(0d, 1d, 0d);
            else
                candidate = new Vector3d(0d, 0d, 1d);

            Vector3d perpendicular = Vector3d.Cross(v, candidate).Normalized;
            return perpendicular.IsFinite ? perpendicular : new Vector3d(0d, 0d, 1d);
        }
    }
}

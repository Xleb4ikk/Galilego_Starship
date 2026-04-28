using System;
using System.Collections.Generic;

namespace Galilego.Physics
{
    public static class PhysicsSolver
    {
        public const double GravitationalConstant = 6.67430e-11d;

        public static Vector3d CalculateAcceleration(Vector3d currentPos, Vector3d anchorPos, double anchorMass)
        {
            if (anchorMass == 0d)
            {
                return Vector3d.Zero;
            }

            Vector3d offset = anchorPos - currentPos;
            double sqrDistance = offset.SqrMagnitude;

            if (sqrDistance <= 0d)
            {
                return Vector3d.Zero;
            }

            double inverseDistance = 1d / Math.Sqrt(sqrDistance);
            double inverseDistanceCubed = inverseDistance / sqrDistance;
            double accelerationScale = GravitationalConstant * anchorMass * inverseDistanceCubed;

            return offset * accelerationScale;
        }

        public static Vector3d CalculateAcceleration(Vector3d currentPos, List<CelestialBody> anchors)
        {
            if (anchors == null)
            {
                throw new ArgumentNullException(nameof(anchors));
            }

            Vector3d totalAcceleration = Vector3d.Zero;

            for (int i = 0; i < anchors.Count; i++)
            {
                CelestialBody anchor = anchors[i];

                if (anchor == null || anchor.Mass == 0d)
                {
                    continue;
                }

                totalAcceleration += CalculateAcceleration(currentPos, anchor.Position, anchor.Mass);
            }

            return totalAcceleration;
        }

        public static IntegrationResult IntegrateRK4(CelestialBody currentBody, List<CelestialBody> anchors, double dt)
        {
            if (currentBody == null)
            {
                throw new ArgumentNullException(nameof(currentBody));
            }

            if (anchors == null)
            {
                throw new ArgumentNullException(nameof(anchors));
            }

            Vector3d position = currentBody.Position;
            Vector3d velocity = currentBody.Velocity;
            double halfDt = dt * 0.5d;
            double sixthDt = dt / 6d;

            Vector3d k1Position = velocity;
            Vector3d k1Velocity = CalculateAcceleration(position, anchors);

            Vector3d k2Position = velocity + (k1Velocity * halfDt);
            Vector3d k2Velocity = CalculateAcceleration(position + (k1Position * halfDt), anchors);

            Vector3d k3Position = velocity + (k2Velocity * halfDt);
            Vector3d k3Velocity = CalculateAcceleration(position + (k2Position * halfDt), anchors);

            Vector3d k4Position = velocity + (k3Velocity * dt);
            Vector3d k4Velocity = CalculateAcceleration(position + (k3Position * dt), anchors);

            Vector3d newPosition = position + ((k1Position + (2d * k2Position) + (2d * k3Position) + k4Position) * sixthDt);
            Vector3d newVelocity = velocity + ((k1Velocity + (2d * k2Velocity) + (2d * k3Velocity) + k4Velocity) * sixthDt);

            return new IntegrationResult(newPosition, newVelocity);
        }

        public static IntegrationResult RK4(CelestialBody currentBody, List<CelestialBody> anchors, double dt)
        {
            return IntegrateRK4(currentBody, anchors, dt);
        }

        public static IntegrationResult RK4(
            CelestialBody currentBody,
            double currentTimeSeconds,
            double dt,
            Func<Vector3d, double, Vector3d> accelerationProvider)
        {
            if (currentBody == null)
            {
                throw new ArgumentNullException(nameof(currentBody));
            }

            return RK4(currentBody.Position, currentBody.Velocity, currentTimeSeconds, dt, accelerationProvider);
        }

        public static IntegrationResult RK4(
            Vector3d position,
            Vector3d velocity,
            double currentTimeSeconds,
            double dt,
            Func<Vector3d, double, Vector3d> accelerationProvider)
        {
            if (accelerationProvider == null)
            {
                throw new ArgumentNullException(nameof(accelerationProvider));
            }

            double halfDt = dt * 0.5d;
            double sixthDt = dt / 6d;

            Vector3d k1Position = velocity;
            Vector3d k1Velocity = accelerationProvider(position, currentTimeSeconds);

            Vector3d k2Position = velocity + (k1Velocity * halfDt);
            Vector3d k2Velocity = accelerationProvider(position + (k1Position * halfDt), currentTimeSeconds + halfDt);

            Vector3d k3Position = velocity + (k2Velocity * halfDt);
            Vector3d k3Velocity = accelerationProvider(position + (k2Position * halfDt), currentTimeSeconds + halfDt);

            Vector3d k4Position = velocity + (k3Velocity * dt);
            Vector3d k4Velocity = accelerationProvider(position + (k3Position * dt), currentTimeSeconds + dt);

            Vector3d newPosition = position + ((k1Position + (2d * k2Position) + (2d * k3Position) + k4Position) * sixthDt);
            Vector3d newVelocity = velocity + ((k1Velocity + (2d * k2Velocity) + (2d * k3Velocity) + k4Velocity) * sixthDt);

            return new IntegrationResult(newPosition, newVelocity);
        }
    }
}

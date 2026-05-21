using System;

namespace Galilego.Core
{
    public sealed class CelestialBody
    {
        public double Mass { get; }
        public double StandardGravitationalParameter { get; }
        public Vector3d Position { get; private set; }
        public Vector3d Velocity { get; private set; }

        public CelestialBody(
            double mass,
            Vector3d position,
            Vector3d velocity,
            double? standardGravitationalParameter = null)
        {
            if (mass < 0d)
            {
                throw new ArgumentOutOfRangeException(nameof(mass), "Mass cannot be negative.");
            }

            Mass = mass;
            StandardGravitationalParameter = standardGravitationalParameter ?? PhysicsSolver.MassToStandardGravitationalParameter(mass);
            Position = position;
            Velocity = velocity;
        }

        public void SetState(Vector3d position, Vector3d velocity)
        {
            Position = position;
            Velocity = velocity;
        }
    }
}

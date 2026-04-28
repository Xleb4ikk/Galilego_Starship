using System;
using UnityEngine;

namespace Galilego.Physics
{
    [Serializable]
    public sealed class ShipSettings
    {
        public Transform VisualTransform;
        public double Mass = 1000d;
        public Vector3d InitialPosition = Vector3d.Zero;
        public Vector3d InitialVelocity = Vector3d.Zero;
    }
}

using System;
using UnityEngine;
using Galilego.Core;

namespace Galilego.Ship
{
    [Serializable]
    public sealed class ShipSettings
    {
        public Transform VisualTransform;
        public double Mass = 1000d;
        public double VisualRadiusMeters = 3d;
        public Vector3d InitialPosition = Vector3d.Zero;
        public Vector3d InitialVelocity = Vector3d.Zero;
    }
}

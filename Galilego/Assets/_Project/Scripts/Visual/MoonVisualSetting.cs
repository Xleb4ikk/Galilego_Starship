using System;
using UnityEngine;

namespace Galilego.MoonVisualSetting
{
    [Serializable]
    public sealed class MoonSettingVisual
    {
        public string Name = "Moon";
        public Transform VisualTransform;
        public double Radius = 1d;
    }
}

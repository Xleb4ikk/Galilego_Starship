using System;
using UnityEngine;

namespace Galilego.PlanetModelInfo
{
    [Serializable]
    public sealed class PlanetModels
    {
        public string Name = "Planet";
        
        public Transform PrefabPlanet;
        public GameObject HighQualityModel;
        public GameObject MidQualityModel;
        public GameObject LowQualityModel;
    }
}

using System.Collections.Generic;

namespace Galilego.Physics
{
    public static class GalileanMoonPresets
    {
        public const double J2000TdbSeconds = 0d;

        public static List<MoonRail> CreateJplGalileanMoons()
        {
            return new List<MoonRail>
            {
                CreateIo(),
                CreateEuropa(),
                CreateGanymede(),
                CreateCallisto()
            };
        }

        private static MoonRail CreateIo()
        {
            return CreateMoon(
                name: "Io",
                mass: 8.932e22d,
                standardGravitationalParameter: 5.95991547e12d,
                radius: 1.82149e6d,
                semiMajorAxis: 4.21800e8d,
                eccentricity: 0.004d,
                inclinationDegrees: 0.0d,
                longitudeOfAscendingNodeDegrees: 0.0d,
                argumentOfPeriapsisDegrees: 49.1d,
                meanAnomalyAtEpochDegrees: 330.9d);
        }

        private static MoonRail CreateEuropa()
        {
            return CreateMoon(
                name: "Europa",
                mass: 4.800e22d,
                standardGravitationalParameter: 3.20271210e12d,
                radius: 1.56080e6d,
                semiMajorAxis: 6.71100e8d,
                eccentricity: 0.009d,
                inclinationDegrees: 0.5d,
                longitudeOfAscendingNodeDegrees: 184.0d,
                argumentOfPeriapsisDegrees: 45.0d,
                meanAnomalyAtEpochDegrees: 345.4d);
        }

        private static MoonRail CreateGanymede()
        {
            return CreateMoon(
                name: "Ganymede",
                mass: 1.4819e23d,
                standardGravitationalParameter: 9.88783275e12d,
                radius: 2.63120e6d,
                semiMajorAxis: 1.07040e9d,
                eccentricity: 0.001d,
                inclinationDegrees: 0.2d,
                longitudeOfAscendingNodeDegrees: 58.5d,
                argumentOfPeriapsisDegrees: 198.3d,
                meanAnomalyAtEpochDegrees: 324.8d);
        }

        private static MoonRail CreateCallisto()
        {
            return CreateMoon(
                name: "Callisto",
                mass: 1.0759e23d,
                standardGravitationalParameter: 7.17928340e12d,
                radius: 2.41030e6d,
                semiMajorAxis: 1.88270e9d,
                eccentricity: 0.007d,
                inclinationDegrees: 0.3d,
                longitudeOfAscendingNodeDegrees: 309.1d,
                argumentOfPeriapsisDegrees: 43.8d,
                meanAnomalyAtEpochDegrees: 87.4d);
        }

        private static MoonRail CreateMoon(
            string name,
            double mass,
            double standardGravitationalParameter,
            double radius,
            double semiMajorAxis,
            double eccentricity,
            double inclinationDegrees,
            double longitudeOfAscendingNodeDegrees,
            double argumentOfPeriapsisDegrees,
            double meanAnomalyAtEpochDegrees)
        {
            MoonRail moon = new MoonRail
            {
                Name = name,
                Mass = mass,
                StandardGravitationalParameter = standardGravitationalParameter,
                Radius = radius,
                SemiMajorAxis = semiMajorAxis,
                Eccentricity = eccentricity,
                InclinationDegrees = inclinationDegrees,
                LongitudeOfAscendingNodeDegrees = longitudeOfAscendingNodeDegrees,
                ArgumentOfPeriapsisDegrees = argumentOfPeriapsisDegrees,
                MeanAnomalyAtEpochDegrees = meanAnomalyAtEpochDegrees,
                EpochTimeSeconds = J2000TdbSeconds
            };

            moon.ApplySemiMajorAxisAndEccentricity();
            moon.SyncMassFromGravitationalParameter();
            return moon;
        }
    }
}

// ============================================================================
// ORBITAL ELEMENTS
// ============================================================================
// Структура для представления орбитальных элементов Кеплера
// Перемещено из Vector3d.cs для архитектурной чистоты (Фаза 2.2)

using System;

namespace Galilego.Core
{
    /// <summary>
    /// Орбитальные элементы Кеплера, описывающие орбиту небесного тела.
    /// </summary>
    public readonly struct OrbitalElements
    {
        public readonly bool IsValid;
        public readonly bool IsBound;
        public readonly double SemiMajorAxis;
        public readonly double Eccentricity;
        public readonly double InclinationDegrees;
        public readonly double LongitudeOfAscendingNodeDegrees;
        public readonly double ArgumentOfPeriapsisDegrees;
        public readonly double TrueAnomalyDegrees;
        public readonly double MeanAnomalyDegrees;
        public readonly double PeriapsisDistance;
        public readonly double ApoapsisDistance;
        public readonly double OrbitalPeriodSeconds;
        public readonly double SpecificOrbitalEnergy;
        public readonly double SpecificAngularMomentum;

        private OrbitalElements(
            bool isValid,
            bool isBound,
            double semiMajorAxis,
            double eccentricity,
            double inclinationDegrees,
            double longitudeOfAscendingNodeDegrees,
            double argumentOfPeriapsisDegrees,
            double trueAnomalyDegrees,
            double meanAnomalyDegrees,
            double periapsisDistance,
            double apoapsisDistance,
            double orbitalPeriodSeconds,
            double specificOrbitalEnergy,
            double specificAngularMomentum)
        {
            IsValid = isValid;
            IsBound = isBound;
            SemiMajorAxis = semiMajorAxis;
            Eccentricity = eccentricity;
            InclinationDegrees = inclinationDegrees;
            LongitudeOfAscendingNodeDegrees = longitudeOfAscendingNodeDegrees;
            ArgumentOfPeriapsisDegrees = argumentOfPeriapsisDegrees;
            TrueAnomalyDegrees = trueAnomalyDegrees;
            MeanAnomalyDegrees = meanAnomalyDegrees;
            PeriapsisDistance = periapsisDistance;
            ApoapsisDistance = apoapsisDistance;
            OrbitalPeriodSeconds = orbitalPeriodSeconds;
            SpecificOrbitalEnergy = specificOrbitalEnergy;
            SpecificAngularMomentum = specificAngularMomentum;
        }

        public static OrbitalElements Invalid => new OrbitalElements(
            false,
            false,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN,
            double.NaN);

        /// <summary>
        /// Вычисляет орбитальные элементы из векторов состояния.
        /// </summary>
        /// <param name="relativePosition">Вектор положения относительно центрального тела в АСТРОДИНАМИЧЕСКОЙ системе (Z-up)</param>
        /// <param name="relativeVelocity">Вектор скорости относительно центрального тела в АСТРОДИНАМИЧЕСКОЙ системе (Z-up)</param>
        /// <param name="standardGravitationalParameter">Стандартный гравитационный параметр μ = G·M центрального тела (м³/с²)</param>
        /// <returns>Орбитальные элементы или Invalid, если расчёт невозможен</returns>
        /// <remarks>
        /// ⚠️ ВАЖНО: КОНТРАКТ АСТРОДИНАМИЧЕСКОЙ СИСТЕМЫ КООРДИНАТ
        /// 
        /// Этот метод ожидает, что оба вектора переданы в АСТРОДИНАМИЧЕСКОЙ системе координат,
        /// где ось +Z ВСЕГДА указывает на север опорной плоскости (эклиптика/экватор).
        /// 
        /// Если ваши векторы находятся в симуляционной системе координат Unity
        /// (где "вверх" может быть Y или Z в зависимости от настроек), вы ДОЛЖНЫ
        /// преобразовать их перед вызовом этого метода:
        /// 
        /// <code>
        /// // ✅ ПРАВИЛЬНО:
        /// var astroPos = universeManager.ConvertSimulationToAstrodynamicFrame(simPos);
        /// var astroVel = universeManager.ConvertSimulationToAstrodynamicFrame(simVel);
        /// var elements = OrbitalElements.FromState(astroPos, astroVel, mu);
        /// 
        /// // ❌ НЕПРАВИЛЬНО (приведёт к неверным наклонению, LAN, аргументу перицентра):
        /// var elements = OrbitalElements.FromState(simPos, simVel, mu);
        /// </code>
        /// 
        /// Формулы орбитальной механики:
        /// - Угловой момент: h = r × v
        /// - Вектор эксцентриситета: e = (v × h)/μ - r/|r|
        /// - Узел: n = k × h, где k = (0, 0, 1) в астродинамической системе
        /// - Наклонение: i = arccos(h_z / |h|)
        /// - Долгота восходящего узла: Ω = arctan2(n_y, n_x)
        /// - Аргумент перицентра: ω = arccos(n · e / (|n| |e|))
        /// </remarks>
        public static OrbitalElements FromState(Vector3d relativePosition, Vector3d relativeVelocity, double standardGravitationalParameter)
        {
            const double epsilon = 1e-10d;

            double radius = relativePosition.Magnitude;
            double speedSquared = relativeVelocity.SqrMagnitude;
            if (radius <= epsilon || standardGravitationalParameter <= 0d)
            {
                return Invalid;
            }

            Vector3d angularMomentum = Vector3d.Cross(relativePosition, relativeVelocity);
            double angularMomentumMagnitude = angularMomentum.Magnitude;
            if (angularMomentumMagnitude <= epsilon)
            {
                return Invalid;
            }

#if UNITY_EDITOR
            // 🛡️ ЗАЩИТНАЯ ПРОВЕРКА: Эвристика для обнаружения непреобразованных векторов
            // Если плоскость орбиты почти перпендикулярна XY (h_z ≈ 0), но имеет большую
            // компоненту по X или Y, возможно, векторы переданы в Y-up системе без преобразования
            Vector3d hNorm = angularMomentum.Normalized;
            if (Math.Abs(hNorm.Z) < 0.1 && (Math.Abs(hNorm.X) > 0.7 || Math.Abs(hNorm.Y) > 0.7))
            {
                UnityEngine.Debug.LogWarning(
                    "[OrbitalElements.FromState] ⚠️ ПОДОЗРЕНИЕ: Векторы могут быть в Y-up системе координат!\n" +
                    $"Угловой момент: h = {angularMomentum}, |h| = {angularMomentumMagnitude:F2}\n" +
                    $"Нормализованный: h_norm = ({hNorm.X:F3}, {hNorm.Y:F3}, {hNorm.Z:F3})\n" +
                    "Убедитесь, что вы преобразовали векторы через UniverseManager.ConvertSimulationToAstrodynamicFrame() " +
                    "перед вызовом FromState!");
            }
#endif

            Vector3d node = Vector3d.Cross(new Vector3d(0d, 0d, 1d), angularMomentum);
            double nodeMagnitude = node.Magnitude;
            Vector3d eccentricityVector =
                (Vector3d.Cross(relativeVelocity, angularMomentum) / standardGravitationalParameter) -
                (relativePosition / radius);

            double eccentricity = eccentricityVector.Magnitude;
            double energy = (0.5d * speedSquared) - (standardGravitationalParameter / radius);
            bool isParabolic = Math.Abs(energy) <= epsilon;
            double semiMajorAxis = isParabolic
                ? double.PositiveInfinity
                : -standardGravitationalParameter / (2d * energy);

            bool isBound = !isParabolic && semiMajorAxis > 0d && eccentricity < 1d;
            double periapsisDistance = (angularMomentumMagnitude * angularMomentumMagnitude) /
                (standardGravitationalParameter * (1d + eccentricity));
            double apoapsisDistance = isBound
                ? semiMajorAxis * (1d + eccentricity)
                : double.PositiveInfinity;
            double orbitalPeriodSeconds = isBound
                ? 2d * Math.PI * Math.Sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / standardGravitationalParameter)
                : double.PositiveInfinity;

            double inclination = SafeAcos(angularMomentum.Z / angularMomentumMagnitude);
            double longitudeOfAscendingNode = nodeMagnitude > epsilon
                ? NormalizeAngle(Math.Atan2(node.Y, node.X))
                : 0d;

            double argumentOfPeriapsis = 0d;
            if (nodeMagnitude > epsilon && eccentricity > epsilon)
            {
                argumentOfPeriapsis = SafeAcos(Vector3d.Dot(node, eccentricityVector) / (nodeMagnitude * eccentricity));
                if (eccentricityVector.Z < 0d)
                {
                    argumentOfPeriapsis = (2d * Math.PI) - argumentOfPeriapsis;
                }
            }

            double trueAnomaly = 0d;
            if (eccentricity > epsilon)
            {
                trueAnomaly = SafeAcos(Vector3d.Dot(eccentricityVector, relativePosition) / (eccentricity * radius));
                if (Vector3d.Dot(relativePosition, relativeVelocity) < 0d)
                {
                    trueAnomaly = (2d * Math.PI) - trueAnomaly;
                }
            }
            else if (nodeMagnitude > epsilon)
            {
                trueAnomaly = SafeAcos(Vector3d.Dot(node, relativePosition) / (nodeMagnitude * radius));
                if (relativePosition.Z < 0d)
                {
                    trueAnomaly = (2d * Math.PI) - trueAnomaly;
                }
            }
            else
            {
                trueAnomaly = NormalizeAngle(Math.Atan2(relativePosition.Y, relativePosition.X));
            }

            double meanAnomaly = double.NaN;
            if (isBound)
            {
                double eccentricAnomaly = 2d * Math.Atan2(
                    Math.Sqrt(1d - eccentricity) * Math.Sin(trueAnomaly * 0.5d),
                    Math.Sqrt(1d + eccentricity) * Math.Cos(trueAnomaly * 0.5d));
                meanAnomaly = NormalizeAngle(eccentricAnomaly - (eccentricity * Math.Sin(eccentricAnomaly)));
            }

            return new OrbitalElements(
                true,
                isBound,
                semiMajorAxis,
                eccentricity,
                RadiansToDegrees(inclination),
                RadiansToDegrees(longitudeOfAscendingNode),
                RadiansToDegrees(argumentOfPeriapsis),
                RadiansToDegrees(NormalizeAngle(trueAnomaly)),
                double.IsNaN(meanAnomaly) ? double.NaN : RadiansToDegrees(meanAnomaly),
                periapsisDistance,
                apoapsisDistance,
                orbitalPeriodSeconds,
                energy,
                angularMomentumMagnitude);
        }

        /// <summary>
        /// Вычисляет радиус сферы влияния (Sphere of Influence) по формуле Лапласа.
        /// </summary>
        /// <param name="semiMajorAxis">Большая полуось орбиты вокруг родительского тела (м)</param>
        /// <param name="orbitingMass">Масса орбитирующего тела (кг)</param>
        /// <param name="parentMass">Масса родительского тела (кг)</param>
        /// <returns>Радиус сферы влияния (м)</returns>
        public static double CalculateSphereOfInfluenceRadius(double semiMajorAxis, double orbitingMass, double parentMass)
        {
            if (semiMajorAxis <= 0d || orbitingMass <= 0d || parentMass <= 0d)
            {
                return 0d;
            }

            return semiMajorAxis * Math.Pow(orbitingMass / parentMass, 0.4d);
        }

        /// <summary>
        /// Вычисляет радиус сферы Хилла (Hill sphere).
        /// </summary>
        /// <param name="semiMajorAxis">Большая полуось орбиты вокруг родительского тела (м)</param>
        /// <param name="eccentricity">Эксцентриситет орбиты</param>
        /// <param name="orbitingMass">Масса орбитирующего тела (кг)</param>
        /// <param name="parentMass">Масса родительского тела (кг)</param>
        /// <returns>Радиус сферы Хилла (м)</returns>
        public static double CalculateHillRadius(double semiMajorAxis, double eccentricity, double orbitingMass, double parentMass)
        {
            if (semiMajorAxis <= 0d || orbitingMass <= 0d || parentMass <= 0d)
            {
                return 0d;
            }

            return semiMajorAxis * (1d - Math.Max(0d, eccentricity)) * Math.Pow(orbitingMass / (3d * parentMass), 1d / 3d);
        }

        private static double SafeAcos(double value)
        {
            return Math.Acos(Math.Max(-1d, Math.Min(1d, value)));
        }

        private static double NormalizeAngle(double angle)
        {
            double twoPi = Math.PI * 2d;
            angle %= twoPi;
            if (angle < 0d)
            {
                angle += twoPi;
            }

            return angle;
        }

        private static double RadiansToDegrees(double radians)
        {
            return radians * (180d / Math.PI);
        }
    }
}

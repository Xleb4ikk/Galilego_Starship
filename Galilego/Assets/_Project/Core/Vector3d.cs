using System;

namespace Galilego.Physics
{
    [Serializable]
    public struct Vector3d
    {
        public static readonly Vector3d Zero = new Vector3d(0d, 0d, 0d);

        public double X;
        public double Y;
        public double Z;

        public bool IsFinite => IsFiniteComponent(X) && IsFiniteComponent(Y) && IsFiniteComponent(Z);
        public double SqrMagnitude => (X * X) + (Y * Y) + (Z * Z);
        public double Magnitude => Math.Sqrt(SqrMagnitude);
        public Vector3d Normalized
        {
            get
            {
                double magnitude = Magnitude;
                return magnitude > 0d ? this / magnitude : Zero;
            }
        }

        public Vector3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vector3d operator +(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }

        public static Vector3d operator -(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        public static Vector3d operator -(Vector3d value)
        {
            return new Vector3d(-value.X, -value.Y, -value.Z);
        }

        public static Vector3d operator *(Vector3d vector, double scalar)
        {
            return new Vector3d(vector.X * scalar, vector.Y * scalar, vector.Z * scalar);
        }

        public static Vector3d operator *(double scalar, Vector3d vector)
        {
            return vector * scalar;
        }

        public static Vector3d operator /(Vector3d vector, double scalar)
        {
            if (scalar == 0d)
            {
                throw new DivideByZeroException("Vector3d cannot be divided by zero.");
            }

            return new Vector3d(vector.X / scalar, vector.Y / scalar, vector.Z / scalar);
        }

        public static double Dot(Vector3d left, Vector3d right)
        {
            return (left.X * right.X) + (left.Y * right.Y) + (left.Z * right.Z);
        }

        public static Vector3d Cross(Vector3d left, Vector3d right)
        {
            return new Vector3d(
                (left.Y * right.Z) - (left.Z * right.Y),
                (left.Z * right.X) - (left.X * right.Z),
                (left.X * right.Y) - (left.Y * right.X));
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }

        private static bool IsFiniteComponent(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }

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

        public static double CalculateSphereOfInfluenceRadius(double semiMajorAxis, double orbitingMass, double parentMass)
        {
            if (semiMajorAxis <= 0d || orbitingMass <= 0d || parentMass <= 0d)
            {
                return 0d;
            }

            return semiMajorAxis * Math.Pow(orbitingMass / parentMass, 0.4d);
        }

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

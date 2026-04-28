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

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }

        private static bool IsFiniteComponent(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}

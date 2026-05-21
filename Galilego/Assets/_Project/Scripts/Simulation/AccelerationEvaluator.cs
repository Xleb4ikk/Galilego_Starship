using Unity.Collections;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    public static class AccelerationEvaluator
    {
        public const double GravitationalConstant = 6.67430e-11d;
        public const double MinSqrDistance = 100.0d;
        public const double G0 = 9.80665d;

        public static double3 CalculateAcceleration(
            double3 shipPos,
            NativeArray<BodyState> bodies)
        {
            double3 total = double3.zero;
            for (int i = 0; i < bodies.Length; i++)
            {
                total += BodyGravity(shipPos, bodies[i].Position, bodies[i].StandardGravitationalParameter);
            }
            return total;
        }

        public static double3 CalculateAcceleration(
            double3 shipPos,
            double bodyPos, double bodySGP)
        {
            return BodyGravity(shipPos, bodyPos, bodySGP);
        }

        private static double3 BodyGravity(double3 shipPos, double3 bodyPos, double sgp)
        {
            if (sgp == 0.0) return double3.zero;
            double3 offset = bodyPos - shipPos;
            double sqrDist = math.lengthsq(offset);
            if (sqrDist <= 0.0 || sqrDist < MinSqrDistance) return double3.zero;

            double invDist = 1.0 / math.sqrt(sqrDist);
            double invDistCubed = invDist / sqrDist;
            double scale = sgp * invDistCubed;
            return offset * scale;
        }

        public static double3 EvaluateMoonPosition(
            ref MoonOrbitData orbit,
            double timeSeconds,
            int planeMapping)
        {
            double sma = math.max(orbit.SemiMajorAxis, 1.0);
            double ecc = math.clamp(orbit.Eccentricity, 0.0, 0.999);
            double mu = orbit.GravitationalParameter;
            double meanMotion = math.sqrt(mu / (sma * sma * sma));
            double meanAnomaly = NormalizeAngle(orbit.MeanAnomalyAtEpochRad + meanMotion * (timeSeconds - orbit.EpochTimeSeconds));
            double eccentricAnomaly = SolveEccentricAnomaly(meanAnomaly, ecc);

            double cosE = math.cos(eccentricAnomaly);
            double sinE = math.sin(eccentricAnomaly);
            double radius = sma * (1.0 - ecc * cosE);
            double yScale = math.sqrt(1.0 - ecc * ecc);

            double3 orbitalPos = new double3(
                sma * (cosE - ecc),
                sma * yScale * sinE,
                0.0);

            double3 worldPos = RotateOrbitalToWorld(orbitalPos,
                orbit.AscendingNodeRad, orbit.InclinationRad, orbit.PeriapsisArgRad);

            return ConvertFrame(worldPos, planeMapping);
        }

        private static double3 RotateOrbitalToWorld(
            double3 orbital,
            double ascendingNode,
            double inclination,
            double periapsis)
        {
            double cosO = math.cos(ascendingNode);
            double sinO = math.sin(ascendingNode);
            double cosI = math.cos(inclination);
            double sinI = math.sin(inclination);
            double cosW = math.cos(periapsis);
            double sinW = math.sin(periapsis);

            double x = ((cosO * cosW) - (sinO * sinW * cosI)) * orbital.x +
                       ((-cosO * sinW) - (sinO * cosW * cosI)) * orbital.y;

            double y = ((sinO * cosW) + (cosO * sinW * cosI)) * orbital.x +
                       ((-sinO * sinW) + (cosO * cosW * cosI)) * orbital.y;

            double z = (sinW * sinI * orbital.x) + (cosW * sinI * orbital.y);

            return new double3(x, y, z);
        }

        private static double SolveEccentricAnomaly(double meanAnomaly, double eccentricity)
        {
            double estimate = eccentricity < 0.8 ? meanAnomaly : math.PI;
            for (int i = 0; i < 8; i++)
            {
                double f = estimate - eccentricity * math.sin(estimate) - meanAnomaly;
                double df = 1.0 - eccentricity * math.cos(estimate);
                estimate -= f / df;
            }
            return estimate;
        }

        private static double NormalizeAngle(double angle)
        {
            double twoPi = math.PI * 2.0;
            angle %= twoPi;
            if (angle < 0.0) angle += twoPi;
            return angle;
        }

        private static double3 ConvertFrame(double3 v, int planeMapping)
        {
            if (planeMapping == 0) return v;
            return new double3(v.x, v.z, v.y);
        }

        public static double3 HermiteInterpolate(
            double3 p0, double3 v0,
            double3 p1, double3 v1,
            double t0, double t1, double t)
        {
            double dt = t1 - t0;
            if (dt <= 0.0) return p0;

            double s = (t - t0) / dt;

            double s2 = s * s;
            double s3 = s2 * s;

            double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
            double h10 = s3 - 2.0 * s2 + s;
            double h01 = -2.0 * s3 + 3.0 * s2;
            double h11 = s3 - s2;

            double3 tan0 = v0 * dt;
            double3 tan1 = v1 * dt;

            return h00 * p0 + h10 * tan0 + h01 * p1 + h11 * tan1;
        }

        public static double3 ComputeCentralVelocity(
            double3 pPrev, double3 pNext,
            double tPrev, double tNext)
        {
            double dt = tNext - tPrev;
            if (dt <= 0.0) return double3.zero;
            return (pNext - pPrev) / dt;
        }
    }
}

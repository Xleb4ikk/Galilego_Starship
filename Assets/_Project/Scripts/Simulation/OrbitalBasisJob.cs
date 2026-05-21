using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Galilego.Simulation
{
    [BurstCompile]
    public struct OrbitalBasisJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<double3> Positions;
        [ReadOnly] public NativeArray<double3> Velocities;

        public NativeArray<double3> Radials;
        public NativeArray<double3> Normals;
        public NativeArray<double3> Progrades;

        public void Execute(int index)
        {
            ComputeBasis(Positions[index], Velocities[index],
                out double3 r, out double3 n, out double3 p);
            Radials[index] = r;
            Normals[index] = n;
            Progrades[index] = p;
        }

        public static void ComputeBasis(
            double3 relativePosition,
            double3 relativeVelocity,
            out double3 radial,
            out double3 normal,
            out double3 prograde)
        {
            double posMag = math.length(relativePosition);
            double velMag = math.length(relativeVelocity);

            if (posMag < 1e-12)
            {
                radial = new double3(1, 0, 0);
                normal = new double3(0, 1, 0);
                prograde = new double3(0, 0, 1);
                return;
            }

            radial = relativePosition / posMag;

            double3 crossRN = math.cross(relativePosition, relativeVelocity);
            double crossMag = math.length(crossRN);
            if (crossMag < 1e-12)
            {
                normal = ComputePerpendicular(radial);
            }
            else
            {
                normal = crossRN / crossMag;
            }

            prograde = math.cross(normal, radial);
            double proMag = math.length(prograde);
            if (proMag < 1e-12)
            {
                prograde = ComputePerpendicular(normal);
            }
            else
            {
                prograde /= proMag;
            }
        }

        private static double3 ComputePerpendicular(double3 v)
        {
            double3 absV = new double3(math.abs(v.x), math.abs(v.y), math.abs(v.z));
            double3 candidate;

            if (absV.x <= absV.y && absV.x <= absV.z)
                candidate = new double3(1, 0, 0);
            else if (absV.y <= absV.x && absV.y <= absV.z)
                candidate = new double3(0, 1, 0);
            else
                candidate = new double3(0, 0, 1);

            double3 perp = math.cross(v, candidate);
            double mag = math.length(perp);
            return mag > 1e-12 ? perp / mag : new double3(0, 0, 1);
        }
    }
}

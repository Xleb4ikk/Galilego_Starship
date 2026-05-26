// ============================================================================
// OPTIMIZATION VALIDATION TESTS
// ============================================================================
// Unit tests to ensure Burst-optimized code produces identical results
// to the original managed code implementation

using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using Galilego.Core;
using Galilego.Simulation;

namespace Galilego.Tests.Editor
{
    [TestFixture]
    public class OptimizationValidationTests
    {
        // Tolerance adjusted for realistic numerical precision
        // Burst and Managed may have tiny differences due to floating-point operations order
        // For orbital mechanics: 1e-3 = 1mm for distances, 0.001 degrees for angles
        private const double TOLERANCE = 1e-3; // Realistic precision tolerance
        private const double ANGLE_TOLERANCE = 0.01; // 0.01 degrees for angles (~175m at 1 million km)
        private const double TIME_TOLERANCE = 0.01; // 0.01 seconds for time periods

        [Test]
        public void OrbitalElements_BurstVsManaged_CircularOrbit()
        {
            // Test case: Circular orbit around Jupiter
            Vector3d pos = new Vector3d(4.22e8, 0, 0); // ~422,000 km (Io's orbit)
            Vector3d vel = new Vector3d(0, 17334, 0); // Circular velocity
            double mu = 1.266865319e17; // Jupiter's μ

            // Managed version
            var managedResult = OrbitalElements.FromState(pos, vel, mu);

            // Burst version
            NativeArray<double3> positions = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double3> velocities = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double> mus = new NativeArray<double>(1, Allocator.TempJob);
            NativeArray<OrbitalElementsData> results = new NativeArray<OrbitalElementsData>(1, Allocator.TempJob);

            positions[0] = new double3(pos.X, pos.Y, pos.Z);
            velocities[0] = new double3(vel.X, vel.Y, vel.Z);
            mus[0] = mu;

            OrbitalElements.CalculateBatch(positions, velocities, mus, results).Complete();
            var burstResult = results[0];

            // Validate
            Assert.AreEqual(1, burstResult.IsValid, "Burst result should be valid");
            Assert.AreEqual(managedResult.SemiMajorAxis, burstResult.SemiMajorAxis, TOLERANCE, "Semi-major axis mismatch");
            Assert.AreEqual(managedResult.Eccentricity, burstResult.Eccentricity, TOLERANCE, "Eccentricity mismatch");
            Assert.AreEqual(managedResult.InclinationDegrees, burstResult.InclinationDegrees, ANGLE_TOLERANCE, "Inclination mismatch");
            Assert.AreEqual(managedResult.OrbitalPeriodSeconds, burstResult.OrbitalPeriodSeconds, TIME_TOLERANCE, "Orbital period mismatch");

            // Cleanup
            positions.Dispose();
            velocities.Dispose();
            mus.Dispose();
            results.Dispose();
        }

        [Test]
        public void OrbitalElements_BurstVsManaged_EllipticalOrbit()
        {
            // Test case: Elliptical orbit (e = 0.3)
            Vector3d pos = new Vector3d(5e8, 0, 0);
            Vector3d vel = new Vector3d(0, 14000, 0);
            double mu = 1.266865319e17;

            var managedResult = OrbitalElements.FromState(pos, vel, mu);

            NativeArray<double3> positions = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double3> velocities = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double> mus = new NativeArray<double>(1, Allocator.TempJob);
            NativeArray<OrbitalElementsData> results = new NativeArray<OrbitalElementsData>(1, Allocator.TempJob);

            positions[0] = new double3(pos.X, pos.Y, pos.Z);
            velocities[0] = new double3(vel.X, vel.Y, vel.Z);
            mus[0] = mu;

            OrbitalElements.CalculateBatch(positions, velocities, mus, results).Complete();
            var burstResult = results[0];

            Assert.AreEqual(1, burstResult.IsValid);
            Assert.AreEqual(managedResult.SemiMajorAxis, burstResult.SemiMajorAxis, TOLERANCE);
            Assert.AreEqual(managedResult.Eccentricity, burstResult.Eccentricity, TOLERANCE);
            Assert.AreEqual(managedResult.PeriapsisDistance, burstResult.PeriapsisDistance, TOLERANCE);
            Assert.AreEqual(managedResult.ApoapsisDistance, burstResult.ApoapsisDistance, TOLERANCE);

            positions.Dispose();
            velocities.Dispose();
            mus.Dispose();
            results.Dispose();
        }

        [Test]
        public void OrbitalElements_BurstVsManaged_InclinedOrbit()
        {
            // Test case: Inclined orbit (45 degrees)
            Vector3d pos = new Vector3d(4e8, 0, 2.83e8); // 45° inclination
            Vector3d vel = new Vector3d(0, 15000, 0);
            double mu = 1.266865319e17;

            var managedResult = OrbitalElements.FromState(pos, vel, mu);

            NativeArray<double3> positions = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double3> velocities = new NativeArray<double3>(1, Allocator.TempJob);
            NativeArray<double> mus = new NativeArray<double>(1, Allocator.TempJob);
            NativeArray<OrbitalElementsData> results = new NativeArray<OrbitalElementsData>(1, Allocator.TempJob);

            positions[0] = new double3(pos.X, pos.Y, pos.Z);
            velocities[0] = new double3(vel.X, vel.Y, vel.Z);
            mus[0] = mu;

            OrbitalElements.CalculateBatch(positions, velocities, mus, results).Complete();
            var burstResult = results[0];

            Assert.AreEqual(1, burstResult.IsValid);
            Assert.AreEqual(managedResult.InclinationDegrees, burstResult.InclinationDegrees, ANGLE_TOLERANCE);
            Assert.AreEqual(managedResult.LongitudeOfAscendingNodeDegrees, burstResult.LongitudeOfAscendingNodeDegrees, ANGLE_TOLERANCE);
            Assert.AreEqual(managedResult.ArgumentOfPeriapsisDegrees, burstResult.ArgumentOfPeriapsisDegrees, ANGLE_TOLERANCE);

            positions.Dispose();
            velocities.Dispose();
            mus.Dispose();
            results.Dispose();
        }

        [Test]
        public void OrbitalElements_BatchProcessing_MultipleOrbits()
        {
            // Test batch processing with 10 different orbits
            int count = 10;
            NativeArray<double3> positions = new NativeArray<double3>(count, Allocator.TempJob);
            NativeArray<double3> velocities = new NativeArray<double3>(count, Allocator.TempJob);
            NativeArray<double> mus = new NativeArray<double>(count, Allocator.TempJob);
            NativeArray<OrbitalElementsData> results = new NativeArray<OrbitalElementsData>(count, Allocator.TempJob);

            // Generate test orbits with varying parameters
            for (int i = 0; i < count; i++)
            {
                double radius = 4e8 + i * 1e8; // Varying orbital radii
                positions[i] = new double3(radius, 0, 0);
                velocities[i] = new double3(0, 15000 - i * 500, 0);
                mus[i] = 1.266865319e17;
            }

            // Execute batch job
            OrbitalElements.CalculateBatch(positions, velocities, mus, results).Complete();

            // Validate all results
            for (int i = 0; i < count; i++)
            {
                Assert.AreEqual(1, results[i].IsValid, $"Result {i} should be valid");
                Assert.Greater(results[i].SemiMajorAxis, 0, $"Result {i} should have positive semi-major axis");
                Assert.GreaterOrEqual(results[i].Eccentricity, 0, $"Result {i} should have non-negative eccentricity");
                Assert.Less(results[i].Eccentricity, 1, $"Result {i} should be elliptical");
            }

            positions.Dispose();
            velocities.Dispose();
            mus.Dispose();
            results.Dispose();
        }

        [Test]
        public void ApsisCalculation_ValidEllipticalOrbit()
        {
            // Test apsis calculation for a known elliptical orbit
            double mu = 1.266865319e17;
            double radius = 7.1492e7; // Jupiter radius

            // Create orbital elements for elliptical orbit
            NativeArray<OrbitalElementsData> elements = new NativeArray<OrbitalElementsData>(1, Allocator.TempJob);
            NativeArray<double> segmentTimes = new NativeArray<double>(1, Allocator.TempJob);
            NativeArray<ApsisResultPair> results = new NativeArray<ApsisResultPair>(1, Allocator.TempJob);

            // Manually construct orbital elements
            elements[0] = new OrbitalElementsData
            {
                IsValid = 1,
                IsBound = 1,
                SemiMajorAxis = 5e8,
                Eccentricity = 0.2,
                EccentricityVector = new double3(0.2, 0, 0),
                MeanAnomalyDegrees = 0,
                OrbitalPeriodSeconds = 100000
            };
            segmentTimes[0] = 0;

            var job = new ApsisCalculationJob
            {
                Elements = elements,
                SegmentStartTimes = segmentTimes,
                Mu = mu,
                CentralBodyRadius = radius,
                CircularOrbitThreshold = 0.001,
                Results = results
            };

            Unity.Jobs.IJobParallelForExtensions.Schedule(job, 1, 1).Complete();

            var result = results[0];
            Assert.AreEqual(1, result.PeValid, "Periapsis should be valid");
            Assert.AreEqual(1, result.ApValid, "Apoapsis should be valid");
            Assert.Greater(result.ApAltitude, result.PeAltitude, "Apoapsis altitude should be greater than periapsis");

            elements.Dispose();
            segmentTimes.Dispose();
            results.Dispose();
        }

        [Test]
        public void ApsisCalculation_CircularOrbit_NoApsides()
        {
            // Test that circular orbits don't produce apsis markers
            double mu = 1.266865319e17;
            double radius = 7.1492e7;

            NativeArray<OrbitalElementsData> elements = new NativeArray<OrbitalElementsData>(1, Allocator.TempJob);
            NativeArray<double> segmentTimes = new NativeArray<double>(1, Allocator.TempJob);
            NativeArray<ApsisResultPair> results = new NativeArray<ApsisResultPair>(1, Allocator.TempJob);

            elements[0] = new OrbitalElementsData
            {
                IsValid = 1,
                IsBound = 1,
                SemiMajorAxis = 5e8,
                Eccentricity = 0.0001, // Nearly circular
                EccentricityVector = new double3(0.0001, 0, 0),
                MeanAnomalyDegrees = 0,
                OrbitalPeriodSeconds = 100000
            };
            segmentTimes[0] = 0;

            var job = new ApsisCalculationJob
            {
                Elements = elements,
                SegmentStartTimes = segmentTimes,
                Mu = mu,
                CentralBodyRadius = radius,
                CircularOrbitThreshold = 0.001,
                Results = results
            };

            Unity.Jobs.IJobParallelForExtensions.Schedule(job, 1, 1).Complete();

            var result = results[0];
            Assert.AreEqual(0, result.PeValid, "Circular orbit should not have periapsis marker");
            Assert.AreEqual(0, result.ApValid, "Circular orbit should not have apoapsis marker");

            elements.Dispose();
            segmentTimes.Dispose();
            results.Dispose();
        }

        [Test]
        public void Performance_BatchVsSingle_OrbitalElements()
        {
            // Performance comparison: batch vs sequential processing
            int count = 100;
            Vector3d[] testPositions = new Vector3d[count];
            Vector3d[] testVelocities = new Vector3d[count];
            double mu = 1.266865319e17;

            // Generate test data
            for (int i = 0; i < count; i++)
            {
                testPositions[i] = new Vector3d(4e8 + i * 1e7, 0, 0);
                testVelocities[i] = new Vector3d(0, 15000 - i * 50, 0);
            }

            // Measure sequential processing time
            var sequentialStart = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < count; i++)
            {
                var _ = OrbitalElements.FromState(testPositions[i], testVelocities[i], mu);
            }
            sequentialStart.Stop();

            // Measure batch processing time
            NativeArray<double3> positions = new NativeArray<double3>(count, Allocator.TempJob);
            NativeArray<double3> velocities = new NativeArray<double3>(count, Allocator.TempJob);
            NativeArray<double> mus = new NativeArray<double>(count, Allocator.TempJob);
            NativeArray<OrbitalElementsData> results = new NativeArray<OrbitalElementsData>(count, Allocator.TempJob);

            for (int i = 0; i < count; i++)
            {
                positions[i] = new double3(testPositions[i].X, testPositions[i].Y, testPositions[i].Z);
                velocities[i] = new double3(testVelocities[i].X, testVelocities[i].Y, testVelocities[i].Z);
                mus[i] = mu;
            }

            var batchStart = System.Diagnostics.Stopwatch.StartNew();
            OrbitalElements.CalculateBatch(positions, velocities, mus, results).Complete();
            batchStart.Stop();

            positions.Dispose();
            velocities.Dispose();
            mus.Dispose();
            results.Dispose();

            // Log performance results
            double speedup = batchStart.ElapsedMilliseconds > 0 
                ? (double)sequentialStart.ElapsedMilliseconds / batchStart.ElapsedMilliseconds 
                : double.PositiveInfinity;
            UnityEngine.Debug.Log($"[Performance Test] Sequential: {sequentialStart.ElapsedMilliseconds}ms, Batch: {batchStart.ElapsedMilliseconds}ms, Speedup: {speedup:F2}x");

            // Note: For small datasets, both may be <1ms, so we just check that batch doesn't crash
            // Real performance gains are visible with larger datasets (1000+ items) or in actual gameplay
            if (sequentialStart.ElapsedMilliseconds > 0 && batchStart.ElapsedMilliseconds > 0)
            {
                // If both are measurable, batch should be faster or at least not slower
                Assert.LessOrEqual(batchStart.ElapsedMilliseconds, sequentialStart.ElapsedMilliseconds * 1.5, 
                    "Batch processing should not be significantly slower than sequential");
            }
            else
            {
                // If too fast to measure, just verify no exceptions occurred
                UnityEngine.Debug.Log("[Performance Test] Both methods completed too quickly to measure (<1ms). This is expected for small datasets.");
            }
        }
    }
}

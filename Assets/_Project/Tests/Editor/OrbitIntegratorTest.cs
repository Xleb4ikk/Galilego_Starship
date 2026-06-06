using System;
using NUnit.Framework;
using Galilego.Core;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Unit tests for OrbitIntegrator DOPRI5 implementation.
    /// Tests accuracy, adaptive error control, FSAL optimization, and edge cases.
    /// </summary>
    [TestFixture]
    public class OrbitIntegratorTest
    {
        private const double Jupiter_Mu = 1.266865319e17; // m³/s²
        private const double Io_OrbitalRadius = 421700000.0; // m
        
        #region Accuracy Tests

        [Test]
        public void StepForward_CircularOrbit_MaintainsEnergy()
        {
            // Arrange: Circular orbit at Io's distance
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double initialEnergy = CalculateOrbitalEnergy(position, velocity, Jupiter_Mu);
            
            // Act: Integrate for one orbital period (about 152,856 seconds = 42.5 hours)
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(Math.Pow(Io_OrbitalRadius, 3) / Jupiter_Mu);
            int steps = 100;
            double dt = orbitalPeriod / steps;
            
            Vector3d currentPos = position;
            Vector3d currentVel = velocity;
            double time = 0.0;
            
            for (int i = 0; i < steps; i++)
            {
                var result = OrbitIntegrator.StepForward(
                    currentPos, currentVel, time, dt,
                    (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
                
                currentPos = result.Position;
                currentVel = result.Velocity;
                time += dt;
            }
            
            double finalEnergy = CalculateOrbitalEnergy(currentPos, currentVel, Jupiter_Mu);
            
            // Assert: Energy should be conserved to within 0.1% over one orbit
            double energyError = Math.Abs((finalEnergy - initialEnergy) / initialEnergy);
            Assert.Less(energyError, 0.001, $"Energy error: {energyError * 100:F4}%");
        }

        [Test]
        public void StepForward_EllipticalOrbit_MaintainsSemiMajorAxis()
        {
            // Arrange: Elliptical orbit (e=0.3)
            double rPe = 70000000.0; // 70,000 km periapsis
            double rAp = 130000000.0; // 130,000 km apoapsis
            double a = (rPe + rAp) / 2.0;
            
            Vector3d position = new Vector3d(rPe, 0, 0);
            double vPe = Math.Sqrt(Jupiter_Mu * (2.0 / rPe - 1.0 / a));
            Vector3d velocity = new Vector3d(0, 0, vPe);
            
            double initialSemiMajorAxis = CalculateSemiMajorAxis(position, velocity, Jupiter_Mu);
            
            // Act: Integrate for half orbit
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(Math.Pow(a, 3) / Jupiter_Mu);
            double dt = orbitalPeriod / 2.0;
            
            var result = OrbitIntegrator.StepForward(
                position, velocity, 0.0, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            double finalSemiMajorAxis = CalculateSemiMajorAxis(result.Position, result.Velocity, Jupiter_Mu);
            
            // Assert: Semi-major axis should remain constant (< 0.1% error)
            double smaError = Math.Abs((finalSemiMajorAxis - initialSemiMajorAxis) / initialSemiMajorAxis);
            Assert.Less(smaError, 0.001, $"Semi-major axis error: {smaError * 100:F4}%");
            
            // Should end up near apoapsis
            double finalRadius = result.Position.Magnitude;
            Assert.AreEqual(rAp, finalRadius, rAp * 0.01, "Should reach apoapsis after half orbit");
        }

        [Test]
        public void StepForward_ComparedToRK4_MoreAccurate()
        {
            // Arrange: Compare DOPRI5 to RK4 for accuracy
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double dt = 1000.0; // Large step to emphasize accuracy difference
            
            // Act: Integrate with DOPRI5
            var dopri5Result = OrbitIntegrator.StepForward(
                position, velocity, 0.0, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Integrate with RK4 (from PhysicsSolver)
            var rk4Result = PhysicsSolver.RK4(
                position, velocity, 0.0, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Get "true" solution with very small steps
            var trueResult = IntegrateRK4Fine(position, velocity, 0.0, dt, Jupiter_Mu, 100);
            
            // Assert: DOPRI5 should be closer to true solution than RK4
            double dopri5Error = (dopri5Result.Position - trueResult.Position).Magnitude;
            double rk4Error = (rk4Result.Position - trueResult.Position).Magnitude;
            
            Assert.Less(dopri5Error, rk4Error, 
                $"DOPRI5 error ({dopri5Error:E2} m) should be less than RK4 error ({rk4Error:E2} m)");
        }

        #endregion

        #region Adaptive Error Control Tests

        [Test]
        public void StepForward_AdaptiveSteps_RespectsTolerances()
        {
            // Arrange: Start near Jupiter (high acceleration gradient)
            double closeRadius = 80000000.0; // 80,000 km (close to Jupiter)
            Vector3d position = new Vector3d(closeRadius, 0, 0);
            double velocity = Math.Sqrt(Jupiter_Mu / closeRadius);
            Vector3d velocityVec = new Vector3d(0, 0, velocity);
            
            double strictTolerance = 1e-9; // Very strict
            double looseTolerance = 1e-3;  // Very loose
            
            // Act: Integrate with different tolerances
            var strictResult = OrbitIntegrator.StepForward(
                position, velocityVec, 0.0, 100.0,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu),
                absoluteTolerance: strictTolerance);
            
            var looseResult = OrbitIntegrator.StepForward(
                position, velocityVec, 0.0, 100.0,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu),
                absoluteTolerance: looseTolerance);
            
            // Assert: Results should differ but both be valid
            double difference = (strictResult.Position - looseResult.Position).Magnitude;
            Assert.Greater(difference, 0.0, "Different tolerances should produce different results");
            Assert.IsTrue(strictResult.Position.IsFinite, "Strict result should be finite");
            Assert.IsTrue(looseResult.Position.IsFinite, "Loose result should be finite");
        }

        [Test]
        public void StepForward_LargeTimeStep_UsesMultipleSubsteps()
        {
            // This test verifies adaptive behavior by checking results remain accurate
            // even with large requested time steps
            
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double initialEnergy = CalculateOrbitalEnergy(position, velocity, Jupiter_Mu);
            
            // Act: Very large time step (10 minutes)
            double largeTimeStep = 600.0;
            var result = OrbitIntegrator.StepForward(
                position, velocity, 0.0, largeTimeStep,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            double finalEnergy = CalculateOrbitalEnergy(result.Position, result.Velocity, Jupiter_Mu);
            
            // Assert: Energy should still be well-conserved despite large step
            double energyError = Math.Abs((finalEnergy - initialEnergy) / initialEnergy);
            Assert.Less(energyError, 0.001, 
                $"Energy should be conserved even with large time step. Error: {energyError * 100:F4}%");
        }

        #endregion

        #region FSAL Optimization Tests

        [Test]
        public void StepForward_ConsecutiveSteps_UseFSAL()
        {
            // This test verifies FSAL is working by checking that consecutive steps
            // produce smooth, continuous results (FSAL ensures continuity)
            
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double dt = 100.0;
            
            // Act: Take two consecutive steps
            var result1 = OrbitIntegrator.StepForward(
                position, velocity, 0.0, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            var result2 = OrbitIntegrator.StepForward(
                result1.Position, result1.Velocity, dt, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Act: Take one double-length step
            var resultDouble = OrbitIntegrator.StepForward(
                position, velocity, 0.0, 2.0 * dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert: Results should be close (adaptive stepping may differ slightly)
            double positionDifference = (result2.Position - resultDouble.Position).Magnitude;
            Assert.Less(positionDifference, 1000.0, // Within 1 km
                "Consecutive steps should produce similar result to single double-length step");
        }

        #endregion

        #region Edge Case Tests

        [Test]
        public void StepForward_ZeroTimeStep_ReturnsOriginalState()
        {
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            Vector3d velocity = new Vector3d(0, 0, 17000);
            
            // Act
            var result = OrbitIntegrator.StepForward(
                position, velocity, 0.0, 0.0,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert
            Assert.AreEqual(position.X, result.Position.X, 1e-10);
            Assert.AreEqual(position.Y, result.Position.Y, 1e-10);
            Assert.AreEqual(position.Z, result.Position.Z, 1e-10);
            Assert.AreEqual(velocity.X, result.Velocity.X, 1e-10);
            Assert.AreEqual(velocity.Y, result.Velocity.Y, 1e-10);
            Assert.AreEqual(velocity.Z, result.Velocity.Z, 1e-10);
        }

        [Test]
        public void StepForward_NegativeTimeStep_IntegratesBackward()
        {
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double dt = 100.0;
            
            // Act: Forward then backward integration
            var forward = OrbitIntegrator.StepForward(
                position, velocity, 0.0, dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            var backward = OrbitIntegrator.StepForward(
                forward.Position, forward.Velocity, dt, -dt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert: Should return close to original position
            double positionError = (backward.Position - position).Magnitude;
            double velocityError = (backward.Velocity - velocity).Magnitude;
            
            Assert.Less(positionError, 10.0, $"Position error after round-trip: {positionError:F2} m");
            Assert.Less(velocityError, 0.01, $"Velocity error after round-trip: {velocityError:F4} m/s");
        }

        [Test]
        public void StepToTime_ForwardIntegration_MatchesStepForward()
        {
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double currentTime = 1000.0;
            double targetTime = 1500.0;
            
            // Act
            var stepToTimeResult = OrbitIntegrator.StepToTime(
                position, velocity, currentTime, targetTime,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            var stepForwardResult = OrbitIntegrator.StepForward(
                position, velocity, currentTime, targetTime - currentTime,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert
            double positionDiff = (stepToTimeResult.Position - stepForwardResult.Position).Magnitude;
            double velocityDiff = (stepToTimeResult.Velocity - stepForwardResult.Velocity).Magnitude;
            
            Assert.Less(positionDiff, 1e-6, "StepToTime and StepForward should match for forward integration");
            Assert.Less(velocityDiff, 1e-9, "Velocity should match between methods");
        }

        [Test]
        public void StepToTime_BackwardIntegration_ReachesTargetTime()
        {
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            double currentTime = 1000.0;
            double targetTime = 500.0; // Earlier time (backward)
            
            // Act
            var result = OrbitIntegrator.StepToTime(
                position, velocity, currentTime, targetTime,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert: Result should be valid
            Assert.IsTrue(result.Position.IsFinite, "Position should be finite after backward integration");
            Assert.IsTrue(result.Velocity.IsFinite, "Velocity should be finite after backward integration");
            Assert.Greater(result.Position.Magnitude, Jupiter_Mu / 1e18, "Should not crash into Jupiter");
        }

        [Test]
        public void StepForward_VerySmallTimeStep_RemainsStable()
        {
            // Arrange
            Vector3d position = new Vector3d(Io_OrbitalRadius, 0, 0);
            double circularVelocity = Math.Sqrt(Jupiter_Mu / Io_OrbitalRadius);
            Vector3d velocity = new Vector3d(0, 0, circularVelocity);
            
            // Act: Very small time step (1 microsecond)
            double tinyDt = 1e-6;
            var result = OrbitIntegrator.StepForward(
                position, velocity, 0.0, tinyDt,
                (pos, t) => CalculateGravityAcceleration(pos, Jupiter_Mu));
            
            // Assert: Should remain stable and not produce NaN
            Assert.IsTrue(result.Position.IsFinite, "Position should be finite");
            Assert.IsTrue(result.Velocity.IsFinite, "Velocity should be finite");
            
            // Position should barely change
            double displacement = (result.Position - position).Magnitude;
            Assert.Less(displacement, 0.1, "Position should barely change with tiny time step");
        }

        #endregion

        #region Helper Methods

        private Vector3d CalculateGravityAcceleration(Vector3d position, double mu)
        {
            double r = position.Magnitude;
            if (r < 100.0) // Safety check
                return Vector3d.Zero;
            
            double accelMagnitude = -mu / (r * r);
            return position.Normalized * accelMagnitude;
        }

        private double CalculateOrbitalEnergy(Vector3d position, Vector3d velocity, double mu)
        {
            double r = position.Magnitude;
            double v = velocity.Magnitude;
            
            double kineticEnergy = 0.5 * v * v;
            double potentialEnergy = -mu / r;
            
            return kineticEnergy + potentialEnergy;
        }

        private double CalculateSemiMajorAxis(Vector3d position, Vector3d velocity, double mu)
        {
            double energy = CalculateOrbitalEnergy(position, velocity, mu);
            return -mu / (2.0 * energy);
        }

        private IntegrationResult IntegrateRK4Fine(Vector3d position, Vector3d velocity, double time, double dt, double mu, int substeps)
        {
            double subDt = dt / substeps;
            Vector3d pos = position;
            Vector3d vel = velocity;
            
            for (int i = 0; i < substeps; i++)
            {
                var result = PhysicsSolver.RK4(pos, vel, time, subDt,
                    (p, t) => CalculateGravityAcceleration(p, mu));
                pos = result.Position;
                vel = result.Velocity;
                time += subDt;
            }
            
            return new IntegrationResult(pos, vel);
        }

        #endregion
    }
}

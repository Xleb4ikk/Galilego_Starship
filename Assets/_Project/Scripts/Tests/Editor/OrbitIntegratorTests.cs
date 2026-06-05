// ============================================================================
// ORBIT INTEGRATOR TESTS
// ============================================================================
// Unit tests for the unified DOPRI5 OrbitIntegrator
// Validates: Requirements 2.1, 2.2, 2.3, 2.4

using NUnit.Framework;
using System;
using Galilego.Core;

namespace Galilego.Tests.Editor
{
    [TestFixture]
    public class OrbitIntegratorTests
    {
        private const double TOLERANCE = 1e-3; // 1mm tolerance for position
        private const double VELOCITY_TOLERANCE = 1e-6; // 1mm/s tolerance for velocity
        
        // Jupiter's standard gravitational parameter
        private const double MU_JUPITER = 1.266865319e17;
        
        // ====================================================================
        // BASIC INTEGRATION TESTS
        // ====================================================================
        
        [Test]
        public void StepForward_ZeroTimeStep_ReturnsUnchangedState()
        {
            // Arrange
            Vector3d pos = new Vector3d(1e8, 0, 0);
            Vector3d vel = new Vector3d(0, 1e4, 0);
            double time = 0.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => Vector3d.Zero;
            
            // Act
            var result = OrbitIntegrator.StepForward(pos, vel, time, 0.0, accel);
            
            // Assert
            Assert.AreEqual(pos.X, result.Position.X, TOLERANCE);
            Assert.AreEqual(pos.Y, result.Position.Y, TOLERANCE);
            Assert.AreEqual(pos.Z, result.Position.Z, TOLERANCE);
            Assert.AreEqual(vel.X, result.Velocity.X, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Y, result.Velocity.Y, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Z, result.Velocity.Z, VELOCITY_TOLERANCE);
        }
        
        [Test]
        public void StepForward_NoAcceleration_LinearMotion()
        {
            // Arrange - free particle moving in straight line
            Vector3d pos = new Vector3d(0, 0, 0);
            Vector3d vel = new Vector3d(1000, 500, 200);
            double time = 0.0;
            double dt = 10.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => Vector3d.Zero;
            
            // Act
            var result = OrbitIntegrator.StepForward(pos, vel, time, dt, accel);
            
            // Assert - position should be pos + vel * dt
            Vector3d expectedPos = pos + vel * dt;
            Assert.AreEqual(expectedPos.X, result.Position.X, TOLERANCE);
            Assert.AreEqual(expectedPos.Y, result.Position.Y, TOLERANCE);
            Assert.AreEqual(expectedPos.Z, result.Position.Z, TOLERANCE);
            Assert.AreEqual(vel.X, result.Velocity.X, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Y, result.Velocity.Y, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Z, result.Velocity.Z, VELOCITY_TOLERANCE);
        }
        
        [Test]
        public void StepForward_ConstantAcceleration_ParabolicMotion()
        {
            // Arrange - constant acceleration (like gravity near surface)
            Vector3d pos = new Vector3d(0, 0, 0);
            Vector3d vel = new Vector3d(0, 100, 0);
            double time = 0.0;
            double dt = 10.0;
            Vector3d g = new Vector3d(0, -10, 0); // 10 m/s² downward
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => g;
            
            // Act
            var result = OrbitIntegrator.StepForward(pos, vel, time, dt, accel);
            
            // Assert - use kinematic equations: x = x0 + v0*t + 0.5*a*t²
            Vector3d expectedPos = pos + vel * dt + g * (0.5 * dt * dt);
            Vector3d expectedVel = vel + g * dt;
            
            Assert.AreEqual(expectedPos.X, result.Position.X, TOLERANCE);
            Assert.AreEqual(expectedPos.Y, result.Position.Y, TOLERANCE);
            Assert.AreEqual(expectedPos.Z, result.Position.Z, TOLERANCE);
            Assert.AreEqual(expectedVel.X, result.Velocity.X, VELOCITY_TOLERANCE);
            Assert.AreEqual(expectedVel.Y, result.Velocity.Y, VELOCITY_TOLERANCE);
            Assert.AreEqual(expectedVel.Z, result.Velocity.Z, VELOCITY_TOLERANCE);
        }
        
        // ====================================================================
        // CIRCULAR ORBIT TESTS
        // ====================================================================
        
        [Test]
        public void StepForward_CircularOrbit_MaintainsOrbitalRadius()
        {
            // Arrange - circular orbit around Jupiter at Io's altitude
            double radius = 4.22e8; // 422,000 km
            double circularVel = Math.Sqrt(MU_JUPITER / radius);
            
            Vector3d pos = new Vector3d(radius, 0, 0);
            Vector3d vel = new Vector3d(0, circularVel, 0);
            double time = 0.0;
            
            // Gravity acceleration toward origin
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                Vector3d direction = -p / r;
                double magnitude = MU_JUPITER / (r * r);
                return direction * magnitude;
            };
            
            // Act - integrate for 1/4 orbital period
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(radius * radius * radius / MU_JUPITER);
            double dt = orbitalPeriod / 4.0;
            
            var result = OrbitIntegrator.StepForward(pos, vel, time, dt, accel);
            
            // Assert - radius should be preserved (circular orbit)
            double finalRadius = result.Position.Magnitude;
            Assert.AreEqual(radius, finalRadius, radius * 1e-6, "Orbital radius should be preserved in circular orbit");
            
            // Velocity magnitude should also be preserved
            double finalSpeed = result.Velocity.Magnitude;
            Assert.AreEqual(circularVel, finalSpeed, circularVel * 1e-6, "Orbital speed should be preserved in circular orbit");
        }
        
        [Test]
        public void StepForward_CircularOrbit_CompletesFullOrbit()
        {
            // Arrange - circular orbit
            double radius = 4.22e8;
            double circularVel = Math.Sqrt(MU_JUPITER / radius);
            
            Vector3d pos = new Vector3d(radius, 0, 0);
            Vector3d vel = new Vector3d(0, circularVel, 0);
            double time = 0.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                return -p * (MU_JUPITER / (r * r * r));
            };
            
            // Act - integrate for full orbital period
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(radius * radius * radius / MU_JUPITER);
            var result = OrbitIntegrator.StepForward(pos, vel, time, orbitalPeriod, accel);
            
            // Assert - should return close to starting position
            double posError = (result.Position - pos).Magnitude;
            Assert.Less(posError, radius * 1e-5, "After one full orbit, should return to starting position");
        }
        
        // ====================================================================
        // ELLIPTICAL ORBIT TESTS
        // ====================================================================
        
        [Test]
        public void StepForward_EllipticalOrbit_ConservesEnergy()
        {
            // Arrange - elliptical orbit (e = 0.3)
            double periapsis = 4e8; // 400,000 km
            double eccentricity = 0.3;
            double semiMajorAxis = periapsis / (1.0 - eccentricity);
            
            Vector3d pos = new Vector3d(periapsis, 0, 0);
            double vPeriapsis = Math.Sqrt(MU_JUPITER * (1.0 + eccentricity) / (semiMajorAxis * (1.0 - eccentricity)));
            Vector3d vel = new Vector3d(0, vPeriapsis, 0);
            
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                return -p * (MU_JUPITER / (r * r * r));
            };
            
            // Calculate initial specific orbital energy
            double r0 = pos.Magnitude;
            double v0 = vel.Magnitude;
            double energy0 = 0.5 * v0 * v0 - MU_JUPITER / r0;
            
            // Act - integrate for half orbital period (to apoapsis)
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / MU_JUPITER);
            var result = OrbitIntegrator.StepForward(pos, vel, 0.0, orbitalPeriod / 2.0, accel);
            
            // Assert - energy should be conserved
            double rf = result.Position.Magnitude;
            double vf = result.Velocity.Magnitude;
            double energyF = 0.5 * vf * vf - MU_JUPITER / rf;
            
            double energyError = Math.Abs(energyF - energy0);
            double energyRelativeError = energyError / Math.Abs(energy0);
            Assert.Less(energyRelativeError, 1e-6, "Orbital energy should be conserved");
        }
        
        // ====================================================================
        // STEP TO TIME TESTS
        // ====================================================================
        
        [Test]
        public void StepToTime_ForwardIntegration_MatchesStepForward()
        {
            // Arrange
            Vector3d pos = new Vector3d(1e8, 0, 0);
            Vector3d vel = new Vector3d(0, 1e4, 0);
            double currentTime = 100.0;
            double targetTime = 200.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => new Vector3d(-1e-3, 0, 0);
            
            // Act
            var resultStepToTime = OrbitIntegrator.StepToTime(pos, vel, currentTime, targetTime, accel);
            var resultStepForward = OrbitIntegrator.StepForward(pos, vel, currentTime, targetTime - currentTime, accel);
            
            // Assert - both methods should give same result
            Assert.AreEqual(resultStepForward.Position.X, resultStepToTime.Position.X, TOLERANCE);
            Assert.AreEqual(resultStepForward.Position.Y, resultStepToTime.Position.Y, TOLERANCE);
            Assert.AreEqual(resultStepForward.Position.Z, resultStepToTime.Position.Z, TOLERANCE);
            Assert.AreEqual(resultStepForward.Velocity.X, resultStepToTime.Velocity.X, VELOCITY_TOLERANCE);
            Assert.AreEqual(resultStepForward.Velocity.Y, resultStepToTime.Velocity.Y, VELOCITY_TOLERANCE);
            Assert.AreEqual(resultStepForward.Velocity.Z, resultStepToTime.Velocity.Z, VELOCITY_TOLERANCE);
        }
        
        [Test]
        public void StepToTime_BackwardIntegration_ReturnsToOriginalState()
        {
            // Arrange
            Vector3d pos0 = new Vector3d(1e8, 0, 0);
            Vector3d vel0 = new Vector3d(0, 1e4, 0);
            double time0 = 0.0;
            double time1 = 100.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                return -p * (MU_JUPITER / (r * r * r));
            };
            
            // Act - integrate forward then backward
            var resultForward = OrbitIntegrator.StepToTime(pos0, vel0, time0, time1, accel);
            var resultBackward = OrbitIntegrator.StepToTime(resultForward.Position, resultForward.Velocity, time1, time0, accel);
            
            // Assert - should return close to original state
            double posError = (resultBackward.Position - pos0).Magnitude;
            double velError = (resultBackward.Velocity - vel0).Magnitude;
            
            Assert.Less(posError, 1e3, "Position error after forward-backward integration should be < 1 km");
            Assert.Less(velError, 1e-3, "Velocity error after forward-backward integration should be < 1 mm/s");
        }
        
        [Test]
        public void StepToTime_SameTime_ReturnsUnchangedState()
        {
            // Arrange
            Vector3d pos = new Vector3d(1e8, 0, 0);
            Vector3d vel = new Vector3d(0, 1e4, 0);
            double time = 100.0;
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => Vector3d.Zero;
            
            // Act
            var result = OrbitIntegrator.StepToTime(pos, vel, time, time, accel);
            
            // Assert
            Assert.AreEqual(pos.X, result.Position.X, TOLERANCE);
            Assert.AreEqual(pos.Y, result.Position.Y, TOLERANCE);
            Assert.AreEqual(pos.Z, result.Position.Z, TOLERANCE);
            Assert.AreEqual(vel.X, result.Velocity.X, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Y, result.Velocity.Y, VELOCITY_TOLERANCE);
            Assert.AreEqual(vel.Z, result.Velocity.Z, VELOCITY_TOLERANCE);
        }
        
        // ====================================================================
        // ADAPTIVE ERROR CONTROL TESTS
        // ====================================================================
        
        [Test]
        public void StepForward_LargeTimeStep_AutomaticallySubdivides()
        {
            // Arrange - very large time step that requires subdivision
            Vector3d pos = new Vector3d(4.22e8, 0, 0);
            double circularVel = Math.Sqrt(MU_JUPITER / 4.22e8);
            Vector3d vel = new Vector3d(0, circularVel, 0);
            
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                return -p * (MU_JUPITER / (r * r * r));
            };
            
            // Act - integrate for very large time step (1 day = 86400s)
            var result = OrbitIntegrator.StepForward(pos, vel, 0.0, 86400.0, accel);
            
            // Assert - should complete without throwing exception
            Assert.IsTrue(result.Position.IsFinite, "Result should be finite");
            Assert.IsTrue(result.Velocity.IsFinite, "Result velocity should be finite");
            
            // Verify orbit is still reasonable
            double finalRadius = result.Position.Magnitude;
            Assert.Greater(finalRadius, 1e8, "Final radius should be reasonable");
            Assert.Less(finalRadius, 1e9, "Final radius should be reasonable");
        }
        
        [Test]
        public void StepForward_TightTolerance_MoreAccurateThanLooseTolerance()
        {
            // Arrange
            double radius = 4.22e8;
            double circularVel = Math.Sqrt(MU_JUPITER / radius);
            Vector3d pos = new Vector3d(radius, 0, 0);
            Vector3d vel = new Vector3d(0, circularVel, 0);
            
            Func<Vector3d, double, Vector3d> accel = (p, t) =>
            {
                double r = p.Magnitude;
                if (r < 1e-6) return Vector3d.Zero;
                return -p * (MU_JUPITER / (r * r * r));
            };
            
            double orbitalPeriod = 2.0 * Math.PI * Math.Sqrt(radius * radius * radius / MU_JUPITER);
            
            // Act - integrate with different tolerances
            var resultTight = OrbitIntegrator.StepForward(pos, vel, 0.0, orbitalPeriod, accel, 
                absoluteTolerance: 1e-9, relativeTolerance: 1e-12);
            var resultLoose = OrbitIntegrator.StepForward(pos, vel, 0.0, orbitalPeriod, accel,
                absoluteTolerance: 1e-3, relativeTolerance: 1e-6);
            
            // Assert - tight tolerance should be closer to initial position
            double errorTight = (resultTight.Position - pos).Magnitude;
            double errorLoose = (resultLoose.Position - pos).Magnitude;
            
            Assert.Less(errorTight, errorLoose, "Tighter tolerance should produce more accurate result");
        }
        
        // ====================================================================
        // ARGUMENT VALIDATION TESTS
        // ====================================================================
        
        [Test]
        public void StepForward_NullAccelerationProvider_ThrowsArgumentNullException()
        {
            Vector3d pos = Vector3d.Zero;
            Vector3d vel = Vector3d.Zero;
            
            Assert.Throws<ArgumentNullException>(() =>
            {
                OrbitIntegrator.StepForward(pos, vel, 0.0, 1.0, null);
            });
        }
        
        [Test]
        public void StepForward_NegativeTimeStep_ThrowsArgumentException()
        {
            Vector3d pos = Vector3d.Zero;
            Vector3d vel = Vector3d.Zero;
            Func<Vector3d, double, Vector3d> accel = (p, t) => Vector3d.Zero;
            
            Assert.Throws<ArgumentException>(() =>
            {
                OrbitIntegrator.StepForward(pos, vel, 0.0, -1.0, accel);
            });
        }
        
        // ====================================================================
        // COMPARISON WITH ANALYTICAL SOLUTIONS
        // ====================================================================
        
        [Test]
        public void StepForward_HarmonicOscillator_MatchesAnalyticalSolution()
        {
            // Arrange - simple harmonic oscillator: x'' = -k*x
            double k = 1.0; // spring constant
            double omega = Math.Sqrt(k); // angular frequency
            
            Vector3d pos0 = new Vector3d(1.0, 0, 0); // initial displacement
            Vector3d vel0 = new Vector3d(0, 0, 0); // start from rest
            
            Func<Vector3d, double, Vector3d> accel = (p, t) => -p * k;
            
            double time = Math.PI / (2.0 * omega); // quarter period
            
            // Act
            var result = OrbitIntegrator.StepForward(pos0, vel0, 0.0, time, accel);
            
            // Assert - analytical solution: x(t) = cos(omega*t), v(t) = -omega*sin(omega*t)
            double expectedX = Math.Cos(omega * time);
            double expectedVx = -omega * Math.Sin(omega * time);
            
            Assert.AreEqual(expectedX, result.Position.X, 1e-6, "Position should match analytical solution");
            Assert.AreEqual(expectedVx, result.Velocity.X, 1e-6, "Velocity should match analytical solution");
        }
    }
}

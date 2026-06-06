using System;

namespace Galilego.Core
{
    /// <summary>
    /// Unified DOPRI5 (Dormand-Prince 5(4)) orbit integrator with adaptive error control.
    /// This implementation EXACTLY matches FullTrajectoryJob.DoPri5Step for consistency.
    /// </summary>
    public static class OrbitIntegrator
    {
        // Configuration constants
        public const double DefaultAbsoluteTolerance = 1e-6;  // 1 mm position error
        public const double DefaultRelativeTolerance = 1e-9;  // 1 ppb relative error
        public const double DefaultMaxStepSize = 600.0;       // 10 minutes
        public const double DefaultMinStepSize = 1e-6;        // 1 microsecond
        
        // DOPRI5 Butcher tableau coefficients - MUST match FullTrajectoryJob exactly
        // See DOPRI5Coefficients.cs for single source of truth
        private const double a21 = DOPRI5Coefficients.a21;
        private const double a31 = DOPRI5Coefficients.a31;
        private const double a32 = DOPRI5Coefficients.a32;
        private const double a41 = DOPRI5Coefficients.a41;
        private const double a42 = DOPRI5Coefficients.a42;
        private const double a43 = DOPRI5Coefficients.a43;
        private const double a51 = DOPRI5Coefficients.a51;
        private const double a52 = DOPRI5Coefficients.a52;
        private const double a53 = DOPRI5Coefficients.a53;
        private const double a54 = DOPRI5Coefficients.a54;
        private const double a61 = DOPRI5Coefficients.a61;
        private const double a62 = DOPRI5Coefficients.a62;
        private const double a63 = DOPRI5Coefficients.a63;
        private const double a64 = DOPRI5Coefficients.a64;
        private const double a65 = DOPRI5Coefficients.a65;
        private const double a71 = DOPRI5Coefficients.a71;
        private const double a73 = DOPRI5Coefficients.a73;
        private const double a74 = DOPRI5Coefficients.a74;
        private const double a75 = DOPRI5Coefficients.a75;
        private const double a76 = DOPRI5Coefficients.a76;
        
        private const double b1 = DOPRI5Coefficients.b1;
        private const double b3 = DOPRI5Coefficients.b3;
        private const double b4 = DOPRI5Coefficients.b4;
        private const double b5 = DOPRI5Coefficients.b5;
        private const double b6 = DOPRI5Coefficients.b6;
        
        private const double bStar1 = DOPRI5Coefficients.bStar1;
        private const double bStar3 = DOPRI5Coefficients.bStar3;
        private const double bStar4 = DOPRI5Coefficients.bStar4;
        private const double bStar5 = DOPRI5Coefficients.bStar5;
        private const double bStar6 = DOPRI5Coefficients.bStar6;
        private const double bStar7 = DOPRI5Coefficients.bStar7;

        /// <summary>
        /// Integrate forward by a target time step using adaptive DOPRI5.
        /// This method EXACTLY matches the logic in FullTrajectoryJob.TryAdvanceStep.
        /// </summary>
        public static IntegrationResult StepForward(
            Vector3d position,
            Vector3d velocity,
            double currentTime,
            double targetDt,
            Func<Vector3d, double, Vector3d> accelerationProvider,
            double absoluteTolerance = DefaultAbsoluteTolerance,
            double relativeTolerance = DefaultRelativeTolerance)
        {
            if (accelerationProvider == null)
                throw new ArgumentNullException(nameof(accelerationProvider));
            
            if (targetDt == 0.0)
                return new IntegrationResult(position, velocity);
            
            // Initialize FSAL (First Same As Last)
            Vector3d fsalAccel = accelerationProvider(position, currentTime);
            bool fsalValid = true;
            
            Vector3d currentPos = position;
            Vector3d currentVel = velocity;
            double time = currentTime;
            double remainingDt = Math.Abs(targetDt);
            double direction = Math.Sign(targetDt);
            double dt = Math.Min(remainingDt, DefaultMaxStepSize);
            
            while (remainingDt > DefaultMinStepSize)
            {
                // Clamp step to remaining time
                dt = Math.Min(dt, remainingDt);
                
                // If FSAL invalid (first step or after reject), compute k1
                if (!fsalValid)
                {
                    fsalAccel = accelerationProvider(currentPos, time);
                    fsalValid = true;
                }
                
                // Perform DOPRI5 step (use absolute dt, direction handled externally)
                var result = DoPri5Step(
                    currentPos, currentVel, time,
                    dt,  // Always positive, direction handled in time update
                    accelerationProvider,
                    fsalAccel,
                    out Vector3d newFsalAccel,
                    out double errorPos,
                    out double errorVel);
                
                // Scaled error (EXACTLY matches FullTrajectoryJob lines 466-468)
                double scalePos = absoluteTolerance + relativeTolerance * Math.Max(currentPos.Magnitude, result.Position.Magnitude);
                double scaleVel = absoluteTolerance + relativeTolerance * Math.Max(currentVel.Magnitude, result.Velocity.Magnitude);
                double normalizedError = Math.Max(errorPos / scalePos, errorVel / scaleVel);
                
                if (normalizedError <= 1.0)
                {
                    // ACCEPT step (matches FullTrajectoryJob lines 470-485)
                    currentPos = result.Position;
                    currentVel = result.Velocity;
                    time += dt * direction;
                    remainingDt -= dt;
                    
                    // FSAL: k7 of accepted step = k1 of next step
                    fsalAccel = newFsalAccel;
                    fsalValid = true;
                    
                    // Step size control (matches FullTrajectoryJob lines 481-484)
                    double stepScale = Math.Min(Math.Max(
                        0.9 * Math.Pow(1.0 / Math.Max(normalizedError, 1e-10), 0.2),
                        0.2), 5.0);
                    dt = Math.Min(Math.Max(dt * stepScale, DefaultMinStepSize), DefaultMaxStepSize);
                }
                else
                {
                    // REJECT step (matches FullTrajectoryJob lines 487-509)
                    double rejectScale = Math.Min(Math.Max(
                        0.9 * Math.Pow(1.0 / Math.Max(normalizedError, 1e-10), 0.2),
                        0.1), 0.5);
                    dt = Math.Max(dt * rejectScale, DefaultMinStepSize);
                    
                    // Force-accept if at minimum step size
                    if (dt <= DefaultMinStepSize)
                    {
                        currentPos = result.Position;
                        currentVel = result.Velocity;
                        time += dt * direction;
                        remainingDt -= dt;
                        fsalAccel = newFsalAccel;
                        fsalValid = true;
                    }
                    else
                    {
                        // REJECT: FSAL invalid for next attempt
                        fsalValid = false;
                    }
                }
            }
            
            return new IntegrationResult(currentPos, currentVel);
        }

        /// <summary>
        /// Integrate to an exact target time.
        /// </summary>
        public static IntegrationResult StepToTime(
            Vector3d position,
            Vector3d velocity,
            double currentTime,
            double targetTime,
            Func<Vector3d, double, Vector3d> accelerationProvider,
            double absoluteTolerance = DefaultAbsoluteTolerance,
            double relativeTolerance = DefaultRelativeTolerance)
        {
            double dt = targetTime - currentTime;
            return StepForward(position, velocity, currentTime, dt, accelerationProvider, absoluteTolerance, relativeTolerance);
        }

        /// <summary>
        /// Perform a single DOPRI5 integration step.
        /// This implementation EXACTLY matches FullTrajectoryJob.DoPri5Step (lines 321-410).
        /// </summary>
        private static DoPri5Result DoPri5Step(
            Vector3d pos,
            Vector3d vel,
            double time,
            double dt,
            Func<Vector3d, double, Vector3d> accelerationProvider,
            Vector3d fsalAccel,
            out Vector3d lastAccel,
            out double errPos,
            out double errVel)
        {
            // Stage 1 (FSAL)
            Vector3d k1v = fsalAccel;
            Vector3d k1p = vel;
            
            // DEBUG: Log first step
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[OI_DEBUG] DoPri5Step called: pos=({pos.X:F3}, {pos.Y:F3}, {pos.Z:F3}), vel=({vel.X:F3}, {vel.Y:F3}, {vel.Z:F3}), dt={dt:F3}");
                UnityEngine.Debug.Log($"[OI_DEBUG] k1p=({k1p.X:F3}, {k1p.Y:F3}, {k1p.Z:F3}), k1v=({k1v.X:F3}, {k1v.Y:F3}, {k1v.Z:F3})");
            }
            
            // Stage 2
            Vector3d pos2 = pos + k1p * (dt * a21);
            Vector3d vel2 = vel + k1v * (dt * a21);
            Vector3d a2 = accelerationProvider(pos2, time + dt * (1.0 / 5.0));
            
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[OI_DEBUG] Stage 2: pos2=({pos2.X:F3}, {pos2.Y:F3}, {pos2.Z:F3}), vel2=({vel2.X:F3}, {vel2.Y:F3}, {vel2.Z:F3})");
            }
            
            // Stage 3
            Vector3d pos3 = pos + (k1p * (dt * a31) + vel2 * (dt * a32));
            Vector3d vel3 = vel + (k1v * (dt * a31) + a2 * (dt * a32));
            Vector3d a3 = accelerationProvider(pos3, time + dt * (3.0 / 10.0));
            
            // Stage 4
            Vector3d pos4 = pos + (k1p * (dt * a41) + vel2 * (dt * a42) + vel3 * (dt * a43));
            Vector3d vel4 = vel + (k1v * (dt * a41) + a2 * (dt * a42) + a3 * (dt * a43));
            Vector3d a4 = accelerationProvider(pos4, time + dt * (4.0 / 5.0));
            
            // Stage 5
            Vector3d pos5 = pos + (k1p * (dt * a51) + vel2 * (dt * a52) + vel3 * (dt * a53) + vel4 * (dt * a54));
            Vector3d vel5 = vel + (k1v * (dt * a51) + a2 * (dt * a52) + a3 * (dt * a53) + a4 * (dt * a54));
            Vector3d a5 = accelerationProvider(pos5, time + dt * (8.0 / 9.0));
            
            // Stage 6
            Vector3d pos6 = pos + (k1p * (dt * a61) + vel2 * (dt * a62) + vel3 * (dt * a63) + vel4 * (dt * a64) + vel5 * (dt * a65));
            Vector3d vel6 = vel + (k1v * (dt * a61) + a2 * (dt * a62) + a3 * (dt * a63) + a4 * (dt * a64) + a5 * (dt * a65));
            Vector3d a6 = accelerationProvider(pos6, time + dt);
            
            // Stage 7
            Vector3d pos7 = pos + (k1p * (dt * a71) + vel3 * (dt * a73) + vel4 * (dt * a74) + vel5 * (dt * a75) + vel6 * (dt * a76));
            Vector3d vel7 = vel + (k1v * (dt * a71) + a3 * (dt * a73) + a4 * (dt * a74) + a5 * (dt * a75) + a6 * (dt * a76));
            Vector3d a7 = accelerationProvider(pos7, time + dt);
            
            // 5th order solution
            Vector3d fifthPos = pos + (k1p * (dt * b1) + vel3 * (dt * b3) + vel4 * (dt * b4) + vel5 * (dt * b5) + vel6 * (dt * b6));
            Vector3d fifthVel = vel + (k1v * (dt * b1) + a3 * (dt * b3) + a4 * (dt * b4) + a5 * (dt * b5) + a6 * (dt * b6));
            
            if (time == 0.0)
            {
                UnityEngine.Debug.Log($"[OI_DEBUG] Result: fifthPos=({fifthPos.X:F3}, {fifthPos.Y:F3}, {fifthPos.Z:F3}), fifthVel=({fifthVel.X:F3}, {fifthVel.Y:F3}, {fifthVel.Z:F3})");
            }
            
            // 4th order solution (for error estimation)
            Vector3d fourthPos = pos + (k1p * (dt * bStar1) + vel3 * (dt * bStar3) + vel4 * (dt * bStar4) + vel5 * (dt * bStar5) + vel6 * (dt * bStar6) + vel7 * (dt * bStar7));
            Vector3d fourthVel = vel + (k1v * (dt * bStar1) + a3 * (dt * bStar3) + a4 * (dt * bStar4) + a5 * (dt * bStar5) + a6 * (dt * bStar6) + a7 * (dt * bStar7));
            
            // Error = difference between 5th and 4th order
            errPos = (fifthPos - fourthPos).Magnitude;
            errVel = (fifthVel - fourthVel).Magnitude;
            
            // FSAL: k7 of this step = k1 of next step (if accepted)
            lastAccel = a7;
            
            return new DoPri5Result(fifthPos, fifthVel);
        }

        /// <summary>
        /// Internal result structure for DOPRI5 step.
        /// </summary>
        private struct DoPri5Result
        {
            public Vector3d Position;
            public Vector3d Velocity;

            public DoPri5Result(Vector3d position, Vector3d velocity)
            {
                Position = position;
                Velocity = velocity;
            }
        }
    }
}

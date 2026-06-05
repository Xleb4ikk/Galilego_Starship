namespace Galilego.Core
{
    /// <summary>
    /// Shared DOPRI5 (Dormand-Prince 5(4)) Butcher tableau coefficients.
    /// 
    /// This class provides a single source of truth for DOPRI5 coefficients used by:
    /// - OrbitIntegrator.StepForward (managed code for runtime simulation)
    /// - FullTrajectoryJob.DoPri5Step (Burst-compiled code for trajectory prediction)
    /// 
    /// Any changes to these coefficients must be synchronized across both implementations
    /// to maintain trajectory consistency.
    /// 
    /// Reference: Dormand, J. R., and Prince, P. J. (1980). "A family of embedded Runge-Kutta formulae."
    /// Journal of Computational and Applied Mathematics, 6(1), 19-26.
    /// </summary>
    public static class DOPRI5Coefficients
    {
        // ═══════════════════════════════════════════════════════════════════
        // Time Points (c coefficients)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>c2 = 1/5 - Time point for stage 2</summary>
        public const double c2 = 1.0 / 5.0;
        
        /// <summary>c3 = 3/10 - Time point for stage 3</summary>
        public const double c3 = 3.0 / 10.0;
        
        /// <summary>c4 = 4/5 - Time point for stage 4</summary>
        public const double c4 = 4.0 / 5.0;
        
        /// <summary>c5 = 8/9 - Time point for stage 5</summary>
        public const double c5 = 8.0 / 9.0;
        
        /// <summary>c6 = 1 - Time point for stage 6</summary>
        public const double c6 = 1.0;
        
        /// <summary>c7 = 1 - Time point for stage 7 (FSAL)</summary>
        public const double c7 = 1.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 2 Coefficients (a2j)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a21 = 1/5</summary>
        public const double a21 = 1.0 / 5.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 3 Coefficients (a3j)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a31 = 3/40</summary>
        public const double a31 = 3.0 / 40.0;
        
        /// <summary>a32 = 9/40</summary>
        public const double a32 = 9.0 / 40.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 4 Coefficients (a4j)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a41 = 44/45</summary>
        public const double a41 = 44.0 / 45.0;
        
        /// <summary>a42 = -56/15</summary>
        public const double a42 = -56.0 / 15.0;
        
        /// <summary>a43 = 32/9</summary>
        public const double a43 = 32.0 / 9.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 5 Coefficients (a5j)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a51 = 19372/6561</summary>
        public const double a51 = 19372.0 / 6561.0;
        
        /// <summary>a52 = -25360/2187</summary>
        public const double a52 = -25360.0 / 2187.0;
        
        /// <summary>a53 = 64448/6561</summary>
        public const double a53 = 64448.0 / 6561.0;
        
        /// <summary>a54 = -212/729</summary>
        public const double a54 = -212.0 / 729.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 6 Coefficients (a6j)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a61 = 9017/3168</summary>
        public const double a61 = 9017.0 / 3168.0;
        
        /// <summary>a62 = -355/33</summary>
        public const double a62 = -355.0 / 33.0;
        
        /// <summary>a63 = 46732/5247</summary>
        public const double a63 = 46732.0 / 5247.0;
        
        /// <summary>a64 = 49/176</summary>
        public const double a64 = 49.0 / 176.0;
        
        /// <summary>a65 = -5103/18656</summary>
        public const double a65 = -5103.0 / 18656.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // Stage 7 Coefficients (a7j) - Same as 5th order weights
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>a71 = 35/384 (same as b1)</summary>
        public const double a71 = 35.0 / 384.0;
        
        // a72 = 0 (omitted)
        
        /// <summary>a73 = 500/1113 (same as b3)</summary>
        public const double a73 = 500.0 / 1113.0;
        
        /// <summary>a74 = 125/192 (same as b4)</summary>
        public const double a74 = 125.0 / 192.0;
        
        /// <summary>a75 = -2187/6784 (same as b5)</summary>
        public const double a75 = -2187.0 / 6784.0;
        
        /// <summary>a76 = 11/84 (same as b6)</summary>
        public const double a76 = 11.0 / 84.0;
        
        // ═══════════════════════════════════════════════════════════════════
        // 5th Order Solution Weights (b coefficients)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>b1 = 35/384</summary>
        public const double b1 = 35.0 / 384.0;
        
        // b2 = 0 (omitted)
        
        /// <summary>b3 = 500/1113</summary>
        public const double b3 = 500.0 / 1113.0;
        
        /// <summary>b4 = 125/192</summary>
        public const double b4 = 125.0 / 192.0;
        
        /// <summary>b5 = -2187/6784</summary>
        public const double b5 = -2187.0 / 6784.0;
        
        /// <summary>b6 = 11/84</summary>
        public const double b6 = 11.0 / 84.0;
        
        // b7 = 0 (omitted in 5th order, used in 4th order)
        
        // ═══════════════════════════════════════════════════════════════════
        // 4th Order Solution Weights (b* coefficients for error estimation)
        // ═══════════════════════════════════════════════════════════════════
        
        /// <summary>bStar1 = 5179/57600</summary>
        public const double bStar1 = 5179.0 / 57600.0;
        
        // bStar2 = 0 (omitted)
        
        /// <summary>bStar3 = 7571/16695</summary>
        public const double bStar3 = 7571.0 / 16695.0;
        
        /// <summary>bStar4 = 393/640</summary>
        public const double bStar4 = 393.0 / 640.0;
        
        /// <summary>bStar5 = -92097/339200</summary>
        public const double bStar5 = -92097.0 / 339200.0;
        
        /// <summary>bStar6 = 187/2100</summary>
        public const double bStar6 = 187.0 / 2100.0;
        
        /// <summary>bStar7 = 1/40</summary>
        public const double bStar7 = 1.0 / 40.0;
    }
}

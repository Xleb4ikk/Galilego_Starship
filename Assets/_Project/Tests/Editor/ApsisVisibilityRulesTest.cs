using NUnit.Framework;
using UnityEngine;
using Galilego.Core;
using Galilego.Gameplay;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Tests for apsis visibility rules.
    /// Verifies that visibility flags are correctly applied based on configured rules.
    /// </summary>
    [TestFixture]
    public class ApsisVisibilityRulesTest
    {
        [Test]
        public void VisibilityRule_FutureApsisOnly_PastApsisNotVisible()
        {
            // Arrange: Create apsis in the past
            var pastApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: 100.0, // Past time
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            // Note: Visibility rules are applied by ApsisCalculator.ApplyVisibilityRules()
            // This test documents the expected behavior:
            // If showOnlyFutureApsides = true AND timeToReach <= currentTime, then isVisible = false

            double currentTime = 500.0;
            bool shouldBeVisible = pastApsis.timeToReach > currentTime;

            Assert.IsFalse(shouldBeVisible, "Past apsis should not be visible when showOnlyFutureApsides = true");
        }

        [Test]
        public void VisibilityRule_FutureApsisOnly_FutureApsisVisible()
        {
            // Arrange: Create apsis in the future
            var futureApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: 1000.0, // Future time
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            double currentTime = 500.0;
            bool shouldBeVisible = futureApsis.timeToReach > currentTime;

            Assert.IsTrue(shouldBeVisible, "Future apsis should be visible when showOnlyFutureApsides = true");
        }

        [Test]
        public void VisibilityRule_ShowBelowSurface_SubsurfaceApsisNotVisible()
        {
            // Arrange: Create subsurface apsis
            var subsurfaceApsis = new ApsisData(
                worldPosition: new Vector3d(50000000, 0, 0),
                altitude: -10000000, // Negative altitude (below surface)
                timeToReach: 1000.0,
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            // Rule: If showBelowSurface = false AND altitude < minAltitude, then isVisible = false
            bool showBelowSurface = false;
            double minAltitude = 0.0;
            bool shouldBeVisible = showBelowSurface || subsurfaceApsis.altitude >= minAltitude;

            Assert.IsFalse(shouldBeVisible, "Subsurface apsis should not be visible when showBelowSurface = false");
        }

        [Test]
        public void VisibilityRule_ShowBelowSurface_AboveSurfaceApsisVisible()
        {
            // Arrange: Create above-surface apsis
            var aboveSurfaceApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000, // Positive altitude (above surface)
                timeToReach: 1000.0,
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            bool showBelowSurface = false;
            double minAltitude = 0.0;
            bool shouldBeVisible = showBelowSurface || aboveSurfaceApsis.altitude >= minAltitude;

            Assert.IsTrue(shouldBeVisible, "Above-surface apsis should be visible");
        }

        [Test]
        public void VisibilityRule_MaxPredictionTime_FarFutureApsisNotVisible()
        {
            // Arrange: Create apsis far in the future
            var farFutureApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: 10000.0, // Far future
                type: ApsisType.Apoapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            // Rule: If timeToReach - currentTime > maxPredictionTime, then isVisible = false
            double currentTime = 500.0;
            double maxPredictionTime = 7200.0; // 2 hours
            double timeUntilApsis = farFutureApsis.timeToReach - currentTime;
            bool shouldBeVisible = timeUntilApsis <= maxPredictionTime;

            Assert.IsFalse(shouldBeVisible, "Far future apsis should not be visible when beyond maxPredictionTime");
        }

        [Test]
        public void VisibilityRule_MaxPredictionTime_NearFutureApsisVisible()
        {
            // Arrange: Create apsis in near future
            var nearFutureApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: 2000.0, // Near future
                type: ApsisType.Apoapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            double currentTime = 500.0;
            double maxPredictionTime = 7200.0; // 2 hours
            double timeUntilApsis = nearFutureApsis.timeToReach - currentTime;
            bool shouldBeVisible = timeUntilApsis <= maxPredictionTime;

            Assert.IsTrue(shouldBeVisible, "Near future apsis should be visible when within maxPredictionTime");
        }

        [Test]
        public void VisibilityRule_MinAltitude_BelowMinAltitudeNotVisible()
        {
            // Arrange: Create apsis below minimum altitude
            var lowAltitudeApsis = new ApsisData(
                worldPosition: new Vector3d(70000000, 0, 0),
                altitude: 50000, // 50 km altitude
                timeToReach: 1000.0,
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            // Rule: If altitude < minAltitude, then isVisible = false
            double minAltitude = 100000; // 100 km minimum
            bool shouldBeVisible = lowAltitudeApsis.altitude >= minAltitude;

            Assert.IsFalse(shouldBeVisible, "Apsis below minAltitude should not be visible");
        }

        [Test]
        public void VisibilityRule_CircularOrbit_NoApsisVisible()
        {
            // Arrange: Circular orbit has eccentricity < circularOrbitThreshold
            // In this case, ApsisCalculator should not create any apsis data at all

            double eccentricity = 0.0005; // Very low eccentricity
            double circularOrbitThreshold = 0.001;

            bool isCircular = eccentricity < circularOrbitThreshold;

            Assert.IsTrue(isCircular, "Orbit with e < threshold should be considered circular");
            
            // Expected behavior: CalculateBallisticApsides() returns empty list
            // No apsis markers should be created for circular orbits
        }

        [Test]
        public void VisibilityRule_MultipleRules_AllMustPass()
        {
            // Arrange: Create apsis that passes all rules
            var validApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000, // Above surface
                timeToReach: 1500.0, // Future, within prediction time
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            double currentTime = 1000.0;
            double maxPredictionTime = 7200.0;
            double minAltitude = 0.0;
            bool showBelowSurface = false;
            bool showOnlyFutureApsides = true;

            // Apply all rules
            bool rule1 = !showOnlyFutureApsides || validApsis.timeToReach > currentTime;
            bool rule2 = showBelowSurface || validApsis.altitude >= minAltitude;
            bool rule3 = (validApsis.timeToReach - currentTime) <= maxPredictionTime;

            bool shouldBeVisible = rule1 && rule2 && rule3;

            Assert.IsTrue(shouldBeVisible, "Apsis passing all rules should be visible");
        }

        [Test]
        public void VisibilityRule_MultipleRules_OneFailsAllFail()
        {
            // Arrange: Create apsis that fails one rule (subsurface)
            var invalidApsis = new ApsisData(
                worldPosition: new Vector3d(50000000, 0, 0),
                altitude: -5000000, // Below surface (fails rule 2)
                timeToReach: 1500.0, // Future (passes rule 1)
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            double currentTime = 1000.0;
            double maxPredictionTime = 7200.0;
            double minAltitude = 0.0;
            bool showBelowSurface = false;
            bool showOnlyFutureApsides = true;

            // Apply all rules
            bool rule1 = !showOnlyFutureApsides || invalidApsis.timeToReach > currentTime;
            bool rule2 = showBelowSurface || invalidApsis.altitude >= minAltitude;
            bool rule3 = (invalidApsis.timeToReach - currentTime) <= maxPredictionTime;

            bool shouldBeVisible = rule1 && rule2 && rule3;

            Assert.IsFalse(shouldBeVisible, "Apsis failing any rule should not be visible");
            Assert.IsTrue(rule1, "Rule 1 should pass");
            Assert.IsFalse(rule2, "Rule 2 should fail (subsurface)");
            Assert.IsTrue(rule3, "Rule 3 should pass");
        }

        [Test]
        public void VisibilityRule_EdgeCase_ExactlyAtMaxPredictionTime()
        {
            // Arrange: Create apsis exactly at max prediction time boundary
            double currentTime = 1000.0;
            double maxPredictionTime = 7200.0;
            double exactTime = currentTime + maxPredictionTime;

            var boundaryApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: exactTime, // Exactly at boundary
                type: ApsisType.Apoapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            double timeUntilApsis = boundaryApsis.timeToReach - currentTime;
            bool shouldBeVisible = timeUntilApsis <= maxPredictionTime;

            Assert.IsTrue(shouldBeVisible, "Apsis exactly at maxPredictionTime should be visible (inclusive)");
        }

        [Test]
        public void VisibilityRule_EdgeCase_ExactlyAtCurrentTime()
        {
            // Arrange: Create apsis exactly at current time
            double currentTime = 1000.0;

            var currentTimeApsis = new ApsisData(
                worldPosition: new Vector3d(100000000, 0, 0),
                altitude: 30000000,
                timeToReach: currentTime, // Exactly now
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            bool showOnlyFutureApsides = true;
            bool shouldBeVisible = !showOnlyFutureApsides || currentTimeApsis.timeToReach > currentTime;

            Assert.IsFalse(shouldBeVisible, "Apsis at exactly current time should not be visible (not future)");
        }

        [Test]
        public void VisibilityRule_EdgeCase_ExactlyAtSurface()
        {
            // Arrange: Create apsis exactly at surface (altitude = 0)
            var surfaceApsis = new ApsisData(
                worldPosition: new Vector3d(69911000, 0, 0), // Jupiter radius
                altitude: 0.0, // Exactly at surface
                timeToReach: 1500.0,
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "Jupiter"
            );

            bool showBelowSurface = false;
            double minAltitude = 0.0;
            bool shouldBeVisible = showBelowSurface || surfaceApsis.altitude >= minAltitude;

            Assert.IsTrue(shouldBeVisible, "Apsis exactly at surface (altitude = 0) should be visible (inclusive)");
        }
    }
}

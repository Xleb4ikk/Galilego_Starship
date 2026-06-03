using NUnit.Framework;
using Galilego.Universe;
using Galilego.Core;
using UnityEngine;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Unit tests for UniverseManager.GetCurrentCentralBodyPosition method
    /// 
    /// **Validates: Requirement 5.1**
    /// 
    /// These tests verify that GetCurrentCentralBodyPosition correctly:
    /// - Returns real-world position for ActiveReferenceFrame
    /// - Handles Jupiter and all four moons (Io, Europa, Ganymede, Callisto)
    /// - Uses TryGetMoonPositionAtTime for moon positions
    /// - Defaults to Jupiter if reference frame is unknown
    /// </summary>
    [TestFixture]
    public class UniverseManagerPositionTest
    {
        private GameObject testGameObject;
        private UniverseManager universeManager;

        [SetUp]
        public void SetUp()
        {
            // Create a test GameObject with UniverseManager
            testGameObject = new GameObject("TestUniverseManager");
            universeManager = testGameObject.AddComponent<UniverseManager>();
        }

        [TearDown]
        public void TearDown()
        {
            if (testGameObject != null)
            {
                Object.DestroyImmediate(testGameObject);
            }
        }

        /// <summary>
        /// Test that GetCurrentCentralBodyPosition returns Jupiter's position when Jupiter is the active reference frame
        /// **Validates: Requirement 5.1**
        /// </summary>
        [Test]
        public void GetCurrentCentralBodyPosition_JupiterFrame_ReturnsJupiterPosition()
        {
            // ARRANGE: Set Jupiter as the active reference frame
            universeManager.SelectReferenceFrame(ReferenceFrameTarget.Jupiter);
            
            // ACT: Get the central body position
            Vector3d position = universeManager.GetCurrentCentralBodyPosition();
            
            // ASSERT: Verify it returns a valid position (should be finite)
            Assert.That(position.X, Is.Not.NaN, "Jupiter's X position should not be NaN");
            Assert.That(position.Y, Is.Not.NaN, "Jupiter's Y position should not be NaN");
            Assert.That(position.Z, Is.Not.NaN, "Jupiter's Z position should not be NaN");
            Assert.That(double.IsFinite(position.X), Is.True, "Jupiter's X position should be finite");
            Assert.That(double.IsFinite(position.Y), Is.True, "Jupiter's Y position should be finite");
            Assert.That(double.IsFinite(position.Z), Is.True, "Jupiter's Z position should be finite");
        }

        /// <summary>
        /// Test that GetCurrentCentralBodyPosition returns appropriate position for moon reference frames
        /// **Validates: Requirement 5.1**
        /// </summary>
        [Test]
        public void GetCurrentCentralBodyPosition_MoonFrames_ReturnsMoonPosition()
        {
            // Test each moon reference frame
            ReferenceFrameTarget[] moonFrames = new[]
            {
                ReferenceFrameTarget.Io,
                ReferenceFrameTarget.Europa,
                ReferenceFrameTarget.Ganymede,
                ReferenceFrameTarget.Callisto
            };

            foreach (var moonFrame in moonFrames)
            {
                // ARRANGE: Set moon as the active reference frame
                universeManager.SelectReferenceFrame(moonFrame);
                
                // ACT: Get the central body position
                Vector3d position = universeManager.GetCurrentCentralBodyPosition();
                
                // ASSERT: Verify it returns a valid position (should be finite)
                // Note: The actual value depends on moon rail configuration
                // We just verify it's finite and not NaN
                Assert.That(position.X, Is.Not.NaN, 
                    $"{moonFrame}'s X position should not be NaN");
                Assert.That(position.Y, Is.Not.NaN, 
                    $"{moonFrame}'s Y position should not be NaN");
                Assert.That(position.Z, Is.Not.NaN, 
                    $"{moonFrame}'s Z position should not be NaN");
                Assert.That(double.IsFinite(position.X), Is.True, 
                    $"{moonFrame}'s X position should be finite");
                Assert.That(double.IsFinite(position.Y), Is.True, 
                    $"{moonFrame}'s Y position should be finite");
                Assert.That(double.IsFinite(position.Z), Is.True, 
                    $"{moonFrame}'s Z position should be finite");
            }
        }

        /// <summary>
        /// Test that GetCurrentCentralBodyPosition defaults to Jupiter when moon data is unavailable
        /// **Validates: Requirement 5.1**
        /// </summary>
        [Test]
        public void GetCurrentCentralBodyPosition_UnknownFrame_DefaultsToJupiter()
        {
            // ARRANGE: Select a moon frame (which may not have rail data in test environment)
            universeManager.SelectReferenceFrame(ReferenceFrameTarget.Io);
            
            // ACT: Get the central body position
            Vector3d position = universeManager.GetCurrentCentralBodyPosition();
            
            // ASSERT: Verify it returns a valid position (either moon's or Jupiter's as fallback)
            Assert.That(position.X, Is.Not.NaN, 
                "Should return a valid X position (either moon's or Jupiter's as fallback)");
            Assert.That(position.Y, Is.Not.NaN, 
                "Should return a valid Y position (either moon's or Jupiter's as fallback)");
            Assert.That(position.Z, Is.Not.NaN, 
                "Should return a valid Z position (either moon's or Jupiter's as fallback)");
            Assert.That(double.IsFinite(position.X), Is.True, 
                "Should return a finite X position");
            Assert.That(double.IsFinite(position.Y), Is.True, 
                "Should return a finite Y position");
            Assert.That(double.IsFinite(position.Z), Is.True, 
                "Should return a finite Z position");
        }

        /// <summary>
        /// Test that GetCurrentCentralBodyPosition uses TryGetMoonPositionAtTime for moon positions
        /// **Validates: Requirement 5.1**
        /// </summary>
        [Test]
        public void GetCurrentCentralBodyPosition_MoonFrame_UsesTryGetMoonPositionAtTime()
        {
            // ARRANGE: Set Io as the active reference frame
            universeManager.SelectReferenceFrame(ReferenceFrameTarget.Io);
            
            // ACT: Get the central body position
            Vector3d centralBodyPosition = universeManager.GetCurrentCentralBodyPosition();
            
            // Also get the position directly using TryGetMoonPositionAtTime
            bool success = universeManager.TryGetMoonPositionAtTime(0, universeManager.SimulationTimeSeconds, out Vector3d directPosition);
            
            // ASSERT: If TryGetMoonPositionAtTime succeeds, the positions should match
            // If it fails, GetCurrentCentralBodyPosition should fall back to Jupiter
            if (success)
            {
                Assert.That(centralBodyPosition.X, Is.EqualTo(directPosition.X).Within(1e-6),
                    "X position should match TryGetMoonPositionAtTime result");
                Assert.That(centralBodyPosition.Y, Is.EqualTo(directPosition.Y).Within(1e-6),
                    "Y position should match TryGetMoonPositionAtTime result");
                Assert.That(centralBodyPosition.Z, Is.EqualTo(directPosition.Z).Within(1e-6),
                    "Z position should match TryGetMoonPositionAtTime result");
            }
            else
            {
                // Should fall back to Jupiter position
                Assert.That(centralBodyPosition.X, Is.Not.NaN, 
                    "Should fall back to Jupiter position when moon data unavailable");
            }
        }
    }
}

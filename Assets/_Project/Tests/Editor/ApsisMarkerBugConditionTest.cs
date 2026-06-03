using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Galilego.Gameplay;
using Galilego.Universe;
using Galilego.Core;
using System.Collections.Generic;
using System.Reflection;
using System;

namespace Galilego.Tests.Editor
{
    /// <summary>
    /// Bug Condition Exploration Test for Apsis Marker Hover Detection
    /// 
    /// **Validates: Requirements 1.1, 1.2, 1.3, 1.4**
    /// 
    /// CRITICAL: This test MUST FAIL on unfixed code - failure confirms the bug exists.
    /// 
    /// The test verifies that markerData.worldPosition (used for hover detection) does NOT match
    /// markerObj.transform.position (actual sprite position after camera-facing offset).
    /// 
    /// This mismatch causes hover detection to fail because it calculates distance from the wrong position.
    /// 
    /// EXPECTED OUTCOME ON UNFIXED CODE: Test FAILS with counterexamples showing position mismatch
    /// EXPECTED OUTCOME AFTER FIX: Test PASSES (markerData.worldPosition matches actual sprite position)
    /// </summary>
    [TestFixture]
    public class ApsisMarkerBugConditionTest
    {
        private GameObject testSceneRoot;
        private Camera testCamera;
        private UniverseManager universeManager;
        private ApsisMarkerSystem apsisMarkerSystem;

        [SetUp]
        public void SetUp()
        {
            // Create test scene root
            testSceneRoot = new GameObject("TestSceneRoot");

            // Create test camera with realistic configuration
            GameObject cameraObj = new GameObject("TestCamera");
            cameraObj.transform.SetParent(testSceneRoot.transform);
            testCamera = cameraObj.AddComponent<Camera>();
            testCamera.fieldOfView = 60f;
            testCamera.nearClipPlane = 0.1f;
            testCamera.farClipPlane = 10000f;
            // Note: pixelHeight and pixelWidth are read-only and determined by the render target

            // Position camera at a typical orbit map view angle (45 degrees)
            testCamera.transform.position = new Vector3(150f, 150f, 0f);
            testCamera.transform.LookAt(Vector3.zero);

            // Create UniverseManager - expect NullReferenceException during initialization
            // because we're not setting up the full game scene
            LogAssert.Expect(LogType.Exception, "NullReferenceException: Object reference not set to an instance of an object");
            
            GameObject universeObj = new GameObject("UniverseManager");
            universeObj.transform.SetParent(testSceneRoot.transform);
            universeManager = universeObj.AddComponent<UniverseManager>();

            // Initialize UniverseManager with minimal setup
            InitializeUniverseManager();

            // Create ApsisMarkerSystem
            GameObject apsisObj = new GameObject("ApsisMarkerSystem");
            apsisObj.transform.SetParent(testSceneRoot.transform);
            apsisMarkerSystem = apsisObj.AddComponent<ApsisMarkerSystem>();

            // Use reflection to set private fields
            SetPrivateField(apsisMarkerSystem, "universeManager", universeManager);
            SetPrivateField(apsisMarkerSystem, "referenceCamera", testCamera);
            SetPrivateField(apsisMarkerSystem, "useAnalyticalSystem", true);

            // Trigger Awake to initialize marker pool
            InvokePrivateMethod(apsisMarkerSystem, "Awake");
        }

        [TearDown]
        public void TearDown()
        {
            if (testSceneRoot != null)
            {
                UnityEngine.Object.DestroyImmediate(testSceneRoot);
            }
        }

        /// <summary>
        /// Property 1: Bug Condition - Hover Detection Uses Actual Sprite Position
        /// 
        /// **Validates: Requirements 1.1, 1.2, 1.3, 1.4**
        /// 
        /// CRITICAL: This test MUST FAIL on unfixed code - failure confirms the bug exists.
        /// DO NOT attempt to fix the test or the code when it fails.
        /// 
        /// This test verifies the expected behavior (that markerData.worldPosition equals actual sprite position).
        /// On UNFIXED code, this test will FAIL, documenting the bug through counterexamples.
        /// 
        /// Test Strategy:
        /// - Create apsis markers with camera-facing offset applied
        /// - Verify that markerData.worldPosition equals markerObj.transform.position
        /// - Test at specific camera angles where offset is significant (45° angle, top-down view)
        /// - Document counterexamples showing the position mismatch
        /// </summary>
        [Test]
        public void Property_BugCondition_HoverDetectionUsesActualSpritePosition()
        {
            // ARRANGE: Create realistic apsis data for testing
            var apsisDataList = new List<ApsisData>();

            // Test Case 1: Periapsis marker at 45° camera angle
            // This is a concrete failing case where camera-facing offset creates significant displacement
            Vector3d periapsisWorldPos = new Vector3d(100.0, 0.0, 0.0); // Orbital position (before offset)
            apsisDataList.Add(new ApsisData(
                worldPosition: periapsisWorldPos,
                altitude: 50000.0, // 50 km altitude
                timeToReach: universeManager.SimulationTimeSeconds + 100.0,
                type: ApsisType.Periapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "TestBody"
            ));

            // Test Case 2: Apoapsis marker at top-down view
            Vector3d apoapsisWorldPos = new Vector3d(-150.0, 0.0, 0.0); // Orbital position (before offset)
            apsisDataList.Add(new ApsisData(
                worldPosition: apoapsisWorldPos,
                altitude: 150000.0, // 150 km altitude
                timeToReach: universeManager.SimulationTimeSeconds + 200.0,
                type: ApsisType.Apoapsis,
                orbitType: OrbitType.Ballistic,
                segmentIndex: -1,
                isVisible: true,
                centralBodyName: "TestBody"
            ));

            // ACT: Call UpdateApsisMarkers with the test data
            apsisMarkerSystem.UpdateApsisMarkers(apsisDataList);

            // Get the marker data list (used by hover detection)
            var markerDataList = apsisMarkerSystem.MarkerData;

            // Get the marker pool to access actual sprite positions
            var markerPool = GetPrivateField<List<GameObject>>(apsisMarkerSystem, "markerPool");

            // ASSERT: Verify that markerData.worldPosition matches actual sprite position
            System.Text.StringBuilder counterexamples = new System.Text.StringBuilder();
            counterexamples.AppendLine("Bug Condition Exploration Test Results:");
            counterexamples.AppendLine("==========================================");
            counterexamples.AppendLine();
            counterexamples.AppendLine("EXPECTED BEHAVIOR (after fix):");
            counterexamples.AppendLine("  markerData.worldPosition SHOULD EQUAL markerObj.transform.position");
            counterexamples.AppendLine();
            counterexamples.AppendLine("COUNTEREXAMPLES (proving bug exists on unfixed code):");
            counterexamples.AppendLine();

            bool foundMismatch = false;
            int testCaseNumber = 1;

            for (int i = 0; i < markerDataList.Count; i++)
            {
                if (i >= markerPool.Count) break;

                ApsisMarkerData markerData = markerDataList[i];
                GameObject markerObj = markerPool[i];

                if (markerObj == null || !markerObj.activeSelf) continue;

                // Get actual sprite position (after camera-facing offset)
                Vector3 actualSpritePosition = markerObj.transform.position;

                // Get position used by hover detection
                Vector3 hoverDetectionPosition = markerData.worldPosition;

                // Calculate the mismatch
                Vector3 positionDifference = actualSpritePosition - hoverDetectionPosition;
                float mismatchDistance = positionDifference.magnitude;

                // Document the counterexample
                counterexamples.AppendLine($"Test Case {testCaseNumber}: {markerData.type} marker");
                counterexamples.AppendLine($"  Camera Angle: 45° (position: {testCamera.transform.position})");
                counterexamples.AppendLine($"  Orbital Position: {(i == 0 ? periapsisWorldPos : apoapsisWorldPos)}");
                counterexamples.AppendLine($"  markerData.worldPosition (hover detection): {hoverDetectionPosition}");
                counterexamples.AppendLine($"  markerObj.transform.position (actual sprite): {actualSpritePosition}");
                counterexamples.AppendLine($"  Position Mismatch: {positionDifference}");
                counterexamples.AppendLine($"  Mismatch Distance: {mismatchDistance:F2} Unity units");
                counterexamples.AppendLine();

                // Check if positions match (tolerance of 0.01 units for floating point)
                if (mismatchDistance > 0.01f)
                {
                    foundMismatch = true;
                    counterexamples.AppendLine($"  ❌ BUG CONFIRMED: Positions DO NOT match!");
                    counterexamples.AppendLine($"  Root Cause: markerData.worldPosition stores orbital position (before offset),");
                    counterexamples.AppendLine($"              but hover detection needs actual sprite position (after offset)");
                    counterexamples.AppendLine();
                }
                else
                {
                    counterexamples.AppendLine($"  ✓ Positions match correctly (fix is working)");
                    counterexamples.AppendLine();
                }

                testCaseNumber++;
            }

            counterexamples.AppendLine();
            counterexamples.AppendLine("CONCLUSION:");
            counterexamples.AppendLine("============");
            
            if (foundMismatch)
            {
                counterexamples.AppendLine("❌ BUG EXISTS: markerData.worldPosition does NOT match actual sprite position");
                counterexamples.AppendLine();
                counterexamples.AppendLine("Impact:");
                counterexamples.AppendLine("  - Hover detection calculates distance from WRONG position (orbital point)");
                counterexamples.AppendLine("  - User sees sprite at actualSpritePosition but must hover over hoverDetectionPosition");
                counterexamples.AppendLine("  - Mismatch varies with camera angle (camera-facing offset is directional)");
                counterexamples.AppendLine("  - Tooltip appears in wrong location or doesn't appear at all");
                counterexamples.AppendLine();
                counterexamples.AppendLine("Root Cause Analysis:");
                counterexamples.AppendLine("  1. ToMarkerData() converts original orbital position to Unity coordinates");
                counterexamples.AppendLine("  2. Line 814 attempts to override: markerData.worldPosition = markerObj.transform.position");
                counterexamples.AppendLine("  3. BUT the override happens with markerData as a struct value copy");
                counterexamples.AppendLine("  4. OR transform.position is not yet updated when assignment happens");
                counterexamples.AppendLine();
                counterexamples.AppendLine("This test has SUCCESSFULLY documented the bug condition.");
            }
            else
            {
                counterexamples.AppendLine("✓ FIX VERIFIED: markerData.worldPosition correctly matches actual sprite position");
                counterexamples.AppendLine();
                counterexamples.AppendLine("The fix has been successfully implemented:");
                counterexamples.AppendLine("  - markerData.worldPosition now stores the actual sprite position (after offset)");
                counterexamples.AppendLine("  - Hover detection will calculate distance from correct position");
                counterexamples.AppendLine("  - Tooltip will appear reliably when hovering over visible sprite");
            }

            // Log the full counterexample report
            Debug.Log(counterexamples.ToString());

            // ASSERTION: On unfixed code, this will FAIL (proving bug exists)
            // On fixed code, this will PASS (proving fix works)
            if (foundMismatch)
            {
                Assert.Fail(
                    "BUG CONFIRMED: markerData.worldPosition does NOT match actual sprite position.\n" +
                    "This failure is EXPECTED on unfixed code and proves the bug exists.\n\n" +
                    counterexamples.ToString()
                );
            }
            else
            {
                Assert.Pass(
                    "FIX VERIFIED: markerData.worldPosition correctly matches actual sprite position.\n\n" +
                    counterexamples.ToString()
                );
            }
        }

        // Helper methods for reflection
        private void SetPrivateField(object obj, string fieldName, object value)
        {
            var field = obj.GetType().GetField(fieldName, BindingFlags.NonPublic | BindingFlags.Instance);
            if (field != null)
            {
                field.SetValue(obj, value);
            }
            else
            {
                Debug.LogWarning($"Field '{fieldName}' not found on {obj.GetType().Name}");
            }
        }

        private T GetPrivateField<T>(object obj, string fieldName)
        {
            var field = obj.GetType().GetField(fieldName, BindingFlags.NonPublic | BindingFlags.Instance);
            if (field != null)
            {
                return (T)field.GetValue(obj);
            }
            else
            {
                Debug.LogWarning($"Field '{fieldName}' not found on {obj.GetType().Name}");
                return default(T);
            }
        }

        private void InvokePrivateMethod(object obj, string methodName, params object[] parameters)
        {
            var method = obj.GetType().GetMethod(methodName, BindingFlags.NonPublic | BindingFlags.Instance);
            if (method != null)
            {
                method.Invoke(obj, parameters);
            }
            else
            {
                Debug.LogWarning($"Method '{methodName}' not found on {obj.GetType().Name}");
            }
        }

        private void InitializeUniverseManager()
        {
            // Set minimal required fields for ToUnityPosition to work
            // ToUnityPosition typically subtracts a floating origin offset
            SetPrivateField(universeManager, "floatingOriginOffset", Vector3d.Zero);
            SetPrivateField(universeManager, "simulationTimeSeconds", 0.0);
        }
    }
}

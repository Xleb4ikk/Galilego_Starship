# DEEP ORBITAL DEBUGGING ANALYSIS
## Complete Analysis of N-Body Orbital Simulation Inconsistencies

---

## EXECUTIVE SUMMARY

**ROOT CAUSE**: When Δv = 0, the green (maneuver) and purple (current) orbits DO NOT match because of **multiple independent inconsistencies** in coordinate systems, velocity references, and integration parameters.

**CRITICAL FINDINGS**: 9 major issues identified, 3 of which are CRITICAL for Δv=0 mismatch.

---

## ISSUE #1: CRITICAL - Velocity Used in CalculateWorldDeltaV is WRONG

### Location: `ManeuverEvaluator.cs` lines ~170 and ~260

### Problem:
```csharp
// CURRENT (WRONG):
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, currentVel, currentNode);
```

`currentVel` is the **barycentric world velocity** of the ship, NOT the velocity relative to the reference frame.

### Why This Matters:
`CalculateWorldDeltaV` computes the orbital basis (prograde, radial, normal) using the `velocity` parameter:
```csharp
Vector3d tangentDir = velocity.Normalized;  // Prograde direction
Vector3d normalDir = Vector3d.Cross(velocity, radialDir).Normalized;  // Normal
```

If `velocity` is the world velocity instead of relative velocity, the prograde direction is computed in the **wrong reference frame**. This causes the Δv vector to be applied in the wrong direction, even when Δv = 0 (because the basis itself is wrong).

### Fix:
```csharp
// CORRECT:
// Get frame velocity at current time
Vector3d frameVel;
if (universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out _, out frameVel, out _, out _, out _))
{
    Vector3d relativeVel = currentVel - frameVel;
    Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, relativeVel, currentNode);
    currentVel += dv;
}
```

### Affected Locations:
1. `ManeuverEvaluator.cs` - Past node handling (line ~170)
2. `ManeuverEvaluator.cs` - Future node handling (line ~260)

---

## ISSUE #2: CRITICAL - RefreshPredictionCache Uses World Velocity

### Location: `FlightPlanUI.cs` - `RefreshPredictionCache()`

### Problem:
```csharp
// CURRENT (WRONG):
Vector3d vel = universeManager.ShipBody.Velocity;  // World velocity!
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToJupiter, vel, node);
orbitAfter = OrbitalElements.FromState(posRelativeToJupiter, (vel + dv) - frameVel, mu);
```

The velocity used for `CalculateWorldDeltaV` is world velocity, but the resulting orbit calculation correctly subtracts `frameVel`. This inconsistency means the Δv direction is computed incorrectly.

### Fix:
```csharp
// CORRECT:
Vector3d vel = universeManager.ShipBody.Velocity;
Vector3d relativeVel = vel - frameVel;
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToJupiter, relativeVel, node);
orbitAfter = OrbitalElements.FromState(posRelativeToJupiter, relativeVel + dv, mu);
```

---

## ISSUE #3: CRITICAL - Timestep Desync Between Predictors

### Location: `ManeuverEvaluator.cs` vs `TrajectoryPredictor.cs`

### Problem:
- `ManeuverEvaluator`: `predictionStepSeconds = 2.0d` (FIXED in code)
- `TrajectoryPredictor`: `predictionStepSeconds = 2.0d` (default)
- `UniverseManager`: `shipPredictionStepSeconds = 10d` (INSPECTOR OVERRIDE!)

The `UniverseManager` configures `TrajectoryPredictor` via:
```csharp
shipTrajectoryPredictor.ConfigurePrediction(
    shipPredictionSteps,
    shipPredictionStepSeconds,  // 10d from inspector!
    ...
);
```

But `ManeuverEvaluator` uses its own `predictionStepSeconds = 2.0d`.

### Why Δv=0 Orbits Differ:
Different timesteps → different integration errors → different orbital paths even with identical initial conditions.

### Fix:
**Option A**: Use identical timesteps (2.0d) for both:
```csharp
// In UniverseManager.cs, change default:
[SerializeField] private double shipPredictionStepSeconds = 2d;
```

**Option B**: Make ManeuverEvaluator read from UniverseManager:
```csharp
// In ManeuverEvaluator.cs:
double majorStep = universeManager.RecommendedSolverStepSeconds;
```

---

## ISSUE #4: Line Renderer Space Inconsistency

### Location: Both predictors use `useWorldSpace = false`

### Problem:
Both `TrajectoryPredictor` and `ManeuverEvaluator` create LineRenderers with `useWorldSpace = false`, meaning they use **local space relative to their parent transform**.

The parent transforms are positioned differently:
- `TrajectoryPredictor`: Parent is `trajectoryVisualRoot`, positioned at `startFramePosition`
- `ManeuverEvaluator`: Each segment line is positioned at `framePos` (updated per sample)

### Why This Matters:
If the frame position updates are not identical (due to timing or interpolation), the orbits will be offset in different local spaces.

### Verification Needed:
Check that both predictors use the **same parent transform** and that `ApplyVisualPosition` is called identically.

---

## ISSUE #5: Stale Frame Fallback

### Location: `ManeuverEvaluator.cs` - framePos initialization

### Problem:
```csharp
Vector3d framePos = Vector3d.Zero;  // Default fallback

// Later:
if (!universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out framePos, out _, out _, out _, out _))
{
    framePos = Vector3d.Zero;  // Silent fallback to zero!
}
```

When `TryGetReferenceStateAtTime` fails, `framePos` remains at its previous value (or Zero), causing **stale frame transforms** to be used silently.

### Fix:
```csharp
// CORRECT: Abort sample on failure
if (!universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out framePos, out _, out _, out _, out _))
{
    Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Skipping sample.");
    continue;  // Skip this sample rather than using stale framePos
}
```

---

## ISSUE #6: Coroutine Desync - Partial LineRenderer Updates

### Location: `ManeuverEvaluator.cs` - `FlushLine()`

### Problem:
```csharp
private void FlushLine(LineRenderer line, List<Vector3> points)
{
    // ...
    line.positionCount = points.Count;
    line.SetPositions(toSet);
}
```

During coroutine execution, `FlushLine` is called multiple times per frame. The LineRenderer is **visible to the camera** between flushes, showing partially-built trajectories.

### Fix:
Use double-buffering:
```csharp
// Add to ManeuverEvaluator:
private LineRenderer activeLine;
private LineRenderer backLine;

// In coroutine:
// Build points in backLine
// When segment complete:
Swap(ref activeLine, ref backLine);
backLine.positionCount = 0;  // Clear back buffer
```

---

## ISSUE #7: Float Precision Loss in Coordinate Conversion

### Location: `UniverseManager.cs` - `ToUnityOffset()`

### Problem:
```csharp
public Vector3 ToUnityOffset(Vector3d realOffset)
{
    Vector3d scaledOffset = realOffset / GetMetersPerVisualUnit();
    return new Vector3((float)scaledOffset.X, (float)scaledOffset.Y, (float)scaledOffset.Z);
}
```

When `realOffset` is large (e.g., Jupiter radius ~70,000 km), subtracting two large numbers and converting to float causes precision loss.

### Example:
```csharp
// Jupiter position: ~70,000,000 meters
// Ship position: ~70,100,000 meters
// Difference: 100,000 meters
// After division by metersPerVisualUnit (100,000): 1.0 Unity unit
// Float precision: ~7 decimal digits → OK for this case

// BUT: If positions are ~1e9 meters apart:
// Difference after division: ~10,000 Unity units
// Float precision at 10,000: ~0.001 units → 100 meters error!
```

### Mitigation:
This is inherent to Unity's float-based rendering. For orbital simulations, keep the camera focused on the region of interest and use floating origin.

---

## ISSUE #8: Orbital Basis Inconsistency in FlightPlan.cs

### Location: `FlightPlan.cs` - `CalculateWorldDeltaV()`

### Problem:
```csharp
// Frenet-Serret frame (Prograde, Normal, Radial)
Vector3d tangentDir = velocity.Normalized;
Vector3d radialDir = position.Normalized;
Vector3d normalDir = Vector3d.Cross(velocity, radialDir).Normalized;
Vector3d binormalDir = Vector3d.Cross(tangentDir, normalDir).Normalized;
```

The binormal is computed as `tangent × normal`, but the input `DvBinormal` is described as "Radial In/Out". This is **semantically incorrect**:
- `binormalDir` should be the radial direction
- But `binormalDir = tangent × normal` is actually **anti-radial** (points away from the central body)

### Fix:
```csharp
// CORRECT: Use consistent orbital basis
Vector3d tangentDir = velocity.Normalized;           // Prograde
Vector3d radialDir = position.Normalized;             // Radial Out
Vector3d normalDir = Vector3d.Cross(tangentDir, radialDir).Normalized;  // Normal (orbit plane perpendicular)

// For Δv application:
return (tangentDir * node.DvTangent) +      // Prograde/Retrograde
       (normalDir * node.DvNormal) +        // Normal/Anti-Normal
       (radialDir * node.DvBinormal);        // Radial In/Out
```

---

## ISSUE #9: UniverseManager Ship Trajectory Configuration

### Location: `UniverseManager.cs` - `EnsureShipTrajectoryVisualizer()`

### Problem:
```csharp
shipTrajectoryPredictor.ConfigurePrediction(
    shipPredictionSteps,        // 1200
    shipPredictionStepSeconds,  // 10d (INSPECTOR VALUE!)
    RecommendedSolverStepSeconds,  // 1d
    true,
    shipPredictionRefreshInterval,
    shipPredictionStepsPerBatch);
```

The `shipPredictionStepSeconds = 10d` is set in the inspector, but `ManeuverEvaluator` uses `2.0d`. This is the **primary cause** of Δv=0 orbit mismatch.

### Fix:
```csharp
// In UniverseManager.cs, change the default:
[SerializeField] private double shipPredictionStepSeconds = 2d;
```

---

## COMPLETE FIX LIST

### Fix 1: ManeuverEvaluator.cs - Use Relative Velocity
```csharp
// In CalculateFullTrajectoryCoroutine(), around line 170:
if (currentNode != null)
{
    if (universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out Vector3d newFramePos, out Vector3d newFrameVel, out _, out _, out _))
    {
        framePos = newFramePos;
        Vector3d posRelativeToBody = currentPos - framePos;
        Vector3d relativeVel = currentVel - newFrameVel;
        Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, relativeVel, currentNode);
        currentVel += dv;
    }
}

// Around line 260 (future node handling):
if (currentNode != null)
{
    Vector3d currentFramePos;
    Vector3d currentFrameVel;
    if (universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out currentFramePos, out currentFrameVel, out _, out _, out _))
    {
        framePos = currentFramePos;
        Vector3d posRelativeToBody = currentPos - framePos;
        Vector3d relativeVel = currentVel - currentFrameVel;
        Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, relativeVel, currentNode);
        currentVel += dv;
    }
}
```

### Fix 2: FlightPlanUI.cs - Use Relative Velocity in RefreshPredictionCache
```csharp
private void RefreshPredictionCache()
{
    // ...
    if (!universeManager.TryGetReferenceState(frame, out _, out Vector3d framePos, out Vector3d frameVel, out double mu, out _, out _)) return;
    
    Vector3d worldPos = universeManager.ShipBody.Position;
    Vector3d vel = universeManager.ShipBody.Velocity;
    Vector3d posRelativeToJupiter = worldPos - framePos;
    Vector3d relativeVel = vel - frameVel;
    
    Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToJupiter, relativeVel, node);
    orbitAfter = OrbitalElements.FromState(posRelativeToJupiter, relativeVel + dv, mu);
}
```

### Fix 3: UniverseManager.cs - Unify Timestep
```csharp
// Change default value:
[SerializeField] private double shipPredictionStepSeconds = 2d;
```

### Fix 4: FlightPlan.cs - Fix Orbital Basis
```csharp
public static Vector3d CalculateWorldDeltaV(Vector3d position, Vector3d velocity, ManeuverNode node)
{
    if (node == null) return Vector3d.Zero;
    if (!position.IsFinite || !velocity.IsFinite) return Vector3d.Zero;
    if (velocity.SqrMagnitude < 0.001d) return Vector3d.Zero;

    Vector3d tangentDir = velocity.Normalized;
    if (!tangentDir.IsFinite) tangentDir = Vector3d.Zero;

    Vector3d radialDir = position.Normalized;
    if (!radialDir.IsFinite) radialDir = Vector3d.Zero;

    // FIX: Normal = tangent × radial (right-hand rule, points "up" from orbit plane)
    Vector3d normalDir = Vector3d.Cross(tangentDir, radialDir).Normalized;
    if (!normalDir.IsFinite) normalDir = Vector3d.Zero;

    // FIX: Use radialDir directly for binormal (Radial In/Out)
    // binormalDir = radialDir (not tangent × normal)
    Vector3d binormalDir = radialDir;
    if (!binormalDir.IsFinite) binormalDir = Vector3d.Zero;

    return (tangentDir * node.DvTangent) +
           (normalDir * node.DvNormal) +
           (binormalDir * node.DvBinormal);
}
```

### Fix 5: ManeuverEvaluator.cs - Remove Stale Frame Fallback
```csharp
// Replace:
if (!universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out framePos, out _, out _, out _, out _))
{
    framePos = Vector3d.Zero;
}

// With:
if (!universeManager.TryGetReferenceStateAtTime(referenceFrame, currentTime, out _, out framePos, out _, out _, out _, out _))
{
    Debug.LogWarning($"ManeuverEvaluator: Failed to get frame state at t={currentTime}. Aborting segment.");
    trajectoryLimitReached = true;
    break;
}
```

---

## CANONICAL ARCHITECTURE RECOMMENDATION

### 1. Immutable Prediction Buffers
```csharp
public sealed class TrajectoryBuffer
{
    public readonly Vector3[] Points;
    public readonly double[] Times;
    public readonly int Count;
    
    public TrajectoryBuffer(Vector3[] points, double[] times, int count)
    {
        Points = points;
        Times = times;
        Count = count;
    }
}

// Double-buffering in predictors:
private TrajectoryBuffer frontBuffer;
private TrajectoryBuffer backBuffer;

// Atomic swap when rebuild complete:
Interlocked.Exchange(ref frontBuffer, backBuffer);
```

### 2. Deterministic Render Pipeline
```csharp
// Single render pass reads from immutable front buffer:
void LateUpdate()
{
    var buffer = GetCurrentBuffer();  // Atomic read
    lineRenderer.positionCount = buffer.Count;
    lineRenderer.SetPositions(buffer.Points);
}
```

### 3. Unified Orbital Basis
```csharp
public static class OrbitalBasis
{
    public static void ComputeBasis(Vector3d relativePosition, Vector3d relativeVelocity, 
        out Vector3d prograde, out Vector3d radial, out Vector3d normal)
    {
        prograde = relativeVelocity.Normalized;
        radial = relativePosition.Normalized;
        normal = Vector3d.Cross(prograde, radial).Normalized;
    }
}
```

### 4. Unified Frame Transform Logic
```csharp
public static class FrameTransform
{
    public static Vector3d ToLocalPosition(Vector3d worldPosition, ReferenceFrameTarget frame, double time)
    {
        // Single source of truth for all frame transforms
    }
    
    public static Vector3d ToLocalVelocity(Vector3d worldVelocity, ReferenceFrameTarget frame, double time)
    {
        // Single source of truth for all frame transforms
    }
}
```

---

## VERIFICATION TEST

After applying all fixes, verify with:

```csharp
// Test: Δv = 0 should produce identical orbits
// 1. Create maneuver node with DvTangent = DvNormal = DvBinormal = 0
// 2. Run simulation
// 3. Compare green (maneuver) and purple (current) orbit point-by-point
// 4. Expected: All points within epsilon (1e-6 Unity units)
```

---

## SUMMARY TABLE

| Issue | Severity | Location | Fix |
|-------|----------|----------|-----|
| #1: Wrong velocity in CalculateWorldDeltaV | CRITICAL | ManeuverEvaluator.cs | Use relativeVel = currentVel - frameVel |
| #2: Wrong velocity in RefreshPredictionCache | CRITICAL | FlightPlanUI.cs | Use relativeVel = vel - frameVel |
| #3: Timestep desync | CRITICAL | UniverseManager.cs | Set shipPredictionStepSeconds = 2d |
| #4: Line renderer space | MEDIUM | Both predictors | Verify identical parent transforms |
| #5: Stale frame fallback | MEDIUM | ManeuverEvaluator.cs | Abort on failure, don't use stale |
| #6: Coroutine
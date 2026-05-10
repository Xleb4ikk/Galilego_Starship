# ORBITAL SIMULATION FIXES - SUMMARY
## All Critical Fixes Applied

---

## ✅ FIXES APPLIED

### Fix 1: ManeuverEvaluator.cs - Relative Velocity for Δv Calculation
**Status**: ✅ APPLIED

**Problem**: `CalculateWorldDeltaV` was receiving world velocity instead of relative velocity, causing incorrect orbital basis computation.

**Changes**:
- Line ~170 (past nodes): Added `frameVel` retrieval and `relativeVel = currentVel - newFrameVel`
- Line ~260 (future nodes): Added `frameVel` retrieval and `relativeVel = currentVel - currentFrameVel`

**Code**:
```csharp
// Before (WRONG):
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, currentVel, currentNode);

// After (CORRECT):
Vector3d relativeVel = currentVel - newFrameVel;
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, relativeVel, currentNode);
```

---

### Fix 2: FlightPlanUI.cs - Relative Velocity in RefreshPredictionCache
**Status**: ✅ APPLIED

**Problem**: `RefreshPredictionCache` was using world velocity for Δv calculation but relative velocity for orbit computation.

**Code**:
```csharp
// Before (WRONG):
Vector3d vel = universeManager.ShipBody.Velocity;
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToJupiter, vel, node);
orbitAfter = OrbitalElements.FromState(posRelativeToJupiter, (vel + dv) - frameVel, mu);

// After (CORRECT):
Vector3d worldVel = universeManager.ShipBody.Velocity;
Vector3d relativeVel = worldVel - frameVel;
Vector3d dv = FlightPlan.CalculateWorldDeltaV(posRelativeToBody, relativeVel, node);
orbitAfter = OrbitalElements.FromState(posRelativeToBody, relativeVel + dv, mu);
```

---

### Fix 3: UniverseManager.cs - Unified Timestep
**Status**: ✅ APPLIED

**Problem**: `shipPredictionStepSeconds = 10d` in inspector caused different integration precision between TrajectoryPredictor and ManeuverEvaluator.

**Code**:
```csharp
// Before (WRONG):
[SerializeField] private double shipPredictionStepSeconds = 10d;

// After (CORRECT):
// ИСПРАВЛЕНИЕ: Унифицирован с ManeuverEvaluator.predictionStepSeconds = 2.0d
// для идентичной точности интеграции при Δv=0
[SerializeField] private double shipPredictionStepSeconds = 2d;
```

---

### Fix 4: FlightPlan.cs - Fixed Orbital Basis
**Status**: ✅ APPLIED

**Problem**: `binormalDir` was computed as `tangent × normal` (incorrect), but `DvBinormal` represents "Radial In/Out" which should use `radialDir`.

**Code**:
```csharp
// Before (WRONG):
Vector3d normalDir = Vector3d.Cross(velocity, radialDir).Normalized;
Vector3d binormalDir = Vector3d.Cross(tangentDir, normalDir).Normalized;

// After (CORRECT):
// Normal = tangent × radial (right-hand rule, "up" from orbit plane)
Vector3d normalDir = Vector3d.Cross(tangentDir, radialDir).Normalized;
// Binormal = radialDir (Radial In/Out direction)
Vector3d binormalDir = radialDir;
```

---

## 🔍 REMAINING ISSUES (Non-Critical)

### Issue 5: Stale Frame Fallback (MEDIUM)
**Location**: `ManeuverEvaluator.cs`
**Problem**: When `TryGetReferenceStateAtTime` fails, `framePos` falls back to `Vector3d.Zero` silently.
**Recommendation**: Abort segment on failure instead of using stale frame position.

### Issue 6: Coroutine Desync (MEDIUM)
**Location**: `ManeuverEvaluator.cs`
**Problem**: LineRenderer shows partially-built trajectories during coroutine execution.
**Recommendation**: Implement double-buffering for trajectory rendering.

### Issue 7: Float Precision (LOW)
**Location**: `UniverseManager.cs` - `ToUnityOffset()`
**Problem**: Large coordinate subtraction causes precision loss when converting to float.
**Recommendation**: Inherent to Unity's float-based rendering. Keep camera focused on region of interest.

---

## 📊 VERIFICATION TEST

After applying all fixes, verify with:

```csharp
// Test: Δv = 0 should produce identical orbits
// 1. Create maneuver node with DvTangent = DvNormal = DvBinormal = 0
// 2. Run simulation
// 3. Compare green (maneuver) and purple (current) orbit point-by-point
// 4. Expected: All points within epsilon (1e-6 Unity units)
```

---

## 📋 SUMMARY TABLE

| Issue | Severity | Status | File |
|-------|----------|--------|------|
| #1: Wrong velocity in CalculateWorldDeltaV | CRITICAL | ✅ FIXED | ManeuverEvaluator.cs |
| #2: Wrong velocity in RefreshPredictionCache | CRITICAL | ✅ FIXED | FlightPlanUI.cs |
| #3: Timestep desync | CRITICAL | ✅ FIXED | UniverseManager.cs |
| #4: Orbital basis binormal direction | CRITICAL | ✅ FIXED | FlightPlan.cs |
| #5: Stale frame fallback | MEDIUM | ⚠️ REMAINS | ManeuverEvaluator.cs |
| #6: Coroutine desync | MEDIUM | ⚠️ REMAINS | ManeuverEvaluator.cs |
| #7: Float precision | LOW | ⚠️ REMAINS | UniverseManager.cs |

---

## 🏗️ CANONICAL ARCHITECTURE RECOMMENDATIONS

### 1. Unified Orbital Basis
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

### 2. Unified Frame Transform
```csharp
public static class FrameTransform
{
    public static (Vector3d position, Vector3d velocity) ToRelative(
        Vector3d worldPos, Vector3d worldVel, ReferenceFrameTarget frame, double time)
    {
        // Single source of truth for all frame transforms
    }
}
```

### 3. Immutable Prediction Buffers
```csharp
public sealed class TrajectoryBuffer
{
    public readonly Vector3[] Points;
    public readonly double[] Times;
    public readonly int Count;
}
```

---

## 🎯 EXPECTED RESULT

After applying fixes #1-4, when Δv = 0:
- Green (maneuver) orbit should match purple (current) orbit
- Sample-by-sample numerical identity within tiny epsilon (~1e-6 Unity units)
- No visible offset between the two orbits in OrbitMap mode

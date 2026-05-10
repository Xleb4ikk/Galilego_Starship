# FINAL ORBITAL SIMULATION ARCHITECTURE
## Complete Architectural Cleanup - Post Fix Summary

---

## ARCHITECTURE OVERVIEW

### Core Principles
1. **Deterministic Prediction**: Δv=0 produces identical orbits (sample-by-sample)
2. **Immutable Rendering**: No partial trajectory visibility during rebuild
3. **Canonical Orbital Basis**: Mathematically correct orbital frame
4. **No Stale State**: Hard fail on frame state errors
5. **Unified Timestep**: Identical integration precision across all predictors

---

## FILE STRUCTURE

### Core/
- `OrbitalBasis.cs` - Canonical orbital basis computation
- `TrajectoryBuffer.cs` - Immutable trajectory buffers with double-buffering
- `Vector3d.cs` - Double-precision vector math
- `PhysicsSolver.cs` - RK4 integration
- `CelestialBody.cs` - Body state management

### Scripts/Gameplay/
- `FlightPlan.cs` - Maneuver nodes with canonical Δv axes (Prograde, Normal, Radial)
- `ManeuverEvaluator.cs` - Deterministic trajectory prediction with back-buffer
- `TrajectoryPredictor.cs` - Current orbit prediction (purple)

### Scripts/Systems/
- `UniverseManager.cs` - Barycentric simulation, reference frames, visual transforms

### Scripts/UI/
- `FlightPlanUI.cs` - Maneuver planner interface

---

## KEY ARCHITECTURAL CHANGES

### 1. Canonical Orbital Basis (OrbitalBasis.cs)

**Before**: Inconsistent basis with `binormalDir = tangent × normal` (wrong)

**After**: Canonical orbital mechanics basis:
```csharp
// Radial: direction from central body to spacecraft
radial = relativePosition.Normalized;

// Normal: angular momentum direction (R × V)
normal = Vector3d.Cross(relativePosition, relativeVelocity).Normalized;

// Prograde: completes right-handed triad (N × R)
prograde = Vector3d.Cross(normal, radial).Normalized;
```

**Properties**:
- Right-handed coordinate system
- Normal points along angular momentum
- Prograde is perpendicular to radial in velocity direction
- Validated orthogonality check in `TryComputeBasis`

### 2. Immutable Trajectory Rendering (ManeuverEvaluator.cs)

**Before**: Direct LineRenderer updates during coroutine (partial visibility)

**After**: Double-buffered rendering:
```csharp
// Back buffer: built by coroutine
Vector3[] backBufferPoints;
double[] backBufferTimes;

// Atomic swap on completion
CompleteBackBuffer(totalPoints);

// Render from front buffer only (in LateUpdate)
```

**Benefits**:
- No partial trajectory visibility
- No coroutine/render race conditions
- Deterministic frame presentation

### 3. No Stale Frame Fallback (ManeuverEvaluator.cs)

**Before**: Silent fallback to `Vector3d.Zero` on frame state failure

**After**: Hard fail with diagnostics:
```csharp
private bool TryUpdateFrameState(ref Vector3d framePos, ref Vector3d frameVel, double time)
{
    if (universeManager.TryGetReferenceStateAtTime(...))
        return true;
    
    // Hard fail - do NOT use stale frame
    return false;
}
```

**Behavior**: Prediction aborts on frame state error, preventing orbit jumps.

### 4. Frame-Locked Reference Frame (ManeuverEvaluator.cs)

**Before**: Reference frame could change during prediction

**After**: Locked at prediction start:
```csharp
// Lock reference frame for entire prediction
lockedReferenceFrame = universeManager.ActiveReferenceFrame;
```

### 5. Unified Timestep (UniverseManager.cs + ManeuverEvaluator.cs)

**Before**: `shipPredictionStepSeconds = 10d` vs `predictionStepSeconds = 2.0d`

**After**: Both use `2.0d`:
```csharp
[SerializeField] private double shipPredictionStepSeconds = 2d;
[SerializeField] private double predictionStepSeconds = 2.0d;
```

### 6. Renamed Δv Axes (FlightPlan.cs)

**Before**: `DvTangent`, `DvBinormal` (semantically incorrect)

**After**: `DvPrograde`, `DvRadial` (canonical names)

**Backward Compatibility**: Obsolete properties maintain serialization:
```csharp
[Obsolete("Use DvPrograde instead")]
public double DvTangent { get => DvPrograde; set => DvPrograde = value; }
```

---

## ORBITAL BASIS VALIDATION

### Canonical Basis Properties
1. **Right-handed**: `radial × prograde = normal`
2. **Orthogonal**: All axes perpendicular (validated in `TryComputeBasis`)
3. **Consistent**: Same basis for all maneuvers and predictions
4. **Physically Correct**: Normal points along angular momentum

### Validation Test
```csharp
// Test: Δv = 0 should produce identical orbits
// 1. Create maneuver node with DvPrograde = DvNormal = DvRadial = 0
// 2. Run simulation
// 3. Compare green (maneuver) and purple (current) orbit point-by-point
// 4. Expected: All points within epsilon (1e-6 Unity units)
```

---

## RENDERING PIPELINE

### Prediction Flow
1. `RequestRecalculation()` → Start coroutine
2. Coroutine builds into back buffer
3. On completion: `CompleteBackBuffer()` → atomic swap
4. `LateUpdate()` renders from front buffer only

### Frame Synchronization
- Reference frame locked at prediction start
- Frame state updated per sample (no stale fallback)
- Display transform: `displayPos = predictedShipPos(t) - framePos(t)`

### Line Renderer Space
- `useWorldSpace = false` (local space)
- Parent transform positioned at frame position
- All segments in identical coordinate space

---

## DETERMINISM GUARANTEES

### What is Deterministic
1. **Initial Conditions**: Same ship state → same prediction
2. **Integration**: Same timestep → same trajectory
3. **Frame Transforms**: Same frame state → same display
4. **Rendering**: Same buffer → same vertices

### What is NOT Deterministic (by design)
1. **UI Timing**: Prediction triggered by user interaction
2. **Frame Rate**: Yield points may vary (but don't affect result)
3. **Floating Point**: Minor variations in last digits (within epsilon)

---

## ERROR HANDLING

### Hard Failures (Abort Prediction)
- Frame state unavailable
- Invalid physics state (NaN/Inf)
- Trajectory point limit exceeded

### Soft Failures (Skip Sample)
- Individual integration step failure
- Single frame state update failure

### Diagnostics
- Warning logs for frame state failures
- Error logs for physics state violations
- Safety limits for iteration counts

---

## PERFORMANCE CHARACTERISTICS

### Prediction
- Max 512 integration steps per frame
- Yield every N steps to maintain framerate
- Back buffer pre-allocated to estimated capacity

### Rendering
- Atomic swap (no partial updates)
- Front buffer read-only during render
- LineRenderer count scales with trajectory length

### Memory
- Back buffer: ~5000 points × (12 + 8) bytes = ~100 KB
- Front buffer: same
- Total: ~200 KB for trajectory data

---

## KNOWN LIMITATIONS

### Float Precision
- Unity uses float for rendering
- Large coordinates (>1e6 units) may show jitter
- Mitigation: Keep camera focused on region of interest

### Timestep Trade-offs
- 2.0s timestep balances precision vs performance
- Longer orbits accumulate more error
- Consider adaptive timestep for high-precision needs

### Reference Frame Changes
- Frame locked at prediction start
- Changing reference frame during prediction requires recalculation
- This is intentional (prevents desync)

---

## FUTURE IMPROVEMENTS

### Floating Origin
- Implement camera-local rendering origin
- Reduces float precision issues
- Requires transform hierarchy changes

### Adaptive Timestep
- Smaller steps near massive bodies
- Larger steps in deep space
- Improves long-orbit precision

### Prediction Caching
- Cache predictions for unchanged flight plans
- Invalidate on node modification
- Reduces redundant computation

### Multi-Segment Rendering
- Split long trajectories across multiple GameObjects
- Reduces per-renderer vertex count
- Improves culling efficiency

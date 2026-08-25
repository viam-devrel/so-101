# Opt-in articulated jaw on the hardware SO-101 gripper

**Date:** 2026-08-24
**Status:** Design, approved for planning
**Scope:** `components/gripper`, `docs/gripper.md`, `CLAUDE.md`

## Purpose

Give the **hardware** SO-101 gripper the same optional 1-DoF revolute jaw the simulated
gripper gained, behind the same `articulated_jaw` config attribute, defaulting to `false`.

This is deliberately **not** a change of default. Making it the default requires owning a
caching policy, a read-failure policy, and grasp retention on real hardware; this spec
lands all three behind a flag so they can be exercised on a real arm safely first.

## Background

The simulated gripper (spec `2026-08-23-articulated-gripper-jaw-design.md`) is now
`InputEnabled` behind `articulated_jaw`, with a branching kinematic model whose TCP stays
invariant while a revolute jaw joint drives the moving mesh.

`TestPlannerJawTravel` measured what the motion planner does with that degree of freedom.
On an unobstructed plan the jaw never moves. Under obstruction the RRT moves it
**4.3%-19.9% of its range**, in both directions, with 8 of 10 seeds returning near their
start. On a simulated gripper that is a curiosity. On hardware holding a part, it is the
grip loosening mid-trajectory — which is the single fact that shapes this design.

Three things are already in place and are reused rather than rebuilt:

- `positionPercent(ctx)` reads the servo's opening; `moveToPercent(ctx, pct)` commands it
  and waits for it to settle. Both go through `servoDo` → `g.arm.DoCommand`.
- `geometry.JawRadiansFromPct` / `JawPctFromRadians` convert between opening percentage
  and jaw radians.
- `components/gripper/gripper_test.go` has a `fakeServoArm` and `newTestGripper` harness.
  The gripper's **only** hardware path is `arm.DoCommand`, so everything in this spec is
  unit-testable without a robot.

## The three decisions this design makes

### 1. Reject planner jaw commands while holding

`GoToInputs` consults `IsHoldingSomething` and returns an error if the gripper has a grip,
rather than loosening it. The motion service surfaces the error and aborts the plan.

Rejected alternatives: *clamp so the jaw never opens further* — the executed trajectory
then silently diverges from the planned one, so the collision geometry the planner reasoned
about no longer matches reality; and *allow with a warning* — which will physically drop
parts.

Accepted cost: an arm move can now fail for a reason that looks unrelated to the gripper.
The error message must say plainly that the gripper is holding something and that the
planner tried to move the jaw.

### 2. Last-known-good on a failed read

`robot/framesystem`'s `CurrentInputs` walks every frame with `len(DoF()) > 0` and
**hard-errors the entire machine's frame system** if that component's `CurrentInputs`
returns an error. One dropped serial frame would therefore abort every arm move on the
machine, including ones with nothing to do with the gripper.

So: on a failed read, return the last successful reading and log a warning. Error **only**
when there has never been a successful read (startup).

Explicitly rejected: falling back to `GripperJointMin`, which is what `jawAngle` does
today. That is fine for a viewer mesh but unsafe for planning — it tells the planner the
jaw is *closed* when it may be wide open, producing wrong collision geometry with no signal.

### 3. Short TTL cache, invalidated on every write

Every `CurrentInputs` call is a `DoCommand` round trip (possibly across gRPC to a remote
arm) plus a serial read on servo 6, serialized behind the controller's `busMu`, on the same
1 Mbaud bus as the five arm servos. Nothing in `internal/controller` or `internal/servocmd`
caches anything.

Today the hardware gripper is 0-DoF, so the frame system skips it and that path is nearly
idle. Making it 1-DoF puts it on every motion request, every `TransformPose`, and — verified
on a live machine — **every 3D-viewer refresh**, because the viewer polls
`GetCurrentInputs` to animate a jointed gripper.

So: cache the reading for `jawCacheTTL = 50ms`, invalidated immediately by any write path.
A value can only be stale while nothing is commanding the jaw.

**This is the assumption most worth challenging.** The planner may act on jaw geometry up
to 50 ms old. With invalidation on every write and no external force on the jaw, that is
believed safe. A const rather than a config attribute until there is evidence it needs
tuning.

## Design

### Config

```go
type SO101GripperConfig struct {
    Arm            string `json:"arm"`
    ServoID        int    `json:"servo_id,omitempty"`
    GripperType    string `json:"gripper_type,omitempty"`
    MeshDetail     string `json:"mesh_detail,omitempty"`
    ArticulatedJaw bool   `json:"articulated_jaw,omitempty"`  // new, default false
}
```

Same name and semantics as the simulated gripper's attribute. Passed as
`BuildGripperModel`'s fourth argument at `gripper.go:142`.

### Position cache

State on `so101Gripper`, guarded by the existing `g.mu`:

```go
jawPct       float64   // last successful reading
jawReadAt    time.Time // when it was taken
jawHaveRead  bool      // false until the first success
```

- `positionPercentCached(ctx)` — returns `jawPct` when `jawHaveRead` and the reading is
  within `jawCacheTTL`; otherwise reads through `positionPercent`, stores, returns. On a
  read error: return `jawPct` with a warning if `jawHaveRead`, else return the error.
- `invalidateJawCache()` — clears freshness (not `jawHaveRead`). Called by `Open`, `Grab`,
  `moveToPercent`, and `GoToInputs`.

Invalidation belongs in `moveToPercent` rather than at each call site, so a future command
path cannot forget it.

### `IsHoldingSomething`

Currently a stub returning `gripper.HoldingStatus{}` — it always reports "not holding",
which makes decision 1 unimplementable and is independently wrong.

`Grab` already carries the real heuristic: `currentPercent - g.closedPosition > 15.0` means
the jaw stopped short of closed, so something is between the fingers. Extract that literal
as `graspThresholdPct = 15.0` and have both use it:

```go
func (g *so101Gripper) IsHoldingSomething(ctx, extra) (gripper.HoldingStatus, error) {
    pct, err := g.positionPercentCached(ctx)
    if err != nil {
        return gripper.HoldingStatus{}, err
    }
    return gripper.HoldingStatus{
        IsHoldingSomething: pct-g.closedPosition > graspThresholdPct,
    }, nil
}
```

This is a scope addition beyond the jaw work, justified because decision 1 depends on it
and because the method currently lies to every caller.

### `CurrentInputs`

```go
func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
    if !g.articulatedJaw {
        return nil, errors.ErrUnsupported
    }
    pct, err := g.positionPercentCached(ctx)
    if err != nil {
        return nil, err   // only reachable before the first successful read
    }
    return []referenceframe.Input{geometry.JawRadiansFromPct(pct)}, nil
}
```

### `GoToInputs`

Mirrors the simulated gripper's implementation, which is already reviewed and
mutation-tested. In order:

1. `errors.ErrUnsupported` when the flag is off.
2. **Validate every step before moving any** — arity against `len(g.model.DoF())`, and
   bounds against the joint limits widened by `jawLimitEpsilon`. A batch that fails halfway
   leaves the jaw somewhere the planner did not intend. (rdk rejects an input exceeding a
   limit by even one ULP, hence the epsilon.)
3. **Reject if holding** — call `IsHoldingSomething`; on true, return an error naming both
   facts: the gripper is holding something, and the planner asked to move the jaw.
4. Clear the `stopped` flag once per batch — not per step, so a mid-batch `Stop` cannot be
   forgotten by the next step.
5. Drive each step through `moveToPercent(geometry.JawPctFromRadians(θ))`, awaiting servo
   settle only on the **final** step. Intermediate targets are waypoints the servo passes
   through anyway; awaiting each would cost a full `gripperSettleTimeoutMs` window per step.
6. Check `stopped` between steps and return an error if set, so the motion service learns
   the trajectory did not complete.

### `Stop`

Sets the `stopped` flag while moving, mirroring `components/simulated/simulated.go:443-453`
("Only set stopped while moving, otherwise the distinction between 'reached the goal' and
'was stopped' is lost").

## Testing

All through the existing `fakeServoArm` harness — no hardware.

| Test | Asserts |
|---|---|
| `TestArticulatedJawConfigHardware` | default false ⇒ 0-DoF model and `ErrUnsupported` from both input methods; true ⇒ 1-DoF and working `CurrentInputs` |
| `TestJawCacheBoundsBusReads` | N rapid `CurrentInputs` calls inside the TTL produce exactly **one** `servo_position` command on the fake arm |
| `TestJawCacheInvalidatedByWrites` | a write path forces the next read to hit the bus |
| `TestCurrentInputsSurvivesReadFailure` | fake arm fails a read ⇒ last-known-good returned, no error; and errors when it has never succeeded |
| `TestGoToInputsRejectsWhileHolding` | fake arm reports a held object ⇒ error, and **no** `servo_move` command issued |
| `TestGoToInputsValidatesBeforeMoving` | a batch whose second step is invalid issues no `servo_move` at all |
| `TestGoToInputsAwaitsOnlyFinalStep` | an N-step batch issues N `servo_move` but only one `servo_wait_stop` |
| `TestIsHoldingSomethingReportsGrasp` | above/below `graspThresholdPct` both directions |
| `TestGoToInputsAbortsOnStop` | `Stop` mid-batch ⇒ error, remaining steps not issued |

Every guard must be **mutation-proven**: break it, show the test fails with the intended
diagnostic, revert. A green test is not evidence it would catch anything — established
during the simulated work, where a pre-existing guard on the shipping gripper's frozen jaw
angle turned out to catch nothing.

## Out of scope

- **Making it the default.** That needs the cache and rejection behaviour exercised on real
  hardware first.
- **Torque- or current-based grasp detection.** The position heuristic is what exists and
  what `Grab` already trusts.
- **Changing `Geometries()`.** It keeps its own `jawAngle` fallback-to-closed, which is
  correct for a mesh.
- **The leader gripper.** It has no jaw; the flag applies but is not useful.

## Risks

| Risk | Mitigation |
|---|---|
| Stale geometry within the TTL window | invalidation on every write; 50 ms; const so it is one place to change |
| One dropped serial read aborts every arm move | last-known-good; error only before the first success |
| Planner loosens a grip mid-move | rejection in `GoToInputs`, guarded by `TestGoToInputsRejectsWhileHolding` |
| Arm move fails confusingly while carrying a part | error names both facts; documented in `docs/gripper.md` |
| Bus saturation from viewer polling | `TestJawCacheBoundsBusReads` pins the read count |
| `Stop` silently advancing a batch | `stopped` flag + `TestGoToInputsAbortsOnStop` |

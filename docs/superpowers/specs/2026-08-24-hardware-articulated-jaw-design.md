# Opt-in articulated jaw on the hardware SO-101 gripper

**Date:** 2026-08-24
**Status:** Design, approved for planning
**Scope:** `components/gripper`, `components/simulated` and `internal/geometry` (relocating
`jawLimitEpsilon`), `docs/gripper.md`, `CLAUDE.md`

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

### 1. Reject planner jaw commands while holding — but only ones that MOVE the jaw

`GoToInputs` returns an error if the gripper has a grip **and** the batch actually asks the
jaw to move beyond `jawHoldToleranceRad`. A batch whose every step is already within that
tolerance of the current angle is a no-op: return `nil` without issuing any servo command.

**The scoping is not optional.** `services/motion/builtin/builtin.go:686-688` skips a
component in a trajectory step only when `len(inputs) == 0` — never when the inputs are
*unchanged*. A 1-DoF gripper appears in every step of every plan (its frame has DoF, so
`LinearInputs`' schema includes it), carrying its current jaw value. `TestPlannerJawTravel`
shows this directly: the `direct` scenario's trajectories carry a `gripper` column in every
step with `0.0000` travel. An unconditional rejection would therefore fail **every arm move
made while holding a part** — it would break pick-and-place, the main reason to have a
gripper at all.

Skipping no-op batches also removes write-side bus load: without it, every arm plan issues
a `servo_move` per step on the same 1 Mbaud bus the arm is using.

**"Current angle" means a fresh `positionPercentCached` read at execute time, not the value
the plan started from.** That matters: if the servo's reported position drifts more than the
tolerance between plan time and execute time — plausible under load with a part clamped
between the fingers — a genuine no-op batch is misclassified as jaw-moving and rejected,
resurrecting the pick-and-place regression intermittently. The fake arm reports an exact
percent, so no unit test can catch this. Mitigation: log at info level whenever a batch is
classified as jaw-moving while holding, including the current and requested angles, so the
first hardware run shows whether 0.01 rad is the right number.

`jawHoldToleranceRad = 0.01` rad (~0.6°). The jaw's range is 1.92 rad and the planner's
measured travel under obstruction is 0.083-0.383 rad, so this cleanly separates "no-op plus
float noise" from "the planner genuinely wants the jaw somewhere else". It is deliberately
much looser than `jawLimitEpsilon` (1e-6), which exists for a different purpose.

Rejected alternatives: *clamp so the jaw never opens further* — the executed trajectory then
silently diverges from the planned one, so the collision geometry the planner reasoned about
no longer matches reality; and *allow with a warning* — which will physically drop parts.

Accepted cost: an arm move that genuinely needs jaw motion fails while carrying a part. The
error must name both facts — the gripper is holding something, and the planner asked to move
the jaw — and say which angle it asked for.

The rejection is scoped to `GoToInputs` only. `Open`, `Grab` and the `set_position`
DoCommand are unaffected, so a user can always release a held part.

**Two consequences that follow from this and should not be "fixed" without thought.** First,
the dominant measured case is an out-and-back batch (8/10 seeds return near their start).
That is *not* a no-op — an intermediate step exceeds tolerance — so while holding it is
rejected despite net jaw motion of ~zero. That is intended: the planner did ask to open the
jaw mid-trajectory. Do not change step 3 to compare only the final step.

Second, issuing only the final target (step 6) means the executed jaw path diverges from the
planned mid-trajectory geometry — the same objection used above to reject *clamping*. The
difference is that a position-controlled STS3215 never traverses superseded waypoints, so
that divergence exists whether or not the intermediate targets are sent; clamping would have
additionally changed the *endpoint*. Sending them would cost N bus writes for no change in
behaviour.

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

So: cache the reading for `jawCacheTTL = 50ms`, invalidated by every *mutating* bus command.

**Invalidation goes in `servoDo`, not `moveToPercent`.** `servoDo` is the single choke point
through which every bus command passes; invalidating there catches paths `moveToPercent`
does not. One such path exists already: `set_position`'s raw form (`gripper.go:396-398`)
calls `servoDo(CmdServoMove, {"raw": ...})` directly. `CmdServoStop` likewise halts the jaw
mid-travel. Invalidate on any command that is not a read.

**Do not cache a reading taken while the jaw is moving.** `moveToPercent` returns when
`servo_wait_stop` reports settled *or* when it times out at `gripperSettleTimeoutMs = 2000`;
after a timeout the servo may still be moving, so a reading captured then would be frozen for
a full TTL. Skip storing whenever `isMoving` is set.

**Back-driving is bounded, not unbounded.** With torque disabled the jaw is freely
back-drivable by hand while nothing writes to the bus (`services/teleop`'s `SetTorqueEnable`
acts on the whole 6-servo group). This is *not* a special case: `jawCacheTTL` expires on wall
clock regardless of writes, so such a reading is stale for at most 50 ms like any other.
Invalidation only ever shortens staleness; it never extends it. Recorded so nobody engineers
around a non-problem or infers an exception to the TTL that does not exist.

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

**Locking discipline is load-bearing here and must not reuse `g.mu`.** `sync.Mutex` is not
reentrant, and `Open` (`:249`), `Grab` (`:266`) and both `DoCommand` write paths (`:359`,
`:388`) already `g.mu.Lock()` at entry *before* calling `moveToPercent`. Invalidating under
`g.mu` from inside that call would deadlock on the very first `Open()`.

So the cache gets its own `jawMu sync.Mutex`, guarding only:

```go
jawPct      float64   // last successful reading
jawReadAt   time.Time // when it was taken
jawHaveRead bool      // false until the first success
```

`jawMu` is **never held across a `DoCommand`**. `positionPercentCached` takes it, checks
freshness, releases; does the bus read unlocked; re-acquires only to store. Two concurrent
callers may both read the bus — acceptable, and far better than serializing a viewer poll
behind an `Open` that is sitting in `servo_wait_stop` for up to 2000 ms. (Today's `jawAngle`
/ `Geometries` / `get_position` take no lock at all, so this preserves that property.)

- `positionPercentCached(ctx)` — returns `jawPct` when `jawHaveRead` and the reading is
  within `jawCacheTTL`; otherwise reads through `positionPercent`. On success, store unless
  `isMoving` is set. On error: return `jawPct` with a warning if `jawHaveRead`, else return
  the error.
- `invalidateJawCache()` — clears freshness, not `jawHaveRead`, and bumps a generation
  counter. Called from `servoDo` for every non-read command.
- **A generation counter is required, not optional.** Because `jawMu` is released across the
  bus read, an invalidation landing inside that window would otherwise be overwritten by the
  in-flight pre-write value and stamped fresh — 50 ms of confidently-wrong geometry, exactly
  what this decision exists to prevent. Capture the generation before the read; at store time,
  discard the result if it changed. The `isMoving` store-guard covers `Open`/`Grab`/
  `GoToInputs`/`set_position` (all set it before `servoDo`) but **not `Stop`**, which clears
  `isMoving` at `gripper.go:302` *before* issuing `servo_stop` at `:303`.
- **Prime the cache with one read at construction.** The constructor already probes servo
  capabilities; adding a position read closes the "error before the first successful read"
  window, which otherwise means the first frame-system sweep after a reconfigure on a flaky
  bus hard-errors the whole machine. A failed prime is logged, not fatal.

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
3. **Detect a no-op batch.** If every step is within `jawHoldToleranceRad` of the current
   angle, return `nil` immediately, issuing no servo command. This is the common case — the
   gripper appears in every trajectory step of every plan — so it must be cheap and must not
   reject.
4. **Reject if holding**, now that we know the batch really moves the jaw: call
   `IsHoldingSomething`; on true, return an error naming the held object and the requested
   angle.
5. `g.isMoving.Store(true)` with a deferred `false`, and clear `stopped` **once per batch**
   (not per step, so a mid-batch `Stop` cannot be forgotten by the next step). Setting
   `isMoving` is required, not incidental: `Stop` only latches `stopped` while moving, so
   without it `TestGoToInputsAbortsOnStop` would pass vacuously.
6. **Issue only the final step's target.** Unlike the simulated interpolator, a
   position-controlled STS3215 simply tracks the most recent command — each `servo_move`
   supersedes the last, so intermediate targets are never traversed. Sending them would spend
   N bus writes to no effect, on the bus the arm is actively using. Await settle once.
7. Check `stopped` before and after the move and return an error if set, so the motion
   service learns the trajectory did not complete.

### `Stop`

Sets the `stopped` flag while moving, mirroring `components/simulated/simulated.go:443-453`
("Only set stopped while moving, otherwise the distinction between 'reached the goal' and
'was stopped' is lost").

## Testing

All through the `fakeServoArm` harness — no hardware. **Two harness changes are needed and
must be budgeted, contrary to "no changes required":**

- `newTestGripper` (`gripper_test.go:151-163`) hard-codes `SO101GripperConfig{Arm: ...}` with
  no way to set `ArticulatedJaw`. It needs to take the config, or gain a variant that does.
- `jawCacheTTL` as a bare const with no clock seam forces real 50 ms sleeps to test expiry.
  Prefer an injectable `now func() time.Time` on the struct, defaulting to `time.Now`, so the
  cache tests are deterministic rather than timing-dependent.

Failing a read is already expressible (`fa.doErr`, which exempts `servo_capabilities` so
construction still succeeds); reporting a held object is expressible via `fa.percent`.

| Test | Asserts |
|---|---|
| `TestArticulatedJawConfigHardware` | default false ⇒ 0-DoF model and `ErrUnsupported` from both input methods; true ⇒ 1-DoF and working `CurrentInputs` |
| `TestJawCacheBoundsBusReads` | N rapid `CurrentInputs` calls inside the TTL produce exactly **one** `servo_position` command on the fake arm |
| `TestJawCacheInvalidatedByWrites` | a write path forces the next read to hit the bus |
| `TestCurrentInputsSurvivesReadFailure` | fake arm fails a read ⇒ last-known-good returned, no error; and errors when it has never succeeded |
| `TestGoToInputsNoOpBatchIssuesNoCommand` | a batch matching the current angle returns nil and issues **no** `servo_move` — the pick-and-place regression guard |
| `TestGoToInputsRejectsWhileHolding` | fake arm reports a held object ⇒ error, and **no** `servo_move` command issued |
| `TestGoToInputsValidatesBeforeMoving` | a batch whose second step is invalid issues no `servo_move` at all |
| `TestGoToInputsIssuesOnlyFinalTarget` | an N-step batch issues exactly **one** `servo_move` (the last step's target) and one `servo_wait_stop` |
| `TestIsHoldingSomethingReportsGrasp` | above/below `graspThresholdPct` both directions |
| `TestGoToInputsAbortsOnStop` | `Stop` mid-batch ⇒ error; and `isMoving` is set during the batch, so the flag can latch at all |
| `TestStopDuringReadDoesNotStampStaleValue` | an invalidation landing mid-read is discarded by the generation check |
| `TestOpenDoesNotDeadlock` | `Open`/`Grab` complete with cache invalidation wired in — the reentrancy guard |
| `TestRawSetPositionInvalidatesCache` | the `servo_position` raw path invalidates, proving `servoDo` is the right choke point |

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
| Rejection breaks every plan while holding | scoped to batches that actually move the jaw; `TestGoToInputsNoOpBatchIssuesNoCommand` |
| Arm move that genuinely moves the jaw fails while carrying | error names the held object and the requested angle; documented in `docs/gripper.md` |
| Cache invalidation deadlocks against `g.mu` | separate `jawMu`, never held across a `DoCommand`; `TestOpenDoesNotDeadlock` |
| A write path bypasses invalidation | invalidate in `servoDo`, the single bus choke point; `TestRawSetPositionInvalidatesCache` |
| Jaw back-driven with torque off | documented, not engineered around — a torque-disabled arm is not under planner control |
| Bus saturation from viewer polling | `TestJawCacheBoundsBusReads` pins the read count |
| `Stop` silently advancing a batch | `stopped` flag + `TestGoToInputsAbortsOnStop` |

# Gripper as an explicit dependency of the arm

**Date:** 2026-08-17
**Status:** Approved design, pending implementation plan
**Affects:** `devrel:so101:gripper`, `devrel:so101:arm`, `devrel:so101:calibration`, `devrel:so101:discovery`

## Problem

Three resources — the arm, the gripper, and the calibration sensor — independently acquire
a "shared" serial controller from a global, port-keyed registry (`registry.go`,
`manager.go`). The sharing is broken in six distinct ways.

**1. The shared mutex is not shared.** `getExistingController` (`registry.go:98`) returns a
brand-new `SafeSoArmController` struct that copies `bus`, `group`, and `calibratedServos`
but gets a zero-value `mu`. Each resource locks its own mutex. Nothing mutually excludes
them; only the feetech bus's internal lock serializes individual transactions.

**2. Calibration state forks.** `arm.go:963` calls `SetCalibration`, which updates the arm's
copy of `s.calibration` *and* the shared `calibratedServos`. The gripper's writes
denormalize against its own now-stale `s.calibration` (`manager.go:97`), while its reads
normalize against the shared per-servo calibration (`manager.go:159`). After a
recalibration the gripper reads with the new calibration and writes with the old one.

**3. Unsynchronized read.** `GetJointPositionsForServos` reads
`s.calibratedServos[servoID].calibration` directly (`manager.go:159`) without taking
`CalibratedServo.mu`, racing `UpdateCalibration`.

**4. Release is a no-op.** `trackCaller` keys `callerPorts` by the PC inside
`GetSharedControllerWithCalibration`; `releaseFromCaller` looks that map up by the PC inside
the caller's `Close`. Those PCs can never be equal, so `ReleaseController` never runs. The
refcount only grows and the bus is never closed. `ForceCloseSharedController` and
`GetSharedController` have no callers at all.

**5. Config triplication.** `port`, `baudrate`, and `timeout` must match byte-for-byte
across all three resources or `getExistingController` fails with a "conflict" error. Worse,
whichever resource is constructed first decides whether calibration is loaded from file or
read from the servo registers, so startup behavior depends on config order.

Beyond the registry, two ownership bugs follow from the gripper holding a full controller:

- `gripper.Stop()` calls `controller.Stop()`, which iterates **every** `calibratedServo` and
  zeroes its velocity. Stopping the gripper kills an in-flight arm move.
- The gripper encodes a percentage into a pseudo-radians value purely to pass it through
  `MoveServosToPositions`, which converts it straight back to a percentage
  (`manager.go:88-92`). The round trip exists only to reuse an arm-shaped API.

**6. The calibration sensor bypasses the controller entirely on its hot paths.** It reaches
past `SafeSoArmController` in **eight** places, taking no controller mutex at all, so those
calls are unserialized against every other resource even in principle:

| Site | Call | Purpose |
|---|---|---|
| `calibration.go:463`, `:565` | `bus.SyncRead` | present positions |
| `:468`, `:570` | `bus.Protocol` | decode the `SyncRead` byte slices |
| `:1103`, `:1165` | `bus.Discover` | full-bus ID sweep |
| `:1045` | `calibratedServos[id].Ping` | `motorSetupVerify` |
| `:1193` | `calibratedServos[id]` → `Ping`/`SetID`/`SetBaudRate` | `assignMotorIDAndBaudrate` |

Site `:1193` is the `SetID`/`SetBaudRate` pair that the entire read blockade below is
justified by. Leaving any of these reaching into the struct would mean the handle is not the
only path to the bus and the lease is not actually total.

Finally, **the arm and the calibration sensor must never drive the bus at the same time**,
and nothing today enforces that. Per-transaction serialization is insufficient: the
calibration workflow holds meaning across many seconds and many transactions.

## Enabling fact: local module dependencies are real Go objects

`rdk@v1.0.0/module/resources.go:298-304`:

> If the dependency is local to this module, add the resource object directly, rather than
> a client object that talks with the viam-server.

So when the gripper declares the arm as a dependency and both are served by this module,
`arm.FromDependencies` yields the actual `*so101` pointer. `DoCommand` between them is a
direct Go method call — no gRPC, no proto conversion, no serialization cost. The same code
still works if the arm is remote, at the cost of a real round trip.

This is the mechanism the ufactory module relies on.

## Design

### Ownership

```
          registry (port-keyed, ref-counted)
            │  ONE *SafeSoArmController
            │  hard lease (motor_setup_*, seconds)
            │  motion hold (calibration workflow, minutes-hours)
            │                    │
   handle ──┤                    ├── handle
            ▼                    ▼
      so101 (arm)         so101CalibrationSensor
            │  ▲
DoCommand ──┘  │  arm.Arm from resource.Dependencies
               │
         so101Gripper   ← owns no bus, no port, no calibration
```

The arm and the calibration sensor share exactly one `*SafeSoArmController` — one mutex, and
after the collapse below, one calibration. The gripper owns nothing on the bus; every
servo-6 operation is a `DoCommand` on its `arm.Arm` dependency, so it inherits both tiers of
exclusivity without knowing they exist.

The calibration sensor deliberately stays on the registry rather than depending on the arm.
Proxying its full surface (homing, range recording, `SetID`, `SetBaudRate`) through
`DoCommand` would be a large protocol for little gain, and it keeps the calibration wizard
configurable without an arm.

### Registry: shared pointer, identity-carrying handles, two-tier exclusivity

```go
// one per port — the shared bus
type ControllerEntry struct {
    controller *SafeSoArmController          // ONE mutex, ONE calibration
    exclusive  busLease                      // HARD: total lockout, motor_setup_* only
    motionHold motionHold                    // SOFT: refuses motion, reads pass through
    handles    map[*ControllerHandle]struct{} // preempt phase needs "every OTHER handle"
    refCount   int64                          // live handles
    pins       int64                          // in-flight acquires; teardown waits on BOTH
    // ...
}

type busLease struct {                        // hard, command-scoped (seconds)
    state  leaseState                          // free | acquiring | held
    holder *ControllerHandle                   // identity is the POINTER, never resource.Name
}

type motionHold struct {                      // soft, workflow-scoped (minutes to hours)
    holder *ControllerHandle
    reason string                              // surfaced in ErrCalibrationInProgress
}

// one per resource — the ONLY way to reach the bus
type ControllerHandle struct {
    entry    *ControllerEntry
    owner    resource.Name       // DIAGNOSTICS ONLY — never an identity key
    preempt  func(context.Context) error
    released atomic.Bool         // read by hook invocation; see teardown rule 3
    once     sync.Once
}

// lifecycle — the preemption hook is supplied at acquire time, not set afterwards, so
// there is no window in which a registered handle can be missed by the preempt phase.
func AcquireController(
    owner resource.Name, config *SoArm101Config,
    calibration SO101FullCalibration, fromFile bool,
    preempt func(context.Context) error,     // nil for resources that need no preemption
) (*ControllerHandle, error)                 // replaces GetSharedControllerWithCalibration
func (h *ControllerHandle) Release()         // see "Handle teardown" below

// exclusivity — both return a release closure
func (h *ControllerHandle) AcquireExclusive(ctx context.Context) (release func(), err error)
func (h *ControllerHandle) AcquireMotionHold(ctx context.Context, reason string) (func(), error)

// bus surface — EVERY method is classified read or motion (see "Two-tier exclusivity")
func (h *ControllerHandle) MoveServoPercent(...) error                   // motion
func (h *ControllerHandle) StopServo(ctx context.Context, id int) error  // motion
func (h *ControllerHandle) ServoPositionPercent(...) (float64, int, error) // read
func (h *ControllerHandle) SyncReadPositions(...) (map[int]int, error)   // read
func (h *ControllerHandle) Discover(ctx context.Context) ([]int, error)  // read
// ... one method per operation the arm and calibration sensor need
```

**The handle carries the full bus surface, and `SafeSoArmController` is reachable only
through it.** This is the answer to "where is exclusivity enforced": on every handle call,
not once at acquire time. It has to be, because every consumer today resolves a controller
once and caches it for its lifetime — `s.controller` (`arm.go:500`), `cs.controller`
(`calibration.go`), and `controllerManualIO.controller` captured at manual-mode entry
(`manual_mode.go:219`). A check that ran only at acquire time would be bypassed by all
three. Caching a *handle* is safe precisely because the handle re-checks per call.

So every `*SafeSoArmController` field in the codebase becomes a `*ControllerHandle`, and all
eight of the calibration sensor's reaches past the controller (problem 6) move onto handle
methods. That is real added scope, but it is not optional: those call sites are exactly the
ones exclusivity exists to govern, and routing them through the handle also puts them behind
the shared mutex for the first time.

`bus.Protocol()` (`calibration.go:468`, `:570`) gets **no** handle equivalent — adding a
passthrough accessor would re-leak the bus and defeat the point. It exists only to
`proto.DecodeWord` the `SyncRead` byte slices into ints, so that decode moves *inside*
`SyncReadPositions`, whose signature already returns `map[int]int`. `Protocol()` disappears
from the sensor entirely.

Handles wrap a **pointer** to the one shared controller rather than copying its fields into
a fresh struct with a fresh mutex, so problem 1 is fixed.

**Identity is the handle pointer, never `resource.Name`.** `resource.Name` is stable across
an `AlwaysRebuild` cycle, so a rebuilt sensor would otherwise impersonate its own
predecessor — its acquire against a hold still recorded to the dead instance would look like
the same owner re-acquiring, and its release would target a record it does not own. `owner`
survives only for log lines and error messages.

### Collapsing the controller's two calibration copies

Sharing the controller pointer does **not**, on its own, fix problem 2 — it relocates it.
`SafeSoArmController` holds calibration twice:

| Copy | Read by | Written by |
|---|---|---|
| `s.calibration` | every **write** path (`manager.go:64`, `:97`, `:124`, `:134`), `getCalibrationForServo` (`:358`), `GetCalibration` (`:351`, consumed by `arm.calculateJointLimits` at `arm.go:376`) | `SetCalibration` only (`:329-349`) |
| `calibratedServos[id].calibration` | the **read** path (`manager.go:159`) | `SetCalibration` *and* the registry's acquire-time update (`registry.go:72-93`) |

The registry's acquire-time update touches `calibratedServos` and `entry.calibration` but
**never** `controller.calibration`. Today each caller gets its own struct copy, which masks
the divergence. Sharing one pointer *unmasks* it: a second `AcquireController` with
`fromFile=true` and a different calibration leaves writes denormalizing against a stale
`s.calibration` while reads normalize against a fresh per-servo one — problem 2's exact
shape, now with poisoned joint limits as a bonus. This is not an edge case:
`discovery.go:179-181` emits `calibration_file` on both the arm and the sensor config, so a
second acquire with `fromFile=true` is the *normal* configuration.

**Fix: collapse to one copy.** `s.calibration` is derivable from the per-servo calibrations,
so delete it and have `getCalibrationForServo`/`GetCalibration` read through
`calibratedServos[id].Calibration()`. The registry's acquire-time update then has exactly
one place to write, and no path can observe a half-updated calibration. If collapsing proves
invasive, the fallback is to route the acquire-time update through `SetCalibration` — but
that leaves two copies and one more chance to desynchronize, so prefer the collapse.

### Handle teardown

`Release()` is where exclusivity state and resource lifecycle meet, and both the arm and the
calibration sensor are `resource.AlwaysRebuild` (`arm.go:201`, `calibration.go:109`) — so a
reconfigure *during* calibration runs `Close` → `Release` on a handle that currently holds
the lease or the motion hold. Three rules, all necessary:

1. **`Release()` drops any lease or hold the handle owns or is acquiring**, and removes the
   handle from `entry.handles`. Otherwise the hold outlives its owner, and with no watchdog
   in the split design nothing recovers it.
2. **An in-flight acquire pins the entry against teardown, via a count separate from
   `refCount`.** Without the pin, a concurrent `Release()` taking the refcount to zero
   deletes the entry and closes the bus while hooks are still running, and the commit phase
   lands on freed memory. Since the pin and the refcount then both gate the same moment,
   ownership of the deferred teardown must be explicit: **whichever of `refCount` and `pins`
   decrements to zero last performs the teardown.** `Release()` is idempotent via `once`, so
   the releasing handle structurally cannot come back and finish the job — the deferred close
   is an entry-level responsibility. This also covers the acquirer being its own last
   releaser: it decrements `refCount`, then `pins`, and tears down on the second.
3. **Hook invocation checks a per-handle `released` flag at call time; hooks are
   contractually safe to invoke on a closing resource.** Deregistration alone cannot work:
   the preempt phase runs without the registry lock, so it iterates a *snapshot* of
   `entry.handles`, and removing a handle from the live map cannot retract it from a snapshot
   already taken. Rather than block `Release()` on in-flight preempts (which reintroduces the
   lock-ordering hazard the phase split exists to avoid), the invocation site skips any handle
   whose `released` flag is set, and hooks tolerate being called on a resource whose `Close`
   has begun. This matches the hook's own semantics — "exited manual mode" is an idempotent
   safe state, so a late or redundant call is harmless.

**The sensor's recording goroutine must be joined.** `so101CalibrationSensor.Close`
(`calibration.go:1227-1243`) cancels `recordingCtx` but never waits for `recordPositions`
(`:544-635`), a 10 ms ticker calling `SyncReadPositions`. There is no `done` channel. Today
this is latent because `ReleaseSharedController()` is a no-op and the bus is never closed
(problem 4). *Making `Release()` work is what makes it reachable*: a reconfigure or removal
during `StateRangeRecording` would close the serial port under a live goroutine mid-read. So
`recordPositions` gains a `done` channel, and `Close` and `stopRangeRecording`
(`calibration.go:645-648`) both join it — the same audit manual mode already passes.

### Two-tier exclusivity

The arm and the calibration sensor must not drive the bus concurrently, but the two hazards
have different shapes and different durations, and applying one mechanism to both is what
made the earlier single-lease design collapse under its own machinery.

**Hard lease — total bus lockout, `motor_setup_*` only.** Justified solely by
`assignMotorIDAndBaudrate` (`calibration.go:1191-1224`), where `SetID` and `SetBaudRate` land
back to back: a *read* arriving between them addresses a servo that just changed identity, so
here reads genuinely must be blocked too. `motorSetupDiscover` sweeps every ID on the bus and
needs the same. These are bounded operations lasting seconds, each contained within a single
DoCommand, so the lease is acquired and released inside a `withLease` helper via `defer`.

Because it is command-scoped rather than workflow-scoped, most of the machinery evaporates:

- **No watchdog.** The lease cannot outlive its command, so there is nothing to time out.
- **No revocation**, and therefore no state-machine desynchronization to repair.
- **No re-entrancy.** `cs.mu` serializes DoCommands and no `motor_setup_*` command invokes
  another, so nesting does not arise. (If that ever changes, the release-closure signature
  already leaves room for a depth counter.)

What survives is **preemption**, because manual mode may be running when motor setup starts.

**Soft motion hold — arm refuses motion, reads keep working.** Held whenever the sensor's
`state != StateIdle`; taken on the first workflow command, released on any transition back to
`StateIdle` (which includes `resetCalibration`, `calibration.go:803-825`, not just save and
abort). This covers the calibration state machine, which spans minutes to hours of human
work: `StateStarted` waits for the user to hand-position the arm, `StateHomingPosition` waits
for them to press a button, `StateRangeRecording` waits while they walk six joints through
their ranges, and `StateCompleted` waits for Save.

The hold blocks exactly what needs blocking — an arm move corrupting a homing offset or a
range recording — while leaving `JointPositions`, `EndPosition`, `Geometries`, and
`CurrentInputs` live. Motion calls return `ErrCalibrationInProgress{Holder, Reason}`.

Scoping it this way is what makes the design tractable. A total lockout held across the whole
workflow would need a liveness signal to distinguish "user is thinking" from "wizard was
abandoned" — and no such signal exists: three of the five non-idle states (`StateStarted`,
`StateHomingPosition`, `StateCompleted`) generate neither bus traffic nor DoCommands while the
human works. A five-minute timer would revoke during `StateCompleted` and discard the ranges
just recorded. The motion hold sidesteps the question entirely: there is nothing to revoke,
because the failure mode of an abandoned wizard is a *soft* one.

**Accepted tradeoff: an abandoned wizard leaves the arm unable to move** until the sensor is
aborted, reset, or reconfigured. Reads keep working throughout, so the machine is not dark,
and `ErrCalibrationInProgress` names the holding sensor and the remedy. This is the price of
having no watchdog, and it is deliberately the trade the split makes: a recoverable soft
block beats a timer that can destroy recorded calibration data.

**Manual mode is refused while either is active.** `enter_manual_mode` returns an error
unless the hard lease is `free` *and* no motion hold is set. Without this the acquire race in
the next section stays open even after preemption.

### Preemption

Acquiring either the hard lease or the motion hold preempts, and preemption is a hook rather
than merely a stop. Stopping servos is not enough, because `enter_manual_mode` launches
`manualSession.loop()` (`manual_mode.go:118`) — a ticker-driven goroutine that keeps reading
positions and writing goals until explicitly stopped. Zeroing velocities does not end it.

To be precise about the failure this prevents: `manualSession.stop()` is *already*
synchronous (`manual_mode.go:113-116` is `m.cancel(); <-m.done`, and `done` closes only after
the deferred `restoreCompliance` completes), so nothing there needs refactoring. The
stranding comes from the **error-exit** path — consecutive denied ticks make `loop()` return
*on its own*, at which point its `restoreCompliance` writes are denied too, leaving the arm
at reduced stiffness with no retry. The fix is about *ordering* preemption ahead of the
holder record, not about manual mode's lifecycle.

Acquisition runs in **three phases**:

1. **Claim** — atomically move the lease from `free` to `acquiring{handle}`. If it is already
   `acquiring` or `held` by a *different handle*, fail immediately. Compare handle
   **pointers**, never `resource.Name`. This closes the window where two acquirers could both
   see "no holder," both run hooks, and both record themselves.
2. **Preempt** — invoke every *other* handle's hook, **with no registry lock held**. The
   `acquiring` state already excludes other acquirers, so the lock buys nothing and costs
   correctness: `enter_manual_mode` takes the arm's `s.mu` (`arm.go:985-987`) and holds it
   through `enterManualLocked` → `manualSession.start()` → synchronous bus I/O, while the hook
   path runs registry → arm `s.mu`. Holding the registry lock across the hook would interlock
   those orders into an ABBA deadlock — the single most likely way this hangs on hardware. For
   the same reason, **the per-call check must be a lock-free atomic read** that can never block
   on an in-progress acquisition.
3. **Commit** — move `acquiring` to `held`.

**During `acquiring`, the check is acquirer-scoped, not open.** The obvious reading — that
`acquiring` blocks nobody, since hooks must be able to reach the bus — leaves a hole exactly
the size of the failure preemption exists to prevent: a concurrent `enter_manual_mode`
(`arm.go:984-987`) can land *after* the arm's hook has run and *before* commit, re-entering
manual mode behind preemption's back. The loop then accumulates denied ticks, exits on its
own, and strands stiffness. So during `acquiring`, permit the acquirer and any handle
currently executing its own hook, and deny everyone else. Combined with `enter_manual_mode`
refusing whenever the lease is not free, the window closes.

**The preempt phase carries a short deadline** (a few seconds). Hook failure is handled below,
but a hook that *hangs* rather than fails would strand the lease in `acquiring` with no
recovery short of restarting the module. Hook work is bounded in practice — manual-mode exit
is a handful of register writes across five servos — so the deadline costs nothing. A timeout
counts as a hook failure.

**If a hook fails, acquisition aborts** and returns an error naming the failing hook. A failed
hook means we could not put the arm into a safe state, and driving calibration against an arm
that is still actively writing is the worse outcome: an unstartable wizard is recoverable by
the user (stop the arm, exit manual mode, retry), while stranded stiffness is not. Hooks that
already succeeded are not rolled back — "exited manual mode" is an idempotent safe state.

**Long-running handle methods re-check per iteration.** The "checked on every call" guarantee
is per *call*, and some calls are not atomic: `WaitForServosToStop` (`manager.go:215-257`)
deliberately drops the lock and polls for up to `timeoutMs`, and `Ping` (`:269-279`) and
`SetTorqueEnable` (`:175-189`) are multi-transaction. These re-check inside their loops rather
than once on entry. This matters most for `servo_wait_stop`, which `Open`/`Grab` now use in
place of the 500 ms sleep.

**The gripper is covered for free.** Its only path to the bus is the arm, so it inherits both
tiers, and `Geometries()` already falls back to a closed jaw when a position read fails.

**Arm construction tolerates both.** Treating `ErrPortLeased` or `ErrCalibrationInProgress`
from `initializeServos()` as fatal would mean reconfiguring during calibration permanently
breaks the arm. It logs a warning and builds.

### The `servo_*` command family

New file `servo_commands.go` holds the command-name constants used by both sides. Dispatch
is a pure function over a small interface so it is testable without hardware:

```go
type servoOps interface {
    MoveServoPercent(ctx context.Context, id int, percent float64, speed int) error
    MoveServoRaw(ctx context.Context, id, raw int) error
    ServoPositionPercent(ctx context.Context, id int) (percent float64, raw int, err error)
    StopServo(ctx context.Context, id int) error
    WaitForServosToStop(ctx context.Context, ids []int, timeoutMs int) error
}

func handleServoCommand(
    ctx context.Context, cmd map[string]any, ops servoOps, ownServos []int,
) (map[string]any, error)
```

| Command | Request | Response |
|---|---|---|
| `servo_capabilities` | — | `{servo_ids: [6], servo_commands: true}` |
| `servo_move` | `servo_id`, `percent` *or* `raw`, optional `speed` | `{percent, raw}` |
| `servo_position` | `servo_id` | `{percent, raw}` |
| `servo_stop` | `servo_id` | `{}` |
| `servo_wait_stop` | `servo_id`, `timeout_ms` | `{}` |

**Numbers must be coerced on both sides — the local and remote paths do not agree.** A local
dependency hands over the real `*so101`, so `DoCommand` arguments stay Go-native (`int` stays
`int`). A remote arm goes through `arm.Client.DoCommand` → structpb, where **every** number
becomes `float64`. The existing codebase is uniformly `float64`-only
(`gripper.go:390`, `:436`, `:475`, `:495`; `arm.go:922`, `:1004`) precisely because those
paths had only ever crossed gRPC. So this design's claim that the same code works against a
remote arm is false unless `servo_id`, `raw`, `speed`, and `timeout_ms` on the request side
and `raw`/`servo_ids` on the response side accept both shapes.

This is a trap for the test plan specifically: a `fakeArm` exercises only the local shape, so
every test listed below would stay green while the remote path was broken. `handleServoCommand`
gets a `numArg(cmd, key)` helper accepting `int`, `int64`, and `float64`, and its table test
covers all three per numeric field.

Guards: `servo_id` must be in 1–6 and must **not** appear in the arm's own `servo_ids`.
Excluding the arm's own servos rather than hardcoding "6" lets a 4-DOF variant configured
with `servo_ids: [1,2,3,4]` legitimately host a gripper on servo 5.

New single-servo methods on `SafeSoArmController` — `MoveServoPercent`, `MoveServoRaw`,
`ServoPositionPercent`, `StopServo` — each take the shared mutex only for the duration of
the bus transaction. The percent path denormalizes through `cal.Denormalize` using the
servo's **own** `NormMode` — not a hardcoded `NormModeRange100`. `NormModeRange100` is
assigned by servo ID == 6 in every path (`config.go:67-70`, `:145-152`, `:348-352`, `:431`;
`calibration.go:722-726`), so hardcoding it would contradict this design's own servo-id
guard: the guard excludes the arm's `servo_ids` rather than hardcoding 6 precisely so a
4-DOF arm can host a gripper on servo 5 — whose calibration is `NormModeDegrees`. Hardcoding
Range100 would write a wrong raw value there, and `handleServoCommand`'s
`[RangeMin, RangeMax]` clamp would be derived from a Degrees calibration. Using the servo's
real `NormMode` keeps the 4-DOF justification honest. The pseudo-radians round trip on the
**write** side (`manager.go:88-92`) becomes unreachable and is deleted.

The **read** side keeps its gripper branch: the calibration sensor still calls
`GetJointPositionsForServos` with servo 6 (`calibration.go:836`). Removing that encoding is
a follow-up, not part of this change.

### Gripper

```go
type SO101GripperConfig struct {
    Arm         string `json:"arm"`                 // required; Validate returns []string{cfg.Arm}
    ServoID     int    `json:"servo_id,omitempty"`  // default 6
    GripperType string `json:"gripper_type,omitempty"`
    MeshDetail  string `json:"mesh_detail,omitempty"`
}
```

`Port`, `Baudrate`, `Timeout`, and `CalibrationFile` are removed. The struct holds an
`arm.Arm` instead of a `*SafeSoArmController`. The constructor resolves the dependency with
`arm.FromDependencies`, probes `servo_capabilities`, and verifies its `servoID` is offered —
so a misconfigured `arm` reference fails at construction rather than at first move.
`Close()` returns nil, since the gripper owns nothing.

Deleted from `gripper.go`: `percentToRadians`, `radiansToPercent`, `openPositionRadians`,
`closedPositionRadians`, and the `getCalibrationForServo` lookup in `set_position`'s
`servo_position` branch (now `servo_move` with `raw`). The gripper holds no calibration
state at all.

Behavior changes:

- `Stop()` issues `servo_stop` for servo 6 only, and no longer kills arm motion.
- `Open`/`Grab` issue `servo_move` then `servo_wait_stop`, replacing the fixed
  `time.Sleep(500 * time.Millisecond)`.

The gripper's own DoCommand surface is unchanged, since it is public API: `get`/`set`,
`get_position`, `set_position` (all three key forms), `controller_status`,
`calibrate_positions`, `set_motion_params`, `get_motion_params`.

Three response-shape subtleties to preserve deliberately rather than by accident:

- `get_position`'s `position_radians` becomes *derived* from percent (`(pct/100*2-1)*π`)
  rather than read as radians. Identical in practice, but it gets an explicit test.
- `controller_status` forwards to the arm's existing handler (`arm.go:897`), which returns
  `arm_servo_ids` where the gripper's returns `servo_id`. The gripper re-shapes the response
  to its documented keys rather than passing the arm's through.
- `set_position` with `servo_position` (raw ticks) currently converts to a percentage,
  clamps to [0,100], then re-denormalizes — which effectively clamps to the calibrated
  range. Routing it to `servo_move` with `raw` writes the value through unclamped.
  `handleServoCommand` clamps `raw` to the servo's calibrated `[RangeMin, RangeMax]` to
  preserve today's protection.

## Testing

`arm.Arm` as the seam makes the gripper unit-testable without hardware for the first time.

- **`fakeArm`** implementing `arm.Arm` and recording DoCommands. Covers: `Validate` requires
  `arm` and returns it as a dependency; probe failure at construction; the exact command
  sequence emitted per gripper operation; `Stop` is servo-6-scoped; percent clamping;
  `ErrPortLeased` propagates to callers; `Geometries()` falls back to a closed jaw when the
  position read fails.
- **`handleServoCommand`** table test against a fake `servoOps`: unknown command, missing
  `servo_id`, out-of-range `servo_id`, `servo_id` owned by the arm, `percent` vs `raw`,
  clamping.
- **Registry**: two acquisitions return the same pointer; `Release` decrements and closes
  the bus once both `refCount` and `pins` reach zero (see teardown rule 2 — the unqualified
  "closes at zero" is only true when no acquire is in flight); double `Release` is a no-op;
  `AcquireExclusive` locks out a second acquirer; a leased handle's read *and* motion calls
  fail with `ErrPortLeased`; release-then-reacquire.
- **Two-tier behavior**, the distinction the whole design rests on: under a motion hold,
  motion calls fail with `ErrCalibrationInProgress` while `ServoPositionPercent`,
  `SyncReadPositions`, and `Discover` still succeed; under the hard lease, both fail; the
  hold is released on *every* path back to `StateIdle` including `resetCalibration`;
  `enter_manual_mode` is refused while either tier is active.
- **Acquire concurrency**, which is where the subtle bugs live: a second acquirer is rejected
  during the `acquiring` phase, not just the `held` phase; preemption hooks run before the
  holder is recorded and the hook's own bus traffic succeeds; **a non-hook, non-acquirer
  motion call during `acquiring` is denied** (the `enter_manual_mode`-lands-mid-acquire hole);
  a failing hook aborts the acquisition and leaves the lease `free`; a hanging hook trips the
  preempt deadline and leaves it `free` rather than stuck in `acquiring`; long-running methods
  re-check per iteration, so a `servo_wait_stop` in flight when a lease is taken fails partway
  rather than running to completion. The ABBA ordering is worth a `-race` test that drives
  `enter_manual_mode` and `AcquireExclusive` concurrently in a loop.
- **Lifecycle**: `Release()` on a handle holding either tier frees it; `Release()` during an
  in-flight acquire does not tear the entry down under the running hooks; a hook target that
  calls `Release()` mid-preempt has its hook *skipped* via the `released` flag rather than
  invoked against a closing resource; teardown happens exactly once when the later of
  `refCount` and `pins` hits zero, including when the acquirer is itself the last releaser; a
  rebuilt handle with the *same* `resource.Name` as a stale holder does not inherit its
  predecessor's hold.
- **Calibration coherence**, guarding the collapse: after an acquire-time calibration update,
  a write and a read of the same servo agree, and `calculateJointLimits` sees the new values.
  This is the test that would have caught problem 2's relocation.
- **Goroutine join**: closing the sensor during `StateRangeRecording` does not close the bus
  under a live `recordPositions` — with `-race`, and with the bus close asserted to happen
  after the goroutine has exited.

`registry_test.go` needs rework, though less than an earlier draft of this spec claimed.
`registry.GetController` is called at exactly **four** sites (60, 115, 157, 396) and its
signature changes. `TestReferenceCountingLogic` and `TestCleanupOnZeroRefs` (173-245) do not
call it — they build `ControllerEntry{}` literals and poke `refCount`/`registry.entries`
directly, so they break on the struct gaining `exclusive`/`motionHold`/`handles`/`pins` and
on `ReleaseController` being replaced. `TestForceCloseController` (246-273) exercises
`registry.ForceCloseController`, a **method** distinct from the package-level
`ForceCloseSharedController` in the delete list; decide its fate explicitly rather than by
accident.

This narrows the gotcha recorded in `CLAUDE.md` about `NewSO101`'s wiring being guarded only
by review — the dispatch and gripper halves become covered. The hardware constructor itself
remains untested, since it still needs serial hardware.

## Migration

This is a breaking config change, but **`meta.json` has no version field** — it carries
`module_id`, `models`, `applications`, `entrypoint`, and `build` only. Versions come from the
GitHub release tag via `.github/workflows/deploy.yml` → `viamrobotics/build-action`, as
`CLAUDE.md` records. So the breaking-change signal is the **release tag and release notes**,
not a file edit. (Two earlier drafts of this spec said "major version bump in `meta.json`";
that instruction was unfollowable.)

- `discovery.go`: the generated gripper config gains `"arm": "so101-arm-<suffix>"` and drops
  `port` and `calibration_file`. It is emitted **only when `hasArm` is also true** — a
  gripper with no arm has nothing to depend on. When servo 6 answers and servo 1 does not,
  log why no gripper config was suggested.
- `README.md`: rewrite the gripper model section, document the `servo_*` commands, and
  describe the arm/calibration exclusivity and `ErrPortLeased` behavior.
- `setup-app`: mentions the gripper in prose only (`StepComplete.svelte`,
  `StepMotorSetup.svelte`, `StepMotorVerify.svelte`); a copy tweak at most.
- Existing gripper configs fail validation with a message naming the fix: set `arm` to the
  `devrel:so101:arm` component name and remove `port`.
- `registry_test.go:398` calls `registry.GetCurrentCalibration`; keep that method or update
  the test. (See Testing for the broader `registry_test.go` rework.)
- **Drive-mode double inversion is silently fixed.** `percentToRadians` and
  `radiansToPercent` each apply `if cal.DriveMode != 0 { negate }` (`gripper.go:571`,
  `:587`), but `MotorCalibration.Denormalize`/`Normalize` already apply drive mode
  (`calibrated_servo.go:86`, `:65`) — so the gripper inverts twice. This is inert today
  because every default and every sensor-written calibration uses `DriveMode: 0`
  (`calibration.go:715`), but deleting these helpers is a real behavior change for anyone
  with a hand-edited nonzero drive mode. Call it out in the release notes.

## Risks and accepted tradeoffs

1. **Reads keep working during calibration; motion does not.** This is what the two-tier
   split buys. Under the motion hold, `JointPositions`, `EndPosition`, `Geometries`, and
   `CurrentInputs` all continue to serve, so the 3D viewer, data capture, and the frame
   system are unaffected for the hours a calibration workflow can span. Total lockout is
   confined to the seconds-long `motor_setup_*` commands, where a brief error is tolerable.
   Had the lockout stayed workflow-scoped, the blast radius would have been much wider than
   "the 3D viewer": data capture on `JointPositions`/`EndPosition`, the frame system's
   `Geometries` call on every shaped resource (rdk `robot/impl/local_robot.go:1439`, which
   for the arm reaches the bus via `CurrentInputs`), and teleop, which would hit
   `maxConsecutiveErrors` and self-stop (`teleop.go:302-323`).
2. **Grab detection may need retuning.** `servo_wait_stop` replacing the 500 ms sleep changes
   the settle timing that Grab's 15% position-difference threshold was tuned against.
3. **The pseudo-radians read path survives** for the calibration sensor. Follow-up.
4. **`SafeSoArmController.logger` becomes first-creator-wins** once the struct is shared
   rather than copied per caller. Move it onto the entry, and log through per-resource
   loggers where the distinction matters.
5. **Hardware verification required** for mid-move preemption, the motion-hold transitions,
   and grab detection. None of these can be covered by the unit tests above.
6. **Two-arm (leader/follower) setups** use two ports, so two registry entries and two
   independent exclusivity states. Verify against a teleop config.
7. **Manual mode is the riskiest interaction.** Preempting a live admittance loop and
   restoring compliance before the lease closes is the one place where getting the ordering
   wrong leaves hardware in a bad state (reduced stiffness, no retry) rather than merely
   erroring. It deserves a dedicated hardware test: enter manual mode, start calibration,
   confirm compliance is restored and the loop has exited before the lease is held.
8. **Scope grew during review.** Routing all eight of the calibration sensor's reaches past
   the controller through handle methods is required for the lease to be meaningful, but it
   means this change touches the sensor's read and motor-setup paths, not just its
   lifecycle. If that proves larger than expected, the fallback is to land steps 1-2
   (registry + lease) as their own PR before the gripper work.
9. **A configured gripper can no longer come up when its arm fails to build.** This is the
   most user-visible consequence of the dependency and it follows directly from making `arm`
   required: rdk returns `DependencyNotReadyError` (`robot/impl/local_robot.go:778-786`) and
   the gripper is never constructed. The arm fails to build when `initializeServosWithRetry`
   exhausts its 3 attempts (`arm.go:517-520`) — and `controller.Ping` pings **all six**
   servos (`manager.go:269-279`), so a dead *gripper* servo already fails the arm — or when
   `motion.FromProvider` cannot resolve (`arm.go:482-490`), or when `use_urdf` is set without
   `VIAM_MODULE_ROOT` (`arm.go:330-332`). Today the gripper survives all of these
   independently. This is inherent to the design rather than a defect, but it must be in the
   README so the failure is diagnosable.
10. **Teleop needs attention the earlier drafts never gave it.** `setLeaderTorque`'s comment
   (`teleop.go:263-266`) justifies itself with "the leader arm + gripper share one
   controller" — a premise this design deletes. The behavior still holds (`SetTorqueEnable`
   → `group.EnableAll` covers all six servos) but for a different reason, so the comment must
   change or it will mislead. Teleop's 20 Hz `get_position`/`set_position` calls also become
   gripper→arm hops; direct Go calls, but worth confirming on hardware at rate. `README.md`'s
   teleop section describes gripper wiring and needs updating alongside.
11. **`arm.Stop()` still stops all six servos** (`manager.go:191-201`). This design fixes the
   gripper→arm direction of that defect and calls it an ownership bug; the arm→gripper
   direction is left as-is. Defensible — stopping the arm arguably *should* stop everything —
   but it should be a decision, not an oversight.
12. **An arm reconfigure builds the gripper twice.** `rebuildResource` cascades
   unconditionally (rdk `module/resources.go:594`) using the `conf` captured at the gripper's
   original `addResource`, so the gripper is rebuilt once from a possibly stale config before
   viam-server reconfigures it again with the current one. Two constructions and two
   `servo_capabilities` probes per arm reconfigure. Harmless given the probe is cheap and
   construction has no side effects on the bus, but it makes construction-time logging noisy
   and is worth knowing before someone debugs it.
13. **The lease is registry-scoped, not OS-scoped — discovery is a known non-goal.**
   `discovery.go:141` opens its own independent `feetech.NewBus` on the port and closes it,
   entirely outside the registry, so a discovery ping can still land mid-`SetID`. This is
   pre-existing and low-frequency (discovery runs on demand, not continuously), and closing
   it would mean routing discovery through the registry for ports it does not yet know are
   SO-101s. Recorded so it is a deliberate boundary rather than an oversight.

## Sequencing

1. **Registry and calibration collapse.** Shared pointer, `ControllerHandle` with the full
   bus surface, `AcquireController`, working `Release` with the `refCount`/`pins` split.
   Collapse the controller's two calibration copies. Convert `arm.go`, `calibration.go`, and
   `manual_mode.go` to store handles, including all eight call sites in the problem-6 table
   (`calibration.go:463`, `:468`, `:565`, `:570`, `:1045`, `:1103`, `:1165`, `:1193`),
   folding the `Protocol()` decode into `SyncReadPositions`. Add the `done` channel to
   `recordPositions` and join it in `Close` and `stopRangeRecording` — this must land in the
   *same* step as the working `Release()`, since that is what makes the use-after-close
   reachable.
2. **Two-tier exclusivity.** Motion hold wired to the sensor's state machine; hard lease
   wired to `motor_setup_*` via `withLease`; three-phase acquire with preemption hooks,
   handle-pointer identity, and the acquirer-scoped check during `acquiring`.
   `enter_manual_mode` refuses while either tier is active. Manual mode needs no lifecycle
   change — `stop()` is already synchronous.

   Three places where correct prose still admits an incorrect implementation, and which
   deserve the closest review at PR time: the **snapshot semantics in the preempt phase**
   (iterating `entry.handles` unlocked, with `released` checked at invocation rather than at
   snapshot), the **pin/refcount split** (teardown owned by whichever counter decrements
   last), and the **read/motion classification** of every handle method — a method filed
   under the wrong tier is a silent hole in exactly one direction.
3. Controller single-servo methods, `handleServoCommand` with numeric coercion, arm DoCommand
   wiring.
4. Gripper rewrite and the config break.
5. `discovery.go`, README, release notes (there is no `meta.json` version to bump).

Steps 1 and 2 are independently valuable and independently testable — together they fix all
six problems with no config break — so they are a natural PR boundary if the whole change
proves too large to review at once. Step 1 alone is also coherent: it fixes problems 1, 2, 3,
4, and 6 and leaves exclusivity for step 2.

## Rejected alternatives

- **Narrow Go interface + type assertion** (the ufactory shape). Fails fast at construction
  and needs no command protocol, but only works when the arm is local, and keeps a
  compile-time coupling that `DoCommand` avoids. The capability probe recovers most of the
  fail-fast benefit.
- **Handing the gripper the `*SafeSoArmController`.** Smallest diff and fixes problems 1–3,
  but leaves the gripper with arm-wide reach — including the `Stop()` that kills arm motion.
- **A separate `bus` resource** owned by all three. Cleanest ownership on paper, but adds a
  resource every user must configure and diverges from the ufactory precedent.
- **Optional `arm` with a legacy fallback.** No breakage, but keeps both code paths alive and
  preserves the races in the fallback path indefinitely.
- **Lifetime exclusivity** (arm and calibration sensor cannot coexist on a port). Simplest
  model, nothing to revoke, but it breaks discovery's generated config and forces users to
  delete the calibration sensor to drive the arm.

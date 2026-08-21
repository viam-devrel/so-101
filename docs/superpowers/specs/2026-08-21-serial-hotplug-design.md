# Serial hotplug: surviving unplug and replug on a live arm

**Date:** 2026-08-21
**Status:** Designed, unbuilt. Blocked on Change B step 1 of the 2026-08-17 spec.
**Affects:** `devrel:so101:arm`, `devrel:so101:gripper`, `devrel:so101:calibration`
**Depends on:** `docs/superpowers/specs/2026-08-17-gripper-arm-dependency-design.md`, Change B step 1

## Problem

Pull the USB cable on a running SO-101 and the module never recovers. Plug it back in and it
still never recovers. The only fix is restarting the module process.

Three separate causes, each verified in the code rather than inferred:

1. **Nothing detects a dead bus.** `feetech.Bus` is opened once in `createNewController` and
   held for the life of the process. When the device disappears the file descriptor goes
   stale; every read and write against it fails forever. Nothing closes the bus, and nothing
   tries to reopen it.

2. **A failed open poisons the port permanently.** `createNewController` stores the error and
   inserts the nil-controller entry into the map anyway (`registry.go:140-145`). Every later
   `GetController` for that port finds the entry, routes to `getExistingController`, and
   returns `"cached controller creation error"` (`registry.go:58`) without retrying the open.
   `NewSO101` does not release on that path either (`arm.go:466`), unlike its two other
   error paths. Two functions *do* evict such an entry — `ReleaseController` and
   `ForceCloseController` — but neither is reachable in production: the first depends on the
   no-op `Release` below, and `ForceCloseSharedController` (`manager.go:524`) has no callers
   anywhere in the repo. Tests do call both (`registry_test.go:87`, `:236`, `:271`), so the
   poisoning fix must not assume they never run.

   This matters more than it looks: viam-server already retries resources that failed to
   build, every 5 seconds, via `completeConfigWorker` (`rdk@v1.0.0
   robot/impl/local_robot.go:427`). The server is doing the right thing and the module
   discards it. An arm plugged in after startup never connects.

3. **Even a working reconnect would not reach the components.** The registry hands each
   caller its own **copy** of `SafeSoArmController` (`registry.go:98`, `:220`). `bus` and
   `group` are pointers copied into each new struct, so swapping either inside the registry
   entry leaves every holder on the old one. `calibratedServos` is the exception: it is a map,
   shared by reference across all copies, which is exactly how `UpdateCalibration`
   (`registry.go:89`) already reaches every holder today. So the servo map is not the
   obstacle — `bus` and `group` are.

## Scope

**In scope.** Unplug then replug on the same port path, recovered on a live component with no
viam-server or module restart. Detection, teardown, reopen, and restoring the arm to a
torque-held state.

**Out of scope**, each an explicit decision rather than an oversight:

- **Port path changes across replug** (`ttyACM0` becoming `ttyACM1`, or a macOS
  `tty.usbmodem*` rename). Identifying the device by USB serial number or VID:PID is a
  separate feature.
- **Building while the device is absent at startup** as a feature. Fixing cause 2 delivers it
  incidentally, since a failed open then leaves no residue for the server's 5s retry to trip
  over, but nothing here is designed around the absent-at-startup case.
- **Detecting a different arm swapped onto the port.** Calibration is preserved across a
  reconnect as-is; a swapped arm gets the previous arm's calibration and is not noticed.
- **New config attributes.** Recovery is always on. A stale-fd bus is never useful, so there
  is nothing worth a knob, and a `reconnect: false` would only restore today's behavior.
- **Teleop auto-resume.** See "Accepted tradeoffs".
- **Fixing the no-op `Release`.** Owned by the prerequisite, not by this change.

## Enabling facts

Each of these was measured or read, not assumed. Several contradict the obvious guess.

**Read errors are unusable as a disconnect signal.** `readRawBytesLocked` swallows any read
error where `n == 0` — it sleeps a millisecond and retries until the deadline, then returns
`ErrNoResponse` (`feetech-servo@v0.6.0 feetech/bus.go:548-580`). An unplugged device and a
single servo that failed to answer produce *the same error value*. Any design that keys
disconnect detection off read failures cannot tell them apart.

**Write errors are reliable.** `sendPacketLocked` wraps them as `"write failed: %w"` with the
underlying syscall error intact (`feetech/bus.go:472-482`). This is the one place the
transport's own failure reaches the caller undisguised.

**`BusConfig.Transport` is exported and `transports.MockTransport` exists.** An unplug and
replug can therefore be simulated in unit tests with no serial hardware, following the
`deadTransport` precedent already in `gripper_range_test.go`. This is what makes the whole
design testable; without it the reconnect path would be review-only, like `NewSO101`'s
`goalCloud` wiring.

**`Release` is a guaranteed no-op for every caller.** `trackCaller` records
`runtime.Caller(3)`, which resolves to `GetSharedControllerWithCalibration`
(`manager.go:517`) from both `GetController` paths, and to `GetController` itself
(`registry.go:49`) from the third path where `createNewController` delegates to
`getExistingController` — never, on any path, to the calling component.
`releaseFromCaller` then looks up `runtime.Caller(2)`, which *is* the component (`arm.go:1081`
inside `Close`). The two program counters can never match, so `callerPorts` always misses and
`ReleaseController` never runs. refCount only grows and the bus is never closed. The prior
spec asserts this; the call-depth arithmetic confirms it independently.

**`WaitForServosToStop` polls lock-free on purpose.** It captures `*CalibratedServo`
references under a brief read lock and then releases it for the whole polling loop
(`manager.go:331-341`), so a concurrent `Stop` is not blocked. It is the only place in the
controller that holds session-derived references across time.

**`servo_capabilities` touches no bus.** It returns `servo_commands` and `servo_ids` from
memory (`servo_commands.go:126-131`). A gripper therefore constructs successfully against an
arm whose port is down.

## Prerequisite: Change B step 1

This design assumes Change B step 1 of the 2026-08-17 spec is built: shared pointer,
`ControllerHandle` carrying the bus surface, `AcquireController`, a working `Release` with the
`refCount`/`pins` split, the collapse of the controller's two calibration copies,
`bus.Protocol()` folded into `SyncReadPositions`, and the `recordPositions` done channel.

**The dependency is correctness, not tidiness.** A session swap is only sound if nothing
holds a raw bus pointer across it. Two things do today: every `SafeSoArmController` copy
holds `bus` by value, and the calibration sensor reaches through the controller into the raw
bus twice per site — `bus.SyncRead` then `bus.Protocol()` (`calibration.go:464`/`:468` and
`:565`/`:570`). Shipping hotplug first would mean shipping a swap that stale
pointers silently bypass — the arm would reconnect while the sensor kept writing to a closed
descriptor. B step 1 removes both leaks; it is the thing that makes the swap airtight.

The prerequisite also simplifies three points that would otherwise land here:

- Reconnect liveness ties to B's real `refCount`, so callbacks are only about callbacks.
- The collapsed calibration gives the reconnect path one authoritative source to re-wrap
  servos around instead of two copies to reconcile.
- The `recordPositions` use-after-close, which a working `Release` makes reachable, is fixed
  by B rather than by this change.

Problem 1 of the prior spec — the copied zero-valued mutexes — is likewise fixed by B, not
here.

## Design

### The swappable unit

```go
// busSession is immutable once published. A reconnect builds a new one and swaps it in;
// it is never mutated in place.
type busSession struct {
    bus    *feetech.Bus
    group  *feetech.ServoGroup
    servos map[int]*CalibratedServo
    gen    uint64                    // monotonic; identifies this session
}
```

`ControllerEntry` gains the session and the connection state:

```go
type ControllerEntry struct {
    session atomic.Pointer[busSession]  // nil == disconnected
    state   connState                   // connected | reconnecting
    fails   atomic.Int32                // consecutive-failure escalation
    // plus B step 1's controller, refCount, pins, handles, leases
}
```

`ControllerHandle`'s methods stop reading `entry.controller`'s bus fields and route through a
single helper:

```go
func (h *ControllerHandle) withSession(f func(*busSession) error) error {
    sess := h.entry.session.Load()
    if sess == nil {
        return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.portPath())
    }
    return h.entry.report(sess.gen, f(sess))
}
```

Loading the session is a single atomic read, so a swap needs no lock against in-flight
callers.

**`report` is generation-scoped, and that is load-bearing.** A caller that loaded generation
*N*, blocked on a serial timeout, and returns a transport-class error after generation *N+1*
has been published must not tear down the fresh session, nor inflate its strike counter.
`report` therefore ignores — logging at Debug — any error whose `gen` is not the live one.

An error from the live generation goes through classification, and on a disconnect verdict
`report` performs the transition itself: compare-and-swap the session pointer to `nil` **for
that generation only**, flip `state` to reconnecting, and start the reconnect goroutine. The
CAS is what makes a simultaneous disconnect verdict from two concurrent callers idempotent —
the loser's CAS fails, so exactly one goroutine starts.

### Detection: three rules

Given that read errors cannot be distinguished from a quiet servo, detection needs both a
fast path and an escalation path.

1. **Transport-class error, disconnect immediately.** An error chain containing
   `*os.SyscallError`, a `serial.PortError`, `os.ErrClosed`, `feetech.ErrBusClosed`, or the
   `"write failed: "` wrap from `sendPacketLocked`. These only happen when the descriptor
   itself is gone.
2. **`ErrNoResponse` or `ErrTimeout`, escalate.** Increment `fails`; at **3 consecutive**,
   declare disconnected. Three strikes because one non-answering servo produces exactly this
   error on a perfectly healthy bus, and a single strike would tear down a working port.
3. **Any success resets `fails` to zero.** Including a success on a different servo — the
   counter is a property of the bus, not of a servo.

**Classify on the sentinel, not on the error type.** A servo fault — `*feetech.ServoError`
with `Status != 0` — is never a disconnect and does not touch the counter. But `*ServoError`
is *also* how a missing sync-read reply arrives (`feetech/bus.go:271` returns
`&ServoError{Op: "sync_read", Err: ErrNoResponse}`), and `*ServoError` implements `Unwrap`, so
`errors.Is(err, ErrNoResponse)` matches it. An implementation that type-switches on
`*ServoError` before checking sentinels gets this exactly backwards and never escalates a
sync-read outage. Check `errors.Is` against the sentinels first; treat `*ServoError` as a
servo fault only when `Status != 0`.

Deliberately rejected: polling the port path's existence. It is traffic-free and catches the
common case, but `/dev/ttyACM0` existing does not prove the servos answer, and after a fast
replug the path exists again while our descriptor is still stale — so it produces both false
negatives and false positives. Reopening unconditionally on escalation handles both.

### The reconnect loop

One goroutine per entry, started by `report` on the transition to disconnected, holding a
**`pin`** for its lifetime so B's teardown cannot close the entry underneath it. A reconnect
loop that outlived `Release` would reopen a port no component holds.

**The shutdown order has to be named, because B's `pins` counter is what gates teardown.**
"Teardown cancels and joins the goroutine" would deadlock: teardown waits on the pin the
goroutine holds, and the goroutine waits to be cancelled by teardown. The order is therefore
`refCount` reaching zero cancels the reconnect loop's context; the loop observes the
cancellation and returns; its deferred pin decrement is the decrement that performs teardown.
The loop never waits on teardown, and teardown never waits on a live loop.

**Acquiring into a disconnected entry restarts the loop.** That cancel trigger makes a
terminal state reachable, because the arm and the calibration sensor are both
`AlwaysRebuild`: a reconfigure while unplugged runs `Close` (`refCount` to zero, loop
cancelled) and then a fresh `AcquireController`. The entry is still present — the exiting
loop's pin is outstanding — so `refCount` climbs back to 1 with no teardown, but the loop is
already on its way out and nothing in `withSession` would restart it, since a `nil` session
returns early without ever reaching `report`. `AcquireController` therefore starts the loop
whenever it joins an entry whose session is `nil` with no live loop. Without that, the new
handle is permanently disconnected with only `reinitialize` to rescue it.

Each attempt:

1. Best-effort `Close` the dead bus, ignoring the error — closing a vanished device usually
   fails and that is not interesting.
2. `busFactory(cfg)` for a fresh bus.
3. Ping every servo in `cfg.ServoIDs`. An open that succeeds while the servos stay silent is
   not a reconnect; USB can enumerate before servo power returns.
4. Build a new `busSession`: raw servos, group, and `CalibratedServo`s re-wrapped around the
   **existing in-memory calibration**, so a `reload_calibration` or a calibration-sensor
   update survives the replug. Registers are deliberately not re-read — that would only
   matter for a swapped arm, which is out of scope, and would risk overwriting good
   calibration with a `0`/`4095` "unset" read.
5. `session.Store(new)`, state to connected, `gen` incremented, **and `fails` reset to
   zero**. The reset is not bookkeeping: the disconnect verdict left `fails` at 3, and the
   loop's own ping in step 3 talks to the raw bus rather than through `report`, so nothing
   else clears it. Without the reset the strike budget is spent for good — the first
   `ErrNoResponse` from one quiet servo after a reconnect is strike 4 and re-declares
   disconnect immediately.
6. Fire reconnect callbacks.

Backoff 250 ms doubling to a 5 s cap, with jitter, retrying indefinitely while a holder
exists — a user may leave the arm unplugged for hours and should not have to restart anything
when they come back. Failed attempts log at Debug, with an Info every ~30 s so a long outage
is neither silent nor a flood.

### Who restores torque

The registry knows nothing about torque or manual mode, so the arm is told and does it
itself:

```go
func (h *ControllerHandle) OnReconnect(fn func()) (unsubscribe func())
```

Registered at acquire, dropped in `Close`. The arm's callback exits manual mode, then re-runs
`initializeServos` against **`s.cancelCtx`** — not the `initCtx` it stores from construction,
which by then is long dead.

That is not a call-site change: `initializeServos` takes no context at all, and
`doServoInitialization` (`arm.go:1120`), `diagnoseConnection` (`:1154`), and
`verifyServoConfig` (`:1182`) each read `ctx := s.initCtx` internally. A `ctx` parameter has
to be threaded through `initializeServos` → `initializeServosWithRetry` →
`doServoInitialization`, and those three assignments replaced. Worth doing regardless: the
existing `reinitialize` DoCommand (`arm.go:927-937`) already runs against the dead `initCtx`
today.

Callbacks fire *after* the session is published and while the reconnect goroutine holds no
entry lock, so the callback's own bus I/O succeeds. The arm's callback takes `s.mu` for the
manual-mode exit and releases it before doing I/O; nothing in the reconnect path takes locks
in the reverse order, and the handle's lock is always innermost.

**The last commanded goal is deliberately not re-sent.** A replug power-cycles the STS3215s,
which come up holding wherever they physically are. Re-sending a stale goal would make the
arm lurch across the gap it sagged through while unpowered.

### Sessions held across time

`WaitForServosToStop` captures the session generation alongside its servo references and
aborts with `ErrPortDisconnected` if `gen` changed mid-wait. Without this it would keep
polling servos belonging to a closed bus, and its best-effort timeout path would return `nil`
— reporting a completed move that never happened.

`manual_mode.go`'s `controllerManualIO` captures a `*SafeSoArmController` today
(`manual_mode.go:218-233`) — i.e. the copy holding the stale `bus`, so it is *not* currently
session-safe. Change B step 1 replaces that field with a `*ControllerHandle`, and once it
does, capturing the handle rather than servos means it needs no further change here.

### Registry changes

- `createNewController` stops inserting the entry on a failed open (`registry.go:140-145`).
  A failed open leaves no residue, so the next `GetController` retries it.
- `ControllerRegistry` gains `busFactory func(feetech.BusConfig) (*feetech.Bus, error)`,
  defaulting to `feetech.NewBus`. This is the test seam.
- `gripper.go:155`'s `ReleaseSharedController()` is deleted. The gripper has held no
  controller since the arm-dependency change, and `releaseFromCaller` no-ops anyway.

### Surface

No new config attributes. One existing DoCommand changes: **`reinitialize` also forces an
immediate reconnect attempt**, skipping the backoff wait, then re-runs init. It is the
operator's escape hatch from watching the log.

`controller_status` is left alone.

## Behavior contract

While disconnected, every bus-touching call returns
`fmt.Errorf("%w: %s", ErrPortDisconnected, port)` **immediately**, rather than stalling a
second per servo on serial timeouts.

| Surface | While disconnected | After replug |
|---|---|---|
| `JointPositions`, `EndPosition`, `CurrentInputs` | `ErrPortDisconnected` | works |
| `MoveToJointPositions`, `MoveThroughJointPositions`, `Stop` | `ErrPortDisconnected` | works |
| `IsMoving` | `false` | unchanged |
| `Kinematics`, `ModelFrame`, `Get3DModels` | unaffected — no bus | unaffected |
| `Geometries` | `ErrPortDisconnected` | works |
| `reinitialize`, `controller_status` | callable | callable |
| Gripper (all motion and position) | `ErrPortDisconnected` via the arm | works |
| Gripper construction | succeeds | succeeds |
| Manual mode | loop exits on its own error limit | dropped; arm returns torque-held |
| Calibration sensor | `ErrPortDisconnected` | works; in-progress workflow abandoned |
| Teleop | stops after its error limit | **stays stopped** until `start` |

`IsMoving` needs no change: both move paths `defer`-reset the atomic (`arm.go:671`, `:694`),
and `Stop` stores `false` eagerly before its bus call (`arm.go:750`) — the more correct order
there anyway, since it should read stopped even if the stop write fails.

**`reinitialize` and `controller_status` are exempt from the fail-fast rule on purpose.**
`reinitialize` is the escape hatch *for* the disconnected state; routing the arm's whole
`DoCommand` surface through `withSession` would make the hatch unreachable exactly when it is
needed. Both must reach the entry directly rather than through a session.

**`Geometries` fails rather than lying.** It routes through `CurrentInputs` to the bus, so the
frame system and 3D viewer lose the arm during an outage. Serving the last known joint
positions from cache would keep the viewer stable while reporting a pose that is usually
wrong — an unpowered arm sags, and operators move it by hand precisely when it is limp. A
frame system fed a confident wrong pose is worse than one fed an error.

## Testing

Every test below runs without serial hardware, via the injected `busFactory` plus a fake
transport whose `Write` returns `syscall.EIO` on demand.

**Classification** — table test over transport errors, `ErrNoResponse`, `ErrTimeout`,
`ErrBusClosed`, `*feetech.ServoError` with status flags, and `nil`.

**Escalation** — a transport error disconnects on the first failure; `ErrNoResponse`
disconnects only on the third; an interleaved success resets the counter so the third
*non-consecutive* failure does not.

**Fail-fast** — a call during an outage returns `ErrPortDisconnected` and returns *fast*:
assert elapsed time is far below the bus timeout. This is the test that catches a regression
to per-servo timeout stalls.

**The swap reaches every holder** — two independently acquired handles on one port; a
reconnect driven through one is observed by the other. This is the direct regression test for
cause 3.

**Calibration survives** — mutate calibration through the handle, force a reconnect, assert
the new session's servos carry the mutated values, not the file or register defaults.

**Callbacks** — fire exactly once per reconnect; an unsubscribed callback does not fire.

**Generation guard** — `WaitForServosToStop` returns `ErrPortDisconnected` when the session is
swapped mid-wait, rather than `nil`.

**Goroutine lifetime** — the last `Release` cancels the loop; assert on a done channel that
the loop exited, that the pin decrement following it is what performs teardown, and that the
port is not reopened after the last release. The direction matters: a test written as
"teardown cancels and joins the loop" encodes the deadlock the design rejects.

**Loop restart on re-acquire** — release the last handle while disconnected, then acquire a
new one, and assert the reconnect loop is running again. This is the `AlwaysRebuild`
reconfigure-while-unplugged path, and without it the new handle never recovers.

**No poisoning** (regression for cause 2) — a `busFactory` that fails once then succeeds:
assert the first `GetController` returns an error, leaves `r.entries` empty, and the second
call opens successfully.

## Risks and accepted tradeoffs

**Three strikes is a heuristic, and it is tunable in exactly one direction.** A genuinely
dead servo on a healthy bus produces `ErrNoResponse` on every read of that servo, so three
consecutive reads of *that* servo will tear down and reopen a working port. Reopening is
cheap and idempotent, and the reconnect's ping step will then fail on that servo and keep
retrying — which is loud, but correct: a servo that never answers is a fault worth surfacing.
The alternative, keying only off transport errors, misses the servo-power-brownout case
entirely.

**Servo power lost while USB stays enumerated** is not the scoped case but falls out
acceptably: the strike rule triggers, the reopen succeeds, the ping fails, and the loop
retries until power returns.

**Teleop does not auto-resume.** `teleop.go:307-314` stops the sync loop after N consecutive
errors and never restarts, so after a replug teleop stays stopped until someone re-issues
`start`. Everything else here self-heals; this one is documented in the README instead.
Deliberate: teleop talks over the arm API rather than the controller, so resuming it is a
separate concern with its own safety question — auto-resuming leader-to-follower mirroring
with no human present moves the follower unexpectedly.

**A calibration workflow interrupted by a replug is abandoned.** The sensor's state machine
keeps its progress in memory and the operator restarts. Checkpointing it across a
disconnect is a larger change than the recovery it would protect.

**Callbacks run on the reconnect goroutine.** A callback that blocks stalls further reconnect
work for that port. Only the arm registers a real one, and it is bounded by
`initializeServos`' own retry budget.

## Sequencing

1. **Change B step 1** (prior spec, designed and unbuilt). Not re-scoped here.
2. **Session indirection.** `busSession`, the entry's session pointer and state, the
   `ErrPortDisconnected` sentinel, `withSession` with generation-scoped `report` across the
   handle's bus surface, and the generation guard in `WaitForServosToStop`. No detection and
   no reconnect yet: the session is always present, so behavior is unchanged and this wide
   mechanical refactor is provable on its own.
3. **Detection and reconnect, in one step.** Classification, the strike counter, the
   `busFactory` seam — used by both `createNewController` and the loop's reopen, which is what
   lets the poisoning test and the reconnect tests share one fake — the loop, backoff, pinning
   with the cancellation order above, restart-on-acquire, and the registry poisoning fix.
4. **Restoration.** `OnReconnect`, the arm's callback, `reinitialize` forcing a reopen, and
   the README note covering recovery and the teleop caveat.

**Detection must not ship without reconnect.** Splitting them would leave a release in which a
disconnect verdict nils the session permanently, with no reopen and no `reinitialize` escape
hatch — and the three-strike false positive from a single genuinely dead servo (see "Risks")
would convert a today-degraded arm into a dead one needing a module restart. The three-strike
rule is only defensible because a reopen follows it.

Step 2 is shippable alone. Step 3 is where the judgment calls live, and where review
attention belongs.

## Rejected alternatives

- **Exit the module process on disconnect.** Far less code, and it reuses the existing init
  path wholesale. But it kills every component in the module, including a second arm on a
  healthy port, and loses in-flight state to recover from a fault that is local to one port.
- **Fail and let viam-server rebuild.** Fixing the poisoned entry would make the 5s retry work
  for construction, but RDK gives a successfully-constructed resource no way to ask to be
  rebuilt, so a live component would stay broken until a config edit or restart.
- **Reopen inline on each failed operation.** No session type and no goroutine: the operation
  reopens and retries once. But every call on a permanently-unplugged arm then pays a port
  open plus re-init, concurrent callers stampede the open, and there is nowhere to put
  backoff.
- **Share one `*SafeSoArmController` and mutate its bus fields under its own mutex.** After
  Change B step 1 the controller is already shared and its calibration already collapsed, so
  the usual objection — per-holder `logger` and `calibration` becoming shared — is moot; the
  prior spec already accepts first-creator-wins on `logger`. The durable reason for an
  immutable session is the pair of properties mutation cannot offer: a lock-free atomic swap
  that in-flight readers never block on, and a generation stamp that lets `report` and
  `WaitForServosToStop` tell a stale caller from a live one.
- **Periodic ping heartbeat.** Notices an outage while the arm is idle, so reconnection
  completes before the first call. Costs bus bandwidth and mutex contention against teleop's
  sync loop and manual mode's tick, both of which already saturate the bus, to buy latency on
  a first call that is about to happen anyway.
- **Watching the device file.** Traffic-free, but path existence proves nothing about the
  servos and is stale-fd-blind after a fast replug.
- **Caching joint positions to keep `Geometries` alive.** Keeps the viewer and frame system
  stable through an outage by reporting a pose that is usually wrong.

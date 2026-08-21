# Bus Ownership (Change B step 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the per-port serial controller genuinely shared — one mutex, one calibration, one path to the bus, and a `Release` that actually releases — and put the bus behind a swappable session while doing it, so serial hotplug can be built on top without touching these 20 methods a second time.

**Architecture:** `ControllerEntry` becomes the single owner of a per-port `busSession` (bus + servo group + calibrated servos) held behind an `atomic.Pointer`. Every resource reaches the bus only through a `ControllerHandle` whose methods load that session, so no caller holds a raw bus pointer. `Release` is rewired off the broken `runtime.Caller` PC map onto real reference counting, with a separate refusable pin count and a once-guarded teardown that the serial-hotplug reconnect loop will attach to.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, `github.com/hipsterbrown/feetech-servo v0.6.0`, `go.bug.st/serial v1.6.4`, `testify`.

**Specs:** `docs/superpowers/specs/2026-08-17-gripper-arm-dependency-design.md` (Change B step 1 — the design of record) and `docs/superpowers/specs/2026-08-21-serial-hotplug-design.md` (the lifecycle requirements this plan must satisfy).

---

## READ THIS FIRST

### What this builds, and what it deliberately does not

This is **Change B step 1 only**, plus the session indirection folded in. From the 2026-08-17 spec it fixes problems **1** (unshared mutex), **2** (forked calibration), **3** (unsynchronized calibration read), **4** (no-op `Release`), and **6** (the sensor's eight bus bypasses).

**Not in scope — Change B step 2 (two-tier exclusivity).** No `busLease`, no `motionHold`, no `entry.handles` snapshot, no preemption hooks, no `withLease` around `motor_setup_*`. The prior spec's teardown rules 1 and 3 exist to serve those mechanisms and are therefore deferred with them; **only teardown rule 2 (the `refCount`/`pins` split) is built here**, because its hazard — a concurrent `Release` closing the bus under an in-flight acquire — exists without any lease. The prior spec's own guidance is that step 1 is mechanical and worth landing alone, and that step 2 should be re-scoped after it.

Also not in scope: problem **5** (config triplication between the arm and the sensor), and the arm/sensor exclusivity requirement itself.

### Why the session is folded in here

Serial hotplug needs the bus behind a swappable session. Building that separately would rewrite all 20 controller method bodies a second time — two chances to drop one of 49 call sites or mis-scope a lock, for no benefit. Folding it in keeps this plan behavior-neutral anyway: **the session is always non-nil until hotplug's detection lands**, so nothing here can observe a disconnected state.

Consequence: `docs/superpowers/plans/2026-08-21-serial-hotplug.md` starts at **its Task 5**. Its Tasks 1-4 and 13 are absorbed here and must be struck from that document (Task 11 of this plan does it).

### Lifecycle contracts this plan owes to hotplug

These are **requirements, not options** — hotplug's reconnect loop is unsafe without each, and three of them were found by review only after being got wrong on paper:

1. **`tryPin() bool` is refusable.** It returns false once teardown has begun, so a reconnect loop is never started for an entry that is going away. The prior spec designs `pins` as a plain counter for in-flight acquires and does not define refusal.
2. **Teardown is once-guarded, not a re-read of the counters.** Whichever of `refCount` and `pins` reaches zero last performs it, and a second attempt must be a no-op.
3. **`Release` reaches the teardown check with no entry lock held**, so a later `cancelReconnect` can be slotted in front of it without inverting the lock order.
4. **`AcquireController` has a post-lock, post-refCount seam.** Hotplug calls `resumeReconnect()` there. It must sit outside `getExistingController`'s `entry.mu` — that function holds the lock under a `defer` for its whole body (`registry.go:52-53`), and calling in from inside self-deadlocks through `tryPin`.
5. **Lock order: `reconnectMu` before `entry.mu`, never the reverse.** Nothing here takes `reconnectMu` (it does not exist yet), but the order is documented now because step 9's `Release` is the one call site that would invert it.

### Project conventions

- Build and test with `go test ./cmd/module/ .` — **never** `./...`. `cmd/cli/` is standalone debug scripts with multiple `func main()` and does not compile as a package.
- `gofmt -s -w .` and `go vet ./cmd/module/ .` before every commit.
- `-race` on every task that touches concurrency.
- `TestEnumerateSerialPorts` (`discovery_test.go`) fails on machines with no serial hardware. Environmental, not yours.
- Never create a `*_arm.go` file — Go treats that as GOARCH=arm-only.

### Risk note

The prior spec warns that Change B's design went through four review rounds **without being built**, and that the spike which finally built Change A found real errors in A's design. Expect the same here. Two specific places where this plan is most likely to be wrong, flagged at their tasks: the calibration collapse's effect on `arm.calculateJointLimits`, and whether `Discover`'s return type can be narrowed (it cannot — see Task 6).

---

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `hotplug_fake_test.go` | create | Concurrency-safe fake `feetech.Transport` answering real protocol packets. Test keystone for this plan *and* hotplug. |
| `bus_session.go` | create | `busSession`, `newBusSession`, `ErrPortDisconnected`, `withSession`/`withSessionValue`. No policy, no goroutines. |
| `bus_session_test.go` | create | Session presence, shared helpers used by every later task. |
| `registry.go` | modify | Entry owns the session, the calibration, `refCount`, `pins`, and teardown. `busFactory` seam. Poisoning fix. `AcquireController`. The PC-keyed machinery is deleted. |
| `registry_test.go` | modify | Poisoning regression; refCount/pins/teardown tests. |
| `manager.go` | modify | `SafeSoArmController` → `ControllerHandle`; all 20 methods route through `withSession`; calibration collapse; five new methods for the sensor. |
| `manager_test.go` | create | Handle sharing, calibration collapse, the new sensor-facing methods. |
| `arm.go` | modify | Field type and acquire/release call sites. |
| `calibration.go` | modify | Eight bypass sites onto handle methods; `recordPositions` done channel. |
| `manual_mode.go` | modify | Field type only. |
| `gripper.go` | modify | Delete the stale `ReleaseSharedController()` at `:155`. |
| `docs/superpowers/plans/2026-08-21-serial-hotplug.md` | modify | Strike the absorbed tasks. |

---

## Task 1: The fake transport

Identical to the serial-hotplug plan's Task 1 — **build it from that document** (`docs/superpowers/plans/2026-08-21-serial-hotplug.md`, Task 1), including `isUnplugged`, `noteOpen`/`openCount`, and the absent-servo test. It is reproduced there in full with its framing rationale.

Why it belongs here too: `transports.MockTransport` has unlocked fields and `transports.Script` is documented single-goroutine, so neither survives `-race`; and this plan's `Release`/teardown tests need a bus they can open and close without hardware.

- [ ] **Step 1:** Build Task 1 of the serial-hotplug plan verbatim (all 6 steps), then return here.
- [ ] **Step 2:** Confirm `go test -run TestFakeTransport -race .` passes all three tests before continuing. Every later task trusts this fake.

---

## Task 2: `busFactory` seam

Build **Task 2 of the serial-hotplug plan** verbatim: add `busFactory func(feetech.BusConfig) (*feetech.Bus, error)` to `ControllerRegistry`, default it to `feetech.NewBus` in `NewControllerRegistry`, and use it in `createNewController`. Includes `shortTimeoutConfig` and the note that `registry_test.go:19` already has `testConfig`.

- [ ] **Step 1:** Build it, then run `go test -run TestRegistry -race .` — all pre-existing `TestRegistry*` must still pass.
- [ ] **Step 2:** Commit as that task specifies.

---

## Task 3: Poisoning fix

Build **Task 3 of the serial-hotplug plan** verbatim: stop inserting the entry when the open fails (`registry.go:140-145`), delete `lastError` and the branch at `registry.go:58`, and update the `ControllerEntry` literal at `registry_test.go:224`.

Do it before the restructure so the diff stays small and the regression test lands against today's code.

- [ ] **Step 1:** Build it.
- [ ] **Step 2:** `go test ./cmd/module/ . -race` passes; commit as that task specifies.

---

## Task 4: One entry, one session, one handle

The big one. All 20 `SafeSoArmController` methods get rewritten **once**. The type keeps its name until Task 7, so the 49 `s.controller.Method(...)` call sites in `arm.go`, `calibration.go`, and `manual_mode.go` are untouched by this task — only the field's *type* changes, and it changes to a same-named type.

**Files:**
- Create: `bus_session.go`, `bus_session_test.go`
- Modify: `registry.go`, `manager.go`

- [ ] **Step 1: Write the failing test**

`bus_session_test.go` — the two session tests plus the **Shared test helpers block** (`testRegistry`, `acquireTestHandle`, `testRegistryAndHandle`, `testHandle`, `cloneCalibration`, `stopReconnectAndWait`) from **Task 4, Step 1 of the serial-hotplug plan**. Copy that block; every later task in both plans uses it. Omit `stopReconnectAndWait` for now — it references reconnect fields that do not exist until hotplug.

Add the test this task is actually about:

```go
func TestAllHoldersShareOneControllerState(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)

	require.Same(t, h1.entry, h2.entry, "one port must mean one entry")
	assert.Same(t, h1.entry.session.Load(), h2.entry.session.Load(),
		"both holders must see the same session -- problem 1 was each getting a private copy")
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run 'TestWithSession|TestAllHoldersShare' -race .`
Expected: FAIL — `undefined: busSession`, `ErrPortDisconnected`, `h.entry`.

- [ ] **Step 3: Write `bus_session.go`**

Take `busSession`, `ErrPortDisconnected`, `withSession`, `withSessionValue`, and `newBusSession` from **Task 4, Step 3 of the serial-hotplug plan**, with one change: the methods hang off `*SafeSoArmController` for now (Task 7 renames the type), and `newBusSession` takes the entry's single calibration.

- [ ] **Step 4: Restructure the entry and the controller**

`ControllerEntry` gains:

```go
	port     string                     // for error messages; no longer only in config
	registry *ControllerRegistry        // for eviction at teardown (Task 9)
	session  atomic.Pointer[busSession] // the single owner of the bus
	gen      atomic.Uint64              // monotonic session counter
```

The registry back-pointer is what lets `teardownIfLast` evict the entry from `r.entries`
without the registry having to poll. It also means the entry reads `e.registry.busFactory`
rather than caching its own copy — supersede the serial-hotplug plan's note about the entry
"needing no registry back-pointer", which predates this teardown design.

`SafeSoArmController` loses `bus`, `group`, `calibratedServos`, **and `mu`**, keeping only:

```go
type SafeSoArmController struct {
	entry  *ControllerEntry
	logger logging.Logger // per-holder, so log lines name the component that acted
}
```

Deleting `mu` is the fix for problem 1: serialization moves to `entry.mu`, which is genuinely shared. `withSession` takes it.

`createNewController` builds the session with `newBusSession` and `entry.session.Store(...)` instead of assembling the three fields inline. The `!fromFile` register-read path stays, but must rebuild the session afterwards so the servos carry the calibration just read.

Both registry return paths (`registry.go:98`, `:220`) now return `&SafeSoArmController{entry: entry, logger: config.Logger}`.

- [ ] **Step 5: Convert all 20 methods**

Every method that touched `s.bus`, `s.group`, or `s.calibratedServos` becomes a `withSession`/`withSessionValue` call. The sample conversion in **Task 4, Step 5 of the serial-hotplug plan** shows the shape. Three rules:

1. **Argument validation stays outside `withSession`** — a malformed call must fail the same way whether or not the port is up, and must never reach the error classifier hotplug adds later.
2. **Do not wrap `Close`.** Closing the bus is teardown, not a session operation. (In fact `SafeSoArmController.Close` becomes dead — nothing should close a *shared* bus from a per-holder method. Delete it and let Task 9's teardown own closing.)
3. **`WaitForServosToStop` keeps its lock-free polling** (`manager.go:331-341`) — capture servos, release the lock, poll. Hotplug adds the generation guard; do not add it here, and do not "fix" the lock-free design.

- [ ] **Step 6: Run the full suite**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -30`
Expected: PASS. This task must be behavior-neutral; any changed behavior is a refactor bug.

- [ ] **Step 7: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add bus_session.go bus_session_test.go registry.go manager.go
git commit -m "refactor: one shared controller state per port, behind a bus session"
```

---

## Task 5: Collapse the two calibrations

Sharing the pointer does not fix problem 2 — it *unmasks* it. `SafeSoArmController` held calibration twice: `s.calibration` (read by every write path) and `calibratedServos[id].calibration` (read by the read path, and the only one the registry's acquire-time update touched). Per-caller struct copies hid the divergence; one shared state exposes it. And it is not an edge case — `discovery.go:179-181` puts `calibration_file` on both the arm and the sensor config, so a second acquire with `fromFile=true` is the normal setup.

**Files:**
- Modify: `manager.go`, `registry.go`
- Create: `manager_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestCalibrationHasASingleSourceOfTruth(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)

	updated := cloneCalibration(DefaultSO101FullCalibration)
	updated.Gripper.RangeMin = 1234
	require.NoError(t, h1.SetCalibration(updated))

	// The read path and the write path must agree, through either holder.
	assert.Equal(t, 1234, h2.GetCalibration().Gripper.RangeMin)
	assert.Equal(t, 1234, h2.entry.session.Load().servos[6].Calibration().RangeMin,
		"the per-servo calibration is the one source of truth; nothing may hold a second copy")
}

func TestConcurrentCalibrationUpdateAndReadIsRaceFree(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	var wg sync.WaitGroup
	for i := 0; i < 20; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			c := cloneCalibration(DefaultSO101FullCalibration)
			c.Gripper.RangeMin = 1000 + i
			_ = h.SetCalibration(c)
		}(i)
		wg.Add(1)
		go func() { defer wg.Done(); _ = h.GetCalibration() }()
	}
	wg.Wait()
}
```

The second test is problem 3: `GetJointPositionsForServos` read `calibratedServos[servoID].calibration` directly without taking `CalibratedServo.mu` (`manager.go:159`), racing `UpdateCalibration`. It only fails under `-race`.

- [ ] **Step 2: Run to verify they fail**

Run: `go test -run 'TestCalibrationHasASingle|TestConcurrentCalibration' -race .`
Expected: the first FAILs on a stale value or a compile error; the second FAILs with a `-race` report.

- [ ] **Step 3: Delete `s.calibration`**

`s.calibration` is derivable, so delete it. `getCalibrationForServo` and `GetCalibration` read through `calibratedServos[id].Calibration()` (that accessor already exists and takes the servo's own lock). The registry's acquire-time update (`registry.go:72-93`) then has exactly one place to write, and no path can observe a half-updated calibration.

Every read of a servo's calibration goes through `Calibration()` — never the struct field. That is problem 3's fix, and it is easy to reintroduce by reaching for `.calibration` out of habit.

The prior spec offers a fallback (route the acquire-time update through `SetCalibration`), but it leaves two copies and one more chance to desynchronize. Take the collapse.

- [ ] **Step 4: Check `calculateJointLimits`**

`arm.go:376` consumes `GetCalibration()` to build joint limits. **This is the likeliest place for the collapse to change behavior**, because it previously read `s.calibration` — the copy the registry's acquire-time update never touched. After the collapse it sees post-update values. Read that function and confirm the new values are the correct ones (they are: the per-servo calibration is what the servos actually use). Run `go test -run TestArm -race .` and read any diff in expected limits carefully rather than adjusting the test to match.

- [ ] **Step 5: Run to verify they pass**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS, including both new tests.

- [ ] **Step 6: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go registry.go manager_test.go
git commit -m "fix: collapse the controller's two calibration copies into one"
```

---

## Task 6: Close the sensor's eight bus bypasses

Problem 6. The sensor reaches past the controller in eight places, taking no controller lock at all — so those calls are unserialized against every other resource *even in principle*. They are also the raw-bus pointers that would let a caller bypass a session swap, which is why hotplug depends on this task specifically.

**Files:**
- Modify: `manager.go`, `calibration.go`
- Modify: `manager_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestSyncReadPositionsDecodesInsideTheHandle(t *testing.T) {
	ft := newFakeTransport()
	ft.setRegister(1, feetech.RegPresentPosition.Address, encodeWordLE(2048))
	h := testHandle(t, ft)

	positions, err := h.SyncReadPositions(context.Background(), []int{1})
	require.NoError(t, err)
	assert.Equal(t, 2048, positions[1],
		"decoding belongs inside the handle; exposing Protocol() would re-leak the bus")
}

func TestPingServoReachesOneServo(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	model, err := h.PingServo(context.Background(), 3)
	require.NoError(t, err)
	assert.Equal(t, fakeModelNumber, model)
}
```

`encodeWordLE` is a two-line test helper using `feetech.NewProtocol(feetech.ProtocolSTS).EncodeWord`.

- [ ] **Step 2: Run to verify they fail**

Run: `go test -run 'TestSyncReadPositions|TestPingServo' -race .`
Expected: FAIL — methods undefined.

- [ ] **Step 3: Add five handle methods**

```go
// SyncReadPositions reads present position for the given servos and returns decoded ticks.
//
// The decode lives HERE on purpose. The sensor previously called bus.SyncRead and then
// bus.Protocol().DecodeWord (calibration.go:463/:468, :565/:570); adding a Protocol()
// passthrough to the handle would re-leak the bus and defeat the point of the handle.
// Protocol() disappears from the sensor entirely.
func (h *ControllerHandle) SyncReadPositions(ctx context.Context, ids []int) (map[int]int, error)

// PingServo pings one servo and returns its model number. Replaces calibration.go:1045's
// reach into calibratedServos.
func (h *ControllerHandle) PingServo(ctx context.Context, id int) (int, error)

// Discover sweeps every bus ID. Replaces calibration.go:1103 and :1165.
func (h *ControllerHandle) Discover(ctx context.Context) ([]feetech.FoundServo, error)

// SetServoID and SetServoBaudRate replace the pair at calibration.go:1193.
func (h *ControllerHandle) SetServoID(ctx context.Context, currentID, targetID int) error
func (h *ControllerHandle) SetServoBaudRate(ctx context.Context, id, baudRate int) error
```

Each body is a `withSession`/`withSessionValue` call over `sess.bus` or `sess.servos[id]`.

**`Discover` must return `[]feetech.FoundServo`, not the `[]int` the prior spec sketches.** Both call sites read `servo.Model.Name` (`calibration.go:1113-1115`, `:1179`), so narrowing to IDs would break them. This is a concrete correction to the design of record.

- [ ] **Step 4: Convert the eight sites**

| Site | Was | Becomes |
|---|---|---|
| `calibration.go:463`+`:468` | `bus.SyncRead` + `bus.Protocol().DecodeWord` | `SyncReadPositions` |
| `:565`+`:570` | same, in `recordPositions` | `SyncReadPositions` |
| `:1045` | `calibratedServos[id].Ping` | `PingServo` |
| `:1103` | `bus.Discover` | `Discover` |
| `:1165` | `bus.Discover` | `Discover` |
| `:1193` | `calibratedServos[id]` → `Ping`/`SetID`/`SetBaudRate` | `PingServo`, `SetServoID`, `SetServoBaudRate` |

Then verify nothing reaches the bus any more:

Run: `grep -n "controller\.bus\|controller\.calibratedServos" *.go`
Expected: **no output.** If anything remains, the handle is not the only path to the bus and hotplug's session swap has a hole.

- [ ] **Step 5: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS.

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go calibration.go manager_test.go
git commit -m "refactor(calibration): reach the bus only through the controller handle"
```

**Accepted carry-forward:** `SetServoID` changes a servo's ID while the session's `servos` map is still keyed by the old one, so the map is stale until the next acquire. That is exactly today's behavior (`calibratedServos` has the same staleness), the motor-setup workflow is explicitly driven and reconfigures afterwards, and fixing it means rebuilding the session mid-workflow. Left as-is deliberately — note it in the PR body rather than fixing it here.

---

## Task 7: Rename to `ControllerHandle` / `AcquireController`

Pure rename plus per-handle identity. No method bodies change. Doing it as its own commit keeps Task 4's behavior-neutral refactor reviewable.

**Files:**
- Modify: `manager.go`, `registry.go`, `arm.go`, `calibration.go`, `manual_mode.go`

- [ ] **Step 1: Rename the type and constructor**

`SafeSoArmController` → `ControllerHandle`. `GetSharedControllerWithCalibration` → `AcquireController`, taking the owner's name for diagnostics:

```go
func AcquireController(
	owner resource.Name, config *SoArm101Config,
	calibration SO101FullCalibration, fromFile bool,
) (*ControllerHandle, error)
```

`ControllerHandle` gains `owner resource.Name` and `released atomic.Bool`.

**`owner` is for diagnostics only — never an identity key.** `resource.Name` is stable across an `AlwaysRebuild` cycle, so a rebuilt resource would otherwise impersonate its predecessor. Identity is the handle pointer. (This matters for step 2's leases; recording it now costs nothing and prevents the wrong habit forming.)

- [ ] **Step 2: Update the three field declarations and their call sites**

`arm.go`, `calibration.go`, `manual_mode.go`: field type `*SafeSoArmController` → `*ControllerHandle`. The 49 `controller.Method(...)` sites are unaffected — same method names on the same field.

Constructors call `AcquireController(name, ...)`. `ReleaseSharedController()` becomes `h.Release()` at `arm.go:1081` and `calibration.go:1239`.

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS.

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go registry.go arm.go calibration.go manual_mode.go
git commit -m "refactor: rename the shared controller to ControllerHandle"
```

---

## Task 8: Join the sensor's recording goroutine

**This must land before Task 9, not after.** `so101CalibrationSensor.Close` (`calibration.go:1227-1243`) cancels `recordingCtx` but never waits for `recordPositions` (`:544-635`), a 10 ms ticker calling `SyncReadPositions`. Today that is latent *only because `Release` is a no-op and the bus is never closed*. Making `Release` work in Task 9 is what makes it reachable: a reconfigure during `StateRangeRecording` would close the serial port under a live goroutine mid-read.

**Files:**
- Modify: `calibration.go`

- [ ] **Step 1: Write the failing test**

```go
func TestClosingDuringRecordingJoinsTheGoroutine(t *testing.T) {
	// Start recording, then Close, and assert the goroutine has exited before Close
	// returns -- otherwise Task 9's bus close lands under a live SyncReadPositions.
	// Assert on the done channel, not on a sleep.
}
```

Write it against the sensor's actual construction path; if the sensor cannot be built without hardware, assert on `recordPositions`' `done` channel directly through a small exported-for-test seam rather than skipping the test.

- [ ] **Step 2: Add the done channel**

`recordPositions` gains a `done chan struct{}` closed on exit. `Close` (`calibration.go:1227`) and `stopRangeRecording` (`:645-648`) both cancel *and then join* it. This is the same audit manual mode already passes (`manual_mode.go:111` — `stop()` is synchronous).

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add calibration.go
git commit -m "fix(calibration): join the recording goroutine before returning from Close"
```

---

## Task 9: Make `Release` real

The heart of it. This is where hotplug's five lifecycle contracts get built.

**Files:**
- Modify: `registry.go`, `manager.go`
- Modify: `registry_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestReleaseClosesTheBusOnlyAfterTheLastHolder(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)
	entry := h1.entry

	h1.Release()
	assert.NotNil(t, entry.session.Load(), "one holder left; the bus must stay open")

	h2.Release()
	r.mu.RLock()
	_, present := r.entries["/dev/fake0"]
	r.mu.RUnlock()
	assert.False(t, present, "the last release must evict the entry")
}

func TestReleaseIsIdempotent(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)
	h.Release()
	h.Release() // a double Close must not double-decrement into a negative refCount
	h2 := acquireTestHandle(t, r)
	assert.NotNil(t, h2.entry.session.Load())
}

func TestTryPinRefusesOnceTeardownHasBegun(t *testing.T) {
	ft := newFakeTransport()
	_, h := testRegistryAndHandle(t, ft)
	entry := h.entry

	h.Release() // last holder: teardown runs
	assert.False(t, entry.tryPin(), "a pin after teardown must be refused, not granted")
}

func TestTeardownRunsExactlyOnce(t *testing.T) {
	ft := newFakeTransport()
	_, h := testRegistryAndHandle(t, ft)
	entry := h.entry

	h.Release()
	// A pin acquired before teardown, released after, must not tear down a second time.
	entry.teardownIfLast()
	entry.teardownIfLast()
	// No panic, no double delete; the fake's Close is counted.
	assert.Equal(t, 1, ft.closeCount())
}
```

Add `closeCount()` to the fake (mutex-guarded counter incremented in `Close`).

- [ ] **Step 2: Run to verify they fail**

Run: `go test -run 'TestRelease|TestTryPin|TestTeardown' -race .`
Expected: FAIL — `Release` is the PC-keyed no-op; `tryPin`/`teardownIfLast` undefined.

- [ ] **Step 3: Build the counters and the teardown**

```go
// ControllerEntry lifecycle:
//   refCount -- live handles
//   pins     -- in-flight users that are NOT handles: an acquire mid-flight today, and
//               serial hotplug's reconnect goroutine later
//   torndown -- once guard
//
// Whichever of refCount and pins reaches zero LAST performs the teardown. Release is
// idempotent via a per-handle once, so the releasing handle structurally cannot come back
// and finish the job -- the deferred close is an entry-level responsibility.
func (e *ControllerEntry) tryPin() bool {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.torndown || e.tearingDown {
		return false // REFUSABLE: hotplug relies on this to not start a doomed loop
	}
	e.pins++
	return true
}

func (e *ControllerEntry) unpin() {
	e.mu.Lock()
	last := e.pins-1 == 0 && e.refCount == 0
	e.pins--
	e.mu.Unlock()
	if last {
		e.teardownIfLast()
	}
}

// teardownIfLast closes the bus and evicts the entry, at most once. It is once-guarded
// rather than a re-read of the counters: the two decrements race, and a predicate that
// re-read them could fire twice.
func (e *ControllerEntry) teardownIfLast() {
	e.mu.Lock()
	if e.torndown || e.refCount > 0 || e.pins > 0 {
		e.mu.Unlock()
		return
	}
	e.torndown = true
	sess := e.session.Swap(nil)
	e.mu.Unlock()

	if sess != nil {
		if err := sess.bus.Close(); err != nil && e.logger != nil {
			e.logger.Warnf("error closing shared bus for port %s: %v", e.port, err)
		}
	}
	e.registry.evict(e.port)
}
```

`Release`:

```go
func (h *ControllerHandle) Release() {
	h.releaseOnce.Do(func() {
		h.released.Store(true)
		e := h.entry
		e.mu.Lock()
		e.refCount--
		last := e.refCount == 0 && e.pins == 0
		e.mu.Unlock()
		// NO entry lock held here. Serial hotplug inserts e.cancelReconnect() at this
		// point, and that path takes reconnectMu then entry.mu -- holding entry.mu here
		// would invert the order. See the lifecycle contracts at the top of this plan.
		if last {
			e.teardownIfLast()
		}
	})
}
```

Name the fields and the once whatever fits, but keep three properties: `Release` is idempotent, it reaches `teardownIfLast` with no lock held, and `teardownIfLast` is once-guarded.

- [ ] **Step 4: Leave the acquire seam**

In `AcquireController`, after `getExistingController` has returned and **its `entry.mu` is released**:

```go
	h, err := r.getExistingController(entry, config, calibration, fromFile)
	if err != nil {
		return nil, err
	}
	// Seam: serial hotplug calls entry.resumeReconnect() here. It MUST stay outside
	// getExistingController, which holds entry.mu under a defer for its whole body
	// (registry.go:52-53) -- calling in from inside self-deadlocks through tryPin.
	return h, nil
```

Leave the comment even though nothing calls it yet. It is the whole reason this seam exists.

- [ ] **Step 5: Run to verify they pass**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -30`
Expected: PASS. Watch for pre-existing tests that assumed a growing refCount — `TestGetControllerStatus` and friends may need updating, and that is a real behavior change, not a test to paper over.

- [ ] **Step 6: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go manager.go registry_test.go
git commit -m "fix(registry): make Release actually release, with refcount and pins"
```

---

## Task 10: Delete the dead machinery

**Files:**
- Modify: `registry.go`, `manager.go`, `gripper.go`

- [ ] **Step 1: Delete**

- `trackCaller`, `releaseFromCaller`, `callerPorts`, `callerMu` (`registry.go`) — the `runtime.Caller` PC map that made `Release` a no-op.
- `ReleaseSharedController`, `GetSharedController` (`manager.go:512`), `ForceCloseSharedController` (`manager.go:524`) — all callerless.
- `ForceCloseController` — keep only if a test needs it; it had no production callers.
- `gripper.go:153-157`'s `ReleaseSharedController()` — the gripper has held no controller since the arm-dependency change (`gripper.go:94` says so), and it no-oped anyway.

- [ ] **Step 2: Verify**

Run: `grep -rn "runtime.Caller\|callerPorts\|ReleaseSharedController\|GetSharedController\b" *.go`
Expected: no output.

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go manager.go gripper.go
git commit -m "chore: delete the PC-keyed release machinery and its callerless helpers"
```

---

## Task 11: Reconcile the serial-hotplug plan

**Files:**
- Modify: `docs/superpowers/plans/2026-08-21-serial-hotplug.md`

- [ ] **Step 1: Strike the absorbed tasks**

Tasks 1, 2, 3, 4, and 13 of that plan are built here. Replace each with a one-line pointer to this plan rather than deleting it, so the task numbers its later steps reference stay stable. Update its "READ THIS FIRST" to say the prerequisite is **built** and name the commit range.

- [ ] **Step 2: Restore `stopReconnectAndWait`**

Task 4 of this plan omitted it (it references reconnect fields that do not exist yet). It belongs with hotplug's Task 6 — note that in the hotplug plan's helper block.

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/plans/2026-08-21-serial-hotplug.md
git commit -m "docs: point the hotplug plan at the built bus-ownership work"
```

---

## Final verification

- [ ] `gofmt -l .` prints nothing
- [ ] `go vet ./cmd/module/ .` is clean
- [ ] `go test ./cmd/module/ . -race` passes (except `TestEnumerateSerialPorts` on hardware-free machines)
- [ ] `go test ./cmd/module/ . -race -count=5` passes — repeats catch a flaky teardown assertion
- [ ] `grep -rn "controller\.bus\|controller\.calibratedServos" *.go` returns nothing — the handle is the only path to the bus
- [ ] `grep -rn "runtime.Caller\|callerPorts" *.go` returns nothing
- [ ] `grep -rn "\.calibration\b" manager.go` returns nothing — every read goes through `Calibration()`
- [ ] The five lifecycle contracts at the top of this plan are each satisfied, checked by reading the code, not by a test

## Hardware verification before merging

The bus is now genuinely closed on the last release, which has never happened in this module before. Tests cannot cover what a real serial port does when closed and reopened by a live viam-server.

- [ ] Configure an arm and a calibration sensor on one port, confirm both work, then remove the sensor from the config and confirm the arm keeps working (the sensor's `Release` must not close a bus the arm holds).
- [ ] Remove both, confirm the port is released — `lsof` shows no handle — then add them back and confirm they reconnect.
- [ ] Run a range-recording calibration and reconfigure mid-recording. Before Task 8 this closed the port under a live goroutine; confirm it now exits cleanly.

## What is deferred to Change B step 2

Restate this in the PR body so nobody reads step 1 as the whole of B:

- The arm and the calibration sensor still **can** drive the bus concurrently. One shared mutex serializes each transaction, but nothing prevents an arm move mid-calibration — that is the hard lease and the soft motion hold, both step 2.
- Teardown rules 1 and 3 of the prior spec (dropping a lease on release; the `released`-flag check at hook-invocation time) are unbuilt, because leases and hooks are unbuilt. `released` is set here but only read by step 2.
- Problem 5 (config triplication between arm and sensor) is untouched.

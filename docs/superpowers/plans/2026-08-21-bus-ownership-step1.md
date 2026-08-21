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
| `bus_session.go` | create | `busSession`, `newBusSession`, `ErrPortDisconnected`, the four `withSession*` accessors, and `report` as a pass-through. No policy, no goroutines. |
| `bus_session_test.go` | create | Session presence, shared helpers used by every later task. |
| `registry.go` | modify | Entry owns the session, the calibration, `refCount`, `pins`, and teardown. `busFactory` seam. Poisoning fix. `AcquireController`. The PC-keyed machinery is deleted. |
| `registry_test.go` | modify | Poisoning regression; refCount/pins/teardown tests. |
| `manager.go` | modify | `SafeSoArmController` → `ControllerHandle`; all 20 methods route through `withSession*`; calibration collapse; seven new methods for the sensor. |
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

The big one. All 20 `SafeSoArmController` methods get rewritten **once**, and the type is renamed to `ControllerHandle` in the same pass — renaming a type while rewriting every one of its methods is one edit, not two. The 49 `s.controller.Method(...)` call sites are unaffected: same method names on the same field, only the field's declared type changes.

**This task is NOT behavior-neutral, and it should not be framed that way in review.** Deleting the per-holder mutex and serializing on a shared one *is* the fix for problem 1 — the whole point is that the arm and the calibration sensor now exclude each other where they previously did not. Expect new contention between them; that is the feature. What must not change is the *result* of any operation.

**Files:**
- Create: `bus_session.go`, `bus_session_test.go`
- Modify: `registry.go`, `manager.go`, `arm.go`, `calibration.go`, `manual_mode.go`

- [ ] **Step 1: Write the failing test**

`bus_session_test.go` — the **Shared test helpers block** from **Task 4, Step 1 of the serial-hotplug plan** (`testRegistry`, `acquireTestHandle`, `testRegistryAndHandle`, `testHandle`, `cloneCalibration`), with two changes: omit `stopReconnectAndWait` (it references reconnect fields that do not exist until hotplug), and give `acquireTestHandle` an owner name, since Task 7 adds that parameter:

```go
func acquireTestHandle(t *testing.T, r *ControllerRegistry) *ControllerHandle {
	t.Helper()
	owner := arm.Named("test-arm-" + t.Name())
	h, err := r.AcquireController(owner, shortTimeoutConfig("/dev/fake0"),
		DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	t.Cleanup(h.Release)
	return h
}
```

Plus the session tests — the nil-session path is the seam hotplug is built on, so it needs
coverage here even though nothing can reach it yet:

```go
func TestWithSessionRunsAgainstTheLiveSession(t *testing.T) {
	h := testHandle(t, newFakeTransport())

	var seen *busSession
	require.NoError(t, h.withSession(func(s *busSession) error {
		seen = s
		return nil
	}))
	require.NotNil(t, seen)
	assert.NotNil(t, seen.bus)
	assert.Len(t, seen.servos, 6)
}

func TestWithSessionFailsFastWhenDisconnected(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	h.entry.session.Store(nil)

	start := time.Now()
	err := h.withSession(func(*busSession) error {
		t.Fatal("the operation must not run without a session")
		return nil
	})
	assert.ErrorIs(t, err, ErrPortDisconnected)
	assert.Less(t, time.Since(start), 10*time.Millisecond,
		"a disconnected call must not pay a serial timeout")
}
```

Plus the test this task is really about:

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

- [ ] **Step 3: Write `bus_session.go` — and settle the locking here**

Take `busSession`, `ErrPortDisconnected`, and `newBusSession` from **Task 4, Step 3 of the serial-hotplug plan**. **Do not** take that plan's `withSession`: it acquires no lock, which was correct only because it was written to sit on top of an already-shared controller. This plan is what makes the controller shared, so the accessor here owns the locking.

```go
// Two mutexes, deliberately. entry.busMu serializes bus WORK; entry.mu guards
// bookkeeping (refCount, pins, torndown, calibration). Keeping them apart is what stops
// a 1-second serial timeout from blocking a Release, an acquire, or a teardown -- and
// what stops withSession from self-deadlocking when it reads calibration.
//
// Lock order: busMu -> entry.mu. Never the reverse.

// withSession runs a bus WRITE. Exclusive, matching today's s.mu.Lock() paths.
func (h *ControllerHandle) withSession(f func(*busSession) error) error {
	h.entry.busMu.Lock()
	defer h.entry.busMu.Unlock()
	return h.runLocked(f)
}

// withSessionRead runs a bus READ. Shared, matching today's s.mu.RLock() paths.
//
// The distinction is worth preserving even though feetech.Bus serializes each individual
// transaction internally (feetech/bus.go's b.mu): what the controller mutex buys is
// MULTI-transaction atomicity, so a write sequence is not interleaved with reads.
func (h *ControllerHandle) withSessionRead(f func(*busSession) error) error {
	h.entry.busMu.RLock()
	defer h.entry.busMu.RUnlock()
	return h.runLocked(f)
}

func (h *ControllerHandle) runLocked(f func(*busSession) error) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	return h.entry.report(sess.gen, f(sess))
}
```

Add `withSessionValue`/`withSessionValueRead` generic variants for the methods that return a value.

**Define `report` here as a pass-through.** The copied method bodies call it, and hotplug gives it teeth later:

```go
// report is where a bus error will become a connection verdict once serial hotplug lands.
// Until then it is the identity function.
//
// It is called with busMu held, so it must never take busMu. Taking entry.mu IS legal and
// hotplug's version does -- that is the declared busMu -> entry.mu order, not a violation.
func (e *ControllerEntry) report(gen uint64, err error) error { return err }
```

- [ ] **Step 4: Restructure the entry and rename the controller**

`ControllerEntry` gains:

```go
	port     string                     // for error messages; no longer only in config
	registry *ControllerRegistry        // for eviction at teardown (Task 9)
	logger   logging.Logger             // was reached via entry.config.Logger
	busMu    sync.RWMutex               // serializes bus work (problem 1's fix)
	session  atomic.Pointer[busSession] // the single owner of the bus
	gen      atomic.Uint64              // monotonic session counter
```

The registry back-pointer is what lets `teardownIfLast` evict the entry from `r.entries`. It also means the entry reads `e.registry.busFactory` rather than caching a copy — supersede the serial-hotplug plan's note about the entry "needing no registry back-pointer", which predates this teardown design.

`createNewController` builds the session via `newBusSession` and `entry.session.Store(...)` instead of assembling `bus`/`group`/`calibratedServos` inline, and **must populate the new entry fields**: `port`, `registry` (itself), and `logger` (from `config.Logger`, which is where `entry.config.Logger` was read from). The `!fromFile` register-read path stays where it is, but must rebuild the session afterwards so the servos carry the calibration just read.

**`ControllerEntry.controller` is deleted.** A `*ControllerHandle` holding only `{entry, logger}` would be circular and meaningless. Three live reads must be converted to `entry.session.Load() != nil`:

| Site | Was | Becomes |
|---|---|---|
| `registry.go:56` | `entry.controller == nil` guard | `entry.session.Load() == nil` |
| `registry.go:302` | `hasController` in `GetControllerStatus` | `entry.session.Load() != nil` |
| `registry.go:277` | `ForceCloseController`'s nil check | same, or delete with the function (Task 10) |

Three test literals construct entries with a `controller` field and need updating: `registry_test.go:223`, `:258`, `:361`.

`SafeSoArmController` is renamed **`ControllerHandle`** and reduced to:

```go
type ControllerHandle struct {
	entry  *ControllerEntry
	logger logging.Logger // per-holder, so log lines name the component that acted
}
```

Both registry return paths (`registry.go:98`, `:220`) return `&ControllerHandle{entry: entry, logger: config.Logger}`.

Rename sites beyond the three component fields — all compile errors, listed so they are not a scavenger hunt: `servo_commands_test.go:220` (`var _ servoOps = (*SafeSoArmController)(nil)`), `manual_mode.go:226` (constructor parameter), and `manager.go:512`'s `GetSharedController` (update it now; Task 10 deletes it).

- [ ] **Step 5: Convert all 20 methods**

Each method that touched `s.bus`, `s.group`, or `s.calibratedServos` becomes a `withSession*` call. **Preserve today's read/write split exactly:** the methods that took `s.mu.Lock()` use `withSession`, the `RLock` methods use `withSessionRead`. **Read each method's current lock before converting it** — do not infer from the name, and do not trust a list. (`Close` is among the `Lock` methods and is deleted rather than converted; see rule 2.)

Three rules:

1. **Argument validation stays outside `withSession`** — a malformed call must fail the same way whether or not the port is up, and must never reach the error classifier hotplug adds later.
2. **`ControllerHandle.Close` is deleted.** Nothing should close a *shared* bus from a per-holder method; Task 9's teardown owns closing.
3. **`WaitForServosToStop` keeps its lock-free polling** (`manager.go:331-341`) — capture servos under the read lock, release it, poll. Hotplug adds the generation guard; do not add it here, and do not "fix" the lock-free design.

- [ ] **Step 6: Run the full suite**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -30`
Expected: PASS. Results must be unchanged; timing and contention may not be.

- [ ] **Step 7: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add bus_session.go bus_session_test.go registry.go manager.go arm.go calibration.go manual_mode.go
git commit -m "refactor: one shared controller state per port, behind a bus session"
```

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
	assert.Equal(t, 1234, h1.entry.calibration.Gripper.RangeMin,
		"the session-rebuild seed must track SetCalibration, or a replug undoes it")

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

**`entry.calibration` is the second copy that survives, and it must not go stale.** `newBusSession(bus, cal, gen)` seeds every servo *from it*, so any session rebuild — the `!fromFile` register-read path here, and hotplug's reconnect later — resurrects whatever it holds. But `SetCalibration` (reached from `arm.go:970`'s `reload_calibration`) writes only the per-servo calibration. Left alone, a `reload_calibration` followed by a replug would silently restore the pre-reload values.

So `SetCalibration` updates **both** under `entry.mu`: the per-servo calibrations (the read/write source of truth) and `entry.calibration` (the session-rebuild seed). `GetControllerStatus`'s custom-vs-default label also reads `entry.calibration`, so it stays meaningful.

**Read calibration off the session, not off the entry.** Every converted method body already
holds `sess`, so `manager.go:63`'s `s.calibration.GetMotorCalibrationByID(servoID)` becomes:

```go
	cal := sess.servos[servoID].Calibration()
```

Do **not** add an `entry.calibrationFor(...)` accessor. Routing writes through `entry.calibration`
while reads go through the per-servo copy is problem 2's exact structure — writes against one
copy, reads against another — surviving only on the discipline that every writer updates both.
Reading from the session makes the single source of truth structural rather than a convention,
and it removes the `busMu → entry.mu` nesting from all 20 method bodies, which is a strictly
simpler lock model than the alternative.

`entry.calibration` is then **purely the session-rebuild seed**: written by `SetCalibration` and
the acquire-time update, read by `newBusSession` and `GetControllerStatus`, and by nothing on a
hot path.

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

Note this is where the sensor's 10 ms `recordPositions` loop starts sharing `busMu` with the arm's motion path for the first time. That contention is the point of problem 6, not a regression.

**Files:**
- Modify: `manager.go`, `calibration.go`
- Modify: `manager_test.go`, `hotplug_fake_test.go`

- [ ] **Step 1: Extend the fake**

Add to `hotplug_fake_test.go` (mutex-guarded, like the rest): `setRegister(id int, address byte, value []byte)`, `closeCount() int` (incremented in `Close`, needed by Task 9), and a package-level `encodeWordLE(v int) []byte` helper over `feetech.NewProtocol(feetech.ProtocolSTS).EncodeWord`. Note them in Task 11 — hotplug's Task 8 would otherwise add `setRegister` a second time.

- [ ] **Step 2: Write the failing tests**

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

func TestServoPresentReportsConfiguredServos(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)
	assert.True(t, h.ServoPresent(6))
	assert.False(t, h.ServoPresent(9), "motorSetupVerify needs absent-vs-unresponsive")
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `go test -run 'TestSyncReadPositions|TestPingServo|TestServoPresent' -race .`
Expected: FAIL — methods undefined.

- [ ] **Step 4: Add seven handle methods**

```go
// SyncReadPositions reads present position for the given servos and returns decoded ticks.
//
// The decode lives HERE on purpose. The sensor previously called bus.SyncRead and then
// bus.Protocol().DecodeWord (calibration.go:463/:468, :565/:570); adding a Protocol()
// passthrough to the handle would re-leak the bus and defeat the point of the handle.
// Protocol() disappears from the sensor entirely.
func (h *ControllerHandle) SyncReadPositions(ctx context.Context, ids []int) (map[int]int, error)

// PingServo pings one servo and returns its model number.
func (h *ControllerHandle) PingServo(ctx context.Context, id int) (int, error)

// DetectServoModel runs the servo's model auto-detection and returns the detected name.
// Wraps the DetectModel + Model().Name pair at calibration.go:1057-1067.
func (h *ControllerHandle) DetectServoModel(ctx context.Context, id int) (string, error)

// ServoPresent reports whether the session has a servo with this ID. No bus traffic.
// motorSetupVerify distinguishes "not in the controller" from "did not answer", so a
// ping-only API cannot express its `not_found` case.
func (h *ControllerHandle) ServoPresent(id int) bool

// Discover sweeps every bus ID. Replaces calibration.go:1103 and :1165.
func (h *ControllerHandle) Discover(ctx context.Context) ([]feetech.FoundServo, error)

// SetServoID and SetServoBaudRate replace the pair at calibration.go:1193.
func (h *ControllerHandle) SetServoID(ctx context.Context, currentID, targetID int) error
func (h *ControllerHandle) SetServoBaudRate(ctx context.Context, id, baudRate int) error
```

Read methods use `withSessionRead`; `SetServoID`/`SetServoBaudRate` are writes and use `withSession`.

**`Discover` must return `[]feetech.FoundServo`, not the `[]int` the prior spec sketches.** Both call sites read `servo.Model.Name` (`calibration.go:1113-1115`, `:1179`), so narrowing to IDs breaks them. A concrete correction to the design of record.

**`SyncReadPositions` passes `dataLen = 2`, not `len(ids)`.** Both existing sites pass `len(cs.cfg.ServoIDs)` as feetech's `dataLen` argument (`calibration.go:463`, `:565`) — wrong but harmless at exactly six servos, since a position is two bytes. Fix it deliberately in the new method; keeping the bug would make the single-id test above panic inside `DecodeWord`.

- [ ] **Step 5: Convert the eight sites**

| Site | Was | Becomes |
|---|---|---|
| `calibration.go:463`+`:468` | `bus.SyncRead` + `bus.Protocol().DecodeWord` | `SyncReadPositions` |
| `:565`+`:570` | same, in `recordPositions` | `SyncReadPositions` |
| `:1045` | `calibratedServos[id]` → `Ping`, `DetectModel`, `Model().Name` | `ServoPresent`, `PingServo`, `DetectServoModel` |
| `:1103` | `bus.Discover` | `Discover` |
| `:1165` | `bus.Discover` | `Discover` |
| `:1193` | `calibratedServos[id]` → `Ping`/`SetID`/`SetBaudRate` | `PingServo`, `SetServoID`, `SetServoBaudRate` |

`motorSetupVerify`'s three result states (`not_found`, `not_responding`, `model_detection_failed`, plus `ok`) must all survive the conversion — check the strings against `calibration.go:1044-1070` rather than reconstructing them.

Then verify nothing reaches the bus any more:

Run: `grep -n "controller\.bus\|controller\.calibratedServos" *.go`
Expected: **no output.** If anything remains, the handle is not the only path to the bus and hotplug's session swap has a hole.

- [ ] **Step 6: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS.

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go calibration.go manager_test.go hotplug_fake_test.go
git commit -m "refactor(calibration): reach the bus only through the controller handle"
```

**Accepted carry-forward:** `SetServoID` changes a servo's ID while the session's `servos` map is still keyed by the old one, so the map is stale until the next acquire. That is exactly today's behavior (`calibratedServos` has the same staleness), the motor-setup workflow is explicitly driven and reconfigures afterwards, and fixing it means rebuilding the session mid-workflow. Left as-is deliberately — note it in the PR body.

## Task 7: `AcquireController` and per-handle identity

The rename landed in Task 4. This task adds the acquire API the tests and hotplug need, and the per-handle identity fields.

**Files:**
- Modify: `manager.go`, `registry.go`, `arm.go`, `calibration.go`

- [ ] **Step 1: Add both forms of the acquire API**

**Two are needed, and the plan's tests depend on the method form.** Tests must drive a registry with an injected `busFactory`, not `globalRegistry`, so the method is the real entry point and the package-level function is a thin delegate:

```go
// AcquireController is the registry-scoped entry point. Tests use this one.
func (r *ControllerRegistry) AcquireController(
	owner resource.Name, config *SoArm101Config,
	calibration SO101FullCalibration, fromFile bool,
) (*ControllerHandle, error)

// AcquireController delegates to the process-wide registry. Production uses this one.
func AcquireController(
	owner resource.Name, config *SoArm101Config,
	calibration SO101FullCalibration, fromFile bool,
) (*ControllerHandle, error) {
	return globalRegistry.AcquireController(owner, config, calibration, fromFile)
}
```

The method wraps today's `GetController` and is where Task 9's seam goes.

`ControllerHandle` gains `owner resource.Name`, `released atomic.Bool`, and `releaseOnce sync.Once`.

**`owner` is for diagnostics only — never an identity key.** `resource.Name` is stable across an `AlwaysRebuild` cycle, so a rebuilt resource would otherwise impersonate its predecessor. Identity is the handle pointer. `released` is written here and read only by Change B step 2; that is intentional.

- [ ] **Step 2: Update the constructors**

`arm.go:466` and `calibration.go:172` call `AcquireController(name, ...)` — both already have a `resource.Name` in scope. `ReleaseSharedController()` becomes `h.Release()` at `arm.go:1081` and `calibration.go:1239`.

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS. `Release` is still effectively a no-op at this point — Task 9 gives it teeth.

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go registry.go arm.go calibration.go
git commit -m "feat(registry): AcquireController with per-handle identity"
```

## Task 8: Join the sensor's recording goroutine

**This must land before Task 9, not after.** `so101CalibrationSensor.Close` (`calibration.go:1227-1243`) cancels `recordingCtx` but never waits for `recordPositions` (`:544-635`), a 10 ms ticker calling `SyncReadPositions`. Today that is latent *only because `Release` is a no-op and the bus is never closed*. Making `Release` work in Task 9 is what makes it reachable: a reconfigure during `StateRangeRecording` would close the serial port under a live goroutine mid-read.

**The obvious implementation deadlocks.** `Close` holds `cs.mu.Lock()` under a defer for its whole body (`calibration.go:1228-1229`), and `stopRangeRecording` runs inside `DoCommand`, which does the same (`:301-302`). Meanwhile `recordPositions` takes `cs.mu.RLock()` at `:556` and `cs.mu.Lock()` at `:610` on **every tick**. Waiting on a `done` channel while holding `cs.mu` hangs whenever the goroutine is mid-tick, and cancelling the context does not help — it is already past its `select`. The failure is a hung test, not a red one.

**Files:**
- Modify: `calibration.go`

- [ ] **Step 1: Write the failing test**

```go
func TestClosingDuringRecordingJoinsTheGoroutine(t *testing.T) {
	// Start range recording, then Close, and assert recordPositions has exited BEFORE
	// Close returns -- otherwise Task 9's bus close lands under a live SyncReadPositions.
	// Assert on the done channel, never on a sleep. Bound the wait and t.Fatal on timeout
	// so the deadlock described above surfaces as a failure rather than a hang.
}
```

Write it against the sensor's real construction path. If the sensor cannot be built without hardware, assert on `recordPositions`' `done` channel through a small test-only seam rather than skipping the test — an untested join is what this task exists to prevent.

- [ ] **Step 2: Add the done channel, and drop the lock before joining**

`recordPositions` gains a `done chan struct{}` closed on exit. Both `Close` and `stopRangeRecording` (`:645-648`) must:

1. take `cs.mu`, copy `recordingCancel` and `done` into locals, clear the fields, **release `cs.mu`**;
2. call cancel;
3. wait on `done` (bounded — log and continue on timeout rather than hanging a reconfigure forever).

**`Close` is the only path that must join.** It is the one that precedes a bus close; `stopRangeRecording` just ends a recording while the port stays open. Since `DoCommand`'s single deferred unlock covers every handler, making `stopRangeRecording` join would mean restructuring that lock scope for no safety gain. Prefer one of:

- join in `Close` only, and have `startRangeRecording` refuse (or join) when a prior goroutine has not exited; or
- have `stopRangeRecording` return a closure that `DoCommand` runs after unlocking.

Take the first unless a test shows the second is needed. This is the same audit manual mode already passes (`manual_mode.go:111` — `stop()` is synchronous).

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race -timeout 120s 2>&1 | tail -20`
Expected: PASS. A timeout here means the lock was still held across the join.

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add calibration.go
git commit -m "fix(calibration): join the recording goroutine before returning from Close"
```

## Task 9: Make `Release` real

The heart of it, and where hotplug's lifecycle contracts get built.

**Files:**
- Modify: `registry.go`, `manager.go`
- Modify: `registry_test.go`

- [ ] **Step 1: Convert `refCount` off `sync/atomic`**

It is an `int64` driven by `atomic` in eight places (`registry.go:19`, `:64`, `:95`, `:210`, `:241`, `:256`, `:282`, `:301`; `manager.go:551`). The teardown logic below needs it read together with `pins` under one lock, and a mixed atomic/mutex field is the kind of thing that reads as correct and is not. Make it a plain `int` guarded by `entry.mu` and convert every site.

- [ ] **Step 2: Write the failing tests**

```go
func TestReleaseClosesTheBusOnlyAfterTheLastHolder(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)
	entry := h1.entry

	h1.Release()
	assert.NotNil(t, entry.session.Load(), "one holder left; the bus must stay open")
	assert.Equal(t, 0, ft.closeCount())

	h2.Release()
	r.mu.RLock()
	_, present := r.entries["/dev/fake0"]
	r.mu.RUnlock()
	assert.False(t, present, "the last release must evict the entry")
	assert.Equal(t, 1, ft.closeCount())
}

func TestReleaseIsIdempotent(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)
	h.Release()
	h.Release() // a double Close must not double-decrement into a negative refCount
	assert.Equal(t, 1, ft.closeCount())

	h2 := acquireTestHandle(t, r)
	assert.NotNil(t, h2.entry.session.Load(), "a fresh acquire must open a fresh entry")
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
	entry.teardownIfLast() // a redundant call must be inert
	entry.teardownIfLast()
	assert.Equal(t, 1, ft.closeCount())
}

func TestAcquireNeverJoinsATornDownEntry(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)
	torn := h.entry

	h.Release()
	h2 := acquireTestHandle(t, r)
	assert.NotSame(t, torn, h2.entry,
		"joining a torn-down entry would hand back a permanently dead handle")
	assert.NotNil(t, h2.entry.session.Load())
}
```

- [ ] **Step 3: Run to verify they fail**

Run: `go test -run 'TestRelease|TestTryPin|TestTeardown|TestAcquireNever' -race .`
Expected: FAIL — `Release` is the PC-keyed no-op; `tryPin`/`teardownIfLast` undefined.

- [ ] **Step 4: Build the counters and the teardown**

```go
// ControllerEntry lifecycle:
//   refCount -- live handles
//   pins     -- in-flight users that are NOT handles. Built and tested here, WIRED BY
//               NOTHING until serial hotplug's reconnect goroutine. See the lifecycle
//               contracts at the top of this plan before looking for a caller.
//   torndown -- once guard; also makes the entry invisible to a new acquire
//
// Whichever of refCount and pins reaches zero LAST performs the teardown. Release is
// idempotent via a per-handle once, so the releasing handle structurally cannot come back
// and finish the job -- the deferred close is an entry-level responsibility.
func (e *ControllerEntry) tryPin() bool {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.torndown {
		return false // REFUSABLE: hotplug relies on this to not start a doomed loop
	}
	e.pins++
	return true
}

func (e *ControllerEntry) unpin() {
	e.mu.Lock()
	e.pins--
	e.mu.Unlock()
	e.teardownIfLast() // safe to call unconditionally; it re-reads both counters
}

// teardownIfLast closes the bus and evicts the entry, at most once.
//
// It takes registry.mu BEFORE entry.mu and evicts inside the same critical section that
// sets torndown. Both halves matter: taking entry.mu first would invert the order against
// GetController (which holds r.mu and then entry.mu), and evicting after releasing the
// locks leaves a window where GetController finds a dead entry, raises its refCount, and
// hands back a permanently disconnected handle -- after which the eviction lands and the
// next acquire opens a SECOND bus on the same port. TestConcurrentRegistryAccess
// (registry_test.go:378-410) hammers exactly this path.
//
// The bus is closed only after both locks are released: Close can block, and nothing else
// needs to wait behind it. In-flight bus ops are safe -- feetech.Bus.Close sets closed
// under its own mutex, so they finish and later ones get ErrBusClosed.
func (e *ControllerEntry) teardownIfLast() {
	r := e.registry
	r.mu.Lock()
	e.mu.Lock()
	if e.torndown || e.refCount > 0 || e.pins > 0 {
		e.mu.Unlock()
		r.mu.Unlock()
		return
	}
	e.torndown = true
	sess := e.session.Swap(nil)
	if r.entries[e.port] == e {
		delete(r.entries, e.port)
	}
	e.mu.Unlock()
	r.mu.Unlock()

	if sess != nil {
		if err := sess.bus.Close(); err != nil && e.logger != nil {
			e.logger.Warnf("error closing shared bus for port %s: %v", e.port, err)
		}
	}
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
		e.mu.Unlock()
		// NO lock held here. Serial hotplug inserts e.cancelReconnect() at this point,
		// and that path takes reconnectMu then entry.mu -- holding entry.mu here would
		// invert the order. See the lifecycle contracts at the top of this plan.
		e.teardownIfLast()
	})
}
```

Calling `teardownIfLast` unconditionally, rather than precomputing a `last` flag, is deliberate: it re-reads both counters under the lock, so a concurrent `unpin` and `Release` in either order reach a correct decision exactly once.

**Belt-and-braces for the acquire path:** have `getExistingController` treat a `torndown` entry as absent and return a sentinel that makes the caller fall through to creating a fresh entry. The eviction above already closes the window; this makes a future refactor that reopens it fail loudly instead of silently.

Handle the sentinel in **both** callers — `GetController` (`registry.go:46`) and `createNewController` (`registry.go:112`). Missing the second surfaces the sentinel as an error to the caller instead of falling through.

- [ ] **Step 5: Retire the old teardown**

`ControllerRegistry.ReleaseController` (`registry.go:229-259`) is the *previous* teardown: it decrements `refCount` atomically, closes the bus, and deletes the entry, entirely outside the new `torndown` guard. Left in place there are two teardown paths — double `bus.Close()`, negative `refCount`.

**It also holds the only reversed lock order in the codebase**: `entry.mu` (`:238`) then `r.mu` (`:249`), against the `r.mu → entry.mu` that both acquire paths and the new teardown use. So retiring it is required for the lock order to be consistent at all, not merely to avoid a double teardown — keeping it as a compatibility shim would reintroduce an ABBA deadlock. Either delete it or reduce it to `entry.mu` decrement + `teardownIfLast()`. Two existing tests drive it (`registry_test.go:87`, `:236`) and must be rewritten against `Release`.

- [ ] **Step 6: Leave the acquire seam**

In `(*ControllerRegistry).AcquireController`, after `GetController` returns — so outside both `r.mu` and `entry.mu`:

```go
	h, err := r.GetController(config.Port, config, calibration, fromFile)
	if err != nil {
		return nil, err
	}
	h.owner = owner
	// Seam: serial hotplug calls h.entry.resumeReconnect() here.
	//
	// It must be HERE and nowhere deeper. getExistingController holds entry.mu under a
	// defer for its whole body (registry.go:52-53), and createNewController calls it
	// (registry.go:112) while holding r.mu -- so a seam placed in either would run under
	// a lock that resumeReconnect's tryPin needs.
	return h, nil
```

Leave the comment even though nothing calls it yet. It is the whole reason this seam exists.

- [ ] **Step 7: Run to verify they pass**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -30`
Expected: PASS. Watch for pre-existing tests that assumed a growing refCount — `TestGetControllerStatus` and friends may need updating, and that is a real behavior change, not a test to paper over.

- [ ] **Step 8: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go manager.go registry_test.go
git commit -m "fix(registry): make Release actually release, with refcount and pins"
```

## Task 10: Delete the dead machinery

**Files:**
- Modify: `registry.go`, `manager.go`, `gripper.go`

- [ ] **Step 1: Delete**

- `trackCaller`, `releaseFromCaller`, `callerPorts`, `callerMu` (`registry.go`) — the `runtime.Caller` PC map that made `Release` a no-op. `registry_test.go:41` asserts `registry.callerPorts != nil` and must go with it.
- `ReleaseSharedController`, `GetSharedController` (`manager.go:512`), `ForceCloseSharedController` (`manager.go:524`) — all callerless.
- `ControllerRegistry.ReleaseController` and `ForceCloseController`, if Task 9 Step 5 did not already fold them in. Neither had a production caller.
- `getCalibrationForServo` (`manager.go:474`) — **delete rather than convert**; its only two callers are commented out (`calibration.go:441`, `:587`).
- `gripper.go:153-157`'s `ReleaseSharedController()` — the gripper has held no controller since the arm-dependency change (`gripper.go:94` says so), and it no-oped anyway.

- [ ] **Step 2: Verify**

Run: `grep -rn "runtime.Caller\|callerPorts\|ReleaseSharedController\|GetSharedController\b\|getCalibrationForServo" *.go`
Expected: no output.

- [ ] **Step 3: Run and commit**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go manager.go gripper.go registry_test.go
git commit -m "chore: delete the PC-keyed release machinery and its callerless helpers"
```

## Task 11: Reconcile the serial-hotplug plan

**Files:**
- Modify: `docs/superpowers/plans/2026-08-21-serial-hotplug.md`

- [ ] **Step 1: Record exactly which halves landed**

"Tasks 1-4 are absorbed" is too coarse to act on. Be specific, because hotplug's Task 5 builds directly on these:

| Hotplug task | Status after this plan |
|---|---|
| Task 1 (fake transport) | **Built**, plus `setRegister`, `closeCount`, `encodeWordLE`, `isUnplugged`, `noteOpen`/`openCount` |
| Task 2 (`busFactory`) | **Built** |
| Task 3 (poisoning fix) | **Built** |
| Task 4 (session + `withSession`) | **Built**, but with `withSession`/`withSessionRead` taking `busMu` — *not* the lock-free version that plan sketches. `report` exists as a pass-through; hotplug's Task 5 replaces its body and must not re-declare it. `entry.calibrationFor` is built. `stopReconnectAndWait` is **not** built (it needs reconnect fields) and moves to hotplug's Task 6. |
| Task 8 (`setRegister`) | The fake helper is built here; the generation guard itself is not |
| Task 13 (gripper release) | **Built** |

- [ ] **Step 2: Update its prerequisite section**

Its "READ THIS FIRST" says the prerequisite is unbuilt and that signatures must be reconciled before Task 4. Replace that with the built commit range, and note the two API facts that differ from what it assumed: `withSession` acquires `busMu`, and `AcquireController` exists in both registry-method and package-level forms.

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/plans/2026-08-21-serial-hotplug.md
git commit -m "docs: point the hotplug plan at the built bus-ownership work"
```

## Final verification

- [ ] `gofmt -l .` prints nothing
- [ ] `go vet ./cmd/module/ .` is clean
- [ ] `go test ./cmd/module/ . -race` passes (except `TestEnumerateSerialPorts` on hardware-free machines)
- [ ] `go test ./cmd/module/ . -race -count=5` passes — repeats catch a flaky teardown assertion
- [ ] `grep -rn "controller\.bus\|controller\.calibratedServos" *.go` returns nothing — the handle is the only path to the bus
- [ ] `grep -rn "runtime.Caller\|callerPorts" *.go` returns nothing
- [ ] Every per-servo calibration read goes through `Calibration()` on a session servo. `entry.calibration` appears only in `SetCalibration`, the acquire-time update, `newBusSession`, and `GetControllerStatus` — never on a bus hot path
- [ ] `grep -rn "calibrationFor" *.go` returns nothing
- [ ] `grep -rn "atomic.*refCount" *.go` returns nothing — one lock, not a mixed field
- [ ] `grep -rn "controller\.controller\|entry.controller" *.go` returns nothing
- [ ] The five lifecycle contracts at the top of this plan are each satisfied, checked by reading the code, not by a test
- [ ] `pins` has no caller, and that is intentional (contract 1)

## Hardware verification before merging

The bus is now genuinely closed on the last release, which has never happened in this module before. Tests cannot cover what a real serial port does when closed and reopened by a live viam-server.

- [ ] **Reconfigure the arm alone and confirm it comes back.** rdk closes a resource before constructing its replacement (`module/resources.go:559`), so with the arm as sole holder this now closes and reopens the port on every config edit. Change an unrelated attribute (e.g. `speed_degs_per_sec`) several times in a row and confirm no `EBUSY`, no permanently dead arm, and no servo jolt. This is the most likely field regression in the whole plan.
- [ ] Configure an arm and a calibration sensor on one port, confirm both work, then remove the sensor from the config and confirm the arm keeps working (the sensor's `Release` must not close a bus the arm holds).
- [ ] Remove both, confirm the port is released — `lsof` shows no handle — then add them back and confirm they reconnect.
- [ ] Run a range-recording calibration and reconfigure mid-recording. Before Task 8 this closed the port under a live goroutine; confirm it now exits cleanly.

## What is deferred to Change B step 2

Restate this in the PR body so nobody reads step 1 as the whole of B:

- The arm and the calibration sensor still **can** drive the bus concurrently. One shared mutex serializes each transaction, but nothing prevents an arm move mid-calibration — that is the hard lease and the soft motion hold, both step 2.
- Teardown rules 1 and 3 of the prior spec (dropping a lease on release; the `released`-flag check at hook-invocation time) are unbuilt, because leases and hooks are unbuilt. `released` is set here but only read by step 2.
- Problem 5 (config triplication between arm and sensor) is untouched.

Also name these two accepted consequences of the shared `busMu`, alongside the 10 ms `recordPositions` note:

- **`Discover` holds `busMu.RLock` for the whole sweep** — seconds on a sparse bus, by `calibration.go:1101`'s own admission — so arm motion blocks during `motor_setup_*`. Correct (they must not share the bus) but newly visible.
- **"A serial timeout cannot block a Release" is approximate, not absolute.** `getExistingController` holds `entry.mu` while `UpdateCalibration` waits on `CalibratedServo.mu`, which `cs.Position` holds across a serial read (`calibrated_servo.go:180-194`). Bounded and cycle-free, so it is a stall rather than a deadlock — but it is a stall.

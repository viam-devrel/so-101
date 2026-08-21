# Serial Hotplug Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a running SO-101 arm survive its USB cable being unplugged and plugged back in, recovering without a viam-server or module restart.

**Architecture:** The `feetech.Bus` and everything derived from it move into an immutable `busSession` held behind an `atomic.Pointer` on the per-port `ControllerEntry`. Bus operations load the session, run, and report their error through a generation-scoped classifier; a transport-class error or three consecutive silent reads nils the session and starts one pinned per-port reconnect goroutine, which reopens with backoff, publishes a fresh session, and notifies subscribers so the arm can restore torque.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, `github.com/hipsterbrown/feetech-servo v0.6.0`, `go.bug.st/serial v1.6.4`, `testify`.

**Spec:** `docs/superpowers/specs/2026-08-21-serial-hotplug-design.md`

---

## READ THIS FIRST

### Prerequisite: Change B step 1 is BUILT

Built by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md`, commits `0bb226a..cee1543`. Start this plan at **Task 5**.

What landed, and where it differs from what this plan assumed:

| This plan's task | Status |
|---|---|
| Task 1 (fake transport) | **Built** — `hotplug_fake_test.go`, plus `isUnplugged`, `noteOpen`/`openCount`, `setRegister`, `closeCount`, and `encodeWordLE` |
| Task 2 (`busFactory`) | **Built** |
| Task 3 (poisoning fix) | **Built** — `lastError` is gone |
| Task 4 (session + `withSession`) | **Built, with two differences below** |
| Task 8's `setRegister` helper | **Built** (the generation guard itself is not) |
| Task 13 (gripper release) | **Built** |

Two API facts that differ from what this plan sketched:

1. **`withSession` acquires a lock.** There are two accessors, `withSession` (exclusive, for bus writes) and `withSessionRead` (shared, for reads), both taking `entry.busMu` — a mutex separate from `entry.mu`, which guards bookkeeping. The lock-free version sketched below was written for an already-shared controller; it is superseded. Lock order is `busMu → entry.mu`, and `report` is called with `busMu` held, so it must never take `busMu` (taking `entry.mu` is fine).
2. **`report` already exists** in `bus_session.go` as a generation-scoped pass-through with the signature `report(gen uint64, err error) error`. Task 5 replaces its **body**; do not re-declare it.

Also note: `entry.calibrationFor` was **not** built — method bodies read calibration off the session they already hold (`sess.servos[id].Calibration()`), which is what keeps the single source of truth structural. And `stopReconnectAndWait` is not built; it belongs with Task 6 here, since it references reconnect fields.

The lifecycle contracts this plan depends on are all in place: `tryPin` is refusable, `teardownIfLast` is once-guarded, `Release` reaches it holding no lock, and `(*ControllerRegistry).AcquireController` carries the documented seam comment where `resumeReconnect()` goes.

### Release boundary

The spec forbids shipping detection without reconnect: a disconnect verdict with no reopen and no escape hatch turns a degraded arm into a dead one. Tasks 1-5 may be released. **Tasks 6-11 must land together.** Commit per task regardless.

### Project conventions

- Build and test with `go test ./cmd/module/ .` — **never** `./...`. `cmd/cli/` is standalone debug scripts with multiple `func main()` and does not compile as a package.
- Format with `gofmt -s -w .` and run `go vet ./cmd/module/ .` before every commit.
- Run concurrency tests with `-race`. Every task here that touches a goroutine says so explicitly.
- `TestEnumerateSerialPorts` in `discovery_test.go` fails on machines with no serial hardware. That is environmental, not a regression you caused.
- A `*_arm.go` filename is treated by Go as a GOARCH=arm-only file. Do not create one.

---

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `hotplug_fake_test.go` | create | Concurrency-safe fake `feetech.Transport` that answers real protocol packets and can be flipped to fail. The keystone: without it nothing else here is testable. |
| `bus_session.go` | create | `busSession`, `ErrPortDisconnected`, `withSession`/`withSessionValue`. No policy, no goroutines. |
| `bus_session_test.go` | create | Session presence/absence and fail-fast timing. |
| `disconnect.go` | create | `classify` and `report` — the only place a disconnect verdict is reached. |
| `disconnect_test.go` | create | Classification table; strike escalation; generation scoping. |
| `reconnect.go` | create | The reconnect loop, backoff, pin/cancel ordering, `OnReconnect` subscription. |
| `reconnect_test.go` | create | Reconnect success, loop lifetime, restart-on-acquire, subscriber fan-out. |
| `registry.go` | modify | Entry gains session + gen + fails + reconnect fields; `busFactory` seam; poisoning fix; acquire starts a loop for a nil-session entry. The spec mentions a `connState` field; it is deliberately not built — `session == nil` plus `reconnectRunning()` already carries that state, and a third source of truth could disagree with them. |
| `registry_test.go` | modify | Poisoning regression. |
| `manager.go` | modify | Handle bus methods route through `withSession`; `WaitForServosToStop` generation guard. |
| `arm.go` | modify | Thread `ctx` through init; subscribe an `OnReconnect` callback; `reinitialize` forces a reopen. |
| `arm_reconnect_test.go` | create | The reconnect callback's logic, behind a seam (no hardware). |
| `gripper.go` | modify | Delete the stale `ReleaseSharedController()` at `:155`. |
| `README.md` | modify | Recovery behavior and the teleop caveat. |

---

## Task 1: The fake transport

> **BUILT** by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md` (commits `0bb226a..cee1543`). Kept for its rationale and code, which the prerequisite plan references. Skip to the next task; see this plan's prerequisite section for the two places the built version differs.

Everything else is untestable without this. `transports.MockTransport` cannot be used: its fields are read and written without a lock, and the reconnect loop runs on its own goroutine, so `-race` would fail. `transports.Script` is documented as single-goroutine only.

**Files:**
- Create: `hotplug_fake_test.go`

- [ ] **Step 1: Write the failing test**

```go
package so_arm

import (
	"context"
	"sync"
	"syscall"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestFakeTransportAnswersPing(t *testing.T) {
	ft := newFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	model, err := bus.Ping(context.Background(), 1)
	require.NoError(t, err)
	assert.Equal(t, 777, model, "fake should report the STS3215 model number")
}

func TestFakeTransportFailsWritesWhenUnplugged(t *testing.T) {
	ft := newFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	ft.unplug()
	_, err = bus.Ping(context.Background(), 1)
	require.Error(t, err)
	assert.ErrorIs(t, err, syscall.EIO, "an unplugged port surfaces its errno through the write path")

	ft.replug()
	_, err = bus.Ping(context.Background(), 1)
	assert.NoError(t, err)
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestFakeTransport -race .`
Expected: FAIL — `undefined: newFakeTransport`.

- [ ] **Step 3: Write the fake**

Append to `hotplug_fake_test.go`. The framing here is derived from `feetech/protocol.go`: a request is `[FF FF][id][len][instruction][params...][checksum]`, and a response puts the status byte where the instruction was, which is why `Encode` is called with `Instruction: 0`.

```go
// fakeTransport answers real Feetech protocol packets from an in-memory register file,
// and can be flipped to fail every write with syscall.EIO the way an unplugged port does
// (go.bug.st/serial's unixPort.Write returns the raw errno; serial_unix.go:112-118).
//
// Safe for concurrent use, unlike transports.MockTransport and transports.Script -- the
// reconnect loop drives a bus from its own goroutine, so -race demands it.
type fakeTransport struct {
	mu        sync.Mutex
	proto     *feetech.Protocol
	registers map[int]map[byte][]byte // servo id -> address -> value
	pending   []byte                  // queued response bytes
	unplugged bool
	writes    int
	opens     int
}

const fakeModelNumber = 777 // ModelSTS3215's model number

func newFakeTransport() *fakeTransport {
	ft := &fakeTransport{
		proto:     feetech.NewProtocol(feetech.ProtocolSTS),
		registers: make(map[int]map[byte][]byte),
	}
	for id := 1; id <= 6; id++ {
		ft.registers[id] = map[byte][]byte{
			feetech.RegModelNumber.Address: ft.proto.EncodeWord(fakeModelNumber),
		}
	}
	return ft
}

func (ft *fakeTransport) unplug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = true
	ft.pending = nil
}

func (ft *fakeTransport) replug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = false
}

func (ft *fakeTransport) isUnplugged() bool {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.unplugged
}

// noteOpen/openCount count reopen attempts, which is the only observable that moves while
// the port is down -- writes fail before they are counted.
func (ft *fakeTransport) noteOpen() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.opens++
}

func (ft *fakeTransport) openCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.opens
}

// writeCount reports how many packets the fake has accepted, so a test can assert that a
// reconnect loop stopped touching the port.
func (ft *fakeTransport) writeCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.writes
}

func (ft *fakeTransport) Write(p []byte) (int, error) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if ft.unplugged {
		return 0, syscall.EIO
	}
	ft.writes++
	ft.reply(p)
	return len(p), nil
}

func (ft *fakeTransport) Read(p []byte) (int, error) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if ft.unplugged {
		return 0, syscall.EIO
	}
	if len(ft.pending) == 0 {
		return 0, nil // no data yet; the bus retries until its deadline
	}
	n := copy(p, ft.pending)
	ft.pending = ft.pending[n:]
	return n, nil
}

func (ft *fakeTransport) Close() error                       { return nil }
func (ft *fakeTransport) SetReadTimeout(time.Duration) error  { return nil }
func (ft *fakeTransport) Flush() error                        { return nil }

// reply parses one request and queues its response. Called with ft.mu held.
func (ft *fakeTransport) reply(req []byte) {
	if len(req) < 6 || req[0] != 0xFF || req[1] != 0xFF {
		return
	}
	id, instruction, params := int(req[2]), req[4], req[5:len(req)-1]

	switch instruction {
	case 0x01: // PING -- status only
		ft.queue(id, nil)
	case 0x02: // READ [address, length]
		if len(params) < 2 {
			return
		}
		ft.queue(id, ft.value(id, params[0], int(params[1])))
	case 0x03, 0x04: // WRITE / REG_WRITE [address, data...]
		if len(params) < 1 {
			return
		}
		if regs, ok := ft.registers[id]; ok {
			regs[params[0]] = append([]byte(nil), params[1:]...)
		}
		ft.queue(id, nil)
	case 0x82: // SYNC_READ [address, length, ids...]
		if len(params) < 2 {
			return
		}
		address, length := params[0], int(params[1])
		for _, raw := range params[2:] {
			ft.queue(int(raw), ft.value(int(raw), address, length))
		}
	case 0x83: // SYNC_WRITE -- no response on the wire
	}
}

// value returns length bytes for a register, zero-filled when unset.
func (ft *fakeTransport) value(id int, address byte, length int) []byte {
	out := make([]byte, length)
	if regs, ok := ft.registers[id]; ok {
		copy(out, regs[address])
	}
	return out
}

func (ft *fakeTransport) queue(id int, params []byte) {
	if _, known := ft.registers[id]; !known {
		return // absent servo: silence, which the bus reports as ErrNoResponse
	}
	ft.pending = append(ft.pending,
		ft.proto.Encode(feetech.Packet{ID: byte(id), Instruction: 0, Parameters: params})...)
}

var _ feetech.Transport = (*fakeTransport)(nil)
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestFakeTransport -race .`
Expected: PASS, both tests.

If `TestFakeTransportAnswersPing` fails on a decode or checksum error, the bug is in `queue`'s framing — compare against `Encode` (`feetech/protocol.go:143`) and `Decode` (`:162`). `Decode` reads `data[4]` as the status byte, so a response must carry `Instruction: 0` there. Do not move on until both tests pass; every later task trusts this fake.

- [ ] **Step 5: Add an absent-servo test**

```go
func TestFakeTransportIsSilentForAbsentServos(t *testing.T) {
	ft := newFakeTransport()
	delete(ft.registers, 4)
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	_, err = bus.Ping(context.Background(), 4)
	assert.ErrorIs(t, err, feetech.ErrNoResponse,
		"a silent servo must be distinguishable from an unplugged port")
}
```

Run: `go test -run TestFakeTransport -race .`
Expected: PASS. This is the case the three-strike rule exists for — a quiet servo on a healthy bus.

- [ ] **Step 6: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add hotplug_fake_test.go
git commit -m "test: concurrency-safe fake feetech transport for hotplug tests"
```

---

## Task 2: `busFactory` seam on the registry

> **BUILT** by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md` (commits `0bb226a..cee1543`). Kept for its rationale and code, which the prerequisite plan references. Skip to the next task; see this plan's prerequisite section for the two places the built version differs.

**Files:**
- Modify: `registry.go` (the `ControllerRegistry` struct, `NewControllerRegistry`, and `createNewController`'s `feetech.NewBus` call at `:140`)
- Modify: `registry_test.go`

- [ ] **Step 1: Write the failing test**

Add to `registry_test.go`:

```go
func TestRegistryUsesInjectedBusFactory(t *testing.T) {
	r := NewControllerRegistry()
	var calls int
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		calls++
		cfg.Transport = newFakeTransport()
		return feetech.NewBus(cfg)
	}

	_, err := r.GetController("/dev/fake0", shortTimeoutConfig("/dev/fake0"),
		DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	assert.Equal(t, 1, calls, "the registry must open buses through the factory")
}
```

`registry_test.go:19` already has `testConfig(port)` — use it rather than adding a second helper. Its `Timeout` is `time.Second`, which is fine here; tests that need a short timeout narrow it:

```go
func shortTimeoutConfig(port string) *SoArm101Config {
	cfg := testConfig(port)
	cfg.Timeout = 50 * time.Millisecond
	return cfg
}
```

Substitute `shortTimeoutConfig` for `testControllerConfig` everywhere below. Note the factory in this test returns a *fresh* fake per call, which is correct **only here** — every reconnect test must do the opposite; see Task 4's helper block.

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestRegistryUsesInjectedBusFactory -race .`
Expected: FAIL — `r.busFactory undefined`.

- [ ] **Step 3: Add the field and use it**

In `registry.go`, add to `ControllerRegistry`:

```go
	// busFactory opens a bus. Injectable so tests can simulate an unplug and replug
	// without serial hardware; production always uses feetech.NewBus.
	busFactory func(feetech.BusConfig) (*feetech.Bus, error)
```

In `NewControllerRegistry`, set `busFactory: feetech.NewBus`. In `createNewController`, replace `bus, err := feetech.NewBus(busConfig)` with `bus, err := r.busFactory(busConfig)`.

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestRegistry -race .`
Expected: PASS, and every pre-existing `TestRegistry*` still passes.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go registry_test.go
git commit -m "refactor(registry): inject the bus factory"
```

---

## Task 3: Poisoning fix

> **BUILT** by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md` (commits `0bb226a..cee1543`). Kept for its rationale and code, which the prerequisite plan references. Skip to the next task; see this plan's prerequisite section for the two places the built version differs.

Independent of the session work, and it is the fix that lets viam-server's 5s `completeConfigWorker` retry actually reach the port.

**Files:**
- Modify: `registry.go:140-145`
- Modify: `registry_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestFailedOpenLeavesNoPoisonedEntry(t *testing.T) {
	r := NewControllerRegistry()
	var attempts int
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		attempts++
		if attempts == 1 {
			return nil, errors.New("no such device")
		}
		cfg.Transport = newFakeTransport()
		return feetech.NewBus(cfg)
	}
	cfg := shortTimeoutConfig("/dev/fake0")

	_, err := r.GetController("/dev/fake0", cfg, DefaultSO101FullCalibration, true)
	require.Error(t, err)

	r.mu.RLock()
	_, cached := r.entries["/dev/fake0"]
	r.mu.RUnlock()
	assert.False(t, cached, "a failed open must leave no entry for the retry to trip over")

	_, err = r.GetController("/dev/fake0", cfg, DefaultSO101FullCalibration, true)
	require.NoError(t, err, "the second attempt must reopen, not replay a cached error")
	assert.Equal(t, 2, attempts)
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestFailedOpenLeavesNoPoisonedEntry -race .`
Expected: FAIL — the entry is cached, and the second call returns `cached controller creation error`.

- [ ] **Step 3: Stop inserting the entry**

In `registry.go`, change the failure branch to not insert:

```go
	bus, err := r.busFactory(busConfig)
	if err != nil {
		// Deliberately no entry: caching the failure here made the port permanently
		// unusable, discarding viam-server's own 5s resource retry
		// (rdk robot/impl/local_robot.go:427).
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}
```

Then delete `lastError` from `ControllerEntry` and the branch that reads it (`registry.go:56-59`) if nothing else uses it — grep first: `grep -rn "lastError" *.go`. Sites that only *clear* it (`entry.lastError = nil`) go away with the field. One test sets it in a `ControllerEntry` literal (`registry_test.go:224`) to make a nil controller "valid" — that literal needs updating too, which is why `registry_test.go` is in this task's file list.

- [ ] **Step 4: Run to verify it passes**

Run: `go test ./cmd/module/ . 2>&1 | tail -20`
Expected: PASS. `TestEnumerateSerialPorts` may fail if this machine has no serial hardware — that is expected.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go registry_test.go
git commit -m "fix(registry): stop caching a failed bus open per port"
```

---

## Task 4: `busSession` and `withSession`

> **BUILT** by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md` (commits `0bb226a..cee1543`). Kept for its rationale and code, which the prerequisite plan references. Skip to the next task; see this plan's prerequisite section for the two places the built version differs.

Behavior-neutral by construction: the session is always non-nil until Task 6, so this is a wide mechanical refactor that must not change what any caller observes.

**Files:**
- Create: `bus_session.go`, `bus_session_test.go`
- Modify: `registry.go` (entry gains the session; `createNewController` builds one), `manager.go` (every bus method)

- [ ] **Step 1: Write the failing test**

`bus_session_test.go`:

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
	elapsed := time.Since(start)

	assert.ErrorIs(t, err, ErrPortDisconnected)
	assert.Less(t, elapsed, 10*time.Millisecond,
		"a disconnected call must not pay a serial timeout")
}
```

**Shared test helpers.** Put these in `bus_session_test.go`; Tasks 5-12 all use them. Write the `AcquireController` call against whatever signature Change B step 1 actually shipped.

```go
// testRegistry returns a registry whose bus factory opens every bus over `ft` -- the SAME
// fake the test holds.
//
// This is the most important detail in the hotplug test suite. A factory that creates a
// fresh fake per call (like Task 2's) makes every reconnect test pass vacuously:
// reconnectOnce's reopen would get a brand-new healthy transport no matter what
// ft.unplug() did, and writeCount() assertions would measure nothing.
//
// The factory also FAILS THE OPEN while the fake is unplugged. That models production,
// where serial.Open on a vanished device node errors -- feetech.NewBus with a non-nil
// Transport never fails on its own, so without this a fresh acquire during an outage would
// publish a live session over a dead port, which cannot happen with real hardware.
func testRegistry(ft *fakeTransport) *ControllerRegistry {
	r := NewControllerRegistry()
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		ft.noteOpen()
		if ft.isUnplugged() {
			return nil, syscall.ENXIO
		}
		cfg.Transport = ft
		return feetech.NewBus(cfg)
	}
	return r
}

// acquireTestHandle acquires one handle on /dev/fake0 and releases it at cleanup.
func acquireTestHandle(t *testing.T, r *ControllerRegistry) *ControllerHandle {
	t.Helper()
	h, err := r.AcquireController(shortTimeoutConfig("/dev/fake0"), DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	t.Cleanup(h.Release)
	return h
}

func testRegistryAndHandle(t *testing.T, ft *fakeTransport) (*ControllerRegistry, *ControllerHandle) {
	t.Helper()
	r := testRegistry(ft)
	return r, acquireTestHandle(t, r)
}

func testHandle(t *testing.T, ft *fakeTransport) *ControllerHandle {
	t.Helper()
	_, h := testRegistryAndHandle(t, ft)
	return h
}

// stopReconnectAndWait cancels the loop AND waits for it to exit. Test-only: production
// never joins the loop (see Task 6 Step 5). Tests need the join to reach a deterministic
// "entry alive, session nil, no loop" state.
// It must go through cancelReconnect rather than cancelling directly, so that `closing`
// gets set. Cancelling without it leaves closing false, the loop's cleanup finds a nil
// session and relaunches itself -- before close(done), so a new loop is already running by
// the time the wait returns -- and the "no loop" state this helper exists to reach is
// unreachable.
//
// Capture `done` BEFORE cancelling: the loop nils reconnectDone on its way out, so reading
// it afterwards races the exit.
func stopReconnectAndWait(t *testing.T, e *ControllerEntry) {
	t.Helper()
	e.reconnectMu.Lock()
	done := e.reconnectDone
	e.reconnectMu.Unlock()
	if done == nil {
		return
	}
	e.cancelReconnect()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("reconnect loop did not exit")
	}
}

// cloneCalibration deep-copies a calibration. SO101FullCalibration's six fields are all
// *MotorCalibration (config.go:33-40), so `cal := DefaultSO101FullCalibration` shares every
// pointer with the package-level default -- mutating the copy corrupts that default for
// every later test in the process, including TestGetControllerStatus, which branches on
// DefaultSO101FullCalibration.ShoulderPan.HomingOffset.
func cloneCalibration(src SO101FullCalibration) SO101FullCalibration {
	cp := func(m *MotorCalibration) *MotorCalibration {
		if m == nil {
			return nil
		}
		out := *m
		return &out
	}
	return SO101FullCalibration{
		ShoulderPan: cp(src.ShoulderPan), ShoulderLift: cp(src.ShoulderLift),
		ElbowFlex: cp(src.ElbowFlex), WristFlex: cp(src.WristFlex),
		WristRoll: cp(src.WristRoll), Gripper: cp(src.Gripper),
	}
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestWithSession -race .`
Expected: FAIL — `undefined: busSession`, `ErrPortDisconnected`.

- [ ] **Step 3: Write `bus_session.go`**

```go
package so_arm

import (
	"errors"
	"fmt"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// ErrPortDisconnected is returned by every bus operation while the port is down, in place
// of stalling a second per servo on serial timeouts.
var ErrPortDisconnected = errors.New("serial port disconnected")

// busSession is one open bus and everything derived from it. Immutable once published: a
// reconnect builds a new one and swaps it in, so an in-flight caller keeps using the
// session it loaded rather than observing a half-swapped one.
type busSession struct {
	bus    *feetech.Bus
	group  *feetech.ServoGroup
	servos map[int]*CalibratedServo
	gen    uint64
}

// withSession runs f against the live session, or fails fast when there is none.
func (h *ControllerHandle) withSession(f func(*busSession) error) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	return h.entry.report(sess.gen, f(sess))
}

// withSessionValue is withSession for operations that return a value.
func withSessionValue[T any](h *ControllerHandle, f func(*busSession) (T, error)) (T, error) {
	var zero T
	sess := h.entry.session.Load()
	if sess == nil {
		return zero, fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	v, err := f(sess)
	return v, h.entry.report(sess.gen, err)
}

// newBusSession assembles a session around an open bus, wrapping each servo in the
// calibration the entry already holds.
func newBusSession(bus *feetech.Bus, cal SO101FullCalibration, gen uint64) *busSession {
	raw := make(map[int]*feetech.Servo, 6)
	servos := make(map[int]*CalibratedServo, 6)
	for id := 1; id <= 6; id++ {
		raw[id] = feetech.NewServo(bus, id, &feetech.ModelSTS3215)
		mc := cal.GetMotorCalibrationByID(id)
		servos[id] = NewCalibratedServo(raw[id], &MotorCalibration{
			ID: mc.ID, DriveMode: mc.DriveMode, HomingOffset: mc.HomingOffset,
			RangeMin: mc.RangeMin, RangeMax: mc.RangeMax, NormMode: mc.NormMode,
		})
	}
	return &busSession{
		bus:    bus,
		group:  feetech.NewServoGroup(bus, raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]),
		servos: servos,
		gen:    gen,
	}
}
```

- [ ] **Step 4: Add the entry fields and a pass-through `report`**

In `registry.go`, add to `ControllerEntry`:

```go
	port    string                     // the port path, for error messages
	session atomic.Pointer[busSession] // nil == disconnected
	gen      atomic.Uint64             // monotonic session counter
```

Add a temporary pass-through in `registry.go` so Task 4 compiles and stays behavior-neutral. Task 5 replaces the body:

```go
// report is where a bus error becomes a connection verdict. Task 5 gives it teeth.
func (e *ControllerEntry) report(gen uint64, err error) error { return err }
```

Rewrite `createNewController` to build the session via `newBusSession` and `entry.session.Store(...)` instead of assembling `bus`/`group`/`calibratedServos` inline. The `!fromFile` register-read path stays where it is, but it must rebuild the session afterwards so the servos carry the calibration that was just read.

- [ ] **Step 5: Convert `manager.go`'s bus methods**

Every method on the handle that touched `s.bus`, `s.group`, or `s.calibratedServos` becomes a `withSession`/`withSessionValue` call. Mechanical; do them all in one pass. Example, from `MoveToJointPositions`:

```go
func (h *ControllerHandle) MoveToJointPositions(ctx context.Context, jointAngles []float64, speed, acc int) error {
	armServoIDs := []int{1, 2, 3, 4, 5}
	if len(jointAngles) != len(armServoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(armServoIDs), len(jointAngles))
	}
	return h.withSession(func(sess *busSession) error {
		rawPositions := make(map[int]int, len(jointAngles))
		for i, servoID := range armServoIDs {
			cal := h.entry.calibrationFor(servoID)
			raw, err := cal.Denormalize(utils.RadToDeg(jointAngles[i]))
			if err != nil {
				return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
			}
			rawPositions[servoID] = raw
		}
		return writePositions(ctx, sess, rawPositions, speed)
	})
}
```

`calibrationFor` is a new accessor on the entry, replacing today's
`s.calibration.GetMotorCalibrationByID(servoID)` (`manager.go:63`) now that the entry owns
the single collapsed calibration:

```go
func (e *ControllerEntry) calibrationFor(servoID int) *MotorCalibration {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.calibration.GetMotorCalibrationByID(servoID)
}
```

Two rules while converting:

1. **Argument validation stays outside `withSession`.** A malformed call should return its own error whether or not the port is up, and must never be reported as a bus error — `report` would count it as a strike.
2. **Do not wrap `Close`.** Closing the bus is session teardown, not a session operation.

- [ ] **Step 6: Run the full suite**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -30`
Expected: PASS. Any behavior change here is a bug in the refactor — the session is never nil yet.

- [ ] **Step 7: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add bus_session.go bus_session_test.go registry.go manager.go
git commit -m "refactor: hold the bus in a swappable busSession"
```

---

## Task 5: `classify` and a generation-scoped `report`

**Files:**
- Create: `disconnect.go`, `disconnect_test.go`

- [ ] **Step 1: Write the classification table test**

```go
func TestClassify(t *testing.T) {
	for _, tc := range []struct {
		name string
		err  error
		want disconnectKind
	}{
		{"nil", nil, kindHealthy},
		{"errno straight from unix.Write", syscall.EIO, kindTransport},
		{"errno behind feetech's write wrap",
			fmt.Errorf("write failed: %w", syscall.ENXIO), kindTransport},
		{"port closed under us", &serial.PortError{}, kindTransport},
		{"bus closed", feetech.ErrBusClosed, kindTransport},
		{"os.ErrClosed", os.ErrClosed, kindTransport},
		{"no response", feetech.ErrNoResponse, kindSilent},
		{"timeout", feetech.ErrTimeout, kindSilent},
		// A missing sync-read reply arrives as a *ServoError wrapping ErrNoResponse
		// (feetech/bus.go:271). Type-switching on *ServoError first would misfile it.
		{"sync-read gap wrapped in ServoError",
			&feetech.ServoError{ID: 3, Op: "sync_read", Err: feetech.ErrNoResponse}, kindSilent},
		{"servo status fault",
			&feetech.ServoError{ID: 3, Op: "read", Status: feetech.ErrOverload}, kindServo},
	} {
		t.Run(tc.name, func(t *testing.T) {
			assert.Equal(t, tc.want, classify(tc.err))
		})
	}
}
```

`feetech.ErrOverload` is a `StatusError` flag constant, declared in `feetech/protocol.go:47`'s block.

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestClassify -race .`
Expected: FAIL — `undefined: classify`.

- [ ] **Step 3: Write `classify`**

```go
package so_arm

import (
	"errors"
	"os"
	"syscall"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.bug.st/serial"
)

type disconnectKind int

const (
	kindHealthy   disconnectKind = iota // nil, or an error that says nothing about the link
	kindServo                           // one servo faulted; the link is fine
	kindSilent                          // nobody answered -- ambiguous, escalates on repeat
	kindTransport                       // the descriptor itself failed; the link is gone
)

// classify decides what a bus error implies about the serial link.
//
// Sentinels are checked BEFORE types on purpose. A missing sync-read reply arrives as a
// *feetech.ServoError wrapping ErrNoResponse (feetech/bus.go:271), and *ServoError
// implements Unwrap, so a type switch placed first would file a whole-bus outage as a
// single servo's fault and never escalate.
func classify(err error) disconnectKind {
	if err == nil {
		return kindHealthy
	}
	if errors.Is(err, feetech.ErrBusClosed) || errors.Is(err, os.ErrClosed) {
		return kindTransport
	}
	if errors.Is(err, feetech.ErrNoResponse) || errors.Is(err, feetech.ErrTimeout) {
		return kindSilent
	}
	// go.bug.st/serial's unixPort.Write returns the bare errno (serial_unix.go:112-118);
	// its Read returns a *PortError once the port is closed.
	var errno syscall.Errno
	if errors.As(err, &errno) {
		return kindTransport
	}
	var syscallErr *os.SyscallError
	if errors.As(err, &syscallErr) {
		return kindTransport
	}
	var portErr *serial.PortError
	if errors.As(err, &portErr) {
		return kindTransport
	}
	var servoErr *feetech.ServoError
	if errors.As(err, &servoErr) && servoErr.Status != 0 {
		return kindServo
	}
	return kindHealthy
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestClassify -race .`
Expected: PASS, all nine cases.

- [ ] **Step 5: Write the `report` tests**

```go
func TestReportDisconnectsImmediatelyOnTransportError(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	gen := h.entry.session.Load().gen

	h.entry.report(gen, syscall.EIO)
	assert.Nil(t, h.entry.session.Load(), "one transport error is enough")
}

func TestReportEscalatesSilenceOnTheThirdStrike(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	gen := h.entry.session.Load().gen

	h.entry.report(gen, feetech.ErrNoResponse)
	h.entry.report(gen, feetech.ErrNoResponse)
	require.NotNil(t, h.entry.session.Load(), "two silent reads are not a disconnect")

	h.entry.report(gen, feetech.ErrNoResponse)
	assert.Nil(t, h.entry.session.Load())
}

func TestReportResetsStrikesOnSuccess(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	gen := h.entry.session.Load().gen

	h.entry.report(gen, feetech.ErrNoResponse)
	h.entry.report(gen, feetech.ErrNoResponse)
	h.entry.report(gen, nil)
	h.entry.report(gen, feetech.ErrNoResponse)
	h.entry.report(gen, feetech.ErrNoResponse)
	assert.NotNil(t, h.entry.session.Load(),
		"strikes must be consecutive, not cumulative")
}

func TestReportIgnoresStaleGenerations(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	live := h.entry.session.Load()

	h.entry.report(live.gen-1, syscall.EIO)
	assert.NotNil(t, h.entry.session.Load(),
		"a caller blocked on a timeout must not tear down the session that replaced its own")
}

func TestReportPassesTheErrorThrough(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	sentinel := errors.New("boom")
	assert.ErrorIs(t, h.entry.report(h.entry.session.Load().gen, sentinel), sentinel)
}
```

- [ ] **Step 6: Run to verify they fail**

Run: `go test -run TestReport -race .`
Expected: FAIL — `report` is still the Task 4 pass-through.

- [ ] **Step 7: Implement `report`**

Replace the stub in `registry.go` (or move it to `disconnect.go`, which reads better — keep the classifier and the verdict together):

```go
// silentStrikesBeforeDisconnect is how many consecutive unanswered reads it takes to call
// the link dead. Three, not one: a single quiet servo on a perfectly healthy bus produces
// ErrNoResponse on every read of that servo, and one strike would tear down a working port.
const silentStrikesBeforeDisconnect = 3

// report classifies a bus error and, on a disconnect verdict, takes the session down.
// It always returns err unchanged so callers can wrap it as they like.
//
// gen is the generation the caller ran against. An error from a superseded session is
// ignored: a caller that blocked on a serial timeout and returned after a reconnect must
// not tear down the session that replaced its own, nor charge it a strike.
func (e *ControllerEntry) report(gen uint64, err error) error {
	live := e.session.Load()
	if live == nil || live.gen != gen {
		if err != nil && e.logger != nil {
			e.logger.Debugf("ignoring error from superseded session %d: %v", gen, err)
		}
		return err
	}

	switch classify(err) {
	case kindHealthy, kindServo:
		e.fails.Store(0)
		return err
	case kindSilent:
		if e.fails.Add(1) < silentStrikesBeforeDisconnect {
			return err
		}
	case kindTransport:
	}

	e.disconnect(live, err)
	return err
}

// disconnect nils the session for one generation and starts the reconnect loop. The CAS is
// what makes two concurrent verdicts idempotent: the loser's swap fails, so exactly one
// caller starts exactly one goroutine.
func (e *ControllerEntry) disconnect(live *busSession, cause error) {
	if !e.session.CompareAndSwap(live, nil) {
		return
	}
	e.fails.Store(0) // the next session starts with a full strike budget
	if e.logger != nil {
		e.logger.Warnf("serial port %s disconnected (%v); reconnecting in the background", e.port, cause)
	}
	_ = live.bus.Close() // best effort: closing a vanished device usually fails
	e.startReconnect()
}
```

`ControllerEntry` needs `fails atomic.Int32` and a `logger logging.Logger`. Add `startReconnect` as a no-op stub in `disconnect.go` and let Task 6 implement it in `reconnect.go`.

- [ ] **Step 8: Run to verify they pass**

Run: `go test -run 'TestReport|TestClassify|TestWithSession' -race .`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add disconnect.go disconnect_test.go registry.go
git commit -m "feat: classify bus errors and take the session down on a disconnect verdict"
```

---

## Task 6: The reconnect loop

**Do not release Tasks 5-11 separately.** After Task 5 a disconnect is permanent; this task is what makes it recoverable.

**Files:**
- Create: `reconnect.go`, `reconnect_test.go`
- Modify: `disconnect.go` (drop the `startReconnect` stub)

- [ ] **Step 1: Write the failing test**

```go
func TestReconnectRepublishesTheSessionAfterAReplug(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)
	before := h.entry.session.Load().gen

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	require.Nil(t, h.entry.session.Load())

	ft.replug()
	require.Eventually(t, func() bool { return h.entry.session.Load() != nil },
		5*time.Second, 20*time.Millisecond, "the loop should reopen once the port is back")

	assert.Greater(t, h.entry.session.Load().gen, before, "a reconnect publishes a new generation")
	assert.NoError(t, h.Ping(context.Background()))
}

func TestReconnectKeepsCalibrationAcrossAReplug(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)

	custom := cloneCalibration(DefaultSO101FullCalibration) // NOT a bare struct copy; see the helper
	custom.Gripper.RangeMin = 1234
	require.NoError(t, h.SetCalibration(custom))

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	ft.replug()
	require.Eventually(t, func() bool { return h.entry.session.Load() != nil },
		5*time.Second, 20*time.Millisecond)

	assert.Equal(t, 1234, h.entry.session.Load().servos[6].Calibration().RangeMin,
		"a reconnect must re-wrap servos in the calibration we already hold, not re-read registers")
}
```

Adjust `SetCalibration`/`Calibration()` to the names Change B step 1 actually shipped.

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestReconnect -race .`
Expected: FAIL — the session stays nil; `startReconnect` is a stub.

- [ ] **Step 3: Write `reconnect.go`**

```go
package so_arm

import (
	"context"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

const (
	reconnectBackoffMin = 250 * time.Millisecond
	reconnectBackoffMax = 5 * time.Second
	// reconnectLogEvery bounds how often a failing attempt is logged at Info, so an hour
	// unplugged is neither silent nor a flood.
	reconnectLogEvery = 30 * time.Second
)

// startReconnect launches the one reconnect goroutine for this entry, if it has none.
//
// The goroutine holds a pin for its whole life so teardown cannot close the entry
// underneath it. The shutdown order is the important part, because pins are what GATE
// teardown: refCount reaching zero cancels this context, the loop returns, and its
// deferred pin release is the decrement that performs teardown. The loop never waits on
// teardown, so there is no cycle.
func (e *ControllerEntry) startReconnect() {
	e.reconnectMu.Lock()
	defer e.reconnectMu.Unlock()

	if e.reconnectCancel != nil {
		return // already running
	}
	if !e.tryPin() {
		return // entry is already tearing down; nothing to reconnect for
	}

	ctx, cancel := context.WithCancel(context.Background())
	e.reconnectCancel = cancel
	done := make(chan struct{})
	e.reconnectDone = done

	go func() {
		defer close(done)
		defer e.unpin()
		defer func() {
			e.reconnectMu.Lock()
			e.reconnectCancel = nil
			e.reconnectDone = nil
			e.reconnectMu.Unlock()
		}()
		e.reconnectLoop(ctx)
	}()
}

// cancelReconnect cancels the loop WITHOUT waiting for it. Called from Release once
// refCount hits zero.
//
// It must not join: the loop's exit path performs the entry teardown (its unpin is the
// last decrement), and teardown takes entry.mu. A caller that joined here while holding
// entry.mu would deadlock, and a caller that joined at all would make Release synchronous
// with a teardown it is supposed to hand off.
func (e *ControllerEntry) cancelReconnect() {
	e.reconnectMu.Lock()
	defer e.reconnectMu.Unlock()
	e.closing = true
	if e.reconnectCancel != nil {
		e.reconnectCancel()
	}
}

// resumeReconnect is called after an acquire raises refCount, with NO entry lock held.
//
// Clearing `closing` is not bookkeeping -- it is the half that makes the flag correct. An
// entry that just gained a handle is by definition not closing, and without the clear the
// flag latches true forever: Release sets it and cancels, the rebuilt component's acquire
// declines on it, the exiting loop's cleanup declines on it too, and that loop's unpin then
// sees refCount 1 so it is not the last decrement and never tears down. Live entry,
// refCount 1, nil session, no loop, permanently -- the same dead end the rejected
// ctx.Err() guard produced, reached through the new predicate.
//
// If the previous loop is still winding down, startReconnectLocked declines (its cancel is
// still set) and the loop's own cleanup relaunches instead, because closing is now false.
// Either way exactly one loop ends up running.
func (e *ControllerEntry) resumeReconnect() {
	e.reconnectMu.Lock()
	defer e.reconnectMu.Unlock()
	e.closing = false
	if e.session.Load() == nil {
		e.startReconnectLocked()
	}
}

func (e *ControllerEntry) reconnectLoop(ctx context.Context) {
	backoff := reconnectBackoffMin
	lastLog := time.Time{}

	for {
		if err := e.reconnectOnce(); err == nil {
			return
		} else if e.logger != nil && time.Since(lastLog) >= reconnectLogEvery {
			e.logger.Infof("serial port %s still unavailable: %v", e.port, err)
			lastLog = time.Now()
		} else if e.logger != nil {
			e.logger.Debugf("reconnect attempt for %s failed: %v", e.port, err)
		}

		select {
		case <-ctx.Done():
			return
		case <-time.After(backoff):
		}
		if backoff *= 2; backoff > reconnectBackoffMax {
			backoff = reconnectBackoffMax
		}
	}
}

// reconnectOnce opens the port, proves the servos answer, and publishes a new session.
func (e *ControllerEntry) reconnectOnce() error {
	cfg := e.busConfig()
	bus, err := e.busFactory(cfg)
	if err != nil {
		return err
	}

	// An open that succeeds while the servos stay silent is not a reconnect: USB can
	// enumerate before servo power comes back.
	ctx, cancel := context.WithTimeout(context.Background(), cfg.Timeout*time.Duration(len(e.servoIDs())+1))
	defer cancel()
	for _, id := range e.servoIDs() {
		if _, err := bus.Ping(ctx, id); err != nil {
			_ = bus.Close()
			return err
		}
	}

	sess := newBusSession(bus, e.currentCalibration(), e.gen.Add(1))
	e.session.Store(sess)
	e.fails.Store(0) // a fresh session gets a full strike budget
	if e.logger != nil {
		e.logger.Infof("serial port %s reconnected (session %d)", e.port, sess.gen)
	}
	e.notifyReconnect()
	return nil
}
```

Add to `ControllerEntry`: `reconnectMu sync.Mutex`, `reconnectCancel context.CancelFunc`, `reconnectDone chan struct{}`, `closing bool` (all four guarded by `reconnectMu`), `reconnectOnceMu sync.Mutex`, and `busFactory` (copied from the registry when the entry is created, so the entry needs no registry back-pointer). `busConfig()`, `servoIDs()`, `currentCalibration()`, and `notifyReconnect()` are small accessors; leave `notifyReconnect()` empty — Task 9 fills it.

**`tryPin`/`unpin`: check what Change B step 1 actually built, and add what it didn't.** That spec designs `pins` as a counter for in-flight *acquires*; it does not define a refusable pin, which is what `startReconnect`'s early return needs. Write the contract down here if it is missing:

- `unpin()` decrements `pins`. Whichever of `refCount` and `pins` reaches zero **last** performs entry teardown (B's teardown rule).
- `tryPin() bool` increments `pins` and returns true, or returns false if teardown has already begun — so a reconnect loop is never started for an entry that is going away.

**Guard `reconnectOnce` with `reconnectOnceMu`.** A forced reconnect (Task 12) and a loop attempt can otherwise both open a bus; `session.Store` is atomic so the state stays sane, but the loser's bus leaks. Hold the mutex across the whole of `reconnectOnce`.

**Close the relaunch race in `startReconnect`, and key it off `closing` — not off the old context.** The deferred cleanup clears `reconnectCancel` *after* `reconnectLoop` returns, so a `disconnect` landing in that window sees a non-nil cancel, returns "already running", and is then orphaned by the exiting loop: a nil session with no loop, which is the permanent outage Task 7 exists to prevent. It is reachable on the success path, since the loop returns immediately after publishing a session a caller may instantly fail against.

Split `startReconnect` so the relaunch happens **without releasing `reconnectMu`** — otherwise a `cancelReconnect` landing in the gap reads a nil `reconnectCancel`, returns without cancelling, and the relaunched loop outlives `Release`, reopening a port no component holds:

```go
func (e *ControllerEntry) startReconnect() {
	e.reconnectMu.Lock()
	defer e.reconnectMu.Unlock()
	e.startReconnectLocked()
}

func (e *ControllerEntry) startReconnectLocked() {
	if e.reconnectCancel != nil || e.closing || !e.tryPin() {
		return
	}
	// ... create ctx/cancel/done, store them, launch the goroutine ...
}
```

and have the loop's cleanup relaunch under the same lock:

```go
		defer func() {
			e.reconnectMu.Lock()
			defer e.reconnectMu.Unlock()
			e.reconnectCancel = nil
			e.reconnectDone = nil
			if e.session.Load() == nil {
				e.startReconnectLocked() // declines if e.closing
			}
		}()
```

**Do not gate the relaunch on `ctx.Err()`.** It looks like the natural way to stop a cancelled loop from resurrecting itself, but it reintroduces the Task 7 failure: `Release` cancels, the rebuilt component's `AcquireController` calls `startReconnect` and is told "already running", then the exiting loop sees `ctx.Err() != nil` and declines — leaving a live entry, refCount 1, nil session, and no loop. `closing` is the correct predicate because `cancelReconnect` sets it and only a teardown sets it, so a cancel-for-shutdown declines while a cancel-then-reacquire does not.

Go runs defers last-in-first-out, so declare this cleanup *after* the `unpin` defer, making it run before the unpin — the relaunch's `tryPin` then still sees `pins >= 1`.

**Lock order: `reconnectMu` before `entry.mu`, never the reverse.** The relaunch path holds `reconnectMu` and calls `tryPin`, which consults teardown state under `entry.mu`. So `cancelReconnect` must never be called while holding `entry.mu` — which is exactly what Step 5's wiring is about.

**Jitter the backoff**, as the spec specifies, so a future multi-port machine cannot synchronize its retries:

```go
		wait := backoff/2 + time.Duration(rand.Int64N(int64(backoff/2)+1))
```

using `math/rand/v2`, and select on `wait` rather than `backoff`.

Delete the `startReconnect` stub from `disconnect.go`.

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestReconnect -race .`
Expected: PASS, both tests.

- [ ] **Step 5: Cancel the loop from `Release`, and let B's last-to-zero rule place the teardown**

This is the one place in the plan where the obvious wiring deadlocks. Read it before writing code.

Do **not** put the cancel inside teardown. The loop holds a pin for its whole life, and B's rule is that whichever of `refCount` and `pins` reaches zero **last** performs teardown — so while the loop runs, teardown cannot execute, and a cancel placed there would never fire.

Do **not** join the loop either. Its exit path performs the teardown and teardown takes `entry.mu`; joining while holding `entry.mu` (as today's `ReleaseController` does, `registry.go:238-239`) deadlocks against it.

What is left is just B's rule, applied unchanged — which also handles the ordinary shutdown where no loop ever ran:

```go
func (h *ControllerHandle) Release() {
	// ... B step 1's once-guard and refCount decrement ...
	if refCount > 0 {
		return
	}
	// Cancel with NO entry lock held: the relaunch path takes reconnectMu then entry.mu,
	// so holding entry.mu here would invert the lock order.
	h.entry.cancelReconnect()

	// B's rule, unchanged: last of refCount/pins to reach zero tears down. With a loop
	// running, pins > 0, so its unpin is last and IT tears down. With no loop -- the
	// normal case, and the one every existing test exercises -- pins is already 0, so
	// this decrement is last and Release tears down here. Skipping this branch would
	// resurrect the never-closed-bus bug that B step 1 exists to fix.
	h.entry.teardownIfLast()
}
```

Match `teardownIfLast` to whatever B step 1 named its last-to-zero check; the point is that `Release` still reaches it. Two constraints on it:

- **It must be a once-style guard, not a bare re-read of the two counters.** The loop's `unpin` can land between `cancelReconnect()` and this call, making the unpin the last decrement and the teardown already done; a predicate that re-reads `pins == 0 && refCount == 0` would then tear down twice. `feetech.Bus.Close()` is idempotent (`feetech/bus.go:98`), so the damage is limited to a double `delete(r.entries, ...)` and double field-nilling — but guard it with a `sync.Once` or a `torndown` flag rather than relying on that.
- **Do not give it a `Locked` suffix.** That convention means "call me with the lock held", which is the opposite of what this path requires, and a future reader following the suffix would reintroduce the inversion.

Then the lifetime test. Note the `Eventually`: `Release` cancels without joining, so the loop's exit is asynchronous by design.

```go
func TestReconnectLoopExitsWhenTheLastHandleGoesAway(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	require.Eventually(t, func() bool { return h.entry.reconnectRunning() },
		time.Second, 10*time.Millisecond)

	h.Release()

	// Eventually, not immediately: Release cancels without joining, because the loop's own
	// exit path performs the teardown. A test asserting a synchronous stop would be
	// asserting a join that must not exist.
	require.Eventually(t, func() bool { return !h.entry.reconnectRunning() },
		2*time.Second, 10*time.Millisecond, "Release must cancel the loop")

	// Count OPEN attempts, not writes: the factory now fails the open while unplugged and
	// fakeTransport.Write returns EIO before incrementing its counter, so writeCount()
	// cannot move while the port is down whether or not the loop is alive.
	before := ft.openCount()
	time.Sleep(4 * reconnectBackoffMin)
	assert.Equal(t, before, ft.openCount(), "no reopen attempts after the last release")
	_ = r
}
```

Add `reconnectRunning()` — `reconnectMu`-guarded `reconnectCancel != nil`. Note the assertion order: `Release` cancels the loop and the loop's pin release performs teardown. A test written as "teardown cancels and joins the loop" would encode the deadlock this design rejects.

- [ ] **Step 6: Run it**

Run: `go test -run TestReconnect -race .`
Expected: PASS.

- [ ] **Step 7: Add the cause-3 regression test**

The spec calls this its direct regression for cause 3 — each holder used to get a private copy of the bus pointer (`registry.go:98`, `:220`), which is the entire reason the session indirection exists. Nothing else in the plan asserts that a second holder observes a swap.

```go
func TestASwapReachesEveryHolder(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r, ft) // a second component on the same port

	before := h2.entry.session.Load().gen

	// Drive the disconnect entirely through h1.
	ft.unplug()
	require.Error(t, h1.Ping(context.Background()))
	assert.ErrorIs(t, h2.Ping(context.Background()), ErrPortDisconnected,
		"the other holder must see the disconnect, not keep writing to a dead descriptor")

	ft.replug()
	require.Eventually(t, func() bool { return h2.entry.session.Load() != nil },
		5*time.Second, 20*time.Millisecond)

	assert.Greater(t, h2.entry.session.Load().gen, before)
	assert.NoError(t, h2.Ping(context.Background()),
		"a reconnect driven through one holder must restore the other")
}
```

Run: `go test -run TestASwapReachesEveryHolder -race .`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add reconnect.go reconnect_test.go disconnect.go registry.go
git commit -m "feat: reconnect a disconnected serial port with backoff"
```

---

## Task 7: Restart the loop on re-acquire

Both the arm and the calibration sensor are `resource.AlwaysRebuild`. A reconfigure while unplugged runs `Close` (refCount to zero, loop cancelled) then a fresh `AcquireController`. The entry survives — the exiting loop's pin is outstanding — so refCount climbs back to 1 with no teardown, but nothing restarts the loop, and `withSession` returns early on a nil session without ever reaching `report`. The new handle would be disconnected forever.

**Files:**
- Modify: `registry.go` (`AcquireController` / `getExistingController`)
- Modify: `reconnect_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestAcquiringADisconnectedEntryRestartsTheLoop(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	require.Nil(t, h.entry.session.Load())

	// Reach the state an AlwaysRebuild cycle during an outage lands in -- entry alive,
	// session nil, no loop -- deterministically. Driving it via a real Release would race:
	// if the loop's unpin tore the entry down first, the re-acquire would open a fresh
	// entry instead of joining this one, and the test would prove nothing. `h` stays held
	// so refCount never reaches zero.
	stopReconnectAndWait(t, h.entry)
	require.False(t, h.entry.reconnectRunning())

	h2 := acquireTestHandle(t, r) // the rebuilt component joins the same entry
	require.Same(t, h.entry, h2.entry, "both handles must share one entry for this to test anything")
	require.True(t, h2.entry.reconnectRunning(),
		"a handle acquired into a disconnected entry must get a running loop")

	ft.replug()
	assert.Eventually(t, func() bool { return h2.entry.session.Load() != nil },
		5*time.Second, 20*time.Millisecond)
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestAcquiringADisconnectedEntry -race .`
Expected: FAIL — `reconnectRunning()` is false after re-acquire.

- [ ] **Step 3: Resume the loop on acquire — from `AcquireController`, not from `getExistingController`**

An entry can be joined while its port is down: an `AlwaysRebuild` cycle during an outage releases the last handle (cancelling the loop) and immediately acquires a new one. `withSession` returns early on a nil session and never reaches `report`, so nothing else would ever restart the loop for the new handle.

**Where the call goes matters more than the call.** The obvious site — the end of the existing-entry acquire path, after refCount is incremented (`registry.go:96-97`) — is inside `getExistingController`, which holds `entry.mu.Lock()` under a `defer` for its entire body (`registry.go:52-53`). Calling `resumeReconnect` there takes `entry.mu` → `reconnectMu` → `entry.mu` again via `tryPin`, which is a self-deadlock on a non-reentrant `sync.RWMutex` and inverts the lock order Task 6 states.

So call it in `AcquireController`, after `getExistingController` has returned and its lock is released:

```go
	h, err := r.getExistingController(entry, config, calibration, fromFile)
	if err != nil {
		return nil, err
	}
	entry.resumeReconnect() // entry.mu is released by now; see Task 6's lock order
	return h, nil
```

`resumeReconnect` is idempotent under `reconnectMu`, so a healthy entry pays one mutex acquisition and one atomic load.

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run 'TestAcquiring|TestReconnect' -race .`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add registry.go reconnect_test.go
git commit -m "fix(registry): restart the reconnect loop when a handle joins a down port"
```

---

## Task 8: `WaitForServosToStop` generation guard

The only place in the controller that holds session-derived references across time: it captures `*CalibratedServo` values under a brief read lock and then polls lock-free by design (`manager.go:331-341`). After a swap those servos belong to a closed bus — and its timeout path returns `nil` (`manager.go:362-365`), so it would report a completed move that never happened.

**Files:**
- Modify: `manager.go`
- Modify: `bus_session_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestWaitForServosToStopAbortsWhenTheSessionIsSwapped(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)
	// Park servo 1 as permanently moving so the wait keeps polling.
	ft.setRegister(1, feetech.RegMoving.Address, []byte{1})

	errCh := make(chan error, 1)
	go func() { errCh <- h.WaitForServosToStop(context.Background(), []int{1}, 5000) }()

	time.Sleep(100 * time.Millisecond)
	live := h.entry.session.Load()
	h.entry.disconnect(live, errors.New("cable pulled"))

	select {
	case err := <-errCh:
		assert.ErrorIs(t, err, ErrPortDisconnected,
			"a swapped session must abort the wait, not let it time out to nil")
	case <-time.After(2 * time.Second):
		t.Fatal("wait did not notice the session swap")
	}
}
```

Add `setRegister(id int, address byte, value []byte)` to the fake — a mutex-guarded write into `registers`. Confirm the real register name for "moving" in the feetech package (`grep -n "RegMoving\|Moving" $(go env GOMODCACHE)/github.com/hipsterbrown/feetech-servo@v0.6.0/feetech/*.go`) and use it in both fake and test.

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestWaitForServosToStopAborts -race .`
Expected: FAIL — the wait runs to its 5s timeout and returns nil.

- [ ] **Step 3: Add the guard**

```go
func (h *ControllerHandle) WaitForServosToStop(ctx context.Context, servoIDs []int, timeoutMs int) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	gen := sess.gen

	ids := make([]int, 0, len(servoIDs))
	servos := make([]*CalibratedServo, 0, len(servoIDs))
	for _, id := range servoIDs {
		if cs, ok := sess.servos[id]; ok {
			ids = append(ids, id)
			servos = append(servos, cs)
		}
	}
	// ... existing deadline and ticker setup ...

	for {
		// The servos captured above belong to `gen`. If the session was swapped, they now
		// point at a closed bus -- and the timeout path below returns nil, which would
		// report a move that never completed.
		if live := h.entry.session.Load(); live == nil || live.gen != gen {
			return fmt.Errorf("%w: %s (session changed while waiting)", ErrPortDisconnected, h.entry.port)
		}
		// ... existing polling body unchanged ...
	}
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestWaitForServos -race .`
Expected: PASS. Also run `go test -run TestArmWait -race .` — `arm_wait_test.go` covers this path.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add manager.go bus_session_test.go hotplug_fake_test.go
git commit -m "fix: abort a settle-wait whose session was swapped underneath it"
```

---

## Task 9: `OnReconnect` subscription

**Files:**
- Modify: `reconnect.go`, `registry.go`
- Modify: `reconnect_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestOnReconnectFiresOncePerReconnect(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)

	var calls atomic.Int32
	unsubscribe := h.OnReconnect(func() { calls.Add(1) })

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	ft.replug()
	require.Eventually(t, func() bool { return calls.Load() == 1 },
		5*time.Second, 20*time.Millisecond)

	unsubscribe()
	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	ft.replug()
	require.Eventually(t, func() bool { return h.entry.session.Load() != nil },
		5*time.Second, 20*time.Millisecond)
	assert.Equal(t, int32(1), calls.Load(), "an unsubscribed callback must not fire")
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestOnReconnect -race .`
Expected: FAIL — `h.OnReconnect` undefined.

- [ ] **Step 3: Implement it**

```go
// OnReconnect registers fn to run after the port comes back and a new session is
// published. Returns an unsubscribe func the holder MUST call in Close -- otherwise a
// rebuilt component's dead predecessor keeps getting notified.
//
// fn runs on the reconnect goroutine with no entry lock held, so its own bus calls
// succeed. It must not block indefinitely: it delays nothing else, but it does hold up
// further reconnect work for this port.
func (h *ControllerHandle) OnReconnect(fn func()) (unsubscribe func()) {
	e := h.entry
	e.subsMu.Lock()
	defer e.subsMu.Unlock()

	if e.subs == nil {
		e.subs = make(map[uint64]func())
	}
	id := e.nextSubID
	e.nextSubID++
	e.subs[id] = fn

	var once sync.Once
	return func() {
		once.Do(func() {
			e.subsMu.Lock()
			delete(e.subs, id)
			e.subsMu.Unlock()
		})
	}
}

// notifyReconnect runs every subscriber against a snapshot, so a callback that
// unsubscribes itself cannot deadlock on subsMu.
func (e *ControllerEntry) notifyReconnect() {
	e.subsMu.Lock()
	fns := make([]func(), 0, len(e.subs))
	for _, fn := range e.subs {
		fns = append(fns, fn)
	}
	e.subsMu.Unlock()

	for _, fn := range fns {
		fn()
	}
}
```

Add `subsMu sync.Mutex`, `subs map[uint64]func()`, `nextSubID uint64` to `ControllerEntry`, and delete the empty `notifyReconnect` stub from Task 6.

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestOnReconnect -race .`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add reconnect.go registry.go reconnect_test.go
git commit -m "feat: notify handle holders after a reconnect"
```

---

## Task 10: Thread `ctx` through the arm's init

Not a call-site change: `initializeServos` takes no context, and `doServoInitialization` (`arm.go:1120`), `diagnoseConnection` (`:1154`), and `verifyServoConfig` (`:1182`) each read `ctx := s.initCtx` internally. `initCtx` is the *construction* context (`arm.go:507-509`), long dead by the time a reconnect happens. The existing `reinitialize` DoCommand (`arm.go:927-937`) already runs against it today, so this is a latent bug fix as much as a prerequisite.

**Files:**
- Modify: `arm.go`

- [ ] **Step 1: Change the signatures**

```go
func (s *so101) initializeServos(ctx context.Context) error {
	return s.initializeServosWithRetry(ctx, 3)
}

func (s *so101) initializeServosWithRetry(ctx context.Context, maxRetries int) error { ... }
func (s *so101) doServoInitialization(ctx context.Context) error { ... }
func (s *so101) diagnoseConnection(ctx context.Context) error { ... }
func (s *so101) verifyServoConfig(ctx context.Context) error { ... }
```

Delete the three internal `ctx := s.initCtx` lines. Delete the `initCtx` field and its assignment — grep to confirm nothing else reads it: `grep -n "initCtx" *.go`.

- [ ] **Step 2: Update the call sites**

`NewSO101` passes its own `ctx`. The `diagnose`, `verify_config`, and `reinitialize` DoCommand cases pass the `ctx` that `DoCommand` already receives. Also replace the retry loop's `time.Sleep(waitTime)` with a ctx-aware wait, so a cancelled context does not sit through the backoff:

```go
			select {
			case <-ctx.Done():
				return ctx.Err()
			case <-time.After(waitTime):
			}
```

- [ ] **Step 3: Build and test**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS. No test constructs the hardware arm, so this is compile-and-regression only.

- [ ] **Step 4: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add arm.go
git commit -m "fix(arm): thread a live context through servo initialization"
```

---

## Task 11: The arm's reconnect callback

No test constructs the hardware arm — it needs serial hardware — so the callback's *logic* goes behind a seam and is tested there, following the `servoOps` precedent in `servo_commands.go:24`. Without the seam this task is review-only, which is how `NewSO101`'s `goalCloud` wiring ended up unguarded.

**Files:**
- Modify: `arm.go`
- Create: `arm_reconnect_test.go`

- [ ] **Step 1: Write the failing test**

```go
package so_arm

// reconnectRecorder records what the callback asked for, standing in for the arm.
type reconnectRecorder struct {
	exited bool
	reason string
	inited int
	initErr error
}

func (r *reconnectRecorder) exitManual(reason string) { r.exited = true; r.reason = reason }
func (r *reconnectRecorder) reinit(context.Context) error { r.inited++; return r.initErr }

func TestReconnectRestoresTorqueAndDropsManualMode(t *testing.T) {
	rec := &reconnectRecorder{}
	logger, logs := logging.NewObservedTestLogger(t)

	restoreAfterReconnect(context.Background(), rec, logger)

	assert.True(t, rec.exited, "manual mode must not survive a replug -- its torque_limit writes did not")
	assert.Equal(t, 1, rec.inited)
	assert.Equal(t, 0, logs.FilterLevelExact(zapcore.WarnLevel).Len())
}

func TestReconnectLogsButDoesNotPanicWhenInitFails(t *testing.T) {
	rec := &reconnectRecorder{initErr: errors.New("still silent")}
	logger, logs := logging.NewObservedTestLogger(t)

	restoreAfterReconnect(context.Background(), rec, logger)

	assert.Equal(t, 1, logs.FilterMessageSnippet("failed to reinitialize").Len(),
		"a failed re-init must be visible; the loop already published the session")
}
```

`zapcore` comes from `go.uber.org/zap`, currently `// indirect` in `go.mod`. It compiles as-is; `go mod tidy` will promote it to a direct dependency.

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestReconnectRestores -race .`
Expected: FAIL — `undefined: restoreAfterReconnect`.

- [ ] **Step 3: Write the seam and the callback**

In `arm.go`:

```go
// reconnectTarget is the slice of the arm restoreAfterReconnect needs. It exists so the
// restore logic is unit-testable: no test can build a hardware arm.
type reconnectTarget interface {
	exitManual(reason string)
	reinit(ctx context.Context) error
}

// restoreAfterReconnect brings the arm back after the port returns. A replug power-cycles
// the servos, so torque is off and manual mode's lowered torque_limit is gone -- manual
// mode is dropped rather than silently resurrected.
//
// It deliberately does NOT re-send the last commanded goal: a power-cycled STS3215 comes up
// holding wherever it physically is, and a stale goal would make the arm lurch across the
// gap it sagged through.
func restoreAfterReconnect(ctx context.Context, t reconnectTarget, logger logging.Logger) {
	t.exitManual("serial port reconnected")
	if err := t.reinit(ctx); err != nil {
		logger.Warnf("failed to reinitialize servos after reconnect: %v", err)
	}
}

func (s *so101) exitManual(reason string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.exitManualLocked(reason)
}

func (s *so101) reinit(ctx context.Context) error { return s.initializeServos(ctx) }

var _ reconnectTarget = (*so101)(nil)
```

`exitManual` takes and releases `s.mu` before `reinit` does bus I/O — never hold `s.mu` across a bus call from this path.

- [ ] **Step 4: Subscribe in the constructor**

In `NewSO101`, after the arm struct exists and before `initializeServos`:

```go
	arm.unsubscribeReconnect = controller.OnReconnect(func() {
		restoreAfterReconnect(arm.cancelCtx, arm, logger)
	})
```

Add the `unsubscribeReconnect func()` field, and in `Close`, before releasing the handle:

```go
	if s.unsubscribeReconnect != nil {
		s.unsubscribeReconnect()
	}
```

Order matters: unsubscribe before `cancelFunc()`, so a callback cannot start against a cancelled context.

- [ ] **Step 5: Run to verify it passes**

Run: `go test ./cmd/module/ . -race 2>&1 | tail -20`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add arm.go arm_reconnect_test.go
git commit -m "feat(arm): restore torque and drop manual mode after a reconnect"
```

---

## Task 12: `reinitialize` forces a reopen

The operator's escape hatch from the backoff wait. It and `controller_status` must reach the entry directly rather than through `withSession` — routing the whole DoCommand surface through the fail-fast rule would make the hatch unreachable exactly when it is needed.

**Files:**
- Modify: `reconnect.go`, `arm.go`
- Modify: `reconnect_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestForceReconnectSkipsTheBackoffWait(t *testing.T) {
	ft := newFakeTransport()
	h := testHandle(t, ft)

	ft.unplug()
	require.Error(t, h.Ping(context.Background()))
	require.Nil(t, h.entry.session.Load())

	ft.replug()
	require.NoError(t, h.ForceReconnect(), "an explicit reconnect must not wait for the loop")
	assert.NotNil(t, h.entry.session.Load())
}

func TestForceReconnectIsANoOpWhenConnected(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	gen := h.entry.session.Load().gen
	require.NoError(t, h.ForceReconnect())
	assert.Equal(t, gen, h.entry.session.Load().gen, "a healthy port must not be churned")
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `go test -run TestForceReconnect -race .`
Expected: FAIL — `h.ForceReconnect` undefined.

- [ ] **Step 3: Implement it**

```go
// ForceReconnect attempts a reopen immediately instead of waiting out the loop's backoff.
// A no-op when the port is already up. The loop keeps running either way -- if this
// attempt publishes a session, reconnectOnce's own success path ends the loop.
func (h *ControllerHandle) ForceReconnect() error {
	if h.entry.session.Load() != nil {
		return nil
	}
	return h.entry.reconnectOnce()
}
```

Concurrent `reconnectOnce` calls are safe: both may open a bus, but `session.Store` is atomic and the loser's bus is orphaned. Task 6 already guards it with `reconnectOnceMu` held for the whole of `reconnectOnce`, so a forced attempt and a loop attempt cannot both leave an unclosed bus behind.

That mutex therefore also covers `notifyReconnect`, so the arm's `initializeServos` retry budget is held across it. That is the spec's accepted tradeoff — a slow callback delays further reconnect work for this port and nothing else — and `ForceReconnect`'s early return means an operator's `reinitialize` never blocks behind it. Do not "fix" the lock scope by moving the notify outside it: subscribers would then be able to observe sessions out of order.

In `arm.go`'s `reinitialize` case, try the reopen first:

```go
	case "reinitialize":
		retries := 3
		if r, ok := cmd["retries"].(float64); ok {
			retries = int(r)
		}
		reopened := true
		if err := s.controller.ForceReconnect(); err != nil {
			reopened = false
			s.logger.Warnf("reinitialize could not reopen %s: %v", s.cfg.Port, err)
		}
		err := s.initializeServosWithRetry(ctx, retries)
		return map[string]interface{}{
			"success":  err == nil,
			"error":    fmt.Sprintf("%v", err),
			"retries":  retries,
			"reopened": reopened,
		}, nil
```

- [ ] **Step 4: Run to verify it passes**

Run: `go test -run TestForceReconnect -race . && go test ./cmd/module/ . -race 2>&1 | tail -10`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add reconnect.go arm.go reconnect_test.go
git commit -m "feat(arm): reinitialize forces an immediate reopen"
```

---

## Task 13: Delete the stale gripper release

> **BUILT** by `docs/superpowers/plans/2026-08-21-bus-ownership-step1.md` (commits `0bb226a..cee1543`). Kept for its rationale and code, which the prerequisite plan references. Skip to the next task; see this plan's prerequisite section for the two places the built version differs.

Independent of everything else — do it any time. The gripper has held no controller since the arm-dependency change (`gripper.go:94` documents it), and `releaseFromCaller` no-ops anyway.

**Files:**
- Modify: `gripper.go:153-157`

- [ ] **Step 1: Delete the call**

```go
	model, err := buildGripperModel(gripperType, meshDetail, conf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}
```

- [ ] **Step 2: Confirm no other stale calls**

Run: `grep -n "ReleaseSharedController" *.go`
Expected: only real holders (`arm.go`, `calibration.go`) — or, after Change B step 1, only `Release()` on a handle.

- [ ] **Step 3: Test and commit**

```bash
go test ./cmd/module/ . 2>&1 | tail -10
gofmt -s -w . && go vet ./cmd/module/ .
git add gripper.go
git commit -m "chore(gripper): drop a controller release the gripper never held"
```

---

## Task 14: Document it

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add a recovery section**

Under the arm model's section, following the existing prose style:

```markdown
### Recovering from an unplugged arm

If the arm's USB cable is pulled, the module notices and starts reopening the port in the
background, retrying until it succeeds. No restart is needed. While the port is down, every
arm and gripper call fails immediately with a "serial port disconnected" error rather than
hanging, and the arm disappears from the 3D viewer — a disconnected arm cannot report where
it is, and reporting a stale pose would be worse.

When the port comes back, the arm re-enables torque and holds wherever it physically is; it
does not move to the position it was last commanded to. If the arm was in manual
(hand-guided) mode, it returns in normal mode, because a replug power-cycles the servos and
clears the register writes manual mode depends on. Calibration is preserved.

To skip the retry wait, send the arm `{"command": "reinitialize"}`.

**Teleop does not resume on its own.** The teleop component stops after repeated errors and
stays stopped, so after a replug send it `{"command": "start"}` again.
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: describe serial reconnect behavior and the teleop caveat"
```

---

## Final verification

- [ ] `gofmt -l .` prints nothing
- [ ] `go vet ./cmd/module/ .` is clean
- [ ] `go test ./cmd/module/ . -race` passes (except `TestEnumerateSerialPorts` on hardware-free machines)
- [ ] `go test -run 'TestReconnect|TestReport|TestClassify|TestWithSession|TestForceReconnect|TestAcquiring|TestFakeTransport' -race -count=5 .` passes — repeated runs are what catch a flaky goroutine assertion
- [ ] `grep -rn "initCtx" *.go` returns nothing
- [ ] `grep -rn "lastError" *.go` returns nothing
- [ ] Every reconnect test's registry came from `testRegistry(ft)`, not a fresh-fake-per-call factory — a test that reopens onto a new fake passes vacuously

## What remains hardware-only

State these in the PR body; they are guarded by review, not by tests:

- `NewSO101`'s `OnReconnect` subscription line. Deleting it leaves the suite green and the arm limp after every replug — the same exposure `resolveGoalCloudConfig` has in that constructor.
- Whether a real unplugged FTDI/CDC device actually surfaces a write errno on this platform. The classifier is tested against `syscall.EIO`; that it is what the kernel returns for *this* hardware is an assumption. **Verify on hardware before merging:** unplug a running arm and confirm the log shows "disconnected" within one command, not a 3-strike escalation from timeouts.
- That a replugged arm's servos come up holding position rather than snapping to a stale goal.

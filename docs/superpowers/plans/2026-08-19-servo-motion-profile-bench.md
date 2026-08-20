# Servo Motion-Profile Bench Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone bench harness that measures what the Feetech STS3215 servos and the USB-serial bus actually do — write-rate ceiling, `GoalTime` accuracy, and acceleration-register behavior — so coordinated arrival, acceleration enforcement, and trajectory execution can be scoped from measurements instead of estimates.

**Architecture:** One self-contained `package main` file, `cmd/cli/profile_bench.go`, that talks to `github.com/hipsterbrown/feetech-servo/feetech` **directly** — never through the module's `SafeSoArmController`, and never importing the root `so_arm` package. It works in raw encoder ticks, so it runs on an uncalibrated arm and its measurements aren't conflated with calibration state. Pure functions (statistics, decode, profile recovery, limit checks) are exercised by a built-in `-selftest` mode that needs no hardware.

**Tech Stack:** Go 1.25, `github.com/hipsterbrown/feetech-servo v0.6.0`, stdlib only otherwise (`flag`, `encoding/csv`, `os/signal`, `time`, `sort`, `math`).

**Spec:** `docs/superpowers/specs/2026-08-19-servo-motion-profile-bench-design.md`

---

## Critical Context (read before Task 1)

Five facts, all verified against source. Getting any of them wrong produces plausible-looking wrong numbers rather than a visible failure — which is the whole risk of this harness living in untested code.

**1. `cmd/cli/` files are compiled individually, never as a package.**
Every file there is `package main` with its own `func main()`. `go vet ./cmd/cli/` fails with `main redeclared in this block`. Verification commands must therefore name the single file:

```bash
go build -o /dev/null cmd/cli/profile_bench.go   # exit 0
go vet cmd/cli/profile_bench.go                  # exit 0
```

Consequence: **`profile_bench.go` cannot call helpers defined in other `cmd/cli/` files.** It must be entirely self-contained.

**2. Do not import the root `so_arm` package.** The existing scripts there are already bit-rotted — `go build -o /dev/null cmd/cli/position_reader.go` fails today with `unknown field DefaultSpeed in struct literal of type so_arm.SoArm101Config`. Depending only on `feetech` keeps this harness immune. A file that imports only `feetech` builds, vets, and `gofmt`s clean (verified).

**3. `MinCommandGap: 0` is silently coerced to 1 ms.** `feetech.NewBus` does this at `bus.go:63-64` with no error:

```go
if cfg.MinCommandGap == 0 {
    cfg.MinCommandGap = time.Millisecond
}
```

Everywhere this harness means "no gap" it MUST pass `time.Nanosecond`, never `0`. A literal `0` makes the sweep's fastest row a mislabeled duplicate of the 1 ms row and caps motion sampling at ~1 kHz instead of the link rate.

**4. The 100 µs turnaround is not additive to the gap.** `sendPacketLocked` stamps `b.lastCmdTime = time.Now()` at `bus.go:486`, *before* `time.Sleep(100 * time.Microsecond)` at `bus.go:489`. The next call's `enforceCommandGap` measures elapsed time that already contains the sleep. Back-to-back transactions period at **~1 ms / ~1000 Hz**, not 1.1 ms / 900 Hz.

**5. The sign-magnitude decode helpers are unexported.** `decodeSignMagnitude` and `decodePositionWord` are package-private (`servo.go:386-400`), so the harness must reimplement them. `Protocol.DecodeWord` *is* exported, as are `Register.SignBit`, `Servo.PositionLimits`, and `Servo.SetTorqueEnabled`.

---

## File Structure

| File | Responsibility |
|---|---|
| Create: `cmd/cli/profile_bench.go` | Everything. Flags, safety, sampling, statistics, the three tests, CSV output, and `-selftest`. Self-contained by necessity (fact 1). |
| Create: `docs/superpowers/results/2026-08-19-bench-run.md` | Task 12 only — the recorded measurements the follow-on work gets scoped from. |

Internally the file is organised in this order, and tasks build it in this order:

1. flags + `main` dispatch
2. `-selftest` registry
3. statistics (percentiles, summaries)
4. decode + sample types
5. bus lifecycle + torque safety + signal handling
6. limit validation + travel planning
7. CSV writers
8. `writerate`
9. sampler loop
10. `goaltime` (single-joint, then `-full-arm`)
11. `accel`

---

## A Note on Testing

The spec deliberately places this harness in `cmd/cli/`, which CI neither compiles nor tests. That decision stands. This plan does **not** add root-package files or CI wiring.

Instead, every pure function gets a case in a built-in `-selftest` mode that runs with no hardware and no serial port. This is the honest analogue of unit tests within the constraint chosen: it costs ~40 lines in the same throwaway file, requires no new packages, and converts "hope the percentile math is right" into "verify it". The percentile, duration, and profile-recovery math is exactly where a silent bug would masquerade as a real measurement.

`-selftest` is the per-task verification gate for Tasks 2, 3, 5, and 6. Tasks that touch hardware I/O are verified by build + vet + a bench run in Task 12.

---

## Task 1: File skeleton, flags, and dispatch

**Files:**
- Create: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Create the file with flags, dispatch, and a self-test registry**

```go
// Command profile_bench characterizes STS3215 servo motion-profile behavior and the
// USB-serial bus, so coordinated arrival, acceleration enforcement, and trajectory
// execution can be scoped from measurements rather than estimates.
//
// See docs/superpowers/specs/2026-08-19-servo-motion-profile-bench-design.md
//
// Run one test at a time:
//
//	go run cmd/cli/profile_bench.go -port=/dev/tty.usbmodemXXXX -test=writerate
//	go run cmd/cli/profile_bench.go -port=/dev/tty.usbmodemXXXX -test=goaltime -move
//	go run cmd/cli/profile_bench.go -selftest
package main

import (
	"flag"
	"fmt"
	"os"
	"strings"
	"time"
)

// noGap is the "no command gap" sentinel. feetech.NewBus coerces a zero MinCommandGap
// back to 1ms silently (bus.go:63), so passing 0 would mislabel the fastest sweep row as
// a duplicate of the 1ms row and cap sampling at ~1kHz. Never pass 0.
const noGap = time.Nanosecond

// stepsPerDegree is the STS3215 encoder resolution: 4096 steps per full revolution.
const stepsPerDegree = 4096.0 / 360.0

type config struct {
	port         string
	test         string
	servo        int
	travelDeg    float64
	move         bool
	fullArm      bool
	assumeLimits string
	outDir       string
	selftest     bool
}

func main() {
	var cfg config
	flag.StringVar(&cfg.port, "port", "", "serial port (e.g. /dev/tty.usbmodem1101)")
	flag.StringVar(&cfg.test, "test", "writerate", "writerate|goaltime|accel|all")
	flag.IntVar(&cfg.servo, "servo", 1, "servo ID for single-joint tests")
	flag.Float64Var(&cfg.travelDeg, "travel", 20, "degrees of travel per trial (accel only; goaltime uses its own 10/20/40 sweep)")
	flag.BoolVar(&cfg.move, "move", false, "REQUIRED to allow any test that moves the arm")
	flag.BoolVar(&cfg.fullArm, "full-arm", false, "coordinated 5-joint goaltime run (needs -move)")
	flag.StringVar(&cfg.assumeLimits, "assume-limits", "", "<min>:<max> raw-tick bounds, for arms whose angle-limit registers read invalid")
	flag.StringVar(&cfg.outDir, "out", ".", "directory for CSV output")
	flag.BoolVar(&cfg.selftest, "selftest", false, "run built-in checks of the pure functions and exit (no hardware)")
	flag.Parse()

	if err := run(cfg); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}

func run(cfg config) error {
	if cfg.selftest {
		return runSelftests()
	}
	if cfg.port == "" {
		return fmt.Errorf("-port is required (or use -selftest)")
	}

	tests, err := selectTests(cfg.test, cfg.move)
	if err != nil {
		return err
	}
	if len(tests) == 0 {
		return fmt.Errorf("no tests to run: %q requires -move", cfg.test)
	}
	fmt.Printf("running: %s\n", strings.Join(tests, ", "))
	return fmt.Errorf("not yet implemented")
}

// selectTests resolves the -test flag into the list that will actually run. Motion tests
// are dropped without -move; "all" without -move degrades to writerate only and says so,
// rather than erroring, so "all" stays useful as a first run.
func selectTests(name string, move bool) ([]string, error) {
	motion := map[string]bool{"goaltime": true, "accel": true}
	var requested []string
	switch name {
	case "all":
		requested = []string{"writerate", "goaltime", "accel"}
	case "writerate", "goaltime", "accel":
		requested = []string{name}
	default:
		return nil, fmt.Errorf("unknown -test %q (want writerate|goaltime|accel|all)", name)
	}

	var out, skipped []string
	for _, t := range requested {
		if motion[t] && !move {
			skipped = append(skipped, t)
			continue
		}
		out = append(out, t)
	}
	if len(skipped) > 0 {
		fmt.Printf("skipping %s (motion tests require -move)\n", strings.Join(skipped, ", "))
	}
	return out, nil
}

// selftestCase is one hardware-free check of a pure function.
type selftestCase struct {
	name string
	fn   func() error
}

// selftests is appended to by each section of this file as it is built.
var selftests []selftestCase

func runSelftests() error {
	failed := 0
	for _, c := range selftests {
		if err := c.fn(); err != nil {
			fmt.Printf("FAIL %s: %v\n", c.name, err)
			failed++
			continue
		}
		fmt.Printf("PASS %s\n", c.name)
	}
	fmt.Printf("\n%d/%d passed\n", len(selftests)-failed, len(selftests))
	if failed > 0 {
		return fmt.Errorf("%d selftest(s) failed", failed)
	}
	return nil
}
```

- [ ] **Step 2: Verify it builds, vets, and is formatted**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output from `gofmt -l`. (`gofmt -l` printing the filename means it needs formatting — run `gofmt -s -w cmd/cli/profile_bench.go`.)

- [ ] **Step 3: Verify the dispatch behaves**

```bash
go run cmd/cli/profile_bench.go -selftest
go run cmd/cli/profile_bench.go -test=all -port=/dev/null
go run cmd/cli/profile_bench.go -test=bogus -port=/dev/null
```

Expected, in order: `0/0 passed`; then `skipping goaltime, accel (motion tests require -move)` followed by `running: writerate` and the `not yet implemented` error; then `error: unknown -test "bogus"` with exit 1.

- [ ] **Step 4: Verify the module's own tests are unaffected**

```bash
go test ./cmd/module/ .
```

Expected: PASS. Nothing in `cmd/cli/` is part of this command; this confirms the new file changed nothing. Note `TestEnumerateSerialPorts` in `discovery_test.go` fails on machines with no serial hardware — that is environment-dependent and not a regression.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): profile_bench skeleton with flags and selftest registry"
```

---

## Task 2: Statistics

Percentiles summarize the write-rate jitter that decides whether timed streaming is viable. Nearest-rank is used (not interpolation) so the reported value is always an observed sample.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the statistics section and its selftests**

Append to the file:

```go
// --- statistics ---

// latencyStats summarizes a set of durations. Nearest-rank percentiles, so every reported
// value is an actually-observed sample rather than an interpolation between two.
type latencyStats struct {
	n      int
	mean   time.Duration
	p50    time.Duration
	p95    time.Duration
	p99    time.Duration
	max    time.Duration
	rateHz float64 // 1/mean
}

// percentile returns the nearest-rank pth percentile of an already-sorted slice.
// p is in [0,100]. Panics on an empty slice, which is a programming error here.
func percentile(sorted []time.Duration, p float64) time.Duration {
	if len(sorted) == 0 {
		panic("percentile: empty slice")
	}
	idx := int(math.Ceil(p/100*float64(len(sorted)))) - 1
	if idx < 0 {
		idx = 0
	}
	if idx >= len(sorted) {
		idx = len(sorted) - 1
	}
	return sorted[idx]
}

func summarize(d []time.Duration) latencyStats {
	if len(d) == 0 {
		return latencyStats{}
	}
	sorted := append([]time.Duration(nil), d...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i] < sorted[j] })

	var total time.Duration
	for _, v := range sorted {
		total += v
	}
	mean := total / time.Duration(len(sorted))

	st := latencyStats{
		n:    len(sorted),
		mean: mean,
		p50:  percentile(sorted, 50),
		p95:  percentile(sorted, 95),
		p99:  percentile(sorted, 99),
		max:  sorted[len(sorted)-1],
	}
	if mean > 0 {
		st.rateHz = float64(time.Second) / float64(mean)
	}
	return st
}
```

Add `"math"` and `"sort"` to the import block.

- [ ] **Step 2: Add the selftest cases**

Append:

```go
func init() {
	selftests = append(selftests,
		selftestCase{"percentile/nearest-rank", func() error {
			// 1..100 ms. Nearest-rank: p50 -> ceil(50)=50 -> idx 49 -> 50ms.
			var d []time.Duration
			for i := 1; i <= 100; i++ {
				d = append(d, time.Duration(i)*time.Millisecond)
			}
			for _, tc := range []struct {
				p    float64
				want time.Duration
			}{
				{50, 50 * time.Millisecond},
				{95, 95 * time.Millisecond},
				{99, 99 * time.Millisecond},
				{100, 100 * time.Millisecond},
				{0, 1 * time.Millisecond},
			} {
				if got := percentile(d, tc.p); got != tc.want {
					return fmt.Errorf("p%v = %v, want %v", tc.p, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"summarize/unsorted-input", func() error {
			// Deliberately unsorted: summarize must sort its own copy and must not
			// mutate the caller's slice.
			in := []time.Duration{
				5 * time.Millisecond, 1 * time.Millisecond, 3 * time.Millisecond,
				2 * time.Millisecond, 4 * time.Millisecond,
			}
			orig := append([]time.Duration(nil), in...)
			st := summarize(in)
			for i := range in {
				if in[i] != orig[i] {
					return fmt.Errorf("summarize mutated its input at %d", i)
				}
			}
			if st.n != 5 {
				return fmt.Errorf("n = %d, want 5", st.n)
			}
			if st.mean != 3*time.Millisecond {
				return fmt.Errorf("mean = %v, want 3ms", st.mean)
			}
			if st.max != 5*time.Millisecond {
				return fmt.Errorf("max = %v, want 5ms", st.max)
			}
			// mean 3ms -> 333.33Hz
			if st.rateHz < 333.0 || st.rateHz > 334.0 {
				return fmt.Errorf("rateHz = %v, want ~333.3", st.rateHz)
			}
			return nil
		}},
		selftestCase{"summarize/empty", func() error {
			if got := summarize(nil); got.n != 0 {
				return fmt.Errorf("n = %d, want 0", got.n)
			}
			return nil
		}},
	)
}
```

- [ ] **Step 3: Run the selftests**

```bash
go run cmd/cli/profile_bench.go -selftest
```

Expected: three `PASS` lines and `3/3 passed`, exit 0.

- [ ] **Step 4: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): nearest-rank percentile and latency summary"
```

---

## Task 3: Sample decoding

`RegPresentPosition` (addr 56, 2 bytes, `SignBit: 15`) and `RegPresentVelocity` (addr 58, 2 bytes, `SignBit: 15`) are adjacent, so one `SyncRead(ctx, 56, 4, ids)` returns both per servo in a single round trip. Both are sign-magnitude, and the library's decoder is unexported (fact 5), so it is reimplemented here.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the decode section**

```go
// --- sample decoding ---

// sample is one timestamped reading of a servo's position and velocity.
type sample struct {
	t   time.Duration // since the start of the trial
	pos int           // raw encoder ticks
	vel int           // raw steps/sec, signed
}

// decodeSignMagnitude mirrors feetech's unexported helper (servo.go:390). The high bit at
// signBit is a sign flag, not a two's-complement extension: the remaining bits are the
// magnitude.
func decodeSignMagnitude(value, signBit int) int {
	if signBit == 0 {
		return value
	}
	signMask := 1 << signBit
	if value&signMask != 0 {
		return -(value & (signMask - 1))
	}
	return value
}

// decodePosVel splits the 4-byte payload of a SyncRead at RegPresentPosition into
// position and velocity. Returns an error rather than silently decoding garbage if the
// servo returned a short payload.
func decodePosVel(proto *feetech.Protocol, data []byte) (pos, vel int, err error) {
	if len(data) < 4 {
		return 0, 0, fmt.Errorf("short payload: %d bytes, want 4", len(data))
	}
	pos = decodeSignMagnitude(int(proto.DecodeWord(data[0:2])), feetech.RegPresentPosition.SignBit)
	vel = decodeSignMagnitude(int(proto.DecodeWord(data[2:4])), feetech.RegPresentVelocity.SignBit)
	return pos, vel, nil
}

func degToSteps(deg float64) int { return int(math.Round(deg * stepsPerDegree)) }
func stepsToDeg(steps int) float64 { return float64(steps) / stepsPerDegree }
```

Add `"github.com/hipsterbrown/feetech-servo/feetech"` to the imports.

- [ ] **Step 2: Add the selftest cases**

```go
func init() {
	selftests = append(selftests,
		selftestCase{"decodeSignMagnitude", func() error {
			for _, tc := range []struct{ in, bit, want int }{
				{100, 15, 100},          // positive
				{0x8064, 15, -100},      // sign bit set -> negative magnitude
				{0, 15, 0},              // zero
				{0x8000, 15, 0},         // negative zero decodes to 0
				{2048, 15, 2048},        // mid-range positive
				{43, 0, 43},             // signBit 0 -> passthrough (odd: also detects a missing guard)
			} {
				if got := decodeSignMagnitude(tc.in, tc.bit); got != tc.want {
					return fmt.Errorf("decodeSignMagnitude(%#x, %d) = %d, want %d",
						tc.in, tc.bit, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"decodePosVel", func() error {
			proto := feetech.NewProtocol(feetech.ProtocolSTS)
			// STS is little-endian: pos = 0x0064 = 100, vel = 0x8064 = -100.
			data := []byte{0x64, 0x00, 0x64, 0x80}
			pos, vel, err := decodePosVel(proto, data)
			if err != nil {
				return err
			}
			if pos != 100 {
				return fmt.Errorf("pos = %d, want 100", pos)
			}
			if vel != -100 {
				return fmt.Errorf("vel = %d, want -100", vel)
			}
			if _, _, err := decodePosVel(proto, []byte{0x00, 0x01}); err == nil {
				return fmt.Errorf("short payload accepted, want error")
			}
			return nil
		}},
		selftestCase{"degToSteps roundtrip", func() error {
			// 360 deg = 4096 steps exactly.
			if got := degToSteps(360); got != 4096 {
				return fmt.Errorf("degToSteps(360) = %d, want 4096", got)
			}
			if got := degToSteps(20); got != 228 { // 20 * 11.3777... = 227.55 -> 228
				return fmt.Errorf("degToSteps(20) = %d, want 228", got)
			}
			if d := stepsToDeg(4096); math.Abs(d-360) > 1e-9 {
				return fmt.Errorf("stepsToDeg(4096) = %v, want 360", d)
			}
			return nil
		}},
	)
}
```

**Note on the byte order:** the selftest asserts little-endian because `feetech`'s STS protocol uses it. If `decodePosVel` fails this case, check `Protocol.byteOrder` for `ProtocolSTS` before changing the test — the test encodes the real wire contract.

- [ ] **Step 3: Run the selftests**

```bash
go run cmd/cli/profile_bench.go -selftest
```

Expected: six `PASS` lines, `6/6 passed`, exit 0.

- [ ] **Step 4: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): sign-magnitude position/velocity decoding"
```

---

## Task 4: Bus lifecycle and torque safety

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the bus section**

```go
// --- bus lifecycle ---

// armServoIDs is the arm chain. Servo 6 is the gripper and is never commanded here.
var armServoIDs = []int{1, 2, 3, 4, 5}

type rig struct {
	bus   *feetech.Bus
	group *feetech.ServoGroup
	proto *feetech.Protocol
	ids   []int
}

// openRig connects to the bus with an explicit command gap. Pass noGap, never 0 — a zero
// value is silently coerced to 1ms by feetech.NewBus (bus.go:63).
func openRig(port string, gap time.Duration, ids []int) (*rig, error) {
	if gap == 0 {
		return nil, fmt.Errorf("internal error: MinCommandGap 0 is coerced to 1ms; pass noGap")
	}
	bus, err := feetech.NewBus(feetech.BusConfig{
		Port:          port,
		BaudRate:      1000000,
		Protocol:      feetech.ProtocolSTS,
		Timeout:       time.Second,
		MinCommandGap: gap,
	})
	if err != nil {
		return nil, fmt.Errorf("open bus on %s: %w", port, err)
	}

	servos := make([]*feetech.Servo, 0, len(ids))
	for _, id := range ids {
		servos = append(servos, feetech.NewServo(bus, id, &feetech.ModelSTS3215))
	}
	return &rig{
		bus:   bus,
		group: feetech.NewServoGroup(bus, servos...),
		proto: bus.Protocol(),
		ids:   ids,
	}, nil
}

func (r *rig) close() error { return r.bus.Close() }

// disableTorque drops torque on every servo in the rig, best-effort. Used both as the
// deliberate resting state for writerate and as the teardown for motion tests.
func (r *rig) disableTorque(ctx context.Context) {
	for _, id := range r.ids {
		s := r.group.ServoByID(id)
		if s == nil {
			continue
		}
		if err := s.SetTorqueEnabled(ctx, false); err != nil {
			fmt.Fprintf(os.Stderr, "warning: disable torque on servo %d: %v\n", id, err)
		}
	}
}

// withSafeShutdown runs fn with a context cancelled on SIGINT/SIGTERM, and guarantees
// torque is dropped on every exit path — normal return, error, or signal.
func (r *rig) withSafeShutdown(fn func(ctx context.Context) error) error {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	defer func() {
		// A fresh context: the shutdown context may already be cancelled, and the
		// torque-off write must still go out.
		r.disableTorque(context.Background())
	}()
	return fn(ctx)
}
```

Add `"context"`, `"os/signal"`, and `"syscall"` to the imports.

- [ ] **Step 2: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 3: Confirm the zero-gap guard fires**

There is no selftest for this (it needs a port), so confirm by inspection that `openRig` rejects `gap == 0` before calling `NewBus`. This guard is the last line of defence against fact 3.

- [ ] **Step 4: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): bus lifecycle with torque-safe shutdown"
```

---

## Task 5: Limit validation and travel planning

Angle-limit registers are not always valid — they legitimately read `0/0` on a multi-turn servo or one with limits disabled, which is why `config.go:424` guards them with `minLimit < maxLimit && maxLimit <= 4095`. Without the same guard here, every target would clamp to zero and every trial would be refused.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the limits section**

```go
// --- limits and travel planning ---

// travelPlan is a validated out-and-back move for one servo.
type travelPlan struct {
	id     int
	start  int // raw ticks, read at startup
	target int // raw ticks
}

// limitsValid mirrors the guard at config.go:424. Angle limits can legitimately read 0/0
// on a multi-turn servo or one with limits disabled; trusting those would clamp every
// target to zero.
func limitsValid(min, max int) bool {
	return min < max && max <= 4095
}

// parseAssumeLimits parses the -assume-limits "<min>:<max>" override.
func parseAssumeLimits(s string) (min, max int, err error) {
	if s == "" {
		return 0, 0, fmt.Errorf("empty")
	}
	parts := strings.Split(s, ":")
	if len(parts) != 2 {
		return 0, 0, fmt.Errorf("want <min>:<max>, got %q", s)
	}
	if min, err = strconv.Atoi(strings.TrimSpace(parts[0])); err != nil {
		return 0, 0, fmt.Errorf("min: %w", err)
	}
	if max, err = strconv.Atoi(strings.TrimSpace(parts[1])); err != nil {
		return 0, 0, fmt.Errorf("max: %w", err)
	}
	if !limitsValid(min, max) {
		return 0, 0, fmt.Errorf("invalid bounds %d:%d (need min < max <= 4095)", min, max)
	}
	return min, max, nil
}

// planTravel validates that start +/- travel fits inside [min,max] and returns the plan.
// It REFUSES rather than clamping: a silently shortened trial reports a real duration for
// a move that never happened, which is worse than no measurement.
func planTravel(id, start, travelSteps, min, max int) (travelPlan, error) {
	if !limitsValid(min, max) {
		return travelPlan{}, fmt.Errorf(
			"servo %d: angle limits read %d/%d, which fail the min<max<=4095 guard; "+
				"pass -assume-limits=<min>:<max> to proceed", id, min, max)
	}
	if start < min || start > max {
		return travelPlan{}, fmt.Errorf(
			"servo %d: start position %d is outside limits [%d,%d]", id, start, min, max)
	}
	target := start + travelSteps
	if target < min || target > max {
		return travelPlan{}, fmt.Errorf(
			"servo %d: travel of %d steps from %d would reach %d, outside limits [%d,%d]; "+
				"reduce -travel or reposition the arm", id, travelSteps, start, target, min, max)
	}
	return travelPlan{id: id, start: start, target: target}, nil
}
```

Add `"strconv"` to the imports.

- [ ] **Step 2: Add the selftest cases**

```go
func init() {
	selftests = append(selftests,
		selftestCase{"limitsValid", func() error {
			for _, tc := range []struct {
				min, max int
				want     bool
			}{
				{0, 4095, true},
				{100, 3000, true},
				{0, 0, false},    // limits disabled / multi-turn
				{3000, 100, false}, // inverted
				{0, 4096, false},   // beyond resolution
				{500, 500, false},  // degenerate
			} {
				if got := limitsValid(tc.min, tc.max); got != tc.want {
					return fmt.Errorf("limitsValid(%d,%d) = %v, want %v",
						tc.min, tc.max, got, tc.want)
				}
			}
			return nil
		}},
		selftestCase{"planTravel/refuses-rather-than-clamps", func() error {
			// Fits: 2048 + 228 = 2276, inside [0,4095].
			p, err := planTravel(1, 2048, 228, 0, 4095)
			if err != nil {
				return fmt.Errorf("valid plan rejected: %w", err)
			}
			if p.target != 2276 {
				return fmt.Errorf("target = %d, want 2276", p.target)
			}
			// Would clip: must error, NOT clamp to 4095.
			if _, err := planTravel(1, 4000, 228, 0, 4095); err == nil {
				return fmt.Errorf("clipping travel accepted, want refusal")
			}
			// Start outside the limits must be refused even when the target lands
			// back inside: -100 + 228 = 128, which looks valid. Only the start guard
			// catches this, so this case is what keeps that guard honest.
			if _, err := planTravel(1, -100, 228, 0, 4095); err == nil {
				return fmt.Errorf("start outside limits accepted, want refusal")
			}
			// Negative direction clips low.
			if _, err := planTravel(1, 100, -228, 0, 4095); err == nil {
				return fmt.Errorf("negative clipping travel accepted, want refusal")
			}
			// Invalid limits (0/0) must be reported as such, not clamp everything to 0.
			_, err = planTravel(1, 2048, 228, 0, 0)
			if err == nil {
				return fmt.Errorf("invalid limits accepted, want refusal")
			}
			if !strings.Contains(err.Error(), "assume-limits") {
				return fmt.Errorf("invalid-limits error should name -assume-limits, got: %v", err)
			}
			return nil
		}},
		selftestCase{"parseAssumeLimits", func() error {
			min, max, err := parseAssumeLimits("100:3000")
			if err != nil || min != 100 || max != 3000 {
				return fmt.Errorf("got (%d,%d,%v), want (100,3000,nil)", min, max, err)
			}
			for _, bad := range []string{"", "100", "100:200:300", "100:50", "a:b", "0:5000"} {
				if _, _, err := parseAssumeLimits(bad); err == nil {
					return fmt.Errorf("parseAssumeLimits(%q) accepted, want error", bad)
				}
			}
			return nil
		}},
	)
}
```

- [ ] **Step 3: Run the selftests**

```bash
go run cmd/cli/profile_bench.go -selftest
```

Expected: nine `PASS` lines, `9/9 passed`, exit 0.

- [ ] **Step 4: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): angle-limit guard and refuse-on-clip travel planning"
```

---

## Task 6: Profile recovery

These are the functions that turn raw samples into the numbers the follow-on work gets scoped from. A bug here looks exactly like a real measurement, so every one has a selftest against a synthetic profile with a hand-computed answer.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the profile section**

```go
// --- profile recovery ---

// velEpsilon is the |velocity| below which a servo is considered stopped. The STS3215
// reports velocity in whole steps/sec, so anything non-zero is real motion.
const velEpsilon = 1

// moveProfile is what one commanded move looks like after the fact.
type moveProfile struct {
	startIdx, endIdx int           // sample indices bounding the motion
	duration         time.Duration // motion start to motion end
	peakVel          int           // max |velocity| observed, steps/sec
	timeToPeak       time.Duration // motion start to first sample within 95% of peak
	overshootSteps   int           // furthest travel beyond target, 0 if none
	moved            bool          // false if the servo never moved at all
}

// analyzeMove recovers the motion profile from a sample series. target is the commanded
// position in raw ticks; direction is inferred from the first and last positions.
func analyzeMove(samples []sample, target int) moveProfile {
	var p moveProfile
	if len(samples) == 0 {
		return p
	}

	first, last := -1, -1
	for i, s := range samples {
		if abs(s.vel) >= velEpsilon {
			if first < 0 {
				first = i
			}
			last = i
		}
	}
	if first < 0 {
		return p // never moved
	}
	p.moved = true
	p.startIdx, p.endIdx = first, last
	p.duration = samples[last].t - samples[first].t

	for _, s := range samples {
		if v := abs(s.vel); v > p.peakVel {
			p.peakVel = v
		}
	}

	// Time to reach 95% of peak, measured from motion start.
	threshold := p.peakVel * 95 / 100
	for i := first; i <= last; i++ {
		if abs(samples[i].vel) >= threshold {
			p.timeToPeak = samples[i].t - samples[first].t
			break
		}
	}

	// Overshoot is travel past the target in the direction of motion.
	forward := samples[last].pos >= samples[first].pos
	for _, s := range samples {
		var over int
		if forward {
			over = s.pos - target
		} else {
			over = target - s.pos
		}
		if over > p.overshootSteps {
			p.overshootSteps = over
		}
	}
	return p
}

// impliedAccel returns the implied acceleration in steps/sec^2 from the ramp to peak
// velocity. Returns 0 when the ramp was instantaneous (below sampling resolution), which
// is itself the finding for Acc=0.
func (p moveProfile) impliedAccel() float64 {
	if p.timeToPeak <= 0 {
		return 0
	}
	return float64(p.peakVel) / p.timeToPeak.Seconds()
}

// clipped reports whether the servo could not honor a commanded GoalTime. The 1.15x + 20ms
// allowance absorbs sampling granularity and the servo's own settling.
//
// Integer math, deliberately: time.Duration(float64(commanded)*1.15) truncates and lands a
// nanosecond short for several round values (450ms yields 537.499999ms, not 537.5ms), which
// makes exact-boundary reasoning wrong in a way that only shows up at the boundary.
func (p moveProfile) clipped(commanded time.Duration) bool {
	if !p.moved || commanded <= 0 {
		return false
	}
	allowed := commanded*115/100 + 20*time.Millisecond
	return p.duration > allowed
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}
```

- [ ] **Step 2: Add the selftest cases**

```go
func init() {
	// synthProfile builds a trapezoidal series: 10ms sampling, ramp 0->1000 steps/s over
	// samples 1..10, cruise, then stop. Hand-computed expectations below.
	synthProfile := func() []sample {
		var out []sample
		pos := 0
		add := func(i, vel int) {
			out = append(out, sample{t: time.Duration(i) * 10 * time.Millisecond, pos: pos, vel: vel})
			pos += vel / 100 // 10ms of travel at vel steps/s
		}
		add(0, 0) // stopped
		for i := 1; i <= 10; i++ {
			add(i, i*100) // ramp: 100..1000
		}
		for i := 11; i <= 15; i++ {
			add(i, 1000) // cruise
		}
		add(16, 0) // stopped
		return out
	}

	selftests = append(selftests,
		selftestCase{"analyzeMove/trapezoid", func() error {
			s := synthProfile()
			p := analyzeMove(s, 1<<30) // target far away -> no overshoot
			if !p.moved {
				return fmt.Errorf("moved = false, want true")
			}
			// Motion spans sample 1 (t=10ms) to sample 15 (t=150ms).
			if want := 140 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			if p.peakVel != 1000 {
				return fmt.Errorf("peakVel = %d, want 1000", p.peakVel)
			}
			// 95% of 1000 = 950; first sample >= 950 is vel=1000 at t=100ms.
			// Measured from motion start (t=10ms) -> 90ms.
			if want := 90 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			// 1000 steps/s over 0.09s.
			if got := p.impliedAccel(); math.Abs(got-11111.11) > 1 {
				return fmt.Errorf("impliedAccel = %v, want ~11111.11", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/never-moved", func() error {
			s := []sample{{t: 0, pos: 100, vel: 0}, {t: 10 * time.Millisecond, pos: 100, vel: 0}}
			p := analyzeMove(s, 100)
			if p.moved {
				return fmt.Errorf("moved = true, want false")
			}
			if p.impliedAccel() != 0 {
				return fmt.Errorf("impliedAccel = %v, want 0", p.impliedAccel())
			}
			if p.clipped(time.Second) {
				return fmt.Errorf("clipped = true for a move that never happened")
			}
			return nil
		}},
		selftestCase{"analyzeMove/overshoot", func() error {
			// Forward motion overshooting a target of 100 by 5.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 50, vel: 500},
				{t: 20 * time.Millisecond, pos: 105, vel: 500},
				{t: 30 * time.Millisecond, pos: 100, vel: -10},
				{t: 40 * time.Millisecond, pos: 100, vel: 0},
			}
			p := analyzeMove(s, 100)
			if p.overshootSteps != 5 {
				return fmt.Errorf("overshootSteps = %d, want 5", p.overshootSteps)
			}
			return nil
		}},
		selftestCase{"moveProfile/clipped", func() error {
			p := moveProfile{moved: true, duration: 500 * time.Millisecond}
			// Commanded 200ms, took 500ms -> clipped.
			if !p.clipped(200 * time.Millisecond) {
				return fmt.Errorf("clipped = false, want true for 500ms vs commanded 200ms")
			}
			// Commanded 800ms, took 500ms -> honored.
			if p.clipped(800 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false for 500ms vs commanded 800ms")
			}
			// Just inside the allowance: 500ms vs commanded 450ms -> 450*1.15+20 = 537ms.
			if p.clipped(450 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false at the allowance boundary")
			}
			return nil
		}},
	)
}
```

- [ ] **Step 2b: Add the adequacy cases**

These eight exist because mutation testing showed the four cases above miss 13 real
defects — including a deleted `!p.moved` guard, an inverted direction inference, and
overshoot that lands after velocity reads zero (the most realistic overshoot signature).
Append a second `init()`:

```go

func init() {
	selftests = append(selftests,
		selftestCase{"analyzeMove/reverse-motion", func() error {
			// Reverse move: negative velocities, overshooting a target of 100 by 5.
			s := []sample{
				{t: 0, pos: 200, vel: 0},
				{t: 10 * time.Millisecond, pos: 150, vel: -500},
				{t: 20 * time.Millisecond, pos: 95, vel: -500},
				{t: 30 * time.Millisecond, pos: 100, vel: 10},
				{t: 40 * time.Millisecond, pos: 100, vel: 0},
			}
			p := analyzeMove(s, 100)
			if !p.moved {
				return fmt.Errorf("moved = false, want true (negative velocities are motion)")
			}
			if p.peakVel != 500 {
				return fmt.Errorf("peakVel = %d, want 500", p.peakVel)
			}
			if want := 20 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			if p.overshootSteps != 5 {
				return fmt.Errorf("overshootSteps = %d, want 5 (reverse overshoot)", p.overshootSteps)
			}
			// Peak reached on the first moving sample: instantaneous ramp -> 0, not +Inf.
			if got := p.impliedAccel(); got != 0 {
				return fmt.Errorf("impliedAccel = %v, want 0 for an instantaneous ramp", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/velEpsilon-boundary", func() error {
			// |vel| == velEpsilon is motion, so the move spans t=10ms..30ms.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 1},
				{t: 20 * time.Millisecond, pos: 5, vel: 500},
				{t: 30 * time.Millisecond, pos: 10, vel: 1},
				{t: 40 * time.Millisecond, pos: 10, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 20 * time.Millisecond; p.duration != want {
				return fmt.Errorf("duration = %v, want %v", p.duration, want)
			}
			return nil
		}},
		selftestCase{"analyzeMove/95-percent-band", func() error {
			// 95% of 1000 is 950, met at t=20ms by vel=960 -- one sample before the peak.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 500},
				{t: 20 * time.Millisecond, pos: 5, vel: 960},
				{t: 30 * time.Millisecond, pos: 15, vel: 1000},
				{t: 40 * time.Millisecond, pos: 25, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 10 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			return nil
		}},
		selftestCase{"analyzeMove/peak-at-last-moving-sample", func() error {
			// The 95% band is first met by the final moving sample.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 0, vel: 100},
				{t: 20 * time.Millisecond, pos: 1, vel: 500},
				{t: 30 * time.Millisecond, pos: 6, vel: 1000},
				{t: 40 * time.Millisecond, pos: 16, vel: 0},
			}
			p := analyzeMove(s, 1<<30)
			if want := 20 * time.Millisecond; p.timeToPeak != want {
				return fmt.Errorf("timeToPeak = %v, want %v", p.timeToPeak, want)
			}
			if got := p.impliedAccel(); math.Abs(got-50000) > 1 {
				return fmt.Errorf("impliedAccel = %v, want ~50000", got)
			}
			return nil
		}},
		selftestCase{"analyzeMove/overshoot-after-motion-ends", func() error {
			// The furthest travel past target lands in a sample already reading vel=0.
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 50, vel: 500},
				{t: 20 * time.Millisecond, pos: 98, vel: 500},
				{t: 30 * time.Millisecond, pos: 103, vel: 0},
				{t: 40 * time.Millisecond, pos: 103, vel: 0},
			}
			p := analyzeMove(s, 100)
			if p.overshootSteps != 3 {
				return fmt.Errorf("overshootSteps = %d, want 3", p.overshootSteps)
			}
			return nil
		}},
		selftestCase{"analyzeMove/bounds-indices", func() error {
			s := []sample{
				{t: 0, pos: 0, vel: 0},
				{t: 10 * time.Millisecond, pos: 5, vel: 500},
				{t: 20 * time.Millisecond, pos: 10, vel: 500},
				{t: 30 * time.Millisecond, pos: 10, vel: 0},
			}
			p := analyzeMove(s, 10)
			if p.startIdx != 1 || p.endIdx != 2 {
				return fmt.Errorf("startIdx,endIdx = %d,%d; want 1,2", p.startIdx, p.endIdx)
			}
			return nil
		}},
		selftestCase{"moveProfile/clipped-allowance", func() error {
			p := moveProfile{moved: true, duration: 500 * time.Millisecond}
			// 400*1.15+20 = 480ms < 500ms -> clipped. Pins the 1.15 factor from above.
			if !p.clipped(400 * time.Millisecond) {
				return fmt.Errorf("clipped = false, want true for 500ms vs commanded 400ms")
			}
			// 420*1.15 = 483ms, +20ms = 503ms > 500ms -> honored. Pins the +20ms.
			if p.clipped(420 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false for 500ms vs commanded 420ms")
			}
			// Exactly at the allowance (300*1.15+20 = 365ms) is not clipped.
			if (moveProfile{moved: true, duration: 365 * time.Millisecond}).clipped(300 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false exactly at the allowance")
			}
			// No GoalTime commanded -> nothing to clip.
			if p.clipped(0) {
				return fmt.Errorf("clipped = true, want false for commanded 0")
			}
			// A profile that never moved is never clipped, whatever its duration.
			if (moveProfile{moved: false, duration: 500 * time.Millisecond}).clipped(200 * time.Millisecond) {
				return fmt.Errorf("clipped = true, want false when moved = false")
			}
			return nil
		}},
		selftestCase{"analyzeMove/empty", func() error {
			if p := analyzeMove(nil, 0); p.moved {
				return fmt.Errorf("moved = true for an empty series")
			}
			return nil
		}},
	)
}
```

**Step 3: Run the selftests**

```bash
go run cmd/cli/profile_bench.go -selftest
```

Expected: twenty-one `PASS` lines, `21/21 passed`, exit 0.

- [ ] **Step 4: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): motion profile recovery with synthetic-profile checks"
```

---

## Task 7: CSV output

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the CSV section**

```go
// --- CSV output ---

// csvWriter wraps a file and encoding/csv, accumulating the first error so callers can
// write a whole table and check once at the end.
type csvWriter struct {
	f      *os.File
	w      *csv.Writer
	err    error
	closed bool
}

func newCSV(dir, name string, header []string) (*csvWriter, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, fmt.Errorf("create out dir %s: %w", dir, err)
	}
	path := filepath.Join(dir, name)
	f, err := os.Create(path)
	if err != nil {
		return nil, fmt.Errorf("create %s: %w", path, err)
	}
	c := &csvWriter{f: f, w: csv.NewWriter(f)}
	c.write(header)
	fmt.Printf("writing %s\n", path)
	return c, nil
}

func (c *csvWriter) write(row []string) {
	if c.err != nil {
		return
	}
	c.err = c.w.Write(row)
}

// close is idempotent, so callers can both `defer c.close()` as a leak guard and call it
// explicitly to check the accumulated error. A CSV write failure must not be silent — the
// files are the durable artifact of this whole harness.
func (c *csvWriter) close() error {
	if c.closed {
		return c.err
	}
	c.closed = true
	c.w.Flush()
	if c.err == nil {
		c.err = c.w.Error()
	}
	if cerr := c.f.Close(); c.err == nil {
		c.err = cerr
	}
	return c.err
}

// writeSamples dumps a sample series with a trial label, for plotting.
func writeSamples(c *csvWriter, trial string, servoID int, samples []sample) {
	for _, s := range samples {
		c.write([]string{
			trial,
			strconv.Itoa(servoID),
			strconv.FormatFloat(s.t.Seconds(), 'f', 6, 64),
			strconv.Itoa(s.pos),
			strconv.Itoa(s.vel),
		})
	}
}

var sampleHeader = []string{"trial", "servo_id", "t_sec", "pos_ticks", "vel_steps_per_sec"}
```

Add `"encoding/csv"` and `"path/filepath"` to the imports.

- [ ] **Step 1b: Add the selftest cases**

These cases pin two things that would otherwise ship untested.

`close()` idempotency: Tasks 10 and 11 both `defer c.close()` AND call `close()`
explicitly inside `firstErr(...)`. Note the explicit call is evaluated first as a return
argument and the deferred one's value is discarded, so a second close's error is never
observed — the guard's real jobs are avoiding a double `f.Close()` and returning the
**same stored error** on a repeat call.

Error paths: `close()`'s doc comment promises a CSV write failure is never silent, since
these files are the harness's durable artifact. Four separate mutants (close returning
nil, `write()` losing its short-circuit, the dropped `w.Error()` capture, and an ignored
`Write` error) all survive unless something actually puts the writer into a failed state.

The round-trip additionally pins the exact `writeSamples` byte format, using two samples
so the loop's multiplicity is covered. All cases run on a temp dir or an `os.Pipe`, no
hardware.

```go
func init() {
	selftests = append(selftests,
		selftestCase{"csvWriter/roundtrip", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", sampleHeader)
			if err != nil {
				return err
			}
			writeSamples(c, "trial1", 3, []sample{
				{t: 1500 * time.Millisecond, pos: -42, vel: 7},
				{t: 2 * time.Second, pos: 100, vel: -8},
			})
			if err := c.close(); err != nil {
				return fmt.Errorf("close: %w", err)
			}
			got, err := os.ReadFile(filepath.Join(dir, "t.csv"))
			if err != nil {
				return err
			}
			want := "trial,servo_id,t_sec,pos_ticks,vel_steps_per_sec\n" +
				"trial1,3,1.500000,-42,7\n" +
				"trial1,3,2.000000,100,-8\n"
			if string(got) != want {
				return fmt.Errorf("file =\n%q\nwant\n%q", got, want)
			}
			return nil
		}},
		selftestCase{"csvWriter/close-is-idempotent", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			if err := c.close(); err != nil {
				return fmt.Errorf("first close: %w", err)
			}
			// Tasks 10 and 11 both defer close() AND call it explicitly inside
			// firstErr(...). The explicit call is evaluated first as a return argument,
			// so the deferred one's value is discarded -- the guard's real jobs are not
			// double-closing the fd, and returning the SAME stored error on a repeat
			// call, which csvWriter/flush-failure-surfaces pins.
			if err := c.close(); err != nil {
				return fmt.Errorf("second close: %w", err)
			}
			return nil
		}},
	)
}

func init() {
	selftests = append(selftests,
		selftestCase{"csvWriter/close-reports-write-failure", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			// Close the file out from under the writer, then write. A silent CSV failure
			// would look like a successful run that happened to produce no data.
			if err := c.f.Close(); err != nil {
				return err
			}
			c.write([]string{"b"})
			if err := c.close(); err == nil {
				return fmt.Errorf("close() = nil, want the underlying write failure")
			}
			return nil
		}},
		selftestCase{"csvWriter/first-error-wins", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			boom := fmt.Errorf("boom")
			c.err = boom
			c.write([]string{"b"}) // must be dropped, not masked by succeeding
			if got := c.close(); got != boom {
				return fmt.Errorf("close() = %v, want the first error (%v)", got, boom)
			}
			got, err := os.ReadFile(filepath.Join(dir, "t.csv"))
			if err != nil {
				return err
			}
			if string(got) != "a\n" {
				return fmt.Errorf("file = %q, want %q (a row written after an error must be dropped)", got, "a\n")
			}
			return nil
		}},
		selftestCase{"csvWriter/flush-failure-surfaces", func() error {
			// A pipe with the read end closed: writes fail with EPIPE while Close() on
			// the write end still succeeds, which isolates a flush error from a close error.
			r, w, err := os.Pipe()
			if err != nil {
				return err
			}
			if err := r.Close(); err != nil {
				return err
			}
			c := &csvWriter{f: w, w: csv.NewWriter(w)}
			c.write([]string{"a"}) // buffered; the failure only appears at flush
			first := c.close()
			if first == nil {
				return fmt.Errorf("close() = nil, want the flush failure")
			}
			// An idempotent close must REPORT the stored error again, not swallow it.
			if second := c.close(); second != first {
				return fmt.Errorf("second close() = %v, want the same error (%v)", second, first)
			}
			return nil
		}},
		selftestCase{"csvWriter/write-error-is-recorded", func() error {
			dir, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(dir)

			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return err
			}
			// An invalid delimiter makes csv.Writer.Write fail without touching the file,
			// so this is the one path where write() recording the error is what surfaces it.
			c.w.Comma = 0
			c.write([]string{"b"})
			c.w.Comma = ','
			if err := c.close(); err == nil {
				return fmt.Errorf("close() = nil, want the recorded write error")
			}
			return nil
		}},
		selftestCase{"newCSV/creates-missing-dir", func() error {
			root, err := os.MkdirTemp("", "profile_bench_selftest")
			if err != nil {
				return err
			}
			defer os.RemoveAll(root)

			// -out may name a directory that does not exist yet; newCSV must create it.
			dir := filepath.Join(root, "nested", "out")
			c, err := newCSV(dir, "t.csv", []string{"a"})
			if err != nil {
				return fmt.Errorf("newCSV into a missing dir: %w", err)
			}
			if err := c.close(); err != nil {
				return err
			}
			if _, err := os.Stat(filepath.Join(dir, "t.csv")); err != nil {
				return fmt.Errorf("file not created: %w", err)
			}
			return nil
		}},
	)
}
```

- [ ] **Step 2: Verify build, vet, format**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go
```

Expected: exit 0, no output.

- [ ] **Step 2b: Run the selftests**

```bash
go run cmd/cli/profile_bench.go -selftest
```

Expected: `35/35 passed`, exit 0.

- [ ] **Step 3: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): CSV output helpers"
```

---

## Task 8: The `writerate` test

No motion. Runs with torque **disabled** so a mis-encoded position cannot actuate anything, and commands the position each servo is already at.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the writerate section**

```go
// --- test: writerate ---

// gapSweep is the set of MinCommandGap values probed. The first entry is the "no gap"
// sentinel: a literal 0 is coerced to 1ms by feetech.NewBus (bus.go:63), which would make
// it a mislabeled duplicate of the 1ms row.
var gapSweep = []time.Duration{
	noGap,
	250 * time.Microsecond,
	500 * time.Microsecond,
	1 * time.Millisecond,
	2 * time.Millisecond,
}

const writeRateIterations = 500

func runWriteRate(cfg config) error {
	out, err := newCSV(cfg.outDir, csvName("writerate", "", 0, false), []string{
		"gap_label", "gap_sec", "n", "mean_sec", "p50_sec", "p95_sec", "p99_sec", "max_sec", "rate_hz",
	})
	if err != nil {
		return err
	}
	defer out.close()

	fmt.Printf("\n%-10s %8s %10s %10s %10s %10s %10s\n",
		"gap", "rate_hz", "mean", "p50", "p95", "p99", "max")

	for _, gap := range gapSweep {
		st, err := measureWriteRate(cfg.port, gap)
		if err != nil {
			return fmt.Errorf("gap %v: %w", gap, err)
		}
		label := gap.String()
		if gap == noGap {
			label = "none"
		}
		fmt.Printf("%-10s %8.1f %10v %10v %10v %10v %10v\n",
			label, st.rateHz, st.mean, st.p50, st.p95, st.p99, st.max)
		out.write([]string{
			label,
			strconv.FormatFloat(gap.Seconds(), 'g', -1, 64),
			strconv.Itoa(st.n),
			strconv.FormatFloat(st.mean.Seconds(), 'f', 9, 64),
			strconv.FormatFloat(st.p50.Seconds(), 'f', 9, 64),
			strconv.FormatFloat(st.p95.Seconds(), 'f', 9, 64),
			strconv.FormatFloat(st.p99.Seconds(), 'f', 9, 64),
			strconv.FormatFloat(st.max.Seconds(), 'f', 9, 64),
			strconv.FormatFloat(st.rateHz, 'f', 2, 64),
		})
	}

	fmt.Printf("\nsanity checks: gap=1ms should read ~1000Hz, gap=2ms ~500Hz.\n")
	fmt.Printf("if gap=none reads the same rate as gap=1ms, the zero-coercion bug is back.\n")
	return out.close()
}

// measureWriteRate opens a fresh bus at the given gap and times back-to-back SetGoals
// writes that command each servo's current position, with torque off.
func measureWriteRate(port string, gap time.Duration) (latencyStats, error) {
	r, err := openRig(port, gap, armServoIDs)
	if err != nil {
		return latencyStats{}, err
	}
	defer r.close()

	var st latencyStats
	err = r.withSafeShutdown(func(ctx context.Context) error {
		// Torque off for the whole test: this measures the wire, not the motor.
		r.disableTorque(ctx)

		positions, err := r.group.Positions(ctx)
		if err != nil {
			return fmt.Errorf("read positions: %w", err)
		}

		goals := make(map[int]feetech.GoalRequest, len(r.ids))
		for _, id := range r.ids {
			pos, ok := positions[id]
			if !ok {
				return fmt.Errorf("servo %d did not report a position", id)
			}
			// Commanding the current position: even with torque somehow enabled, there
			// is nowhere to move to.
			goals[id] = feetech.GoalRequest{Position: pos, Speed: 100, Acc: 10}
		}

		latencies := make([]time.Duration, 0, writeRateIterations)
		for i := 0; i < writeRateIterations; i++ {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			start := time.Now()
			if err := r.group.SetGoals(ctx, goals); err != nil {
				return fmt.Errorf("SetGoals iteration %d: %w", i, err)
			}
			latencies = append(latencies, time.Since(start))
		}
		st = summarize(latencies)
		return nil
	})
	return st, err
}
```

- [ ] **Step 2: Wire it into `run`**

Replace the `return fmt.Errorf("not yet implemented")` line in `run` with a dispatch loop:

```go
	for _, t := range tests {
		var err error
		switch t {
		case "writerate":
			err = runWriteRate(cfg)
		case "goaltime":
			err = runGoalTime(cfg)
		case "accel":
			err = runAccel(cfg)
		}
		if err != nil {
			return fmt.Errorf("%s: %w", t, err)
		}
	}
	return nil
```

`runGoalTime` and `runAccel` do not exist yet. Add temporary stubs returning `fmt.Errorf("not yet implemented")` so the file keeps building; Tasks 9–11 replace them.

- [ ] **Step 3: Verify build, vet, format, and that selftests still pass**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go && \
  go run cmd/cli/profile_bench.go -selftest
```

Expected: exit 0, no `gofmt` output, `35/35 passed`.

- [ ] **Step 4: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): writerate sweep over MinCommandGap"
```

---

## Task 9: The sampler

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the sampler section**

```go
// --- sampling ---

// sampleUntilStopped polls position and velocity as fast as the link allows until every
// servo has reported stopped for settleWindow consecutive reads, or timeout elapses.
//
// One SyncRead of 4 bytes at RegPresentPosition returns position AND velocity per servo,
// because addresses 56 and 58 are adjacent. Each sample carries its own measured
// timestamp, so sampling jitter appears in the data rather than being assumed uniform.
func sampleUntilStopped(
	ctx context.Context,
	r *rig,
	ids []int,
	timeout time.Duration,
) (map[int][]sample, error) {
	out := make(map[int][]sample, len(ids))
	t0 := time.Now()
	deadline := t0.Add(timeout)
	stoppedRuns := 0
	sawMotion := false

	for {
		if ctx.Err() != nil {
			return out, ctx.Err()
		}
		data, err := r.bus.SyncRead(ctx, feetech.RegPresentPosition.Address, 4, ids)
		now := time.Since(t0)
		if err != nil {
			return out, fmt.Errorf("sync read: %w", err)
		}

		allStopped := true
		for _, id := range ids {
			pos, vel, derr := decodePosVel(r.proto, data[id])
			if derr != nil {
				return out, fmt.Errorf("servo %d: %w", id, derr)
			}
			out[id] = append(out[id], sample{t: now, pos: pos, vel: vel})
			if abs(vel) >= velEpsilon {
				allStopped = false
				sawMotion = true
			}
		}

		if allStopped {
			stoppedRuns++
			// Require motion to have started before treating "stopped" as "finished",
			// so the initial pre-motion latency does not end the trial immediately.
			if sawMotion && stoppedRuns >= settleWindow {
				return out, nil
			}
		} else {
			stoppedRuns = 0
		}

		if time.Now().After(deadline) {
			if !sawMotion {
				return out, fmt.Errorf("no motion detected within %v", timeout)
			}
			return out, nil // timed out mid-move; caller sees it in the profile
		}
	}
}

// settleWindow is how many consecutive all-stopped reads end a trial. The STS3215 reports
// zero velocity briefly at direction changes and at the start of a ramp, so a single
// zero read is not conclusive.
const settleWindow = 5
```

- [ ] **Step 2: Verify build, vet, format, selftests**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go && \
  go run cmd/cli/profile_bench.go -selftest
```

Expected: exit 0, no `gofmt` output, `35/35 passed`.

- [ ] **Step 3: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): position+velocity sampler with settle detection"
```

---

## Task 10: The `goaltime` test

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add shared motion setup**

```go
// --- motion setup ---

// prepareTravel reads the start position and angle limits for each servo and validates
// the requested travel. Returns an error rather than clamping (see planTravel).
func prepareTravel(ctx context.Context, r *rig, ids []int, travelSteps int, assume string) ([]travelPlan, error) {
	positions, err := r.group.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("read positions: %w", err)
	}

	var overrideMin, overrideMax int
	haveOverride := false
	if assume != "" {
		overrideMin, overrideMax, err = parseAssumeLimits(assume)
		if err != nil {
			return nil, fmt.Errorf("-assume-limits: %w", err)
		}
		haveOverride = true
	}

	plans := make([]travelPlan, 0, len(ids))
	for _, id := range ids {
		start, ok := positions[id]
		if !ok {
			return nil, fmt.Errorf("servo %d did not report a position", id)
		}
		min, max := overrideMin, overrideMax
		if !haveOverride {
			s := r.group.ServoByID(id)
			if s == nil {
				return nil, fmt.Errorf("servo %d not in group", id)
			}
			if min, max, err = s.PositionLimits(ctx); err != nil {
				return nil, fmt.Errorf("read limits for servo %d: %w", id, err)
			}
		}
		p, err := planTravel(id, start, travelSteps, min, max)
		if err != nil {
			return nil, err
		}
		plans = append(plans, p)
	}
	return plans, nil
}

// moveTimeout bounds a trial generously: commanded time plus slack, floored so that a
// clipped move still has room to finish and be recognised as clipped.
func moveTimeout(commanded time.Duration) time.Duration {
	t := commanded*3 + 2*time.Second
	if t < 5*time.Second {
		t = 5 * time.Second
	}
	return t
}
```

- [ ] **Step 2: Add the goaltime test**

```go
// --- test: goaltime ---

var goalTimeSweepMs = []int{200, 400, 800, 1600, 3000}
var goalTimeTravelDeg = []float64{10, 20, 40}

// goalTimeAcc is held fixed across the GoalTime sweep so timing is the only variable.
const goalTimeAcc = 20

func runGoalTime(cfg config) error {
	// commanded is the set this test actually drives; the rig always contains the whole
	// arm so holdPose can keep the untested joints powered. Without that, a single-joint
	// accel run measures a collapsing chain rather than a posed arm.
	commanded := []int{cfg.servo}
	if cfg.fullArm {
		commanded = armServoIDs
	}
	ids := commanded

	r, err := openRig(cfg.port, noGap, armServoIDs, !cfg.stockTransport) // fast-flush samples ~10x faster
	if err != nil {
		return err
	}
	defer r.close()

	summary, err := newCSV(cfg.outDir, csvName("goaltime", "", cfg.servo, cfg.fullArm), []string{
		"trial", "servo_id", "commanded_ms", "travel_deg", "measured_ms",
		"ratio_measured_over_commanded", "goal_time_exceeded", "peak_vel", "overshoot_steps",
	})
	if err != nil {
		return err
	}
	defer summary.close()

	samplesCSV, err := newCSV(cfg.outDir, csvName("goaltime", "_samples", cfg.servo, cfg.fullArm), sampleHeader)
	if err != nil {
		return err
	}
	defer samplesCSV.close()

	fmt.Printf("\n%-22s %10s %10s %8s %8s\n", "trial", "commanded", "measured", "ratio", "clipped")

	runErr := r.withSafeShutdown(func(ctx context.Context) error {
		if err := r.holdPose(ctx); err != nil {
			return err
		}

		for _, travelDeg := range goalTimeTravelDeg {
			for _, ms := range goalTimeSweepMs {
				if ctx.Err() != nil {
					return ctx.Err()
				}
				trial := fmt.Sprintf("t%.0fdeg_%dms", travelDeg, ms)
				if err := goalTimeTrial(ctx, r, cfg, ids, travelDeg, ms, trial, summary, samplesCSV); err != nil {
					return err
				}
			}
		}
		return nil
	})

	// Surface CSV write failures: the files are this harness's durable artifact, so a
	// silent failure would look like a successful run with no data.
	return firstErr(runErr, summary.close(), samplesCSV.close())
}

// firstErr returns the first non-nil error, so a run error is not masked by a close error
// and vice versa.
func firstErr(errs ...error) error {
	for _, e := range errs {
		if e != nil {
			return e
		}
	}
	return nil
}

// goalTimeTrial runs one out-and-back at a commanded time. In -full-arm mode each joint
// gets a DIFFERENT travel under the SAME commanded time, which is what makes per-joint
// arrival spread meaningful.
func goalTimeTrial(
	ctx context.Context,
	r *rig,
	cfg config,
	ids []int,
	travelDeg float64,
	ms int,
	trial string,
	summary, samplesCSV *csvWriter,
) error {
	steps := degToSteps(travelDeg)
	plans, err := prepareTravel(ctx, r, ids, steps, cfg.assumeLimits)
	if err != nil {
		return err
	}

	goals := make(map[int]feetech.GoalRequest, len(plans))
	targets := make(map[int]int, len(plans))
	for i, p := range plans {
		target := p.target
		if cfg.fullArm {
			// Stagger travel per joint so arrival spread is observable: joint i travels
			// (i+1)/len of the nominal distance, all under the same commanded time.
			//
			// No re-validation needed, and deliberately no second planTravel call: Go
			// truncates integer division toward zero, so scaled is bounded by steps with
			// a matching sign, putting p.start+scaled inside (p.start, p.target] — a
			// range planTravel already validated above. A second call with its own limits
			// could fail and silently fall back to the un-staggered target, collapsing the
			// spread to ~0 and reporting perfect coordinated arrival from a test that
			// never staggered anything.
			scaled := steps * (i + 1) / len(plans)
			target = p.start + scaled
		}
		targets[p.id] = target
		goals[p.id] = feetech.GoalRequest{Position: target, Time: ms, Acc: goalTimeAcc}
	}

	if err := r.group.SetGoals(ctx, goals); err != nil {
		return fmt.Errorf("%s: SetGoals: %w", trial, err)
	}
	commanded := time.Duration(ms) * time.Millisecond
	series, err := sampleUntilStopped(ctx, r, ids, moveTimeout(commanded))
	if err != nil {
		return fmt.Errorf("%s: %w", trial, err)
	}

	var arrivals []time.Duration
	for _, p := range plans {
		s := series[p.id]
		writeSamples(samplesCSV, trial, p.id, s)
		prof := analyzeMove(s, targets[p.id])
		ratio := 0.0
		if commanded > 0 {
			ratio = prof.duration.Seconds() / commanded.Seconds()
		}
		// Arrival spread must be measured in ABSOLUTE time, not per-joint duration.
		// Every joint shares this trial's sampling t0, so the coordinated-arrival
		// question is "did they all finish at the same moment", which is the spread of
		// end timestamps. prof.duration subtracts each joint's own motion-onset
		// detection latency, and under -full-arm that latency is systematically
		// joint-dependent (same commanded time, different travel -> gentler ramps cross
		// velEpsilon later), so a duration spread is biased.
		if prof.moved {
			arrivals = append(arrivals, s[prof.endIdx].t)
		}
		summary.write([]string{
			trial, strconv.Itoa(p.id), strconv.Itoa(ms),
			strconv.FormatFloat(travelDeg, 'f', 1, 64),
			strconv.FormatFloat(prof.duration.Seconds()*1000, 'f', 1, 64),
			strconv.FormatFloat(ratio, 'f', 3, 64),
			strconv.FormatBool(prof.exceededGoalTime(commanded)),
			strconv.Itoa(prof.peakVel),
			strconv.Itoa(prof.overshootSteps),
		})
		if len(ids) == 1 {
			fmt.Printf("%-22s %10v %10v %8.3f %8v\n",
				trial, commanded, prof.duration, ratio, prof.exceededGoalTime(commanded))
		}
	}

	if len(ids) > 1 {
		if len(arrivals) < len(plans) {
			// Never report a spread over a subset: a joint that did not move at all
			// would silently tighten the number that decides coordinated arrival.
			fmt.Printf("%-22s %10v arrival spread: UNRELIABLE, only %d of %d joints moved\n",
				trial, commanded, len(arrivals), len(plans))
		} else {
			min, max := arrivals[0], arrivals[0]
			for _, a := range arrivals[1:] {
				if a < min {
					min = a
				}
				if a > max {
					max = a
				}
			}
			fmt.Printf("%-22s %10v arrival spread across %d joints: %v (absolute)\n",
				trial, commanded, len(arrivals), max-min)
		}
	}

	// Return to start under the same commanded time, so the next trial begins where this
	// one did.
	back := make(map[int]feetech.GoalRequest, len(plans))
	for _, p := range plans {
		back[p.id] = feetech.GoalRequest{Position: p.start, Time: ms, Acc: goalTimeAcc}
	}
	if err := r.group.SetGoals(ctx, back); err != nil {
		return fmt.Errorf("%s: return SetGoals: %w", trial, err)
	}
	if _, err := sampleUntilStopped(ctx, r, ids, moveTimeout(commanded)); err != nil {
		return fmt.Errorf("%s: return: %w", trial, err)
	}
	return nil
}
```

Remove the `runGoalTime` stub added in Task 8.

- [ ] **Step 3: Verify build, vet, format, selftests**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go && \
  go run cmd/cli/profile_bench.go -selftest
```

Expected: exit 0, no `gofmt` output, `35/35 passed`.

- [ ] **Step 4: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): GoalTime accuracy sweep with arrival-spread reporting"
```

---

## Task 11: The `accel` test

Single joint, fixed travel, fixed `Speed` — acceleration shapes the ramp toward a velocity target, so `Time` is left at zero here.

**Files:**
- Modify: `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the accel test**

```go
// --- test: accel ---

// Descending: Acc=0 is an unlimited ramp and the jerkiest command in the harness, so it
// runs LAST rather than as the first motion of a bench session.
var accSweep = []int{255, 200, 100, 50, 20, 10, 5, 2, 1, 0}

// accelSpeedSteps caps the velocity target during the acceleration sweep. Acc=0 means an
// unlimited ramp; pairing that with a high velocity target is the one genuinely jerky
// combination in this harness, so the ceiling stays modest.
const accelSpeedSteps = 1000

func runAccel(cfg config) error {
	// Only cfg.servo is commanded, but the rig holds the whole arm so holdPose can keep
	// the other joints powered -- otherwise this sweep measures a sagging chain, which is
	// exactly wrong for the gravity-loaded joints the mapping depends on.
	ids := []int{cfg.servo}
	r, err := openRig(cfg.port, noGap, armServoIDs, !cfg.stockTransport) // fast-flush samples ~10x faster
	if err != nil {
		return err
	}
	defer r.close()

	summary, err := newCSV(cfg.outDir, csvName("accel", "", cfg.servo, false), []string{
		"trial", "servo_id", "acc_units", "travel_deg", "peak_vel_steps_per_sec",
		"time_to_peak_sec", "implied_accel_steps_per_sec2", "steps_per_sec2_per_unit",
		"ramp_below_sampling_resolution", "duration_sec", "overshoot_steps",
	})
	if err != nil {
		return err
	}
	defer summary.close()

	samplesCSV, err := newCSV(cfg.outDir, csvName("accel", "_samples", cfg.servo, false), sampleHeader)
	if err != nil {
		return err
	}
	defer samplesCSV.close()

	fmt.Printf("\n%-10s %10s %12s %14s %12s\n",
		"acc", "peak_vel", "time_to_peak", "implied_a", "a_per_unit")

	runErr := r.withSafeShutdown(func(ctx context.Context) error {
		if err := r.holdPose(ctx); err != nil {
			return err
		}

		steps := degToSteps(cfg.travelDeg)
		for _, acc := range accSweep {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			trial := fmt.Sprintf("acc%d", acc)

			plans, err := prepareTravel(ctx, r, ids, steps, cfg.assumeLimits)
			if err != nil {
				return err
			}
			p := plans[0]

			goal := map[int]feetech.GoalRequest{
				p.id: {Position: p.target, Speed: accelSpeedSteps, Acc: acc},
			}
			if err := r.group.SetGoals(ctx, goal); err != nil {
				return fmt.Errorf("%s: SetGoals: %w", trial, err)
			}
			series, err := sampleUntilStopped(ctx, r, ids, 10*time.Second)
			if err != nil {
				return fmt.Errorf("%s: %w", trial, err)
			}

			s := series[p.id]
			writeSamples(samplesCSV, trial, p.id, s)
			prof := analyzeMove(s, p.target)
			implied := prof.impliedAccel()
			perUnit := 0.0
			if acc > 0 {
				perUnit = implied / float64(acc)
			}
			fmt.Printf("%-10d %10d %12v %14.1f %12.1f\n",
				acc, prof.peakVel, prof.timeToPeak, implied, perUnit)
			summary.write([]string{
				trial, strconv.Itoa(p.id), strconv.Itoa(acc),
				strconv.FormatFloat(cfg.travelDeg, 'f', 1, 64),
				strconv.Itoa(prof.peakVel),
				strconv.FormatFloat(prof.timeToPeak.Seconds(), 'f', 6, 64),
				strconv.FormatFloat(implied, 'f', 1, 64),
				strconv.FormatFloat(perUnit, 'f', 1, 64),
				strconv.FormatBool(prof.rampBelowSamplingResolution()),
				strconv.FormatFloat(prof.duration.Seconds(), 'f', 6, 64),
				strconv.Itoa(prof.overshootSteps),
			})

			// Return to start at a fixed, gentle profile so the return leg does not
			// contaminate the next trial.
			back := map[int]feetech.GoalRequest{
				p.id: {Position: p.start, Speed: 500, Acc: 20},
			}
			if err := r.group.SetGoals(ctx, back); err != nil {
				return fmt.Errorf("%s: return SetGoals: %w", trial, err)
			}
			if _, err := sampleUntilStopped(ctx, r, ids, 10*time.Second); err != nil {
				return fmt.Errorf("%s: return: %w", trial, err)
			}
		}

		fmt.Printf("\nthe library documents Acc as ~100 steps/s^2 per unit; " +
			"compare the a_per_unit column and note where it saturates.\n")
		fmt.Printf("rows with ramp_below_sampling_resolution=true reached peak within one " +
			"sample: implied_accel reads 0 because it was too fast to measure, NOT because " +
			"there was no acceleration. Read peak_vel for those.\n")
		return nil
	})

	return firstErr(runErr, summary.close(), samplesCSV.close())
}
```

Remove the `runAccel` stub added in Task 8.

- [ ] **Step 2: Verify build, vet, format, selftests**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  gofmt -l cmd/cli/profile_bench.go && \
  go run cmd/cli/profile_bench.go -selftest
```

Expected: exit 0, no `gofmt` output, `35/35 passed`.

- [ ] **Step 3: Confirm the module's tests still pass**

```bash
go test ./cmd/module/ .
```

Expected: PASS (modulo the environment-dependent `TestEnumerateSerialPorts`).

- [ ] **Step 4: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): acceleration register sweep"
```

---

## Task 12: Bench run and recorded results

This is the actual deliverable. Everything before it is instrumentation.

**Files:**
- Create: `docs/superpowers/results/2026-08-19-bench-run.md`

- [ ] **Step 1: Find the serial port**

```bash
ls /dev/tty.usb* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

- [ ] **Step 2: Run the no-motion test first**

```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=writerate -out=./bench-out
```

The table has TWO blocks, one per transport. Read them differently:

- **`stock` rows are expected to be FLAT at ~95 Hz regardless of gap.** That is feetech's
  `SerialTransport.Flush` waiting out a 10 ms read timeout before every packet, and it is
  the single most actionable finding this test produces — not a fault. Do **not** stop the
  run because these rows fail to track the gap.
- **`fastflush` rows should track the gap**: ~1000 Hz at 1 ms, ~500 Hz at 2 ms. Only if a
  *fastflush* `none` row reads the same rate as its own `1ms` row has the `MinCommandGap`
  zero-coercion been reintroduced — stop and fix that.
- A 5-servo `SetGoals` is 48 bytes ≈ 480 µs of wire time, so ~2080 Hz is the physical
  ceiling. Anything faster is host-side buffering, not the bus.

- [ ] **Step 3: Clear the workspace and position the joint, then run the motion tests**

Two things to know before the motion runs:

- **The harness refuses to overwrite CSVs.** Filenames carry the servo and mode
  and the transport (`accel_servo2_fastflush.csv`,
  `goaltime_fullarm_fastflush_samples.csv`), so the servo 1/2/3 sweep below no longer
  collides with itself, and stock vs fast-flush runs stay separate — but re-running the *same* configuration into the same
  `-out` will fail rather than clobber. Move the old files aside or pass a new `-out`.
- **The harness verifies the arm returned to start after every trial** and aborts if a
  joint is more than ~1 degree off, rather than measuring the rest of the sweep from a
  contaminated position. If you see that abort, the arm is fighting something.
- **Filenames and a CSV column record which transport produced the data.** The transport
  changes the sample rate 3-10x and therefore changes `measured_ms`, `peak_vel` and
  `time_to_peak_sec`; `n_samples` per trial is recorded so you can see the resolution
  behind each number.
- **After ANY run — including `writerate` — the servos keep `RegAcceleration` and
  `RegGoalVelocity` set.** Both are SRAM, so power-cycle the arm before using the shipped
  module, or its motion stays acceleration-limited in a way it normally is not.
- **`-test=all` runs the motion tests first and `writerate` last**, because `writerate`
  deliberately leaves the arm limp; running it first would have the motion tests hold and
  measure a collapsed pose.
- **Torque is released when each run ends**, on the normal path as well as on Ctrl-C. For
  the gravity-loaded joints below the arm will drop the moment the summary finishes
  printing, so support it. The harness prints a reminder when it powers up. Note also that
  a second Ctrl-C does nothing while a run is shutting down; if it wedges on a serial read,
  use `kill`.

Before running `goaltime`, position the tested joint with at least **40° of positive
headroom** inside its angle limits. For the `-full-arm` run in Step 4, **every** arm joint
needs that headroom: `prepareTravel` validates the full un-staggered travel for all five,
even though joints 1-4 only move a fraction of it. The sweep runs 10°, 20°, and 40° travels and the harness
**refuses rather than clamps** (by design), so a joint parked near its upper limit completes
the 10° and 20° trials and then aborts partway through the run.


```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=goaltime -move -servo=1 -out=./bench-out
go run cmd/cli/profile_bench.go -port=<PORT> -test=accel -move -servo=1 -out=./bench-out
```

Then repeat for the gravity-loaded joints, which are the ones that matter for a real `acceleration_degs_per_sec_per_sec` mapping:

```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=accel -move -servo=2 -out=./bench-out
go run cmd/cli/profile_bench.go -port=<PORT> -test=accel -move -servo=3 -out=./bench-out
```

- [ ] **Step 4: Run the coordinated-arrival test with the workspace clear**

```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=goaltime -move -full-arm -out=./bench-out
```

- [ ] **Step 5: Record the results**

The directory does not exist yet:

```bash
mkdir -p docs/superpowers/results
```

Write `docs/superpowers/results/2026-08-19-bench-run.md` covering:

- Hardware and host: arm (leader/follower), USB adapter, OS, port.
- **Write rate:** the table, and the achievable setpoint rate with jitter. p95/p99 matter more than the mean.
- **GoalTime:** over which (time, travel) combinations it was honored, where it clipped, and the per-joint arrival spread from the `-full-arm` run. This decides whether coordinated arrival is viable.
- **Acceleration:** measured steps/s² per `Acc` unit for servos 1, 2, and 3; whether the ~100 figure holds; where it saturates; and **whether the mapping is per-joint or global** — this is the open question the spec flagged. Ignore `implied_accel` on rows where `ramp_below_sampling_resolution` is true: those reached peak within one sample, so the 0 means "too fast to measure", not "no acceleration" — read `peak_vel` instead.
- **Scope recommendation:** which of the three follow-on stages the numbers support (coordinated arrival, acceleration enforcement, trajectory execution), and what each would cost.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/results/2026-08-19-bench-run.md
git commit -m "docs: recorded servo motion-profile bench measurements"
```

---

## Done When

- [ ] `go build -o /dev/null cmd/cli/profile_bench.go` exits 0
- [ ] `go vet cmd/cli/profile_bench.go` exits 0
- [ ] `gofmt -l cmd/cli/profile_bench.go` prints nothing
- [ ] `go run cmd/cli/profile_bench.go -selftest` reports `35/35 passed`
- [ ] `go test ./cmd/module/ .` passes (modulo `TestEnumerateSerialPorts` without serial hardware)
- [ ] All three tests have been run against real hardware and their CSVs exist
- [ ] `docs/superpowers/results/2026-08-19-bench-run.md` records the numbers and a scope recommendation
- [ ] No file outside `cmd/cli/` and `docs/` was modified

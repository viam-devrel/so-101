# Per-Joint Motion Profiles Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the SO-101 hardware arm's joints arrive together by scaling each joint's speed *and* acceleration by its share of the travel, write the acceleration register for the first time, and honor the per-joint fields in `arm.MoveOptions`.

**Architecture:** A new pure file `motion_profile.go` owns all the profile math (`k = dᵢ/d_max` scaling, unit conversion, clamping, cap reduction) and is unit-tested with no hardware. A new controller method `MoveServosWithProfiles` writes per-servo speed and acceleration in one `SetGoals` sync write. `arm.go` computes travels and wires the two together. `writePositions` is untouched, so the gripper's single-servo paths are unaffected.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, `github.com/hipsterbrown/feetech-servo` v0.6.0 → **v0.6.1**.

**Spec:** `docs/superpowers/specs/2026-08-20-per-joint-motion-profiles-design.md`
**Measurements this rests on:** `docs/superpowers/results/2026-08-19-bench-run.md`

---

## Critical Context (read before Task 1)

**1. Two register values mean the opposite of what they look like.** Both have already caused real bugs in this project.

- **`Speed: 0` means MAXIMUM, not stopped.** `speed.go:19`'s `minSpeedSteps = 1` floors this; `degPerSecToStepsPerSec` applies it.
- **`Acc: 0` means UNLIMITED, not zero.** A profile that rounds to 0 becomes the *jerkiest* possible command instead of the gentlest.

A short-travel joint scaled by a small `k` can round into either. Both floors are load-bearing.

**2. Use `degPerSecToStepsPerSec`, never `resolveSpeedDegsPerSec`.** Both are in `speed.go`. The latter floors at `minSpeedDegsPerSec = 3.0`, which would destroy scaling for any joint below `k ≈ 0.06`. The former floors at 1 step/s (≈0.088 °/s).

**3. The acceleration register saturates at ~50.** Above the knee it does nothing. The shipped default of 500 °/s² maps to `Acc 49` deliberately — 600 °/s² would map to 59.6 and clamp, which clamps the *reference* joint (`k = 1`) while leaving lower-`k` joints exactly scaled, breaking coordination at the default. A test pins this.

**4. `writePositions` must not change.** Two of its four callers are single-servo gripper paths (`manager.go:134`, `:154`) where coordination is meaningless. Add a new method alongside it.

**5. Verify feetech facts against the module cache**, `$(go env GOMODCACHE)/github.com/hipsterbrown/feetech-servo@v0.6.1/`, not the local checkout at `~/src/feetech-servo` — go.mod has no `replace` directive.

**6. Build and test with `go test ./cmd/module/ .`**, never `./...` — `cmd/cli/` holds standalone debug scripts with multiple `func main()` and will not compile as a package. Lint with `gofmt -s -w .` and `go vet`.

---

## File Structure

| File | Responsibility |
|---|---|
| Create: `motion_profile.go` | All profile math. Pure: no I/O, no hardware. `k`-scaling, deg→steps, deg/s²→`Acc`, clamping, `MoveOptions` cap reduction. |
| Create: `motion_profile_test.go` | Unit tests for the above. Table-driven, matching `speed_test.go`'s style. |
| Modify: `manager.go` | Add `ServoProfile` + `MoveServosWithProfiles` using `SetGoals`. `writePositions` untouched. |
| Modify: `arm.go` | `moveJoints` takes a from/to pair; `MoveToJointPositions` reads position; `MoveThroughJointPositions` uses deltas and honors `MoveOptions`; config default and **both** bounds checks. |
| Modify: `go.mod`, `go.sum` | feetech-servo v0.6.1. |
| Modify: `README.md`, `CLAUDE.md` | Remove the "not yet enforced" note; record that hardware now matches the simulated arm. |
| Modify: `cmd/cli/profile_bench.go` | Add a `readrate` test for the hardware phase. |

---

## Task 1: Bump feetech-servo to v0.6.1

The version bump is a **correctness dependency**, not just performance: the spec makes the position read unconditional, including on the `wait: false` teleop path. That read costs ~12 ms on v0.6.0's flush and ~1 ms on v0.6.1's.

**Files:** Modify `go.mod`, `go.sum`

- [ ] **Step 1: Bump**

```bash
go get github.com/hipsterbrown/feetech-servo@v0.6.1
```

- [ ] **Step 2: Confirm the flush fix is actually in the module cache**

```bash
sed -n '/func (t \*SerialTransport) Flush/,/^}/p' \
  "$(go env GOMODCACHE)/github.com/hipsterbrown/feetech-servo@v0.6.1/transports/serial_os.go"
```

Expected: a body that calls `t.port.ResetInputBuffer()`. If it still contains a `for` loop reading into a buffer, the wrong version was fetched — stop and report.

- [ ] **Step 3: Confirm `SetGoals` exists with the expected signature**

```bash
grep -n -A3 "func (g \*ServoGroup) SetGoals" \
  "$(go env GOMODCACHE)/github.com/hipsterbrown/feetech-servo@v0.6.1/feetech/group.go"
```

Expected: `func (g *ServoGroup) SetGoals(ctx context.Context, goals map[int]GoalRequest) error`.

- [ ] **Step 4: Verify nothing broke**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
```

Expected: PASS. Note `TestEnumerateSerialPorts` in `discovery_test.go` fails on machines with no serial hardware — that alone is not a regression.

- [ ] **Step 5: Commit**

```bash
git add go.mod go.sum
git commit -m "chore: bump feetech-servo to v0.6.1 for the flush fix"
```

---

## Task 2: Acceleration unit conversion

**Files:** Create `motion_profile.go`, `motion_profile_test.go`

- [ ] **Step 1: Write the failing test**

Create `motion_profile_test.go`:

```go
package so_arm

import "testing"

func TestDegPerSecSqToAccUnits(t *testing.T) {
	tests := []struct {
		name  string
		degs2 float64
		want  int
	}{
		// (degs2 * 4096/360 - 386) / 108, rounded, clamped to [1,50].
		{"shipped default 500 stays below the clamp", 500, 49},
		{"old documented default 100", 100, 7},
		{"config minimum 50", 50, 2},
		{"600 would clamp, which is why it is not the default", 600, 50},
		{"unreachable low value clamps to the floor, not to 0", 10, 1},
		{"zero clamps to the floor, never to 0 (0 means UNLIMITED)", 0, 1},
		{"absurdly high clamps to the knee", 100000, 50},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := degPerSecSqToAccUnits(tt.degs2); got != tt.want {
				t.Errorf("degPerSecSqToAccUnits(%v) = %d, want %d", tt.degs2, got, tt.want)
			}
		})
	}
}
```

- [ ] **Step 2: Run it and confirm it fails**

```bash
go test -count=1 -run TestDegPerSecSqToAccUnits .
```

Expected: FAIL — `undefined: degPerSecSqToAccUnits`.

- [ ] **Step 3: Implement**

Create `motion_profile.go`:

```go
package so_arm

import "math"

// Acceleration-register calibration, measured on hardware 2026-08-19 (see
// docs/superpowers/results/2026-08-19-bench-run.md).
//
// The register is linear from Acc 1 to about 50 and saturates above it. A single global
// slope and offset are used rather than a per-joint table: only three of five joints were
// measured, and the 89-128 steps/s^2 per unit spread implies roughly 10% arrival-timing
// error between joints -- small against a current spread of 50-100% of move duration.
const (
	accStepsPerUnit = 108.0 // mean measured slope, steps/s^2 per Acc unit
	accOffsetSteps  = 386.0 // mean measured offset, steps/s^2

	// minAccUnits is 1, NOT 0. Acc 0 means an UNLIMITED ramp -- the jerkiest possible
	// command, the exact opposite of what a caller asking for gentle acceleration wants.
	minAccUnits = 1

	// maxAccUnits is the measured knee. Above it the register has no further effect, so
	// scaling silently stops working.
	maxAccUnits = 50
)

// degPerSecSqToAccUnits converts an angular acceleration into the servo's Acc register
// value, clamped to the range where the register actually does something.
func degPerSecSqToAccUnits(degsPerSecSq float64) int {
	steps := degsPerSecSq * stepsPerDegree
	units := int(math.Round((steps - accOffsetSteps) / accStepsPerUnit))
	if units < minAccUnits {
		units = minAccUnits
	}
	if units > maxAccUnits {
		units = maxAccUnits
	}
	return units
}
```

`stepsPerDegree` already exists in `speed.go`.

- [ ] **Step 4: Run it and confirm it passes**

```bash
go test -count=1 -run TestDegPerSecSqToAccUnits .
```

Expected: PASS, 7 subtests.

- [ ] **Step 5: Verify and commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
git add motion_profile.go motion_profile_test.go
git commit -m "feat(arm): acceleration register unit conversion"
```

---

## Task 3: Coordinated profile scaling

The core of the feature. `kᵢ = dᵢ / d_max`, then `vᵢ = v_max·kᵢ` and `aᵢ = a_max·kᵢ`, which makes every joint's profile a time-scaled copy of the same shape and so gives matching durations in both the triangular and trapezoidal regimes.

**Files:** Modify `motion_profile.go`, `motion_profile_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `motion_profile_test.go`:

```go
func TestCoordinatedProfiles(t *testing.T) {
	const (
		refSpeed = 50.0  // deg/s
		refAccel = 500.0 // deg/s^2
	)

	t.Run("equal travels give equal profiles", func(t *testing.T) {
		got := coordinatedProfiles([]float64{10, 10, 10}, refSpeed, refAccel, nil)
		if len(got) != 3 {
			t.Fatalf("len = %d, want 3", len(got))
		}
		for i, p := range got {
			if p != got[0] {
				t.Errorf("joint %d = %+v, want %+v (equal travels must scale equally)", i, p, got[0])
			}
		}
		// The reference joint runs at the full reference.
		if want := degPerSecToStepsPerSec(refSpeed); got[0].speedSteps != want {
			t.Errorf("speedSteps = %d, want %d", got[0].speedSteps, want)
		}
	})

	t.Run("4:1 travel gives 4:1 speed and acceleration", func(t *testing.T) {
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, nil)
		// k = 1.0 and 0.25.
		if want := degPerSecToStepsPerSec(50); got[0].speedSteps != want {
			t.Errorf("long joint speed = %d, want %d", got[0].speedSteps, want)
		}
		if want := degPerSecToStepsPerSec(12.5); got[1].speedSteps != want {
			t.Errorf("short joint speed = %d, want %d", got[1].speedSteps, want)
		}
		if want := degPerSecSqToAccUnits(500); got[0].accUnits != want {
			t.Errorf("long joint acc = %d, want %d", got[0].accUnits, want)
		}
		if want := degPerSecSqToAccUnits(125); got[1].accUnits != want {
			t.Errorf("short joint acc = %d, want %d", got[1].accUnits, want)
		}
	})

	t.Run("direction does not matter, only magnitude", func(t *testing.T) {
		fwd := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, nil)
		rev := coordinatedProfiles([]float64{-40, -10}, refSpeed, refAccel, nil)
		for i := range fwd {
			if fwd[i] != rev[i] {
				t.Errorf("joint %d: forward %+v != reverse %+v", i, fwd[i], rev[i])
			}
		}
	})

	t.Run("all joints stationary is a no-op", func(t *testing.T) {
		if got := coordinatedProfiles([]float64{0, 0, 0}, refSpeed, refAccel, nil); got != nil {
			t.Errorf("got %+v, want nil (nothing to command)", got)
		}
	})

	t.Run("a single moving joint runs at the full reference", func(t *testing.T) {
		got := coordinatedProfiles([]float64{0, 20, 0}, refSpeed, refAccel, nil)
		if want := degPerSecToStepsPerSec(refSpeed); got[1].speedSteps != want {
			t.Errorf("moving joint speed = %d, want %d", got[1].speedSteps, want)
		}
	})

	t.Run("stationary joints never round into the 0 sentinels", func(t *testing.T) {
		// Speed 0 means MAXIMUM and Acc 0 means UNLIMITED, so a k=0 joint must floor to 1
		// on both, not to 0. Harmless on the wire because its goal equals its position.
		got := coordinatedProfiles([]float64{20, 0}, refSpeed, refAccel, nil)
		if got[1].speedSteps != minSpeedSteps {
			t.Errorf("stationary speedSteps = %d, want %d (0 would mean MAX SPEED)",
				got[1].speedSteps, minSpeedSteps)
		}
		if got[1].accUnits != minAccUnits {
			t.Errorf("stationary accUnits = %d, want %d (0 would mean UNLIMITED)",
				got[1].accUnits, minAccUnits)
		}
	})

	t.Run("a tiny travel floors instead of rounding into the sentinels", func(t *testing.T) {
		// k = 0.0001: speed scales to 0.005 deg/s and acceleration to 0.05 deg/s^2, both
		// of which round to 0 without the floors.
		got := coordinatedProfiles([]float64{100, 0.01}, refSpeed, refAccel, nil)
		if got[1].speedSteps < minSpeedSteps {
			t.Errorf("speedSteps = %d, want >= %d", got[1].speedSteps, minSpeedSteps)
		}
		if got[1].accUnits < minAccUnits {
			t.Errorf("accUnits = %d, want >= %d", got[1].accUnits, minAccUnits)
		}
	})

	t.Run("the shipped default leaves the reference joint UNCLAMPED", func(t *testing.T) {
		// This is why the default is 500 and not 600. If the reference joint (k=1) clamps
		// at maxAccUnits while lower-k joints keep their exact scaled value, coordination
		// silently breaks at the default -- the reference arrives late and every joint
		// above k~0.85 collapses to the same register value.
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, defaultAccelDegsPerSecSq, nil)
		if got[0].accUnits >= maxAccUnits {
			t.Errorf("reference joint accUnits = %d, which is at the clamp of %d; "+
				"the default acceleration is too high and coordination is broken",
				got[0].accUnits, maxAccUnits)
		}
	})
}
```

- [ ] **Step 2: Run and confirm failure**

```bash
go test -count=1 -run TestCoordinatedProfiles .
```

Expected: FAIL — `undefined: coordinatedProfiles`, `undefined: jointProfile`, `undefined: defaultAccelDegsPerSecSq`.

- [ ] **Step 3: Implement**

Append to `motion_profile.go`:

```go
// defaultAccelDegsPerSecSq is the shipped default, deliberately chosen so it maps to an Acc
// value BELOW maxAccUnits. 600 deg/s^2 maps to 59.6 and would clamp -- clamping the
// reference joint (k=1) while lower-k joints keep their exact scaled value breaks the
// coordination this whole mechanism exists to provide.
const defaultAccelDegsPerSecSq = 500.0

// jointProfile is one servo's commanded motion profile, in register units.
type jointProfile struct {
	speedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	accUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// coordinatedProfiles computes a profile per joint so that every joint finishes its travel
// at the same moment.
//
// Each joint is scaled by its share of the longest travel, k = d/d_max, applied to BOTH
// speed and acceleration. Scaling both makes each joint's profile a time-scaled copy of the
// same shape, which yields equal durations whether the motion is acceleration-limited
// (t = 2*sqrt(d/a); k cancels inside the radical) or speed-limited
// (t = d/v + v/a; k cancels term by term). Scaling speed alone would coordinate only the
// speed-limited case -- and at the defaults the crossover is 5 degrees, so most
// motion-planner waypoints fall in the acceleration-limited regime.
//
// travelsDeg may hold negative values; only magnitude matters. caps may be nil.
// Returns nil when no joint moves, in which case the caller should command nothing.
func coordinatedProfiles(travelsDeg []float64, refSpeedDegsPerSec, refAccelDegsPerSecSq float64, caps []jointLimits) []jointProfile {
	maxTravel := 0.0
	for _, d := range travelsDeg {
		if a := math.Abs(d); a > maxTravel {
			maxTravel = a
		}
	}
	if maxTravel == 0 {
		return nil
	}

	speed, accel := reduceReference(travelsDeg, maxTravel, refSpeedDegsPerSec, refAccelDegsPerSecSq, caps)

	out := make([]jointProfile, len(travelsDeg))
	for i, d := range travelsDeg {
		k := math.Abs(d) / maxTravel
		out[i] = jointProfile{
			// degPerSecToStepsPerSec, NOT resolveSpeedDegsPerSec: the latter floors at
			// 3 deg/s, which would destroy scaling for any joint below k ~ 0.06.
			speedSteps: degPerSecToStepsPerSec(speed * k),
			accUnits:   degPerSecSqToAccUnits(accel * k),
		}
	}
	return out
}
```

Add a temporary stub so this task compiles; Task 4 implements it properly:

```go
// reduceReference is implemented in Task 4.
func reduceReference(travelsDeg []float64, maxTravel, speed, accel float64, caps []jointLimits) (float64, float64) {
	return speed, accel
}

// jointLimits is defined properly in Task 4.
type jointLimits struct{}
```

- [ ] **Step 4: Run and confirm all subtests pass**

```bash
go test -count=1 -run TestCoordinatedProfiles . -v 2>&1 | tail -20
```

Expected: PASS, 8 subtests.

- [ ] **Step 5: Verify and commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
git add motion_profile.go motion_profile_test.go
git commit -m "feat(arm): coordinated per-joint speed and acceleration scaling"
```

---

## Task 4: `MoveOptions` caps reduce the shared reference

Per-joint caps are constraints, not targets. Honoring one by slowing a single joint would destroy the coordination Task 3 just built, so instead the caps lower the shared reference that every joint derives from.

**Files:** Modify `motion_profile.go`, `motion_profile_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `motion_profile_test.go`:

```go
func TestReduceReferenceAndCaps(t *testing.T) {
	const refSpeed, refAccel = 50.0, 500.0

	t.Run("a cap on the reference joint slows everyone proportionally", func(t *testing.T) {
		caps := []jointLimits{{maxSpeedDegsPerSec: 25}, {}}
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, caps)
		// k = 1.0 and 0.25; the cap halves the reference, so both halve.
		if want := degPerSecToStepsPerSec(25); got[0].speedSteps != want {
			t.Errorf("capped joint = %d, want %d", got[0].speedSteps, want)
		}
		if want := degPerSecToStepsPerSec(6.25); got[1].speedSteps != want {
			t.Errorf("uncapped joint = %d, want %d (it must slow too, to stay coordinated)",
				got[1].speedSteps, want)
		}
	})

	t.Run("a cap on a SHORT-travel joint still slows the whole move", func(t *testing.T) {
		// This is the case that distinguishes reference-reduction from per-joint clamping.
		// The short joint would have run at 12.5 deg/s; capping it at 5 forces the shared
		// reference down to 5/0.25 = 20 deg/s, so the long joint drops from 50 to 20.
		caps := []jointLimits{{}, {maxSpeedDegsPerSec: 5}}
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, caps)
		if want := degPerSecToStepsPerSec(20); got[0].speedSteps != want {
			t.Errorf("long joint = %d, want %d (a short joint's cap must bind everyone)",
				got[0].speedSteps, want)
		}
		if want := degPerSecToStepsPerSec(5); got[1].speedSteps != want {
			t.Errorf("capped short joint = %d, want %d", got[1].speedSteps, want)
		}
	})

	t.Run("every joint ends at or below its own cap", func(t *testing.T) {
		caps := []jointLimits{{maxSpeedDegsPerSec: 30}, {maxSpeedDegsPerSec: 8}, {maxSpeedDegsPerSec: 40}}
		travels := []float64{40, 20, 10}
		got := coordinatedProfiles(travels, refSpeed, refAccel, caps)
		for i := range travels {
			limit := degPerSecToStepsPerSec(caps[i].maxSpeedDegsPerSec)
			if got[i].speedSteps > limit {
				t.Errorf("joint %d = %d steps/s, exceeds its cap of %d", i, got[i].speedSteps, limit)
			}
		}
	})

	t.Run("a stationary joint's cap cannot bind", func(t *testing.T) {
		// k = 0 would divide by zero. A stationary joint moves at 0 regardless, so no cap
		// on it can be violated and it must be excluded from the reduction.
		caps := []jointLimits{{}, {maxSpeedDegsPerSec: 0.001}}
		got := coordinatedProfiles([]float64{40, 0}, refSpeed, refAccel, caps)
		if want := degPerSecToStepsPerSec(refSpeed); got[0].speedSteps != want {
			t.Errorf("moving joint = %d, want %d (a stationary joint's cap must not bind)",
				got[0].speedSteps, want)
		}
	})

	t.Run("acceleration caps reduce the reference the same way", func(t *testing.T) {
		caps := []jointLimits{{maxAccelDegsPerSecSq: 250}, {}}
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, caps)
		if want := degPerSecSqToAccUnits(250); got[0].accUnits != want {
			t.Errorf("capped joint acc = %d, want %d", got[0].accUnits, want)
		}
		if want := degPerSecSqToAccUnits(62.5); got[1].accUnits != want {
			t.Errorf("uncapped joint acc = %d, want %d", got[1].accUnits, want)
		}
	})

	t.Run("a zero cap means uncapped, not stopped", func(t *testing.T) {
		caps := []jointLimits{{maxSpeedDegsPerSec: 0}, {}}
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, caps)
		if want := degPerSecToStepsPerSec(refSpeed); got[0].speedSteps != want {
			t.Errorf("got %d, want %d (an unset cap must not pin the arm to zero)",
				got[0].speedSteps, want)
		}
	})
}
```

- [ ] **Step 2: Run and confirm failure**

```bash
go test -count=1 -run TestReduceReferenceAndCaps .
```

Expected: FAIL — `unknown field maxSpeedDegsPerSec in struct literal`.

- [ ] **Step 3: Implement**

Replace the Task 3 stubs in `motion_profile.go`:

```go
// jointLimits is an optional per-joint cap derived from arm.MoveOptions. A zero field means
// "no cap" -- never "stop", which is why every check below tests for > 0.
type jointLimits struct {
	maxSpeedDegsPerSec   float64
	maxAccelDegsPerSecSq float64
}

// reduceReference lowers the shared speed and acceleration reference until no per-joint cap
// would be exceeded.
//
// Caps are constraints, not targets. Clamping one joint to its cap would leave it out of
// step with the others and destroy the coordination; instead the most-constrained joint
// sets the pace for everyone. Since joint i is commanded v_max*k_i, requiring
// v_max <= cap_i/k_i for every moving joint guarantees every joint honors its own cap.
//
// Stationary joints are excluded: k = 0 would divide by zero, and a joint that is not moving
// cannot violate a speed or acceleration cap anyway.
func reduceReference(travelsDeg []float64, maxTravel, speed, accel float64, caps []jointLimits) (float64, float64) {
	for i, d := range travelsDeg {
		if i >= len(caps) {
			break
		}
		k := math.Abs(d) / maxTravel
		if k == 0 {
			continue
		}
		if c := caps[i].maxSpeedDegsPerSec; c > 0 && c/k < speed {
			speed = c / k
		}
		if c := caps[i].maxAccelDegsPerSecSq; c > 0 && c/k < accel {
			accel = c / k
		}
	}
	return speed, accel
}
```

- [ ] **Step 4: Run and confirm all pass**

```bash
go test -count=1 -run "TestCoordinatedProfiles|TestReduceReferenceAndCaps" . -v 2>&1 | tail -25
```

Expected: PASS, all subtests from both.

- [ ] **Step 5: Verify and commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
git add motion_profile.go motion_profile_test.go
git commit -m "feat(arm): MoveOptions caps reduce the shared motion reference"
```

---

## Task 5: Convert `arm.MoveOptions` into per-joint limits

**Files:** Modify `motion_profile.go`, `motion_profile_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `motion_profile_test.go`:

```go
func TestJointLimitsFromMoveOptions(t *testing.T) {
	t.Run("nil options means no caps", func(t *testing.T) {
		got, err := jointLimitsFromMoveOptions(nil, 5)
		if err != nil || got != nil {
			t.Errorf("got (%v, %v), want (nil, nil)", got, err)
		}
	})

	t.Run("a scalar applies to every joint", func(t *testing.T) {
		opts := &arm.MoveOptions{MaxVelRads: utils.DegToRad(30)}
		got, err := jointLimitsFromMoveOptions(opts, 3)
		if err != nil {
			t.Fatal(err)
		}
		for i, l := range got {
			if math.Abs(l.maxSpeedDegsPerSec-30) > 1e-9 {
				t.Errorf("joint %d = %v, want 30", i, l.maxSpeedDegsPerSec)
			}
		}
	})

	t.Run("a per-joint slice IGNORES the scalar, per arm.proto", func(t *testing.T) {
		// arm.proto: the scalar is "Ignored when max_vel_degs_per_sec_joints is set".
		// RDK deliberately does not enforce this, so this driver must.
		opts := &arm.MoveOptions{
			MaxVelRads:       utils.DegToRad(30),
			MaxVelRadsJoints: []float64{utils.DegToRad(10), utils.DegToRad(20)},
		}
		got, err := jointLimitsFromMoveOptions(opts, 2)
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(got[0].maxSpeedDegsPerSec-10) > 1e-9 {
			t.Errorf("joint 0 = %v, want 10 (the scalar 30 must be ignored)", got[0].maxSpeedDegsPerSec)
		}
		if math.Abs(got[1].maxSpeedDegsPerSec-20) > 1e-9 {
			t.Errorf("joint 1 = %v, want 20", got[1].maxSpeedDegsPerSec)
		}
	})

	t.Run("a wrong-length slice is rejected", func(t *testing.T) {
		// RDK sizes the slice from whatever the client sent with no DoF check, so silently
		// ignoring or truncating would violate a cap the caller believes is in force.
		opts := &arm.MoveOptions{MaxVelRadsJoints: []float64{1, 2, 3}}
		if _, err := jointLimitsFromMoveOptions(opts, 5); err == nil {
			t.Error("a 3-entry slice on a 5-DoF arm was accepted, want an error")
		}
	})

	t.Run("velocity and acceleration are independent", func(t *testing.T) {
		opts := &arm.MoveOptions{
			MaxVelRadsJoints: []float64{utils.DegToRad(10), utils.DegToRad(20)},
			MaxAccRads:       utils.DegToRad(400),
		}
		got, err := jointLimitsFromMoveOptions(opts, 2)
		if err != nil {
			t.Fatal(err)
		}
		// Per-joint velocity set, scalar acceleration still applies to all.
		if math.Abs(got[0].maxSpeedDegsPerSec-10) > 1e-9 {
			t.Errorf("joint 0 speed = %v, want 10", got[0].maxSpeedDegsPerSec)
		}
		for i, l := range got {
			if math.Abs(l.maxAccelDegsPerSecSq-400) > 1e-9 {
				t.Errorf("joint %d accel = %v, want 400", i, l.maxAccelDegsPerSecSq)
			}
		}
	})
}
```

Add to the test file's imports: `"math"`, `"go.viam.com/rdk/components/arm"`, `"go.viam.com/rdk/utils"`.

- [ ] **Step 2: Run and confirm failure**

```bash
go test -count=1 -run TestJointLimitsFromMoveOptions .
```

Expected: FAIL — `undefined: jointLimitsFromMoveOptions`.

- [ ] **Step 3: Implement**

Append to `motion_profile.go`:

```go
// jointLimitsFromMoveOptions converts arm.MoveOptions into per-joint caps.
//
// Per arm.proto the scalar limit is IGNORED when the corresponding per-joint slice is set,
// rather than merged with it. RDK's own conversion explicitly leaves that to the
// implementer (components/arm/pb_helpers.go), so this driver enforces it.
//
// Returns an error for a per-joint slice whose length does not match the arm's DoF: RDK
// sizes the slice from whatever the client sent without checking, and silently ignoring or
// truncating it would produce motion that violates a cap the caller believes is in force.
func jointLimitsFromMoveOptions(opts *arm.MoveOptions, dof int) ([]jointLimits, error) {
	if opts == nil {
		return nil, nil
	}
	limits := make([]jointLimits, dof)

	if n := len(opts.MaxVelRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"MoveOptions.MaxVelRadsJoints has %d entries but the arm has %d joints", n, dof)
		}
		for i, v := range opts.MaxVelRadsJoints {
			limits[i].maxSpeedDegsPerSec = utils.RadToDeg(v)
		}
	} else if opts.MaxVelRads > 0 {
		d := utils.RadToDeg(opts.MaxVelRads)
		for i := range limits {
			limits[i].maxSpeedDegsPerSec = d
		}
	}

	if n := len(opts.MaxAccRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"MoveOptions.MaxAccRadsJoints has %d entries but the arm has %d joints", n, dof)
		}
		for i, a := range opts.MaxAccRadsJoints {
			limits[i].maxAccelDegsPerSecSq = utils.RadToDeg(a)
		}
	} else if opts.MaxAccRads > 0 {
		d := utils.RadToDeg(opts.MaxAccRads)
		for i := range limits {
			limits[i].maxAccelDegsPerSecSq = d
		}
	}

	return limits, nil
}
```

Add to `motion_profile.go`'s imports: `"fmt"`, `"go.viam.com/rdk/components/arm"`, `"go.viam.com/rdk/utils"`.

- [ ] **Step 4: Run and confirm passing**

```bash
go test -count=1 -run TestJointLimitsFromMoveOptions . -v 2>&1 | tail -15
```

Expected: PASS, 5 subtests.

- [ ] **Step 5: Verify and commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
git add motion_profile.go motion_profile_test.go
git commit -m "feat(arm): convert MoveOptions into per-joint limits"
```

---

## Task 6: Controller method that writes profiles

**Files:** Modify `manager.go`

- [ ] **Step 1: Add the method**

Add to `manager.go`, immediately after `MoveServosToPositions` (which ends near `manager.go:106`):

```go
// ServoProfile is one servo's speed and acceleration for a coordinated move.
type ServoProfile struct {
	SpeedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	AccUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// MoveServosWithProfiles commands each servo to its angle under its OWN speed and
// acceleration, in a single SetGoals sync write.
//
// This exists alongside writePositions rather than replacing it. writePositions can only
// issue one shared speed and cannot set acceleration at all, which is why joints arrive at
// different times today; but two of its callers are single-servo gripper paths where
// coordination is meaningless, so it stays as it is.
//
// Time is always written as 0. RegGoalTime is inert on STS servos -- FEETECH's own
// SMS_STS driver hardcodes 0 there and exposes no time parameter (see the 2026-08-19 bench
// results). Writing anything else would be silently ignored.
func (s *SafeSoArmController) MoveServosWithProfiles(
	ctx context.Context,
	servoIDs []int,
	jointAngles []float64,
	profiles []ServoProfile,
) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(servoIDs) != len(jointAngles) || len(servoIDs) != len(profiles) {
		return fmt.Errorf("servo IDs (%d), angles (%d) and profiles (%d) must be the same length",
			len(servoIDs), len(jointAngles), len(profiles))
	}

	goals := make(map[int]feetech.GoalRequest, len(servoIDs))
	for i, servoID := range servoIDs {
		normalized := utils.RadToDeg(jointAngles[i])
		if isGripperServo(servoID) {
			normalized = (jointAngles[i]/math.Pi + 1.0) / 2.0 * 100.0
		}
		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(normalized)
		if err != nil {
			return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
		}
		goals[servoID] = feetech.GoalRequest{
			Position: raw,
			Speed:    profiles[i].SpeedSteps,
			Acc:      profiles[i].AccUnits,
			Time:     0,
		}
	}
	return s.group.SetGoals(ctx, goals)
}
```

- [ ] **Step 2: Verify it builds and nothing regressed**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
```

Expected: PASS.

- [ ] **Step 3: Confirm `writePositions` is untouched**

```bash
git diff manager.go | grep -c "^-" 
```

Expected: `0` — this task is a pure addition. If any line was removed, an existing path was changed; stop and report.

- [ ] **Step 4: Commit**

```bash
git add manager.go
git commit -m "feat(controller): MoveServosWithProfiles writes per-servo speed and acceleration"
```

---

## Task 7: Wire `MoveToJointPositions`

**Files:** Modify `arm.go`

- [ ] **Step 1: Reshape `moveJoints`**

Replace `moveJoints`' signature and profile computation at `arm.go:613`. The function currently takes `(ctx, clampedPositions, speedDegsPerSec, wait)`; it now takes an explicit from/to pair plus the reference limits.

```go
// moveJoints commands a coordinated move from `from` to `to` (both radians, per arm servo),
// scaling every joint's speed and acceleration by its share of the longest travel so all
// joints arrive together. When wait is true it blocks until the arm stops.
//
// The caller must hold s.moveLock and manage s.isMoving.
func (s *so101) moveJoints(
	ctx context.Context,
	from, to []float64,
	speedDegsPerSec, accelDegsPerSecSq float64,
	caps []jointLimits,
	wait bool,
) error {
	travelsDeg := make([]float64, len(to))
	maxTravelDeg := 0.0
	for i := range to {
		d := math.Abs(to[i]-from[i]) * 180.0 / math.Pi
		travelsDeg[i] = d
		if d > maxTravelDeg {
			maxTravelDeg = d
		}
	}

	profiles := coordinatedProfiles(travelsDeg, speedDegsPerSec, accelDegsPerSecSq, caps)
	if profiles == nil {
		return nil // already there; nothing to command
	}

	servoProfiles := make([]ServoProfile, len(profiles))
	for i, p := range profiles {
		servoProfiles[i] = ServoProfile{SpeedSteps: p.speedSteps, AccUnits: p.accUnits}
	}

	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, servoProfiles); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	if !wait {
		return nil
	}
	timeout := moveTimeoutMs(maxTravelDeg, speedDegsPerSec)
	s.logger.Debugf("moveJoints: ref %.1f deg/s, %.0f deg/s^2, max travel %.1f deg, wait timeout %d ms",
		speedDegsPerSec, accelDegsPerSecSq, maxTravelDeg, timeout)
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, timeout)
}
```

- [ ] **Step 2: Update `MoveToJointPositions`**

Replace its body from the `s.mu.RLock()` block onward (`arm.go:679-684`):

```go
	s.mu.RLock()
	speed := float64(s.defaultSpeed)
	accel := float64(s.defaultAcc)
	s.mu.RUnlock()

	// The position read is now unconditional, where it used to happen only when waiting.
	// Coordination needs a travel reference, and on v0.6.1 this read costs about 1ms
	// against roughly 12ms on v0.6.0 -- which is why the version bump is a correctness
	// dependency, not just a performance one.
	current, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to read positions for coordinated move: %w", err)
	}

	return s.moveJoints(ctx, current, clamped, speed, accel, nil, parseWaitExtra(extra))
```

- [ ] **Step 3: Verify it builds**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go build ./cmd/module/ .
```

Expected: exit 0. `MoveThroughJointPositions` still calls the old signature and will fail to compile — fix it in Task 8, or add a temporary adapter if the build must stay green between tasks.

- [ ] **Step 4: Run the tests**

```bash
go test -count=1 ./cmd/module/ .
```

Expected: PASS. `arm_move_to_position_test.go` and `arm_wait_test.go` exercise this path — if either fails, read the failure before changing anything: it may be pinning the old uncoordinated behavior deliberately.

- [ ] **Step 5: Commit**

```bash
git add arm.go
git commit -m "feat(arm): coordinate MoveToJointPositions"
```

---

## Task 8: Wire `MoveThroughJointPositions` and honor `MoveOptions`

**Files:** Modify `arm.go`

- [ ] **Step 1: Replace the body from the options block (`arm.go:700`) through the waypoint loop**

```go
	s.mu.RLock()
	defaultSpeed := float64(s.defaultSpeed)
	defaultAccel := float64(s.defaultAcc)
	s.mu.RUnlock()

	caps, err := jointLimitsFromMoveOptions(options, len(s.armServoIDs))
	if err != nil {
		return err
	}
	speed, accel := defaultSpeed, defaultAccel
	if options != nil && len(options.MaxVelRadsJoints) == 0 && options.MaxVelRads > 0 {
		speed = resolveSpeedDegsPerSec(options.MaxVelRads, defaultSpeed)
	}

	// The first waypoint has no predecessor to difference against, so its travel is
	// measured from where the arm actually is. GoToInputs routinely emits single-waypoint
	// streams, which would otherwise have no travel reference at all. Every later segment
	// uses the planner's own deltas, so this is one read per CALL, not per waypoint.
	from, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to read positions for coordinated move: %w", err)
	}

	for idx, jointPositions := range positions {
		clamped, err := s.clampPositions(jointPositions)
		if err != nil {
			return err
		}
		isLast := idx == len(positions)-1
		if err := s.moveJoints(ctx, from, clamped, speed, accel, caps, isLast); err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
		from = clamped
	}
	return nil
```

Note `from = clamped` at the end of each iteration: subsequent segments difference against the previous *commanded* waypoint.

- [ ] **Step 2: Verify build, vet, format, tests**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
```

Expected: PASS.

- [ ] **Step 3: Confirm `manual_mode.go` still behaves**

`manual_mode.go:266` calls `MoveServosToPositions(ctx, ids, rads, 0, 0)` and must keep today's behavior — it was not part of this change.

```bash
grep -n "MoveServosToPositions" manual_mode.go arm.go
```

Expected: `manual_mode.go:266` only. If `arm.go` still calls it, a path was missed.

- [ ] **Step 4: Commit**

```bash
git add arm.go
git commit -m "feat(arm): coordinate waypoint streams and honor MoveOptions per-joint caps"
```

---

## Task 9: Acceleration config default and bounds

Both bounds checks must change together. They are in two places and it is easy to miss the second.

**Files:** Modify `arm.go`

- [ ] **Step 1: Update the config default and bounds (`arm.go:430-436`)**

```go
	accelerationDegsPerSec := conf.AccelerationDegsPerSec
	if accelerationDegsPerSec == 0 {
		accelerationDegsPerSec = defaultAccelDegsPerSecSq
	}
	if accelerationDegsPerSec < minAccelDegsPerSecSq || accelerationDegsPerSec > maxAccelDegsPerSecSq {
		return nil, fmt.Errorf(
			"acceleration_degs_per_sec_per_sec must be between %.0f and %.0f degrees/second^2, got %.1f",
			minAccelDegsPerSecSq, maxAccelDegsPerSecSq, accelerationDegsPerSec)
	}
```

- [ ] **Step 2: Update the `set_acceleration` DoCommand bounds (`arm.go:1025-1029`)**

```go
			if acc, ok := accVal.(float64); ok {
				if acc < minAccelDegsPerSecSq || acc > maxAccelDegsPerSecSq {
					return nil, fmt.Errorf(
						"acceleration must be between %.0f and %.0f degrees/second^2, got %.1f",
						minAccelDegsPerSecSq, maxAccelDegsPerSecSq, acc)
				}
```

- [ ] **Step 3: Add the bounds constants to `motion_profile.go`**

```go
// Configured-acceleration bounds, in deg/s^2. Both ends are set by what the register can
// actually express: Acc 1 delivers about 43 deg/s^2, so the old minimum of 10 was
// unreachable by more than 4x; Acc 50 delivers about 508, so values above that do nothing.
const (
	minAccelDegsPerSecSq = 50.0
	maxAccelDegsPerSecSq = 500.0
)
```

- [ ] **Step 4: Add a test pinning the bounds against what the register can express**

Append to `motion_profile_test.go`:

```go
func TestAccelBoundsAreReachable(t *testing.T) {
	// A configured value the register cannot express would silently clamp, so the validated
	// bounds must both land strictly inside [minAccUnits, maxAccUnits].
	if got := degPerSecSqToAccUnits(minAccelDegsPerSecSq); got <= minAccUnits {
		t.Errorf("minimum %.0f deg/s^2 maps to Acc %d, at or below the floor of %d",
			minAccelDegsPerSecSq, got, minAccUnits)
	}
	if got := degPerSecSqToAccUnits(maxAccelDegsPerSecSq); got >= maxAccUnits {
		t.Errorf("maximum %.0f deg/s^2 maps to Acc %d, at or above the knee of %d",
			maxAccelDegsPerSecSq, got, maxAccUnits)
	}
	if defaultAccelDegsPerSecSq < minAccelDegsPerSecSq || defaultAccelDegsPerSecSq > maxAccelDegsPerSecSq {
		t.Errorf("default %.0f is outside the validated range [%.0f, %.0f]",
			defaultAccelDegsPerSecSq, minAccelDegsPerSecSq, maxAccelDegsPerSecSq)
	}
}
```

- [ ] **Step 5: Verify and commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ . && go test -count=1 ./cmd/module/ .
git add arm.go motion_profile.go motion_profile_test.go
git commit -m "feat(arm): acceleration default 500 deg/s^2, bounds 50-500"
```

---

## Task 10: `readrate` bench test

The spec's decision to make the position read unconditional — including on the `wait: false` teleop path — rests on that read costing ~1 ms on v0.6.1. **That figure was extrapolated from the bench run's WRITE table.** Reads carry a response packet and were never measured. This task adds the measurement.

**Files:** Modify `cmd/cli/profile_bench.go`

- [ ] **Step 1: Add the test**

Append a `readrate` section modelled on `runWriteRate`. It sweeps the same `gapSweep` across both transports, times `bus.SyncRead(ctx, feetech.RegPresentPosition.Address, 4, ids)` — the same 4-byte position+velocity read the sampler uses — and reports the same percentile table.

Key differences from `writerate` to get right:

- A read has a **response packet**, so unlike `SetGoals` this is a genuine round trip. That is exactly what makes it worth measuring separately.
- Run with torque **disabled**, like `writerate` — this reads only, so nothing moves.
- Reuse `summarize` and the existing CSV shape; name the file via `csvName("readrate", "", 0, false, "both")`.
- Register it in `selectTests` as a **non-motion** test, alongside `writerate`.

- [ ] **Step 2: Verify build, vet, format, selftests**

```bash
gofmt -s -w cmd/cli/profile_bench.go && \
  go build -o /dev/null cmd/cli/profile_bench.go && \
  go vet cmd/cli/profile_bench.go && \
  go run cmd/cli/profile_bench.go -selftest 2>&1 | tail -2
```

Expected: exit 0; selftest count unchanged (this adds no pure functions).

- [ ] **Step 3: Confirm the dispatch accepts it**

```bash
go run cmd/cli/profile_bench.go -test=readrate -port=/dev/null 2>&1 | head -2
```

Expected: a real serial-port error, not "unknown -test".

- [ ] **Step 4: Commit**

```bash
git add cmd/cli/profile_bench.go
git commit -m "feat(bench): readrate test to measure SyncRead round-trip latency"
```

---

## Task 11: Documentation

**Files:** Modify `README.md`, `CLAUDE.md`

- [ ] **Step 1: `README.md`**

Replace the `acceleration_degs_per_sec_per_sec` row (`README.md:69`) and the paragraph at `README.md:104` that says it is "validated and stored but not yet enforced". It is now enforced. Document: default 500, range 50–500, that the register saturates so values above ~508 do nothing, and that lowering it trades speed for smoothness (a 20° move takes 25% longer at 500 than with an unlimited ramp).

Update the speed paragraph at `README.md:94`, which currently states that joints move independently and finish at different times. They now arrive together.

- [ ] **Step 2: `CLAUDE.md`**

The "Gotchas" section documents the simulated/hardware divergence as intentional — the simulated arm coordinating arrival while hardware uses independent-joint speed. That divergence is now resolved; both coordinate. Rewrite that bullet, and remove the note that `acceleration_degs_per_sec_per_sec` "is validated and stored on the hardware arm but not yet written to servo hardware".

Add the two register sentinels to Gotchas — `Speed: 0` means maximum and `Acc: 0` means unlimited — since both are the kind of trap that has already caused bugs here.

- [ ] **Step 3: Verify and commit**

```bash
go test -count=1 ./cmd/module/ .
git add README.md CLAUDE.md
git commit -m "docs: acceleration is enforced and joints now arrive together"
```

---

## Task 12: Hardware verification

**This is the gate on whether the approach is worth committing to.** Everything before it is unverified against a real arm.

**Files:** Create `docs/superpowers/results/2026-08-20-coordination-verification.md`

- [ ] **Step 1: Measure read latency first**

```bash
mkdir -p bench-out-v2
go run cmd/cli/profile_bench.go -port=<PORT> -test=readrate -out=./bench-out-v2
```

**This validates or refutes a load-bearing assumption.** The spec makes the position read unconditional on the strength of it costing ~1 ms on v0.6.1, extrapolated from write timings.

- If the `fastflush` rows read **≲2 ms**, the assumption holds and the unconditional read is fine on the teleop path.
- If they read **≫2 ms**, the spec's documented fallback applies: skip coordination when `wait` is false and retain uniform-speed behavior there. Record the number and take that branch rather than shipping a teleop regression.
- The `stock` rows should be flat at ~80 Hz, confirming this arm's baseline matches the earlier run.

- [ ] **Step 2: Re-run the write-rate sweep as a v0.6.1 regression check**

```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=writerate -out=./bench-out-v2
```

Compare against the 2026-08-19 table. The `fastflush` rows should be unchanged; the `stock` rows should now **match** them, because v0.6.1 fixed the flush that made them differ. If the stock rows are still ~80 Hz, the module did not pick up v0.6.1 — stop and check `go.mod`.

- [ ] **Step 3: Clear the workspace and measure arrival spread**

Position the arm with ≥40° of positive headroom on **every** joint, then:

```bash
go run cmd/cli/profile_bench.go -port=<PORT> -test=goaltime -move -full-arm -out=./bench-out-v2
```

**The bench harness commands servos directly and does not go through the driver**, so this measures the *servos'* behavior, not the new coordination code. Its value here is as a control: it should reproduce the 260–520 ms spread from 2026-08-19. If it does not, something else changed and the comparison below is invalid.

- [ ] **Step 4: Measure the driver's coordination through the Viam API**

The harness cannot exercise `coordinatedProfiles`. Drive the arm through the module and observe:

1. Build and run the module: `make bin/arm`, then configure `devrel:so101:arm` against `<PORT>`.
2. Command a move where joints have deliberately unequal travel — e.g. joint 1 by 8°, joint 5 by 40° — via `MoveToJointPositions`.
3. Record when each joint stops, using `cmd/cli/servo_info.go` polling or a `goaltime`-style sample capture.

**Success criterion:** arrival spread falls from the measured 260–520 ms to **roughly 10% of move duration** — the residual predicted by using a single global acceleration constant instead of a per-joint table.

If the spread does not improve, the `k`-scaling assumption is wrong somewhere and this is the point to find out, before more is built on it.

- [ ] **Step 5: Record the results**

Write `docs/superpowers/results/2026-08-20-coordination-verification.md` covering: measured read latency and whether the unconditional read survives it; the writerate before/after confirming v0.6.1; the control arrival spread; the driver's coordinated spread against the 10% target; and a go/no-go on the approach.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/results/2026-08-20-coordination-verification.md
git commit -m "docs: hardware verification of per-joint coordination"
```

---

## Done When

- [ ] `gofmt -l .` prints nothing; `go vet ./cmd/module/ .` exits 0
- [ ] `go test -count=1 ./cmd/module/ .` passes (modulo `TestEnumerateSerialPorts` without serial hardware)
- [ ] `go run cmd/cli/profile_bench.go -selftest` still passes
- [ ] `writePositions` is unchanged — `git diff <base> -- manager.go | grep "^-"` shows no removals from it
- [ ] Read latency measured, and the unconditional read either confirmed or the documented fallback taken
- [ ] Arrival spread measured through the driver and compared against the 10% target
- [ ] `docs/superpowers/results/2026-08-20-coordination-verification.md` records the numbers and a go/no-go

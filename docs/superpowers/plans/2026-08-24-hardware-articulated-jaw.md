# Hardware Articulated Jaw Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the hardware SO-101 gripper an optional 1-DoF revolute jaw behind an `articulated_jaw` config attribute (default `false`), so the motion planner can drive the real jaw — safely.

**Architecture:** Reuse the simulated gripper's shape (`articulated_jaw` flag, validate-all-before-any, `stopped` abort) and add the three things hardware forces: a 50 ms position cache with its own mutex and a generation counter, last-known-good on read failure, and refusing planner jaw commands that would actually move the jaw while holding a part.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, testify. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-08-24-hardware-articulated-jaw-design.md` — read it first.

**Branch:** `hardware-articulated-jaw`, stacked on `gripper-inputs` (PR #39). Open the PR with `--base gripper-inputs`.

---

## Background you must have before starting

Read the spec. These five facts are load-bearing and each has already caused a wrong design once:

1. **`g.mu` is not reusable for the cache.** `Open` (`gripper.go:249`), `Grab` (`:266`) and both `DoCommand` write paths (`:359`, `:388`) `g.mu.Lock()` at entry and *then* call `moveToPercent`. `sync.Mutex` is not reentrant, so invalidating under `g.mu` from inside that call deadlocks on the first `Open()`. The cache gets its own `jawMu`.
2. **`jawMu` must never be held across a `DoCommand`.** A viewer poll would otherwise block behind an `Open` sitting in `servo_wait_stop` for up to 2000 ms, stalling the whole frame system.
3. **The planner calls `GoToInputs` on every plan, with an unchanged jaw value.** `services/motion/builtin/builtin.go:686-688` skips a component only when `len(inputs) == 0`, never when unchanged. A 1-DoF gripper is in every step of every plan. **An unconditional reject-while-holding breaks pick-and-place.**
4. **Invalidation belongs in `servoDo`, not `moveToPercent`.** `set_position`'s raw form (`gripper.go:396-398`) calls `servoDo` directly, bypassing `moveToPercent`. `servoDo` is the single choke point every bus command passes through.
5. **rdk rejects a joint input over a limit by even one ULP.** Hence `jawLimitEpsilon`.

**Run tests with** `go test ./...`; **lint with** `gofmt -s -w . && go vet ./...`. `referenceframe.Input` is a type alias for `float64`.

**Staging:** stage explicit paths. Never `git add -A` or `git commit -a`.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `internal/geometry/gripper.go` | jaw geometry, conversions, shared constants | Modify — gains `JawLimitEpsilon` |
| `components/simulated/simulated_gripper.go` | simulated gripper | Modify — uses the relocated constant |
| `components/gripper/gripper.go` | hardware gripper | Modify — flag, cache, inputs, holding |
| `components/gripper/gripper_test.go` | hardware gripper tests + fake arm | Modify — harness seams, new tests |
| `docs/gripper.md`, `CLAUDE.md` | docs | Modify |

---

## Task 1: Relocate `jawLimitEpsilon` to `internal/geometry`

Both grippers need it; a divergent copy is the bad outcome. Pure move, no behaviour change.

**Files:** `internal/geometry/gripper.go`, `components/simulated/simulated_gripper.go`

- [ ] **Step 1: Move the constant**

Cut the `jawLimitEpsilon` block from `components/simulated/simulated_gripper.go:355-365` and paste it into `internal/geometry/gripper.go` after `JawPctFromRadians`, renamed **exported**:

```go
// JawLimitEpsilon tolerates a planner-issued input that lands just outside a joint limit due to
// float rounding (e.g. left-associative (range*i)/steps in the angle sweep, which overshoots by
// ~2.22e-16 -- one ULP -- at i==steps). rdk's frame system rejects an input outside its limit by
// even that 1 ULP.
//
// 1e-6 is deliberately far wider than a ULP (about 4.5e9 ULP at GripperJointMax) -- it absorbs
// ordinary float rounding, not machine precision, so do not "tighten" it toward a literal ULP.
// What bounds the jaw is JawPctFromRadians's saturation to 0/100, not this epsilon's width.
const JawLimitEpsilon = 1e-6
```

- [ ] **Step 2: Update the simulated gripper**

Replace its one use (`simulated_gripper.go:441`) with `geometry.JawLimitEpsilon`. Update the comment reference at `simulated_gripper_test.go:297` if it names the old identifier.

- [ ] **Step 3: Verify nothing moved**

Run: `go test ./... && gofmt -s -l . && go vet ./...`
Expected: PASS, empty gofmt output. No test expectations should need changing — if one does, stop and report.

- [ ] **Step 4: Commit**

```bash
git add internal/geometry/gripper.go components/simulated/simulated_gripper.go components/simulated/simulated_gripper_test.go
git commit -m "refactor(geometry): share JawLimitEpsilon between both grippers"
```

---

## Task 2: Test harness seams

Three seams the existing harness lacks. Test-only — no production code changes.

**Files:** `components/gripper/gripper_test.go`

- [ ] **Step 1: Let `newTestGripper` take a config**

`newTestGripper` (`gripper_test.go:151-163`) hard-codes `SO101GripperConfig{Arm: fa.name.Name}`. Add a variant that accepts one, keeping the existing signature working so current callers are untouched:

```go
func newTestGripperWithConfig(t *testing.T, fa *fakeServoArm, cfg SO101GripperConfig) *so101Gripper {
	t.Helper()
	cfg.Arm = fa.name.Name
	deps := resource.Dependencies{fa.name: fa}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &cfg,
	}
	g, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return g.(*so101Gripper)
}
```

Then reduce `newTestGripper` to `return newTestGripperWithConfig(t, fa, SO101GripperConfig{})`.

- [ ] **Step 2: Add a read hook to the fake**

So a test can act *while* a bus read is in flight. Add `onRead func()` to `fakeServoArm` and call it in the `CmdServoPosition` branch:

```go
	case servocmd.CmdServoPosition:
		if f.onRead != nil {
			f.onRead()
		}
		return map[string]any{"percent": f.percent, "raw": f.raw}, nil
```

The hook runs while the fake holds `f.mu`, so a hook must **not** issue another command through the fake. Task 4's test calls `g.invalidateJawCache()` directly, which takes only `jawMu` — no deadlock.

- [ ] **Step 3: Verify**

Run: `go test ./components/gripper/ -v`
Expected: PASS, all existing tests unchanged.

- [ ] **Step 4: Commit**

```bash
git add components/gripper/gripper_test.go
git commit -m "test(gripper): add config and read-hook seams to the fake arm harness"
```

---

## Task 3: `articulated_jaw` config flag

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestArticulatedJawConfigHardware covers the flag end to end. The ErrUnsupported half matters as
// much as the enabled half: robot/framesystem's CurrentInputs walks every frame with DoF and hard-
// errors if the component is not InputEnabled, so a 1-DoF model with unwired inputs would break
// the WHOLE machine's frame system, arm moves included.
func TestArticulatedJawConfigHardware(t *testing.T) {
	ctx := context.Background()

	t.Run("default is static", func(t *testing.T) {
		g := newTestGripper(t, newFakeServoArm())
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		assert.Empty(t, m.DoF(), "articulated_jaw must default to false")

		_, err = g.CurrentInputs(ctx)
		assert.ErrorIs(t, err, errors.ErrUnsupported)
		assert.ErrorIs(t, g.GoToInputs(ctx, []referenceframe.Input{0}), errors.ErrUnsupported)
	})

	t.Run("articulated", func(t *testing.T) {
		g := newTestGripperWithConfig(t, newFakeServoArm(), SO101GripperConfig{ArticulatedJaw: true})
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		require.Len(t, m.DoF(), 1)
	})
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `go test ./components/gripper/ -run TestArticulatedJawConfigHardware`
Expected: FAIL — `unknown field ArticulatedJaw`

- [ ] **Step 3: Implement**

Add to `SO101GripperConfig`:

```go
	// ArticulatedJaw gives the gripper a 1-DoF revolute jaw joint in its kinematic model and makes
	// it InputEnabled, so the motion planner sees the jaw as a variable it may drive. Off by
	// default: the jaw does not affect the TCP, so the planner gains a degree of freedom that
	// changes only collision geometry. See docs/gripper.md.
	ArticulatedJaw bool `json:"articulated_jaw,omitempty"`
```

Add `articulatedJaw bool` to `so101Gripper`, set it in the constructor, and pass `cfg.ArticulatedJaw` as `BuildGripperModel`'s fourth argument (`gripper.go:142`).

- [ ] **Step 4: Verify**

Run: `go test ./components/gripper/ -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): add the articulated_jaw config flag"
```

---

## Task 4: Position cache

The heart of this change. Read the spec's Decision 3 before starting.

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing tests**

```go
// TestJawCacheBoundsBusReads pins the reason the cache exists: every read is a DoCommand round
// trip plus a serial read on a bus shared with five arm servos, and the 3D viewer polls
// CurrentInputs continuously.
func TestJawCacheBoundsBusReads(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	before := countCommands(fa, servocmd.CmdServoPosition)
	for i := 0; i < 20; i++ {
		_, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
	}
	assert.Equalf(t, before, countCommands(fa, servocmd.CmdServoPosition),
		"20 calls inside the TTL should have hit the bus zero extra times")
}

// TestJawCacheExpires uses the injectable clock rather than sleeping.
func TestJawCacheExpires(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	base := time.Now()
	g.now = func() time.Time { return base }
	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	n := countCommands(fa, servocmd.CmdServoPosition)

	g.now = func() time.Time { return base.Add(jawCacheTTL + time.Millisecond) }
	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition), "expired cache must re-read")
}

// TestRawSetPositionInvalidatesCache proves servoDo is the right choke point: this path bypasses
// moveToPercent entirely.
func TestRawSetPositionInvalidatesCache(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	n := countCommands(fa, servocmd.CmdServoPosition)

	_, err = g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position", "servo_position": 2500.0,
	})
	require.NoError(t, err)

	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition),
		"a raw set_position must invalidate the cache")
}

// TestCurrentInputsSurvivesReadFailure: framesystem.CurrentInputs hard-errors the entire machine's
// frame system when a DoF-bearing component errors, so one dropped serial frame must not abort
// every arm move.
func TestCurrentInputsSurvivesReadFailure(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 40
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	good, err := g.CurrentInputs(ctx)
	require.NoError(t, err)

	fa.setErr(errors.New("bus timeout"))
	g.invalidateJawCache()
	after, err := g.CurrentInputs(ctx)
	require.NoError(t, err, "a dropped read must not break the frame system")
	assert.InDelta(t, good[0], after[0], 1e-12, "should serve the last known good reading")
}

// TestCurrentInputsErrorsBeforeFirstRead is the one case where erroring is correct.
func TestCurrentInputsErrorsBeforeFirstRead(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
	fa.setErr(errors.New("bus down"))
	g.clearJawCacheForTest() // drops jawHaveRead, simulating a never-successful start
	_, err := g.CurrentInputs(context.Background())
	assert.Error(t, err, "with no reading ever taken there is nothing honest to return")
}

// TestStopDuringReadDoesNotStampStaleValue closes the window opened by releasing jawMu across the
// bus read: an invalidation landing mid-read must not be overwritten by the in-flight pre-write
// value and stamped fresh.
func TestStopDuringReadDoesNotStampStaleValue(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 10
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	fa.onRead = func() { g.invalidateJawCache() } // lands inside the unlocked window
	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	fa.onRead = nil

	n := countCommands(fa, servocmd.CmdServoPosition)
	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition),
		"a reading invalidated mid-flight must not have been stamped fresh")
}
```

Add the helpers:

```go
func countCommands(fa *fakeServoArm, name string) int {
	n := 0
	for _, c := range fa.issued() {
		if c == name {
			n++
		}
	}
	return n
}
```

and `setErr` on the fake (guarded by `f.mu`), plus a tiny `clearJawCacheForTest()` on the gripper that zeroes `jawHaveRead`.

- [ ] **Step 2: Run and watch them fail**

Run: `go test ./components/gripper/ -run 'TestJawCache|TestRawSetPosition|TestCurrentInputs|TestStopDuringRead'`
Expected: FAIL — undefined symbols.

- [ ] **Step 3: Implement**

Struct fields on `so101Gripper`:

```go
	// jawMu guards the position cache ONLY. It is deliberately not g.mu: Open/Grab/DoCommand hold
	// g.mu across moveToPercent, and sync.Mutex is not reentrant, so invalidating under g.mu would
	// deadlock. jawMu is never held across a DoCommand -- a viewer poll must not block behind an
	// Open sitting in servo_wait_stop.
	jawMu       sync.Mutex
	jawPct      float64
	jawReadAt   time.Time
	jawHaveRead bool
	jawGen      uint64

	now func() time.Time // injectable for tests; time.Now in production
```

Set `now: time.Now` in the constructor.

```go
// jawCacheTTL bounds how stale a jaw reading may be. It expires on wall clock regardless of
// writes, so invalidation only ever shortens staleness -- including a jaw back-driven by hand
// with torque off.
const jawCacheTTL = 50 * time.Millisecond

func (g *so101Gripper) invalidateJawCache() {
	g.jawMu.Lock()
	defer g.jawMu.Unlock()
	g.jawReadAt = time.Time{}
	g.jawGen++
}

// positionPercentCached returns the jaw opening, reading the bus at most once per jawCacheTTL.
func (g *so101Gripper) positionPercentCached(ctx context.Context) (float64, error) {
	g.jawMu.Lock()
	if g.jawHaveRead && !g.jawReadAt.IsZero() && g.now().Sub(g.jawReadAt) < jawCacheTTL {
		pct := g.jawPct
		g.jawMu.Unlock()
		return pct, nil
	}
	gen, havePrev, prev := g.jawGen, g.jawHaveRead, g.jawPct
	g.jawMu.Unlock()

	pct, err := g.positionPercent(ctx) // bus read, jawMu NOT held
	if err != nil {
		if havePrev {
			g.logger.Warnf("jaw position read failed, serving last known good %.1f%%: %v", prev, err)
			return prev, nil
		}
		return 0, err
	}

	g.jawMu.Lock()
	defer g.jawMu.Unlock()
	// Discard if the jaw was commanded while this read was in flight, or if it is still moving --
	// moveToPercent returns on a servo_wait_stop timeout too, after which the servo may be settling.
	if g.jawGen == gen && !g.isMoving.Load() {
		g.jawPct, g.jawReadAt, g.jawHaveRead = pct, g.now(), true
	}
	return pct, nil
}
```

Call `g.invalidateJawCache()` at the top of `servoDo` for every command except `CmdServoPosition` and `CmdServoCapabilities`.

In the constructor, after `probeServoSupport`, prime the cache — a failure is logged, not fatal:

```go
	if _, err := g.positionPercentCached(ctx); err != nil {
		logger.Warnf("could not prime the jaw position cache: %v", err)
	}
```

- [ ] **Step 4: Verify**

Run: `go test ./components/gripper/ -v && go test -race ./components/gripper/`
Expected: PASS, no race.

- [ ] **Step 5: Mutation-prove three guards**

Each: break it, record the actual failure output, revert.
1. Drop the `g.jawGen == gen` check → `TestStopDuringReadDoesNotStampStaleValue` must fail.
2. Invalidate in `moveToPercent` instead of `servoDo` → `TestRawSetPositionInvalidatesCache` must fail.
3. Return the error instead of `prev` on a failed read → `TestCurrentInputsSurvivesReadFailure` must fail.

- [ ] **Step 6: Commit**

```bash
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): cache the jaw position with a 50ms TTL"
```

---

## Task 5: `IsHoldingSomething`

Currently a stub returning `gripper.HoldingStatus{}` — it always reports "not holding", which makes Task 7's guard unimplementable and is independently wrong.

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestIsHoldingSomethingReportsGrasp: Grab already infers a grasp from the jaw stopping short of
// closed. IsHoldingSomething was a stub that always said "not holding"; GoToInputs's safety guard
// depends on it telling the truth.
func TestIsHoldingSomethingReportsGrasp(t *testing.T) {
	ctx := context.Background()
	for _, tc := range []struct {
		name    string
		percent float64
		holding bool
	}{
		{"jaw fully closed", 0, false},
		{"just under the threshold", graspThresholdPct - 1, false},
		{"just over the threshold", graspThresholdPct + 1, true},
		{"jaw wide open on a big part", 60, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			fa.percent = tc.percent
			g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
			st, err := g.IsHoldingSomething(ctx, nil)
			require.NoError(t, err)
			assert.Equal(t, tc.holding, st.IsHoldingSomething)
		})
	}
}
```

- [ ] **Step 2: Run and watch it fail**

Run: `go test ./components/gripper/ -run TestIsHoldingSomethingReportsGrasp`
Expected: FAIL — `undefined: graspThresholdPct`, and the stub reports false throughout.

- [ ] **Step 3: Implement**

Extract the literal from `Grab` (`gripper.go:286`):

```go
// graspThresholdPct is how far short of closed the jaw must stop for Grab to conclude something
// is between the fingers.
const graspThresholdPct = 15.0
```

Use it in `Grab` in place of the local `threshold := 15.0`, and implement:

```go
// IsHoldingSomething infers a grasp the same way Grab does: the jaw stopped short of closed, so
// something is between the fingers. Reads through the cache -- GoToInputs calls this on every
// planner-issued batch.
func (g *so101Gripper) IsHoldingSomething(
	ctx context.Context, extra map[string]interface{},
) (gripper.HoldingStatus, error) {
	pct, err := g.positionPercentCached(ctx)
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{IsHoldingSomething: pct-g.closedPosition > graspThresholdPct}, nil
}
```

- [ ] **Step 4: Verify + commit**

Run: `go test ./components/gripper/ -v`

```bash
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): implement IsHoldingSomething instead of stubbing it"
```

---

## Task 6: `CurrentInputs`

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestHardwareCurrentInputsTracksJaw(t *testing.T) {
	ctx := context.Background()
	for _, pct := range []float64{0, 50, 95} {
		fa := newFakeServoArm()
		fa.percent = pct
		g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		require.Len(t, in, 1)
		assert.InDelta(t, geometry.JawRadiansFromPct(pct), in[0], 1e-9)
	}
}
```

- [ ] **Step 2: Run, fail, implement**

```go
func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	if !g.articulatedJaw {
		return nil, errors.ErrUnsupported
	}
	pct, err := g.positionPercentCached(ctx)
	if err != nil {
		return nil, err // only reachable before the first successful read
	}
	return []referenceframe.Input{geometry.JawRadiansFromPct(pct)}, nil
}
```

- [ ] **Step 3: Verify + commit**

```bash
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): report the jaw angle from CurrentInputs"
```

---

## Task 7: `GoToInputs`

The safety-critical one. **Order matters** — validate, then no-op check, then holding check.

**Files:** `components/gripper/gripper.go`, `components/gripper/gripper_test.go`

- [ ] **Step 1: Write the failing tests**

```go
// TestGoToInputsNoOpBatchIssuesNoCommand is the pick-and-place regression guard, and the most
// important test in this file. builtin.go skips a component in a trajectory step only when
// len(inputs) == 0 -- never when unchanged -- so a 1-DoF gripper appears in EVERY step of EVERY
// plan carrying its current jaw value. Rejecting those would fail every arm move made while
// holding a part.
func TestGoToInputsNoOpBatchIssuesNoCommand(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 60 // wide enough that IsHoldingSomething reports true
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	current := geometry.JawRadiansFromPct(60)
	require.NoError(t, g.GoToInputs(ctx,
		[]referenceframe.Input{current},
		[]referenceframe.Input{current + jawHoldToleranceRad/2},
	), "a no-op batch must succeed even while holding")

	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"a no-op batch must not touch the bus; issued %v", fa.issued())
}

// TestGoToInputsRejectsWhileHolding: the planner moves the jaw 4-20%% of its range under
// obstruction, which on hardware means loosening a grip mid-trajectory.
func TestGoToInputsRejectsWhileHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 60
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	err := g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "holding")
	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"a rejected batch must not have moved the jaw")
}

// TestGoToInputsMovesWhenNotHolding is the complement -- the guard must not block ordinary use.
func TestGoToInputsMovesWhenNotHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 0 // closed, nothing held
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax}))
	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoMove))
}

// TestGoToInputsIssuesOnlyFinalTarget: a position-controlled STS3215 tracks the most recent
// command, so intermediate targets are never traversed. Sending them spends bus writes -- on the
// bus the arm is actively using -- for no change in behaviour.
func TestGoToInputsIssuesOnlyFinalTarget(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	steps := [][]referenceframe.Input{}
	for _, pct := range []float64{20, 40, 60, 80} {
		steps = append(steps, []referenceframe.Input{geometry.JawRadiansFromPct(pct)})
	}
	require.NoError(t, g.GoToInputs(ctx, steps...))

	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoMove))
	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoWaitStop))
	cmd := fa.lastCommand(servocmd.CmdServoMove)
	assert.InDelta(t, 80.0, cmd["percent"], 0.01, "must command the FINAL step's target")
}

// TestGoToInputsValidatesBeforeMoving: a batch that fails halfway leaves the jaw somewhere the
// planner did not intend.
func TestGoToInputsValidatesBeforeMoving(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	err := g.GoToInputs(ctx,
		[]referenceframe.Input{geometry.JawRadiansFromPct(50)},
		[]referenceframe.Input{geometry.GripperJointMax + 0.5}, // invalid
	)
	require.Error(t, err)
	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"nothing may move when any step is invalid")
}

// TestGoToInputsToleratesULPOvershoot: rdk rejects an input over a limit by even one ULP, and a
// planner trajectory riding a limit can produce exactly that.
func TestGoToInputsToleratesULPOvershoot(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
	over := math.Nextafter(geometry.GripperJointMax, math.Inf(1))
	assert.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{over}))
}

// TestGoToInputsAbortsOnStop. Note GoToInputs must set isMoving, or Stop never latches and this
// passes vacuously.
func TestGoToInputsAbortsOnStop(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	fa.onRead = nil
	fa.onMove = func() { _ = g.Stop(context.Background(), nil) } // fires mid-batch
	err := g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	assert.Error(t, err, "a stopped batch must report that it did not complete")
}
```

Add `onMove func()` to the fake, called in the `CmdServoMove` branch, mirroring `onRead`.

- [ ] **Step 2: Run and watch them fail**

Run: `go test ./components/gripper/ -run TestGoToInputs`
Expected: FAIL — `ErrUnsupported` and undefined `jawHoldToleranceRad`.

- [ ] **Step 3: Implement**

```go
// jawHoldToleranceRad is how far a commanded jaw angle must differ from the current one to count
// as actually moving the jaw. 0.01 rad is ~0.5%% of the 1.92 rad range and about 6 servo ticks --
// well above quantization, well below the 0.083-0.383 rad the planner was measured moving under
// obstruction.
//
// Numerically identical to rdk's defaultExecuteEpsilon and COMPLETELY unrelated to it: that one
// bounds how far a plan's first step may sit from CurrentInputs, on a path Move() does not take.
const jawHoldToleranceRad = 0.01

func (g *so101Gripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	if !g.articulatedJaw {
		return errors.ErrUnsupported
	}
	limits := g.model.DoF()
	if len(limits) != 1 {
		return fmt.Errorf("articulated gripper model has %d DoF, want 1", len(limits))
	}
	jaw := limits[0]

	// Validate EVERY step before moving any of them.
	for i, step := range inputSteps {
		if len(step) != 1 {
			return fmt.Errorf("step %d: got %d inputs, the jaw takes 1", i, len(step))
		}
		if step[0] < jaw.Min-geometry.JawLimitEpsilon || step[0] > jaw.Max+geometry.JawLimitEpsilon {
			return fmt.Errorf("step %d: jaw angle %.4f rad is outside [%.4f, %.4f]",
				i, step[0], jaw.Min, jaw.Max)
		}
	}
	if len(inputSteps) == 0 {
		return nil
	}

	current, err := g.positionPercentCached(ctx)
	if err != nil {
		return err
	}
	currentRad := geometry.JawRadiansFromPct(current)

	// A no-op batch is the common case: the gripper appears in every trajectory step of every
	// plan carrying its unchanged value. It must neither reject nor touch the bus.
	moves := false
	for _, step := range inputSteps {
		if math.Abs(step[0]-currentRad) > jawHoldToleranceRad {
			moves = true
			break
		}
	}
	if !moves {
		return nil
	}

	held, err := g.IsHoldingSomething(ctx, nil)
	if err != nil {
		return err
	}
	final := inputSteps[len(inputSteps)-1][0]
	if held.IsHoldingSomething {
		g.logger.Infof("refusing planner jaw command while holding: current %.4f rad, requested %.4f rad",
			currentRad, final)
		return fmt.Errorf(
			"gripper is holding something; refusing the planner's request to move the jaw from "+
				"%.4f to %.4f rad. Release the object or disable articulated_jaw", currentRad, final)
	}
	g.logger.Infof("jaw GoToInputs: %d step(s), %.4f -> %.4f rad", len(inputSteps), currentRad, final)

	g.mu.Lock()
	defer g.mu.Unlock()
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)
	g.clearStopped()

	// Only the final target: a position servo never traverses superseded waypoints.
	if err := g.moveToPercent(ctx, geometry.JawPctFromRadians(final)); err != nil {
		return err
	}
	if g.wasStopped() {
		return errors.New("jaw motion was stopped before completing the trajectory")
	}
	return nil
}
```

Add `stopped bool` guarded by `g.mu`, with `clearStopped`/`wasStopped`/`markStopped` helpers, and set it in `Stop` **only while moving** (mirroring `components/simulated/simulated.go:443-453`).

**Careful:** `Stop` is called from another goroutine while `GoToInputs` holds `g.mu`. `markStopped` must not take `g.mu`, or `Stop` deadlocks. Use `atomic.Bool` for `stopped`, matching `isMoving`.

- [ ] **Step 4: Verify**

Run: `go test ./components/gripper/ -v && go test -race ./components/gripper/`

- [ ] **Step 5: Mutation-prove the two guards that matter**

1. Remove the no-op early return → `TestGoToInputsNoOpBatchIssuesNoCommand` must fail. **This is the pick-and-place regression; record the output.**
2. Remove the holding check → `TestGoToInputsRejectsWhileHolding` must fail.

- [ ] **Step 6: Commit**

```bash
git add components/gripper/gripper.go components/gripper/gripper_test.go
git commit -m "feat(gripper): wire GoToInputs with a grasp-retention guard"
```

---

## Task 8: Deadlock guard + docs

**Files:** `components/gripper/gripper_test.go`, `docs/gripper.md`, `CLAUDE.md`

- [ ] **Step 1: Write the deadlock test**

It must fail by *diagnostic*, not by hanging — a deadlock would otherwise stall until `go test`'s 10-minute panic, which cannot satisfy the mutation-proof rule.

```go
// TestOpenDoesNotDeadlock guards the reentrancy hazard: Open/Grab hold g.mu across moveToPercent,
// which invalidates the cache. Invalidating under g.mu would deadlock on the first call.
func TestOpenDoesNotDeadlock(t *testing.T) {
	g := newTestGripperWithConfig(t, newFakeServoArm(), SO101GripperConfig{ArticulatedJaw: true})
	done := make(chan error, 1)
	go func() { done <- g.Open(context.Background(), nil) }()
	select {
	case err := <-done:
		require.NoError(t, err)
	case <-time.After(5 * time.Second):
		t.Fatal("Open deadlocked: cache invalidation must not take g.mu")
	}
}
```

Mutation-prove it by making `invalidateJawCache` take `g.mu`; confirm the 5s timeout fires with that message, then revert.

- [ ] **Step 2: Document**

`docs/gripper.md` — add `articulated_jaw` to the attribute table and a short subsection covering: default off and why; that the planner may command jaw motion during an arm move; that a command which would move the jaw **while holding** is refused with an error, and no-op commands are not; that inputs are radians; and that a jaw reading may be up to 50 ms stale.

`CLAUDE.md` — extend the gripper gotcha to say the hardware gripper is 0-DoF *by default* with `articulated_jaw` as opt-in, and add one gotcha:

```markdown
- **A 1-DoF gripper gets `GoToInputs` on EVERY plan, with an unchanged value.**
  `services/motion/builtin/builtin.go:686-688` skips a component in a trajectory step only when
  `len(inputs) == 0`, never when the inputs are unchanged, and a DoF-bearing frame is in every
  step. So any policy that rejects planner jaw commands must first exclude no-op batches, or it
  fails every arm move (`TestGoToInputsNoOpBatchIssuesNoCommand`).
```

- [ ] **Step 3: Final verification**

Run: `go test -count=1 ./... && go test -race ./components/gripper/ && gofmt -s -l . && go vet ./...`
Expected: PASS, empty gofmt output.

- [ ] **Step 4: Commit**

```bash
git add components/gripper/gripper_test.go docs/gripper.md CLAUDE.md
git commit -m "docs: document articulated_jaw on the hardware gripper"
```

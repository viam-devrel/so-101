# Approach-Axis Orientation Planning via PoseCloud — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the blanket `position_only` goal metric in both SO-101 arm models' `MoveToPosition` with a configurable approach-axis cone, expressed as a `referenceframe.PoseCloud` attached to the goal.

**Architecture:** A new pure helper (`goal_cloud.go`) converts a degrees-based cone into a `PoseCloud` and applies a precedence table over the caller's `extra` map. Both arm models resolve their config into a shared `goalCloudConfig` at construction and delegate to that helper. The module only *sends* the cloud; viam-server plans, so honoring it requires viam-server >= 0.127.0.

**Tech Stack:** Go 1.25, `go.viam.com/rdk` (bumped v0.123.0 → v1.0.0), `testify` for assertions.

**Spec:** `docs/superpowers/specs/2026-07-15-pose-cloud-orientation-design.md` — read it first. The planner semantics are counterintuitive and were established empirically; do not "simplify" the mapping.

---

## Critical context (read before touching code)

Three facts drive every design choice. All were verified empirically against rdk v1.0.0:

1. **`PoseCloud.Theta` encodes the AZIMUTH of a tilt, not roll.** Measured exactly: `Theta = 90 - azimuth`, independent of the tilt's magnitude (about X → `90`; about Y → `0`; diagonal → `45`; azimuth 270 → `-180`). Since azimuth sweeps the full `[-180, 180]`, **only `Theta: 180` admits every azimuth** — only 180 gives an *isotropic* cone. Beware the tempting shorthand "smaller Theta rejects tilts": it is **false** (a `Theta: 0` cloud still accepts a 5° tilt about Y). Smaller values silently carve out a wedge of tilt *directions*. Consequence: roll cannot be constrained alongside tilt. This is approach-axis planning with **free roll**.
2. **`defaultEpsilon = 0.001` is ADDED to every leeway**, not just zeroed ones. Zero `X`/`Y`/`Z` therefore demands a 1-micron match, so positional leeway is **mandatory**. The effective cone is `acos(cos(tol) - 0.001)` — always slightly wider than requested.
3. **`OZ` alone defines the cone.** `OX`/`OY` are set to `1` (unconstrained). Valid across `[0, 180]`, saturating at `acos(-0.999) = 177.4374°`.

A cloud that never matches is **worse than no cloud**: the planner silently reverts to strict 6-DOF scoring and every move fails.

---

## File Structure

| File | Responsibility |
|---|---|
| `goal_cloud.go` (create) | The whole concern: defaults, cone→cloud math, `pose_cloud` parsing, precedence, error wrapping. Pure — no serial port, no motion service, no hardware. |
| `goal_cloud_test.go` (create) | Cone semantics via `PoseInCloud`, regression guards, precedence, parsing, proto round-trip. |
| `arm.go` (modify) | Config fields + `Validate` rules; `goalCloud` field; `MoveToPosition` delegates to the helper. |
| `simulated.go` (modify) | Same three changes as `arm.go`. |
| `arm_move_to_position_test.go` (create) | Both models send a destination carrying the expected `GoalCloud`. |
| `go.mod` / `go.sum` (modify) | RDK bump. |
| `README.md`, `CLAUDE.md` (modify) | Docs. |

---

## Task 1: Bump RDK to v1.0.0

Its own commit, ahead of the feature, so a bisect can separate "the upgrade broke it" from "the cone broke it". Verified to need **zero source changes**.

**Files:**
- Modify: `go.mod`, `go.sum`
- Modify: `CLAUDE.md` (header pins `v0.123.0`)

- [ ] **Step 1: Record the baseline**

Run: `go build ./cmd/module/ . && go test ./cmd/module/ . && go vet ./cmd/module/ .`
Expected: all pass. (`cmd/cli/` is excluded deliberately — it has multiple `func main()` and will not compile as a package. Never use `./...`.)

- [ ] **Step 2: Bump**

```bash
GOFLAGS=-mod=mod go get go.viam.com/rdk@v1.0.0
GOFLAGS=-mod=mod go mod tidy
```

Expected `go.mod` result: `rdk v0.123.0→v1.0.0`, `api v0.1.539→v0.1.566`, `utils v0.4.19→v0.6.6`, `go` directive `1.25.1→1.25.9`.

- [ ] **Step 3: Verify nothing broke**

Run: `go build ./cmd/module/ . && go test ./cmd/module/ . && go vet ./cmd/module/ .`
Expected: all pass, **no source edits required**. If anything fails, STOP and report — the spec's premise was that this bump is source-clean.

Note: `TestEnumerateSerialPorts` in `discovery_test.go` fails on machines with no serial hardware. That is environment-dependent and **not** a regression.

- [ ] **Step 4: Update the CLAUDE.md version pin**

In `CLAUDE.md`, change `go.viam.com/rdk v0.123.0` to `go.viam.com/rdk v1.0.0`.

- [ ] **Step 5: Commit**

```bash
git add go.mod go.sum CLAUDE.md
git commit -m "chore: bump rdk to v1.0.0

Needed for referenceframe.PoseCloud support: v0.123.0 has the type but no
PoseInCloud, and its ProtobufToPoseInFrame silently drops GoalCloud on decode.
Builds, tests, and vets clean with no source changes."
```

- [ ] **Step 6: Verify CI**

Push the branch and confirm `.github/workflows` passes. The `go` directive moved 1.25.1→1.25.9 and nothing pins a Go version, so confirm the build-action's toolchain resolves it rather than assuming.

---

## Task 2: `goalCloudConfig` + `resolveGoalCloudConfig`

Single owner of defaulting **and** the two construction warnings. `Validate` has no logger, which is why the warnings live here.

**Files:**
- Create: `goal_cloud.go`
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the failing test**

Import only what this task uses — Go fails the build on unused imports, and `gofmt` does
not strip them. Later tasks add more (`math`, `r3`, `referenceframe`, `spatialmath` in
Task 3; `require` in Task 5).

```go
package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestResolveGoalCloudConfigDefaults(t *testing.T) {
	// Zero and unset are indistinguishable for a float64 with omitempty; both default.
	got := resolveGoalCloudConfig(0, 0, nil)
	assert.Equal(t, defaultOrientationToleranceDeg, got.OrientationToleranceDeg)
	assert.Equal(t, defaultPositionToleranceMM, got.PositionToleranceMM)
}

func TestResolveGoalCloudConfigKeepsExplicitValues(t *testing.T) {
	got := resolveGoalCloudConfig(12.5, 0.25, nil)
	assert.Equal(t, 12.5, got.OrientationToleranceDeg)
	assert.Equal(t, 0.25, got.PositionToleranceMM)
}

// The warnings are the production path -- both tests above pass a nil logger and so
// exercise only the early return. resolveGoalCloudConfig is the SINGLE owner of these
// thresholds; nothing downstream re-checks them, so nothing downstream would catch a
// regression either. Uses logging.NewObservedTestLogger (rdk logging/logging.go:145).
func TestResolveGoalCloudConfigWarnsAtSaturationBoundary(t *testing.T) {
	// The boundary is acos(-0.999) = 177.4374 -- exactly the band a literal ">= 177.44"
	// check would miss, which is why poseCloudSaturationCos is a cosine.
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.438, 0, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("saturates").Len(), "just inside the boundary must warn")

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.40, 0, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("saturates").Len(), "just outside must not warn")
}

func TestResolveGoalCloudConfigWarnsOnLargePositionTolerance(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 25, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("is large").Len())

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 10, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("is large").Len(), "at the threshold must not warn")
}

func TestResolveGoalCloudConfigDefaultsDoNotWarn(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 0, logger)
	assert.Equal(t, 0, logs.Len(), "the defaults must be quiet")
}
```

The warning tests need `"go.viam.com/rdk/logging"` in the test import block as well.

- [ ] **Step 2: Run test to verify it fails**

Run: `go test . -run TestResolveGoalCloudConfig -v`
Expected: FAIL — `undefined: resolveGoalCloudConfig`

- [ ] **Step 3: Write the implementation**

Create `goal_cloud.go`:

```go
package so_arm

import (
	"math"

	"go.viam.com/rdk/logging"
)

const (
	// defaultOrientationToleranceDeg is the approach-axis cone half-angle used when
	// orientation_tolerance_deg is zero or unset.
	defaultOrientationToleranceDeg = 30.0

	// defaultPositionToleranceMM is the per-axis positional leeway used when
	// position_tolerance_mm is zero or unset. It must stay comfortably above the IK
	// solver's convergence: a cloud the solver can never land inside is worse than no
	// cloud at all, because planning reverts to strict 6-DOF scoring and always fails.
	defaultPositionToleranceMM = 1.0

	// warnPositionToleranceMM is the point above which a cloud is loose enough that the
	// planner may report success visibly far from the goal.
	warnPositionToleranceMM = 10.0

	// poseCloudSaturationCos is the cosine at which the cone accepts every orientation.
	// PoseInCloud adds a 0.001 epsilon to the OZ leeway and the maximum possible
	// deviation is 2.0, so saturation begins where 1-cos(a)+0.001 >= 2. Expressed as a
	// cosine rather than the rounded 177.44 degrees: the true threshold is 177.4374, and
	// a literal degree comparison would miss the [177.4374, 177.44) band.
	poseCloudSaturationCos = -0.999
)

// goalCloudConfig is the resolved tolerance pair, letting both arm models share one code
// path despite their separate config structs. Defaults are already applied.
type goalCloudConfig struct {
	OrientationToleranceDeg float64
	PositionToleranceMM     float64
}

// resolveGoalCloudConfig applies defaults to zero or unset values and emits the two
// construction warnings. It is the single owner of both, so each constructor is one call
// and neither arm model re-implements the thresholds. logger may be nil (unit tests).
func resolveGoalCloudConfig(tolDeg, posTolMM float64, logger logging.Logger) goalCloudConfig {
	cfg := goalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: posTolMM}
	if cfg.OrientationToleranceDeg == 0 {
		cfg.OrientationToleranceDeg = defaultOrientationToleranceDeg
	}
	if cfg.PositionToleranceMM == 0 {
		cfg.PositionToleranceMM = defaultPositionToleranceMM
	}

	if logger == nil {
		return cfg
	}
	if cfg.PositionToleranceMM > warnPositionToleranceMM {
		logger.Warnf("position_tolerance_mm=%g is large; the planner may report success up to %.2fmm "+
			"from the goal (the cloud is a per-axis box, so the worst-case corner is sqrt(3) times this)",
			cfg.PositionToleranceMM, math.Sqrt(3)*cfg.PositionToleranceMM)
	}
	// Cite 177.4374 (the true acos(-0.999) threshold), NOT the rounded 177.44 used in
	// user-facing docs: this guard fires across [177.4374, 177.44), so quoting 177.44 here
	// would contradict the action taken. %g likewise avoids printing 177.4374 as "177.4".
	if math.Cos(utils.DegToRad(cfg.OrientationToleranceDeg)) <= poseCloudSaturationCos {
		logger.Warnf("orientation_tolerance_deg=%g saturates the approach-axis cone (>= 177.4374); "+
			"every orientation will be accepted, which is equivalent to ignoring orientation",
			cfg.OrientationToleranceDeg)
	}
	return cfg
}
```

**Use `utils.DegToRad`, do not declare a local `degToRad`.** `go.viam.com/rdk/utils.DegToRad`
is identical and the package already uses it (`manager.go:130`, `speed.go`); a second
spelling of deg→rad in one package invites a third. The import block is therefore:

```go
import (
	"math"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/utils"
)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test . -run TestResolveGoalCloudConfig -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add goal_cloud.go goal_cloud_test.go
git commit -m "feat(arm): add goalCloudConfig with defaulting and construction warnings"
```

---

## Task 3: `coneToPoseCloud` + verified cone semantics

The heart of the change. Every assertion below is a **measured** value, not a derivation. If one fails, the mapping is wrong — do not adjust the expectation to match.

**Files:**
- Modify: `goal_cloud.go`
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the failing tests**

Add to `goal_cloud_test.go`. Extend the import block to exactly:

```go
import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)
```

(`logging` and `utils` arrive in Task 2 via the warning tests and `utils.DegToRad`; listed
here so the block is complete.)

```go
// testGoal is a tool pointing straight down, a typical SO-101 grasp pose.
func testGoal() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 300, Y: 0, Z: 200},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0},
	)
}

// tiltedBy returns testGoal() tilted by deg about the axis (ax, ay, 0).
func tiltedBy(ax, ay, deg float64) spatialmath.Pose {
	return spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
		&spatialmath.R4AA{RX: ax, RY: ay, RZ: 0, Theta: utils.DegToRad(deg)}))
}

func coneCloud(tolDeg float64) *referenceframe.PoseCloud {
	return coneToPoseCloud(goalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: 1.0})
}

func TestConeToPoseCloudFields(t *testing.T) {
	c := coneCloud(30)
	assert.Equal(t, 1.0, c.X)
	assert.Equal(t, 1.0, c.Y)
	assert.Equal(t, 1.0, c.Z)
	// OX/OY are deliberately unconstrained: OZ alone defines the cone.
	assert.Equal(t, 1.0, c.OX)
	assert.Equal(t, 1.0, c.OY)
	assert.InDelta(t, 1-math.Cos(utils.DegToRad(30)), c.OZ, 1e-12)
	// MUST be 180: Theta encodes tilt AZIMUTH, not roll. See the spec.
	assert.Equal(t, 180.0, c.Theta)
	// X, Y, Z, OX, OY, OZ, Theta is the COMPLETE field set in rdk v1.0.0. Do not look for
	// a ReferenceFrame field: v0.123.0 had one, v1.0.0 removed it, and the struct's
	// free-floating doc comment about reference frames misleadingly survives.
}

func TestConeBoundsTiltIsotropically(t *testing.T) {
	c := coneCloud(30)
	for _, azDeg := range []float64{0, 45, 90, 135, 180, 270} {
		ax, ay := math.Cos(utils.DegToRad(azDeg)), math.Sin(utils.DegToRad(azDeg))
		assert.True(t, c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 29)),
			"29deg tilt at azimuth %.0f should be accepted", azDeg)
		assert.False(t, c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 31)),
			"31deg tilt at azimuth %.0f should be rejected", azDeg)
	}
}

func TestConeBoundsAbove90Degrees(t *testing.T) {
	// The cone does NOT go degenerate above 90, despite OZ = 1-cos(a) exceeding 1.
	assert.True(t, coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 89)))
	assert.False(t, coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 91)))
	assert.True(t, coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 119)))
	assert.False(t, coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 121)))
}

func TestConeSaturationBoundary(t *testing.T) {
	// Saturation begins at acos(-0.999) = 177.4374, NOT 179.
	assert.False(t, coneCloud(177.40).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"177.40 must still reject a 180deg tilt")
	assert.True(t, coneCloud(177.44).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"177.44 must be saturated")
}

// TestSaturationConstantMatchesConeMapping ties Task 2's poseCloudSaturationCos to this
// task's cone mapping. The constant is only correct while coneToPoseCloud maps the cone to
// OZ = 1-cos(a); if that formula ever changes, resolveGoalCloudConfig would keep warning at
// the wrong angle with nothing to catch it. The tests above use literal degrees and would
// not notice. This one derives the angle FROM the constant, so the two cannot drift apart.
func TestSaturationConstantMatchesConeMapping(t *testing.T) {
	satDeg := math.Acos(poseCloudSaturationCos) * 180 / math.Pi // 177.4374...
	assert.True(t, coneCloud(satDeg+0.001).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"at the angle where resolveGoalCloudConfig warns, the cone must actually be saturated")
	assert.False(t, coneCloud(satDeg-0.01).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"just below that angle the cone must still bound tilt")
}

func TestConeRollIsFree(t *testing.T) {
	c := coneCloud(30)
	for _, roll := range []float64{0, 45, 90, 179} {
		p := spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
			&spatialmath.R4AA{RX: 0, RY: 0, RZ: 1, Theta: utils.DegToRad(roll)}))
		assert.True(t, c.PoseInCloud(testGoal(), p), "roll %.0f should be accepted", roll)
	}
}

func TestConeTiltPlusRollStaysBounded(t *testing.T) {
	c := coneCloud(30)
	for _, roll := range []float64{0, 70, 170} {
		for _, tc := range []struct {
			tilt float64
			want bool
		}{{20, true}, {29, true}, {31, false}} {
			p := spatialmath.Compose(testGoal(), spatialmath.Compose(
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RX: 1, Theta: utils.DegToRad(tc.tilt)}),
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RZ: 1, Theta: utils.DegToRad(roll)})))
			assert.Equal(t, tc.want, c.PoseInCloud(testGoal(), p),
				"tilt %.0f roll %.0f", tc.tilt, roll)
		}
	}
}

func TestConeEffectiveWideningFormula(t *testing.T) {
	// The epsilon is ADDED to the leeway, so the enforced cone is always slightly wider
	// than requested: acos(cos(tol) - 0.001). Measured by binary search on acceptance.
	for _, tc := range []struct{ requested, want float64 }{
		{0, 2.5626}, {1, 2.7508}, {2.6, 3.6509}, {10, 10.3247}, {30, 30.1144}, {90, 90.0573},
	} {
		c := coneCloud(tc.requested)
		lo, hi := 0.0, 180.0
		for i := 0; i < 60; i++ {
			mid := (lo + hi) / 2
			if c.PoseInCloud(testGoal(), tiltedBy(1, 0, mid)) {
				lo = mid
			} else {
				hi = mid
			}
		}
		assert.InDelta(t, tc.want, lo, 0.01, "effective cone for requested %.1f", tc.requested)
	}
}

func TestConePositionalBoxNotRadius(t *testing.T) {
	c := coneCloud(30)
	offset := func(dx, dy, dz float64) spatialmath.Pose {
		g := testGoal()
		return spatialmath.NewPose(
			r3.Vector{X: g.Point().X + dx, Y: g.Point().Y + dy, Z: g.Point().Z + dz},
			g.Orientation())
	}
	// The epsilon is additive: at X=1.0 the true bound is 1.001mm.
	assert.True(t, c.PoseInCloud(testGoal(), offset(1.0009, 0, 0)))
	assert.False(t, c.PoseInCloud(testGoal(), offset(1.0015, 0, 0)))
	// Per-axis box, not a radius: all three axes at the bound is accepted at norm ~1.73mm.
	assert.True(t, c.PoseInCloud(testGoal(), offset(1.0005, 1.0005, 1.0005)))
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test . -run TestCone -v`
Expected: FAIL — `undefined: coneToPoseCloud`

- [ ] **Step 3: Write the implementation**

Add to `goal_cloud.go` (add `"go.viam.com/rdk/referenceframe"` to imports):

```go
// coneToPoseCloud converts an approach-axis cone half-angle into a PoseCloud.
//
// Every field here is load-bearing and was established empirically; see
// docs/superpowers/specs/2026-07-15-pose-cloud-orientation-design.md before changing any
// of them:
//
//   - X/Y/Z must be > 0. PoseInCloud adds a 0.001 epsilon to each leeway, so a zero
//     leeway demands a 1-micron match and the cloud would never match.
//   - OX/OY are deliberately 1 (unconstrained). OZ alone defines the cone.
//   - Theta must be exactly 180. For a pure tilt, the relative Theta reports the tilt's
//     AZIMUTH (about X -> 90, about Y -> 0), not the roll -- and it does so independent of
//     the tilt's magnitude. Azimuth sweeps the full [-180, 180], so ONLY 180 admits every
//     azimuth; that is, only 180 yields an isotropic cone. Smaller values do not reject
//     tilts outright -- they silently carve out a wedge of tilt DIRECTIONS (Theta=90 still
//     accepts a tilt about X, but rejects azimuths past 180). TestConeBoundsTiltIsotropically
//     pins this: at azimuth 270 the reported Theta is exactly -180.
//     The cost is that roll cannot be constrained at all, which is why this is
//     approach-axis planning with free roll.
func coneToPoseCloud(cfg goalCloudConfig) *referenceframe.PoseCloud {
	return &referenceframe.PoseCloud{
		X:     cfg.PositionToleranceMM,
		Y:     cfg.PositionToleranceMM,
		Z:     cfg.PositionToleranceMM,
		OX:    1,
		OY:    1,
		OZ:    1 - math.Cos(utils.DegToRad(cfg.OrientationToleranceDeg)),
		Theta: 180,
	}
}
```

These seven are the complete field set in rdk v1.0.0. (v0.123.0 also had a
`ReferenceFrame`; v1.0.0 removed it, but left the struct's doc comment about reference
frames in place, which reads as though the field still exists. It does not.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test . -run TestCone -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add goal_cloud.go goal_cloud_test.go
git commit -m "feat(arm): add coneToPoseCloud approach-axis cone mapping"
```

---

## Task 4: Regression guards for the two counterintuitive findings

These encode *why* the mapping looks as it does. Without them, a future "simplification" of `Theta: 180` → `0` silently breaks every move while the other tests still pass.

**Files:**
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the guards**

```go
// TestRegressionThetaMustBe180 documents why coneToPoseCloud sets Theta: 180.
// PoseCloud's Theta encodes the AZIMUTH of a tilt, not roll, so any smaller leeway
// rejects tilts before the OX/OY/OZ cone is consulted.
func TestRegressionThetaMustBe180(t *testing.T) {
	broken := coneCloud(30)
	broken.Theta = 0 // the "obvious simplification"
	assert.False(t, broken.PoseInCloud(testGoal(), tiltedBy(1, 0, 5)),
		"with Theta=0 even a 5deg tilt is rejected, defeating the cone entirely")

	// Proof of the cause: a pure tilt about X reports Theta=90, about Y reports Theta=0.
	ovX := spatialmath.PoseBetween(testGoal(), tiltedBy(1, 0, 5)).Orientation().OrientationVectorDegrees()
	assert.InDelta(t, 90.0, ovX.Theta, 1e-6, "tilt about X reports Theta=90, not roll")
	ovY := spatialmath.PoseBetween(testGoal(), tiltedBy(0, 1, 5)).Orientation().OrientationVectorDegrees()
	assert.InDelta(t, 0.0, ovY.Theta, 1e-6, "tilt about Y reports Theta=0")
}

// TestRegressionPositionalLeewayIsMandatory documents why X/Y/Z must be > 0: the 0.001
// epsilon means a zero leeway demands a 1-micron match, so the cloud never matches and
// planning silently reverts to strict 6-DOF scoring.
func TestRegressionPositionalLeewayIsMandatory(t *testing.T) {
	broken := coneCloud(30)
	broken.X, broken.Y, broken.Z = 0, 0, 0
	g := testGoal()
	nudged := spatialmath.NewPose(
		r3.Vector{X: g.Point().X + 0.01, Y: g.Point().Y, Z: g.Point().Z}, g.Orientation())
	assert.False(t, broken.PoseInCloud(g, nudged),
		"with zero positional leeway even a 0.01mm offset is rejected")
}
```

- [ ] **Step 2: Run and verify they pass**

Run: `go test . -run TestRegression -v`
Expected: PASS (these assert current behavior; they should pass immediately)

- [ ] **Step 3: Commit**

```bash
git add goal_cloud_test.go
git commit -m "test(arm): guard the Theta=180 and positional-leeway invariants"
```

---

## Task 5: `pose_cloud` parsing

The expert-mode escape hatch. Keys are **lowercase**, matching `referenceframe.PoseCloud`'s JSON tags. Unknown keys are rejected because the Viam docs render them `OX`/`OY` and protobuf JSON spells them `o_x` — a silently-ignored typo would produce a 1-micron cloud that fails every move. `reference_frame` is rejected under the same rule: it is not a `PoseCloud` field in v1.0.0.

Note this task's tests are the first to use `require`; add `"github.com/stretchr/testify/require"` to the import block.

**Files:**
- Modify: `goal_cloud.go`
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestParsePoseCloudValid(t *testing.T) {
	// extra arrives via structpb, so every JSON number is a float64.
	got, err := parsePoseCloud(map[string]interface{}{
		"x": 2.0, "y": 2.0, "z": 0.5, "ox": 1.0, "oy": 1.0, "oz": 0.1, "theta": 180.0,
	})
	require.NoError(t, err)
	assert.Equal(t, 2.0, got.X)
	assert.Equal(t, 0.5, got.Z)
	assert.Equal(t, 0.1, got.OZ)
	assert.Equal(t, 180.0, got.Theta)
}

func TestParsePoseCloudOmittedFieldsStayZero(t *testing.T) {
	got, err := parsePoseCloud(map[string]interface{}{"oz": 0.1})
	require.NoError(t, err)
	assert.Equal(t, 0.0, got.X, "omitted fields stay zero -- sharp, but faithful passthrough")
}

func TestParsePoseCloudRejects(t *testing.T) {
	for name, input := range map[string]interface{}{
		"not an object":       "nope",
		"docs-style casing":   map[string]interface{}{"OX": 1.0},
		"protobuf casing":     map[string]interface{}{"o_x": 1.0},
		"not a field in v1.0": map[string]interface{}{"reference_frame": 1.0},
		"unknown key":         map[string]interface{}{"wobble": 1.0},
		"non-numeric":         map[string]interface{}{"oz": "0.1"},
		"negative":            map[string]interface{}{"oz": -0.1},
		"NaN":                 map[string]interface{}{"oz": math.NaN()},
		"Inf":                 map[string]interface{}{"oz": math.Inf(1)},
	} {
		t.Run(name, func(t *testing.T) {
			_, err := parsePoseCloud(input)
			assert.Error(t, err)
		})
	}
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test . -run TestParsePoseCloud -v`
Expected: FAIL — `undefined: parsePoseCloud`

- [ ] **Step 3: Write the implementation**

Add to `goal_cloud.go` (add `"fmt"` and `"sort"` to imports):

```go
// poseCloudSetters maps the accepted pose_cloud keys to their fields. The names match
// referenceframe.PoseCloud's own JSON tags exactly, and are all lowercase. Note the Viam
// docs render these as OX/OY and protobuf JSON spells them o_x/oX; neither is accepted,
// which is why unknown keys are an error rather than ignored. This is also the complete
// field set: "reference_frame" is not a PoseCloud field in rdk v1.0.0.
var poseCloudSetters = map[string]func(*referenceframe.PoseCloud, float64){
	"x":     func(p *referenceframe.PoseCloud, v float64) { p.X = v },
	"y":     func(p *referenceframe.PoseCloud, v float64) { p.Y = v },
	"z":     func(p *referenceframe.PoseCloud, v float64) { p.Z = v },
	"ox":    func(p *referenceframe.PoseCloud, v float64) { p.OX = v },
	"oy":    func(p *referenceframe.PoseCloud, v float64) { p.OY = v },
	"oz":    func(p *referenceframe.PoseCloud, v float64) { p.OZ = v },
	"theta": func(p *referenceframe.PoseCloud, v float64) { p.Theta = v },
}

func poseCloudKeyList() string {
	keys := make([]string, 0, len(poseCloudSetters))
	for k := range poseCloudSetters {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return strings.Join(keys, ", ")
}

// parsePoseCloud converts a caller-supplied extra["pose_cloud"] into a PoseCloud.
// Omitted fields stay zero, which -- given the additive 0.001 epsilon -- means
// near-zero leeway. That is faithful passthrough and the point of an escape hatch, but it
// is sharp: a cloud whose leeways are all zero will never match.
func parsePoseCloud(raw interface{}) (*referenceframe.PoseCloud, error) {
	m, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("pose_cloud must be an object, got %T", raw)
	}
	pc := &referenceframe.PoseCloud{}
	for k, rv := range m {
		set, known := poseCloudSetters[k]
		if !known {
			return nil, fmt.Errorf("pose_cloud: unknown key %q; valid keys are %s (all lowercase; "+
				"the Viam docs' OX/OY casing and protobuf's o_x are not accepted)", k, poseCloudKeyList())
		}
		v, ok := rv.(float64)
		if !ok {
			return nil, fmt.Errorf("pose_cloud: %q must be a number, got %T", k, rv)
		}
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf("pose_cloud: %q must be finite, got %v", k, v)
		}
		if v < 0 {
			return nil, fmt.Errorf("pose_cloud: %q must not be negative, got %v", k, v)
		}
		set(pc, v)
	}
	return pc, nil
}
```

Also add `"strings"` to the import block.

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test . -run TestParsePoseCloud -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add goal_cloud.go goal_cloud_test.go
git commit -m "feat(arm): parse the raw pose_cloud escape hatch from extra"
```

---

## Task 6: `buildMoveDestination` + precedence + `goalPath`

**Files:**
- Modify: `goal_cloud.go`
- Test: `goal_cloud_test.go`

The precedence table:

| `extra` contains | Destination | Extras forwarded | `goalPath` |
|---|---|---|---|
| neither key | cone from config | all caller keys verbatim | `pathCone` |
| `pose_cloud` only | raw cloud | all keys **except** `pose_cloud` | `pathRawCloud` |
| `goal_metric_type` only | **no cloud** | all keys verbatim | `pathMetricType` |
| **both** | — | — | **error** |

- [ ] **Step 1: Write the failing tests**

```go
func TestBuildMoveDestinationConePath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	require.NoError(t, err)
	assert.Equal(t, pathCone, path)
	assert.Equal(t, "myarm_origin", dest.Parent(), "the frame name is passed through unmodified")
	require.NotNil(t, dest.GoalCloud)
	assert.Equal(t, 180.0, dest.GoalCloud.Theta)
	assert.NotContains(t, planExtra, "goal_metric_type", "the cone replaces position_only")
}

func TestBuildMoveDestinationForwardsCallerExtras(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	_, planExtra, _, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"timeout": 5.0})
	require.NoError(t, err)
	assert.Equal(t, 5.0, planExtra["timeout"], "unrelated caller keys still pass through")
}

func TestBuildMoveDestinationRawCloudPath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}, "timeout": 5.0})
	require.NoError(t, err)
	assert.Equal(t, pathRawCloud, path)
	require.NotNil(t, dest.GoalCloud)
	assert.Equal(t, 0.5, dest.GoalCloud.OZ, "the raw cloud replaces the cone entirely")
	assert.NotContains(t, planExtra, "pose_cloud", "pose_cloud is not a planner key")
	assert.Equal(t, 5.0, planExtra["timeout"])
}

func TestBuildMoveDestinationMetricTypePath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"goal_metric_type": "position_only"})
	require.NoError(t, err)
	assert.Equal(t, pathMetricType, path)
	assert.Nil(t, dest.GoalCloud, "no cloud: position_only sets orientScale=0, making a cloud meaningless")
	assert.Equal(t, "position_only", planExtra["goal_metric_type"], "the caller's metric is forwarded")
}

func TestBuildMoveDestinationRejectsBothKeys(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	_, _, _, err := buildMoveDestination("a_origin", testGoal(), cfg, map[string]interface{}{
		"pose_cloud":       map[string]interface{}{"oz": 0.5},
		"goal_metric_type": "position_only",
	})
	require.Error(t, err, "incoherent: a raw cloud plus a metric that ignores clouds")
}

func TestBuildMoveDestinationDoesNotMutateCallerExtra(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	extra := map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}}
	_, _, _, err := buildMoveDestination("a_origin", testGoal(), cfg, extra)
	require.NoError(t, err)
	assert.Contains(t, extra, "pose_cloud", "the caller's map must not be mutated")
}

func TestBuildMoveDestinationPropagatesParseError(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, _, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"OX": 1.0}})
	require.Error(t, err)
	assert.Nil(t, dest)
	assert.Nil(t, planExtra)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test . -run TestBuildMoveDestination -v`
Expected: FAIL — `undefined: buildMoveDestination`

- [ ] **Step 3: Write the implementation**

Add to `goal_cloud.go` (add `"go.viam.com/rdk/spatialmath"` to imports):

```go
const (
	extraKeyPoseCloud      = "pose_cloud"
	extraKeyGoalMetricType = "goal_metric_type"
)

// goalPath reports which precedence row buildMoveDestination took, so callers can wrap
// failures appropriately without re-inspecting extra (which would duplicate the
// precedence logic across both arm models and let it drift).
type goalPath int

const (
	pathCone       goalPath = iota // cone from config
	pathRawCloud                   // caller-supplied pose_cloud
	pathMetricType                 // caller-supplied goal_metric_type; no cloud sent
)

// buildMoveDestination applies the precedence table (see the plan/spec).
//
// originFrame is the FULLY-FORMED frame name (e.g. "myarm_origin"); this function does
// not append "_origin". It is passed to NewPoseInFrame unmodified.
//
// On success it returns a destination, a NEW extras map (the caller's map is never
// mutated), and the path taken. On error it returns (nil, nil, 0, err).
func buildMoveDestination(originFrame string, pose spatialmath.Pose, cfg goalCloudConfig,
	extra map[string]interface{},
) (*referenceframe.PoseInFrame, map[string]interface{}, goalPath, error) {
	rawCloud, hasCloud := extra[extraKeyPoseCloud]
	_, hasMetric := extra[extraKeyGoalMetricType]

	if hasCloud && hasMetric {
		return nil, nil, 0, fmt.Errorf(
			"extra cannot contain both %q and %q: %q makes the planner ignore goal clouds, "+
				"so the combination is incoherent", extraKeyPoseCloud, extraKeyGoalMetricType,
			extraKeyGoalMetricType)
	}

	planExtra := make(map[string]interface{}, len(extra))
	for k, v := range extra {
		if k == extraKeyPoseCloud {
			continue // consumed here; not a planner key
		}
		planExtra[k] = v
	}

	switch {
	case hasMetric:
		// The caller's metric wins and no cloud is sent: position_only sets orientScale=0,
		// which would make any cloud meaningless.
		return referenceframe.NewPoseInFrame(originFrame, pose), planExtra, pathMetricType, nil
	case hasCloud:
		pc, err := parsePoseCloud(rawCloud)
		if err != nil {
			return nil, nil, 0, err
		}
		return referenceframe.NewPoseInFrameWithGoalCloud(originFrame, pose, pc), planExtra, pathRawCloud, nil
	default:
		return referenceframe.NewPoseInFrameWithGoalCloud(originFrame, pose, coneToPoseCloud(cfg)),
			planExtra, pathCone, nil
	}
}

// wrapMoveErr adds actionable guidance to a planning failure. The cone-specific wording
// applies only on the cone path: on the other paths it would misdirect, telling a caller
// who just passed position_only to pass position_only, or blaming a config tolerance that
// had no bearing on a raw-cloud failure.
//
// The cone message names BOTH tolerances. Either can cause the failure: too tight a cone
// leaves no reachable orientation, and too small a position tolerance means the solver can
// never land inside the cloud (which reverts planning to strict 6-DOF scoring and fails
// every move). Naming only the orientation knob would point at the wrong one half the time.
func wrapMoveErr(err error, path goalPath, cfg goalCloudConfig) error {
	if err == nil {
		return nil
	}
	switch path {
	case pathCone:
		return fmt.Errorf("move to position failed with orientation_tolerance_deg=%g, "+
			"position_tolerance_mm=%g (approach-axis cone): %w; widen either tolerance "+
			"(too small a position_tolerance_mm means the solver can never land inside the "+
			"goal cloud), pass extra {\"goal_metric_type\": \"position_only\"} to ignore "+
			"orientation, or check that viam-server is >= 0.127.0 (older servers silently "+
			"ignore goal clouds)",
			cfg.OrientationToleranceDeg, cfg.PositionToleranceMM, err)
	case pathRawCloud:
		return fmt.Errorf("move to position failed with the caller-supplied pose_cloud: %w; "+
			"widen the cloud, or check that viam-server is >= 0.127.0 (older servers silently "+
			"ignore goal clouds)", err)
	default:
		return err
	}
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `go test . -run TestBuildMoveDestination -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add goal_cloud.go goal_cloud_test.go
git commit -m "feat(arm): add buildMoveDestination precedence and error wrapping"
```

---

## Task 7: Proto round-trip guard

CLAUDE.md's module-boundary gotcha: a component ships over gRPC, and in-memory state does not survive. An in-process assertion would miss a serialization break entirely. This is the test that proves the cloud actually reaches viam-server.

**Files:**
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the test**

```go
// TestGoalCloudSurvivesSerialization guards the module-boundary gotcha: the cloud is only
// useful if it crosses gRPC. An in-process check would pass even if it never shipped.
func TestGoalCloudSurvivesSerialization(t *testing.T) {
	cfg := resolveGoalCloudConfig(30, 1.0, nil)
	dest, _, _, err := buildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	require.NoError(t, err)

	proto := referenceframe.PoseInFrameToProtobuf(dest)
	require.NotNil(t, proto.GoalCloud, "the cloud must be serialized onto the wire")

	got := referenceframe.ProtobufToPoseInFrame(proto)
	require.NotNil(t, got.GoalCloud, "the cloud must survive the round trip")

	want := dest.GoalCloud
	assert.InDelta(t, want.X, got.GoalCloud.X, 1e-9)
	assert.InDelta(t, want.Y, got.GoalCloud.Y, 1e-9)
	assert.InDelta(t, want.Z, got.GoalCloud.Z, 1e-9)
	assert.InDelta(t, want.OX, got.GoalCloud.OX, 1e-9)
	assert.InDelta(t, want.OY, got.GoalCloud.OY, 1e-9)
	assert.InDelta(t, want.OZ, got.GoalCloud.OZ, 1e-9)
	assert.InDelta(t, want.Theta, got.GoalCloud.Theta, 1e-9)
}
```

- [ ] **Step 2: Run and verify**

Run: `go test . -run TestGoalCloudSurvives -v`
Expected: PASS. If the round-trip fails, the RDK bump did not land — recheck Task 1.

- [ ] **Step 3: Commit**

```bash
git add goal_cloud_test.go
git commit -m "test(arm): guard that the goal cloud survives gRPC serialization"
```

---

## Task 8: Config fields and `Validate` on both models

**Files:**
- Modify: `arm.go:70-100` (`SO101ArmConfig`), `arm.go:103+` (`Validate`)
- Modify: `simulated.go:42-71` (`SO101SimulatedArmConfig`), `simulated.go:75-91` (`Validate`)
- Test: `goal_cloud_test.go`

- [ ] **Step 1: Write the failing tests**

```go
func TestArmConfigValidateToleranceRanges(t *testing.T) {
	base := func() *SO101ArmConfig { return &SO101ArmConfig{Port: "/dev/null"} }

	for name, mutate := range map[string]func(*SO101ArmConfig){
		"negative orientation": func(c *SO101ArmConfig) { c.OrientationToleranceDeg = -1 },
		"orientation over 180": func(c *SO101ArmConfig) { c.OrientationToleranceDeg = 181 },
		"NaN orientation":      func(c *SO101ArmConfig) { c.OrientationToleranceDeg = math.NaN() },
		"negative position":    func(c *SO101ArmConfig) { c.PositionToleranceMM = -0.1 },
		"NaN position":         func(c *SO101ArmConfig) { c.PositionToleranceMM = math.NaN() },
	} {
		t.Run(name, func(t *testing.T) {
			c := base()
			mutate(c)
			_, _, err := c.Validate("")
			assert.Error(t, err)
		})
	}

	t.Run("valid bounds accepted", func(t *testing.T) {
		c := base()
		c.OrientationToleranceDeg, c.PositionToleranceMM = 180, 75
		_, _, err := c.Validate("")
		assert.NoError(t, err)
	})
}

func TestSimulatedConfigValidateToleranceRanges(t *testing.T) {
	c := &SO101SimulatedArmConfig{OrientationToleranceDeg: 181}
	_, _, err := c.Validate("")
	assert.Error(t, err)

	c = &SO101SimulatedArmConfig{PositionToleranceMM: -1}
	_, _, err = c.Validate("")
	assert.Error(t, err)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `go test . -run "ConfigValidateToleranceRanges" -v`
Expected: FAIL — unknown fields `OrientationToleranceDeg` / `PositionToleranceMM`

- [ ] **Step 3: Add the config fields**

Add to **both** `SO101ArmConfig` (arm.go) and `SO101SimulatedArmConfig` (simulated.go):

```go
	// OrientationToleranceDeg is the half-angle, in degrees, of the cone of acceptable
	// end-effector approach directions around the goal orientation. Roll about that axis
	// is NOT constrained. Valid range [0, 180]. Zero or unset means
	// defaultOrientationToleranceDeg (30).
	//
	// The planner adds a 0.001 epsilon to the OZ leeway, so the cone actually enforced is
	// acos(cos(tol) - 0.001): always slightly wider than requested (30 gives ~30.11) and
	// never narrower than ~2.56. At or above 177.44 every orientation is accepted.
	//
	// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds.
	OrientationToleranceDeg float64 `json:"orientation_tolerance_deg,omitempty"`

	// PositionToleranceMM is the per-axis positional leeway of the goal cloud, applied as
	// a box along the goal frame's axes (worst-case corner deviation is sqrt(3) times this
	// value). It must be large enough for the IK solver to land inside, or the cloud never
	// matches and every move fails. Zero or unset means defaultPositionToleranceMM (1.0).
	PositionToleranceMM float64 `json:"position_tolerance_mm,omitempty"`
```

- [ ] **Step 4: Add the shared validation helper**

Add to `goal_cloud.go`:

```go
// validateGoalCloudTolerances is shared by both arm models' Validate methods. Note NaN
// must be rejected explicitly: NaN comparisons are always false, so a NaN would slip
// through the range checks.
func validateGoalCloudTolerances(tolDeg, posTolMM float64) error {
	if math.IsNaN(tolDeg) || tolDeg < 0 || tolDeg > 180 {
		return fmt.Errorf("orientation_tolerance_deg must be in [0, 180], got %v", tolDeg)
	}
	if math.IsNaN(posTolMM) || posTolMM < 0 {
		return fmt.Errorf("position_tolerance_mm must not be negative, got %v", posTolMM)
	}
	return nil
}
```

Call it from **both** `Validate` methods, before the `deps` construction and after the existing checks:

```go
	if err := validateGoalCloudTolerances(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM); err != nil {
		return nil, nil, err
	}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `go test . -run "ConfigValidateToleranceRanges" -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add goal_cloud.go arm.go simulated.go goal_cloud_test.go
git commit -m "feat(arm): add orientation_tolerance_deg and position_tolerance_mm config"
```

---

## Task 9: Wire the hardware arm's `MoveToPosition`

**Files:**
- Modify: `arm.go:138-160` (struct), `arm.go:353+` (`NewSO101`), `arm.go:482-504` (`MoveToPosition`)
- Test: `arm_move_to_position_test.go` (create)

- [ ] **Step 1: Write the failing test**

Create `arm_move_to_position_test.go`. Note the package at `go.viam.com/rdk/testutils/inject/motion` is named `inject`, so the alias is required. Tests live in package `so_arm`, so the struct can be built directly — no serial port needed.

```go
package so_arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/motion"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

// captureMotion returns an injected motion service that records the last MoveReq.
func captureMotion(got *motion.MoveReq) *injectmotion.MotionService {
	ms := injectmotion.NewMotionService("builtin")
	ms.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
		*got = req
		return true, nil
	}
	return ms
}

func TestHardwareArmMoveToPositionSendsGoalCloud(t *testing.T) {
	var got motion.MoveReq
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, a.MoveToPosition(context.Background(), testGoal(), nil))

	assert.Equal(t, "myarm", got.ComponentName)
	assert.Equal(t, "myarm_origin", got.Destination.Parent())
	require.NotNil(t, got.Destination.GoalCloud, "the cone must ship to the planner")
	assert.Equal(t, 180.0, got.Destination.GoalCloud.Theta)
	assert.NotContains(t, got.Extra, "goal_metric_type", "the cone replaces position_only")
}

func TestHardwareArmMoveToPositionHonorsMetricTypeOverride(t *testing.T) {
	var got motion.MoveReq
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, a.MoveToPosition(context.Background(), testGoal(),
		map[string]interface{}{"goal_metric_type": "position_only"}))

	assert.Nil(t, got.Destination.GoalCloud, "no cloud when the caller picks the metric")
	assert.Equal(t, "position_only", got.Extra["goal_metric_type"])
}
```

`testGoal()` comes from `goal_cloud_test.go` — same package `so_arm`, so it resolves without import.

- [ ] **Step 2: Run test to verify it fails**

Run: `go test . -run TestHardwareArmMoveToPosition -v`
Expected: FAIL — unknown field `goalCloud`

- [ ] **Step 3: Add the struct field**

In `arm.go`, add to the `so101` struct **above the `mu sync.RWMutex` line** (i.e. near
`cfg`/`opMgr`, not inside the mutex-guarded block) — the field is immutable after
construction, and placing it under `mu` would imply otherwise:

```go
	// goalCloud is the resolved approach-axis tolerance pair. Set once in NewSO101 and
	// never mutated (the model is resource.AlwaysRebuild), so it needs no mutex.
	goalCloud goalCloudConfig
```

- [ ] **Step 4: Resolve it in the constructor**

In `NewSO101`, add to the `&so101{...}` literal:

```go
		goalCloud:    resolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
```

- [ ] **Step 5: Rewrite `MoveToPosition`**

Replace `arm.go:482-504` entirely:

```go
// MoveToPosition moves the arm's end-effector to the target pose.
//
// The SO-101 is a 5-DOF arm, so most six-DOF pose targets are unreachable. Rather than
// discard orientation wholesale, the planner is given an approach-axis cone (a
// referenceframe.PoseCloud) around the goal orientation: the tool's pointing direction is
// constrained to within orientation_tolerance_deg, while roll about that axis is free.
//
// Callers may bypass the cone via extra: "goal_metric_type" restores the old
// orientation-agnostic behavior, and "pose_cloud" supplies a raw cloud. See goal_cloud.go.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds, which makes
// planning revert to strict six-DOF scoring and fail.
func (s *so101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	dest, planExtra, path, err := buildMoveDestination(
		fmt.Sprintf("%v_origin", s.Name().Name), pose, s.goalCloud, extra)
	if err != nil {
		return err
	}
	s.logger.Debugf("MoveToPosition goal cloud: %+v", dest.GoalCloud)

	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().Name,
		Destination:   dest,
		Extra:         planExtra,
	})
	return wrapMoveErr(err, path, s.goalCloud)
}
```

Remove the now-unused `referenceframe` import from `arm.go` only if nothing else uses it — it almost certainly still does.

- [ ] **Step 6: Run tests to verify they pass**

Run: `go test . -run TestHardwareArmMoveToPosition -v`
Expected: PASS

- [ ] **Step 7: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add arm.go arm_move_to_position_test.go
git commit -m "feat(arm): plan MoveToPosition with an approach-axis cone"
```

---

## Task 10: Wire the simulated arm's `MoveToPosition`

Identical shape to Task 9. The simulated model must stay a faithful stand-in for motion-plan testing.

**Files:**
- Modify: `simulated.go:114-140` (struct), `simulated.go:145+` (`newSimulatedSO101`), `simulated.go:296-318` (`MoveToPosition`)
- Test: `arm_move_to_position_test.go`

- [ ] **Step 1: Write the failing test**

```go
func TestSimulatedArmMoveToPositionSendsGoalCloud(t *testing.T) {
	var got motion.MoveReq
	s := &simulatedSO101{
		name:      arm.Named("simarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, s.MoveToPosition(context.Background(), testGoal(), nil))

	assert.Equal(t, "simarm_origin", got.Destination.Parent())
	require.NotNil(t, got.Destination.GoalCloud)
	assert.Equal(t, 180.0, got.Destination.GoalCloud.Theta)
}

func TestSimulatedArmMoveToPositionRequiresMotionService(t *testing.T) {
	s := &simulatedSO101{name: arm.Named("simarm"), logger: logging.NewTestLogger(t)}
	err := s.MoveToPosition(context.Background(), testGoal(), nil)
	require.Error(t, err, "the nil-motion guard must still fire")
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test . -run TestSimulatedArmMoveToPosition -v`
Expected: FAIL — unknown field `goalCloud`

- [ ] **Step 3: Add the struct field**

In `simulated.go`, add to `simulatedSO101` **above the `// mu guards the fields below.`
comment** (near `motion`/`speed`), for the same reason as Task 9:

```go
	// goalCloud is the resolved approach-axis tolerance pair. Set once in
	// newSimulatedSO101 and never mutated (resource.AlwaysRebuild), so it needs no mutex.
	goalCloud goalCloudConfig
```

- [ ] **Step 4: Resolve it in the constructor**

In `newSimulatedSO101`, add to the `&simulatedSO101{...}` literal:

```go
		goalCloud:        resolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
```

- [ ] **Step 5: Rewrite `MoveToPosition`**

Replace `simulated.go:296-318`, keeping the nil-motion guard:

```go
// MoveToPosition moves the arm's end effector to the target pose using the motion service.
//
// As with the devrel:so101:arm model, the planner is given an approach-axis cone around
// the goal orientation rather than discarding orientation via "position_only": the tool's
// pointing direction is constrained to within orientation_tolerance_deg, while roll about
// that axis is free. Callers may bypass the cone via extra; see goal_cloud.go.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds.
func (s *simulatedSO101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if s.motion == nil {
		return errors.New("MoveToPosition requires a motion service, which was not available at construction")
	}

	dest, planExtra, path, err := buildMoveDestination(
		fmt.Sprintf("%v_origin", s.name.Name), pose, s.goalCloud, extra)
	if err != nil {
		return err
	}
	s.logger.Debugf("MoveToPosition goal cloud: %+v", dest.GoalCloud)

	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.name.Name,
		Destination:   dest,
		Extra:         planExtra,
	})
	return wrapMoveErr(err, path, s.goalCloud)
}
```

- [ ] **Step 6: Run the full suite**

Run: `go test ./cmd/module/ .`
Expected: PASS (except the environment-dependent `TestEnumerateSerialPorts`)

- [ ] **Step 7: Commit**

```bash
gofmt -s -w . && go vet ./cmd/module/ .
git add simulated.go arm_move_to_position_test.go
git commit -m "feat(simulated): plan MoveToPosition with an approach-axis cone"
```

---

## Task 11: README

**Files:**
- Modify: `README.md` — the `## Model devrel:so101:arm` and `## Model devrel:so101:simulated` sections

- [ ] **Step 1: Document both attributes in both model sections**

Follow the existing attribute-table style. Cover, for each model:

- `orientation_tolerance_deg` — approach-axis cone half-angle. **Roll is not constrained.** Range `[0, 180]`, default `30`. `0` means the default, **not** exact. The enforced cone is `acos(cos(tol) - 0.001)`, so always slightly wider than requested (30 → ~30.11) and never narrower than ~2.56. At `>= 177.44` every orientation is accepted.
- `position_tolerance_mm` — per-axis box leeway along the goal frame's axes, default `1.0`. Worst-case corner deviation is `sqrt(3)` times the value (~1.73mm at the default). Too small and the cloud never matches, so every move fails.

- [ ] **Step 2: Add a short subsection on the escape hatch and server requirement**

State plainly:
- Goal clouds require **viam-server >= 0.127.0**. Older servers accept and silently ignore them, which makes planning revert to strict six-DOF scoring and fail.
- `extra: {"goal_metric_type": "position_only"}` restores the previous orientation-agnostic behavior per-call.
- `extra: {"pose_cloud": {...}}` supplies a raw cloud. Keys are **lowercase**: `x`, `y`, `z`, `ox`, `oy`, `oz`, `theta`. The Viam docs' `OX` casing and protobuf's `o_x` are **rejected**. Omitted fields mean near-zero leeway — expert mode.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: document orientation_tolerance_deg and position_tolerance_mm"
```

---

## Task 12: CLAUDE.md gotcha

This is the hard-won knowledge that makes the code look wrong-but-correct. Without it, the next reader "simplifies" `Theta: 180` → `0` and breaks every move while most tests still pass.

**Files:**
- Modify: `CLAUDE.md` — the `## Gotchas` section

- [ ] **Step 1: Add the gotcha**

```markdown
- **`referenceframe.PoseCloud` semantics are counterintuitive and were established
  empirically** (see `docs/superpowers/specs/2026-07-15-pose-cloud-orientation-design.md`;
  `goal_cloud.go` depends on all of it):
  - `PoseInCloud` compares the *relative* pose `PoseBetween(goal, candidate)`, where zero
    deviation is the orientation vector `(0,0,1)`.
  - **`Theta` encodes the AZIMUTH of a tilt, not roll** (tilt about X → `Theta=90`, about
    Y → `Theta=0`). So `coneToPoseCloud` must set `Theta: 180`; anything less rejects tilts
    by azimuth before the cone is consulted. The cost is that roll cannot be constrained
    alongside tilt — hence "approach-axis planning with free roll".
  - **`OZ` alone defines the cone**; `OX`/`OY` are set to `1` (unconstrained). Valid across
    `[0, 180]`, saturating at `acos(-0.999) = 177.4374°` — check saturation with
    `cos(tol) <= -0.999`, not a literal degree comparison.
  - **`defaultEpsilon = 0.001` is ADDED to every leeway**, not just zeroed ones. Zero
    `X`/`Y`/`Z` therefore demands a 1-micron match, so positional leeway is mandatory; and
    the effective cone is `acos(cos(tol) - 0.001)`, always slightly wider than requested
    (floor ~2.56°).
  - The cloud only *zeroes the score* when a candidate is inside it. A cloud that never
    matches is **worse than no cloud**: planning reverts to strict six-DOF scoring and
    fails. `position_only` sets `orientScale=0`, which makes any cloud meaningless — the
    two are mutually exclusive.
  - **There is no `ReferenceFrame` field.** v0.123.0 had one; v1.0.0 removed it while
    leaving the struct's doc comment about reference frames in place as free-floating
    prose, so it reads as though the field still exists. The complete field set is
    `X, Y, Z, OX, OY, OZ, Theta`.
- **Goal clouds require viam-server >= 0.127.0.** RDK v0.125.0–v0.126.1 read `GoalCloud` in
  `armplanning` but never decode it in `ProtobufToPoseInFrame`, so the cloud is silently
  inert there; v0.127.0 is the first release with both halves. The module only *sends* the
  cloud, so its own RDK version has no bearing on whether the server honors it. The minimum
  is declared Registry-side — `meta.json` has no field for it.
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record PoseCloud semantics and the viam-server floor in CLAUDE.md"
```

---

## Task 13: Release checklist — declare the minimum viam-server version

**This is a manual post-publish step.** It cannot be carried by `meta.json`, `.github/workflows/deploy.yml`, or `viamrobotics/build-action`. If it is skipped, users on older servers hit exactly the silent-ignore failure this design exists to make loud.

- [ ] **Step 1: Verify the viam-server release exists**

Confirm a viam-server **0.127.0** release exists before publishing the constraint. viam-server releases track RDK version numbers, but confirm rather than assume.

- [ ] **Step 2: Publish, then set the constraint**

After the GitHub release runs `.github/workflows/deploy.yml` and the version uploads to the registry, set the minimum viam-server version to **0.127.0** on the module's Registry page.

- [ ] **Step 3: Verify end-to-end on real hardware**

The unit tests prove the cloud is *built and shipped* correctly; they cannot prove the arm plans well. On a real SO-101 against viam-server >= 0.127.0:

- A reachable pose with a plausible approach direction plans and executes.
- The tool's approach axis visibly respects the cone.
- Tighten `orientation_tolerance_deg` until planning fails, confirming the failure is loud and the error names the tolerance.
- **Tune `defaultPositionToleranceMM`.** The 1.0 default is a starting point, not a derived value: its worst-case corner deviation (~1.73mm) is comparable to the arm's repeatability rather than safely below it. Record what value actually works and update the default if warranted.

---

## Verification checklist

- [ ] `go build ./cmd/module/ .` passes
- [ ] `go test ./cmd/module/ .` passes (`TestEnumerateSerialPorts` may fail without serial hardware — not a regression)
- [ ] `go vet ./cmd/module/ .` passes
- [ ] `gofmt -s -w .` leaves no diff
- [ ] CI green (watch the `go` directive 1.25.1→1.25.9)
- [ ] Hardware verification from Task 13 done, and the positional default revisited

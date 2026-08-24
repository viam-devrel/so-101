# Articulated Gripper Jaw Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulated SO-101 gripper `InputEnabled` behind an `articulated_jaw` config flag, with a real revolute jaw joint in its kinematic model, so we can measure what viam-server's motion planner does with a DoF that changes collision geometry but not the goal pose.

**Architecture:** `internal/geometry.BuildGripperModel` gains a `jawDoF bool`. When false it builds today's 0-DoF chain unchanged. When true it builds a *branching* SVA model — a static body chain that forks into a static `tcp` leaf and a `jaw_mount → jaw_joint → gripper_moving` branch — declared with `output_frames: ["tcp"]` so the model's `Transform` stays the (jaw-invariant) TCP. The simulated gripper wires `CurrentInputs`/`GoToInputs` through the existing percent-based interpolator via a shared bijection, and logs every planner-issued jaw trajectory.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, testify (`require`/`assert`), `go.viam.com/rdk/referenceframe` SVA models, `go.viam.com/rdk/motionplan/armplanning`.

**Spec:** `docs/superpowers/specs/2026-08-23-articulated-gripper-jaw-design.md`

---

## Background you need before starting

Read the spec. These four facts are load-bearing and easy to get wrong:

1. **`JointConfig.Min`/`Max` in SVA JSON are DEGREES.** `referenceframe/frame_json.go:152` applies `utils.DegToRad` when building the `Limit`. `GripperJointMin`/`GripperJointMax` in `internal/geometry/gripper.go` are **radians**. Convert on the way in, and assert the *parsed* limits.
2. **A link's `geometry` offset is measured from its PARENT frame, not its own pose.** Pinned by RDK's `TestGripperModelJSON`. This is why `gripper_moving` gets an identity-ish link under `jaw_joint` with the mesh pose living in the geometry offset.
3. **A model ships to viam-server as `ModelConfig().OriginalFile.Bytes`.** Anything built only in memory is lost across the module gRPC boundary while in-process tests stay green. Hence the SVA-JSON round-trip and Task 5.
4. **A model with two leaves needs `output_frames`.** `referenceframe/model_json.go` requires exactly one leaf when `output_frames` is empty, and accepts exactly one entry when it isn't.

**Run tests with:** `go test ./...` (or `make test`). **Lint with:** `gofmt -s -w . && go vet ./...` (or `make lint`).

`referenceframe.Input` is a **type alias** for `float64` (`input.go:18`), so plain float64 literals work in `[]referenceframe.Input{...}`.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `internal/geometry/gripper.go` | Gripper meshes, kinematic model, jaw-angle bijection | Modify — split mesh helpers, add `jawDoF` branch, add bijection |
| `internal/geometry/gripper_frame_test.go` | Model shape / frame-system contracts | Modify — add articulated-model tests |
| `internal/geometry/serialization_test.go` | Module-boundary guards | Modify — add articulated round-trip |
| `internal/geometry/planner_jaw_test.go` | **The experiment** | Create |
| `components/simulated/simulated_gripper.go` | Simulated gripper component | Modify — config flag, input wiring, instrumentation |
| `components/simulated/simulated_gripper_test.go` | Component behaviour | Modify — config, inputs, non-blocking, instrumentation |
| `components/gripper/gripper.go` | Hardware gripper | Modify — new `BuildGripperModel` arg (`false`), adopt bijection |
| `docs/simulated.md` | User docs | Modify |
| `CLAUDE.md` | Repo gotchas | Modify |

---

## Task 1: Jaw-angle bijection

The percent↔radian conversion is currently inlined **twice** — `components/simulated/simulated_gripper.go:235` and `components/gripper/gripper.go:318-319`. Both become one shared helper before anything else depends on it.

**Files:**
- Modify: `internal/geometry/gripper.go`
- Modify: `components/simulated/simulated_gripper.go:233-236`
- Modify: `components/gripper/gripper.go:315-320`
- Test: `internal/geometry/gripper_frame_test.go`

- [ ] **Step 1: Write the failing test**

Append to `internal/geometry/gripper_frame_test.go`:

```go
// TestJawAngleBijection pins the percent<->radian mapping shared by the model builder, both
// gripper components, and GoToInputs. A drift here silently rescales every jaw command.
func TestJawAngleBijection(t *testing.T) {
	assert.InDelta(t, GripperJointMin, JawRadiansFromPct(0), 1e-12)
	assert.InDelta(t, GripperJointMax, JawRadiansFromPct(100), 1e-12)

	for _, pct := range []float64{0, 12.5, 50, 87.3, 100} {
		back := JawPctFromRadians(JawRadiansFromPct(pct))
		assert.InDeltaf(t, pct, back, 1e-9, "round trip lost %v", pct)
	}

	// Out-of-range inputs clamp rather than extrapolating into the servo stops.
	assert.InDelta(t, GripperJointMin, JawRadiansFromPct(-25), 1e-12)
	assert.InDelta(t, GripperJointMax, JawRadiansFromPct(150), 1e-12)
	assert.InDelta(t, 0.0, JawPctFromRadians(GripperJointMin-1), 1e-12)
	assert.InDelta(t, 100.0, JawPctFromRadians(GripperJointMax+1), 1e-12)
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `go test ./internal/geometry/ -run TestJawAngleBijection`
Expected: FAIL — `undefined: JawRadiansFromPct`

- [ ] **Step 3: Implement**

In `internal/geometry/gripper.go`, after the `GripperJointMin`/`GripperJointMax` const block:

```go
// JawRadiansFromPct maps a gripper opening percentage (0 closed, 100 open) onto the jaw
// joint angle in radians, clamping out-of-range input. This and its inverse are the single
// definition of the jaw mapping. Named for radians because gripper.go carries a second,
// unrelated percent->radian mapping over the servo shaft's +/-pi.
func JawRadiansFromPct(pct float64) float64 {
	p := math.Max(0, math.Min(100, pct)) / 100.0
	return GripperJointMin + p*(GripperJointMax-GripperJointMin)
}

// JawPctFromRadians is the inverse of JawRadiansFromPct. It SATURATES out-of-range angles to
// 0/100 rather than rejecting them, so callers needing limit enforcement must validate first.
func JawPctFromRadians(rad float64) float64 {
	pct := 100.0 * (rad - GripperJointMin) / (GripperJointMax - GripperJointMin)
	return math.Max(0, math.Min(100, pct))
}
```

- [ ] **Step 4: Verify it passes**

Run: `go test ./internal/geometry/ -run TestJawAngleBijection -v`
Expected: PASS

- [ ] **Step 5: Adopt at both existing call sites**

`components/simulated/simulated_gripper.go` — replace the inlined arithmetic in `Geometries`:

```go
	g.mu.Lock()
	pct := g.currentPct
	g.mu.Unlock()
	return geometry.BuildGripperMeshes(g.gripperType, g.meshDetail, geometry.JawRadiansFromPct(pct))
```

`components/gripper/gripper.go` — in `jawAngle`, replace the final two lines:

```go
	return geometry.JawRadiansFromPct(percent)
```

(Keep the existing early `return geometry.GripperJointMin` on the error path. Drop the now-unused `math` import from `gripper.go` **only if** nothing else in the file uses it — check with `go build ./...`.)

- [ ] **Step 6: Full suite must stay green**

Run: `go test ./... && gofmt -s -w . && go vet ./...`
Expected: PASS, no diff from gofmt

- [ ] **Step 7: Commit**

```bash
git add internal/geometry/gripper.go internal/geometry/gripper_frame_test.go components/simulated/simulated_gripper.go components/gripper/gripper.go
git commit -m "refactor(geometry): share the jaw percent<->angle bijection"
```

---

## Task 2: Add `jawDoF` parameter, default path unchanged

Thread the parameter through without changing any behaviour yet. This isolates the call-site churn from the topology change.

**Files:**
- Modify: `internal/geometry/gripper.go:140`
- Modify: `components/simulated/simulated_gripper.go:107`
- Modify: `components/gripper/gripper.go:142`
- Test: `internal/geometry/gripper_frame_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestZeroDoFModelUnchanged guards the default path: with jawDoF false the model must stay
// exactly what it is today -- no DoF, both meshes, frame at the TCP. Every existing machine
// config depends on this.
func TestZeroDoFModelUnchanged(t *testing.T) {
	for _, gt := range []string{FollowerGripper, LeaderGripper} {
		t.Run(gt, func(t *testing.T) {
			m, err := BuildGripperModel(gt, LowDetail, "gripper", false)
			require.NoError(t, err)
			assert.Empty(t, m.DoF(), "the default gripper model must remain 0-DoF")

			pose, err := m.Transform(nil)
			require.NoError(t, err)
			want := GripperTCPPose.Point()
			if gt == LeaderGripper {
				want = r3.Vector{}
			}
			assert.Less(t, pose.Point().Sub(want).Norm(), 1e-9)

			gif, err := m.Geometries(nil)
			require.NoError(t, err)
			assert.Len(t, gif.Geometries(), len(gripperStaticMeshNames(gt))+1)
		})
	}
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `go test ./internal/geometry/ -run TestZeroDoFModelUnchanged`
Expected: FAIL — too many arguments to `BuildGripperModel`, and `undefined: gripperStaticMeshNames`

- [ ] **Step 3: Extract the mesh-name helpers**

In `internal/geometry/gripper.go`, above `BuildGripperMeshes`:

```go
// gripperStaticMeshNames returns the static body mesh names for a gripper variant. The
// follower's body is one piece; the leader's is the wrist-roll part (which connects to the
// arm) plus the handle attached to it.
func gripperStaticMeshNames(gripperType string) []string {
	if gripperType == LeaderGripper {
		return []string{"leader_wrist_roll", "leader_body"}
	}
	return []string{"follower_body"}
}

// gripperMovingMeshName returns the moving part's mesh name: the jaw for the follower, the
// trigger for the leader.
func gripperMovingMeshName(gripperType string) string {
	if gripperType == LeaderGripper {
		return "leader_trigger"
	}
	return "follower_jaw"
}
```

Replace the inline `staticNames` / `movingName` block at the top of `BuildGripperMeshes` with calls to these two.

- [ ] **Step 4: Add the parameter**

Change the signature to `func BuildGripperModel(gripperType, meshDetail, name string, jawDoF bool) (referenceframe.Model, error)`. Leave the body alone for now — the parameter is unused, so add a temporary `_ = jawDoF` line **only if** the compiler complains (it won't; unused *parameters* are legal in Go).

Update both call sites to pass `false`:
- `components/simulated/simulated_gripper.go:107`
- `components/gripper/gripper.go:142`

Update the four existing calls in `internal/geometry/gripper_frame_test.go` (lines ~92, ~124, ~149, ~174) to pass `false`.

- [ ] **Step 5: Verify**

Run: `go test ./... && go vet ./...`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor(geometry): thread jawDoF through BuildGripperModel"
```

---

## Task 3: Build the articulated topology

**Files:**
- Modify: `internal/geometry/gripper.go`
- Test: `internal/geometry/gripper_frame_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestArticulatedGripperModelShape is the core contract of the jaw-DoF experiment. Note the
// limits assertion: JointConfig.Min/Max are DEGREES in SVA JSON (frame_json.go:152 applies
// DegToRad), while GripperJointMin/Max are radians. Asserting the PARSED limits is what
// catches a missing conversion -- checking the written values would not.
func TestArticulatedGripperModelShape(t *testing.T) {
	for _, gt := range []string{FollowerGripper, LeaderGripper} {
		t.Run(gt, func(t *testing.T) {
			m, err := BuildGripperModel(gt, LowDetail, "gripper", true)
			require.NoError(t, err)

			dof := m.DoF()
			require.Len(t, dof, 1, "articulated gripper must expose exactly one jaw DoF")
			assert.InDeltaf(t, GripperJointMin, dof[0].Min, 1e-9,
				"jaw lower limit %v rad; degrees/radians conversion is wrong", dof[0].Min)
			assert.InDeltaf(t, GripperJointMax, dof[0].Max, 1e-9,
				"jaw upper limit %v rad; degrees/radians conversion is wrong", dof[0].Max)

			// The whole point of output_frames: the TCP must not move with the jaw.
			want := GripperTCPPose.Point()
			if gt == LeaderGripper {
				want = r3.Vector{}
			}
			for _, theta := range []float64{GripperJointMin, 0, GripperJointMax} {
				p, err := m.Transform([]referenceframe.Input{theta})
				require.NoError(t, err)
				assert.Lessf(t, p.Point().Sub(want).Norm(), 1e-9,
					"TCP moved to %v at jaw=%.4f rad; it must be invariant", p.Point(), theta)
			}

			// The jaw mesh, by contrast, must move. Note WHAT is asserted here:
			// gripperMovingVisualPose is a pure +Z translation and the joint rotates about
			// local +Z, so Rz(theta) leaves the mesh's pose ORIGIN exactly where it was --
			// comparing Pose().Point() would compare two identical points and can never pass.
			// The jaw swings because the mesh VERTICES are offset from that origin, so compare
			// the transformed vertices (and the orientation, which does change).
			closed, err := m.Geometries([]referenceframe.Input{GripperJointMin})
			require.NoError(t, err)
			opened, err := m.Geometries([]referenceframe.Input{GripperJointMax})
			require.NoError(t, err)
			cg := closed.GeometryByName("gripper:gripper_moving")
			og := opened.GeometryByName("gripper:gripper_moving")
			require.NotNil(t, cg)
			require.NotNil(t, og)

			oriD := spatialmath.QuatToR3AA(
				spatialmath.OrientationBetween(cg.Pose().Orientation(), og.Pose().Orientation()).Quaternion()).Norm()
			assert.InDeltaf(t, GripperJointMax-GripperJointMin, oriD, 1e-6,
				"jaw rotated %.4f rad between the limits, want the full %.4f rad range",
				oriD, GripperJointMax-GripperJointMin)

			// meshWorldPoints already exists at gripper_frame_test.go:17.
			cp := meshWorldPoints(cg.(*spatialmath.Mesh))
			op := meshWorldPoints(og.(*spatialmath.Mesh))
			require.Equal(t, len(cp), len(op))
			var maxMoved float64
			for i := range cp {
				maxMoved = math.Max(maxMoved, cp[i].Sub(op[i]).Norm())
			}
			assert.Greaterf(t, maxMoved, 1.0,
				"no jaw vertex moved more than %.4fmm between the closed and open limits", maxMoved)
		})
	}
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `go test ./internal/geometry/ -run TestArticulatedGripperModelShape`
Expected: FAIL — `DoF()` is empty (the parameter is still ignored)

- [ ] **Step 3: Implement the branch**

In `internal/geometry/gripper.go`, factor the body-mesh construction out of `BuildGripperMeshes` so both callers share it:

```go
// buildGripperBodyMeshes returns the static body meshes, posed in the gripper component's
// (tool) frame.
func buildGripperBodyMeshes(gripperType, meshDetail string) ([]spatialmath.Geometry, error) {
	bodyPose := spatialmath.Compose(toolFromGripperLink, gripperBodyPose)
	names := gripperStaticMeshNames(gripperType)
	geoms := make([]spatialmath.Geometry, 0, len(names))
	for i, name := range names {
		ply, err := gripperMeshPLY(meshDetail, name)
		if err != nil {
			return nil, err
		}
		m, err := spatialmath.NewMeshFromProto(bodyPose,
			&commonpb.Mesh{ContentType: "ply", Mesh: ply}, fmt.Sprintf("gripper_body_%d", i))
		if err != nil {
			return nil, err
		}
		geoms = append(geoms, m)
	}
	return geoms, nil
}

// buildGripperMovingMesh returns the moving part's mesh at the given pose. The two callers
// compute that pose by DIFFERENT routes -- BuildGripperMeshes composes it by hand, the
// articulated model lets the frame system compose it from the jaw joint -- which is what makes
// TestModelGeometriesMatchBuiltMeshes a real check rather than a tautology.
func buildGripperMovingMesh(gripperType, meshDetail string, pose spatialmath.Pose) (spatialmath.Geometry, error) {
	ply, err := gripperMeshPLY(meshDetail, gripperMovingMeshName(gripperType))
	if err != nil {
		return nil, err
	}
	return spatialmath.NewMeshFromProto(pose,
		&commonpb.Mesh{ContentType: "ply", Mesh: ply}, "gripper_moving")
}
```

Rewrite `BuildGripperMeshes` to use them (behaviour identical):

```go
func BuildGripperMeshes(gripperType, meshDetail string, jawAngle float64) ([]spatialmath.Geometry, error) {
	geoms, err := buildGripperBodyMeshes(gripperType, meshDetail)
	if err != nil {
		return nil, err
	}
	movingPose := spatialmath.Compose(toolFromGripperLink, spatialmath.Compose(
		spatialmath.Compose(gripperJointPose,
			spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Yaw: jawAngle})),
		gripperMovingVisualPose,
	))
	moving, err := buildGripperMovingMesh(gripperType, meshDetail, movingPose)
	if err != nil {
		return nil, err
	}
	return append(geoms, moving), nil
}
```

Add the frame names and the branch builder:

```go
const (
	gripperJawMountFrame = "jaw_mount"
	gripperJawJointFrame = "jaw_joint"
	gripperJawLinkFrame  = "gripper_moving"
)

// appendJawBranch hangs the revolute jaw branch off parent:
//
//	parent -> jaw_mount (static) -> jaw_joint (revolute, +Z) -> gripper_moving (mesh)
//
// jaw_mount carries the joint origin already corrected into the tool frame, so the joint's
// rotation axis is plain local +Z -- matching the EulerAngles{Yaw: theta} that
// BuildGripperMeshes applies at the same point in its chain. gripper_moving's mesh lives in
// the GEOMETRY offset, which SVA measures from the link's PARENT frame (the joint's output),
// so the mesh swings with the joint.
func appendJawBranch(cfg *referenceframe.ModelConfigJSON, gripperType, meshDetail, parent string) error {
	mountPose := spatialmath.Compose(toolFromGripperLink, gripperJointPose)
	mountOrient, err := spatialmath.NewOrientationConfig(mountPose.Orientation())
	if err != nil {
		return fmt.Errorf("encoding the jaw mount orientation: %w", err)
	}
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID:          gripperJawMountFrame,
		Parent:      parent,
		Translation: mountPose.Point(),
		Orientation: mountOrient,
	})

	moving, err := buildGripperMovingMesh(gripperType, meshDetail, gripperMovingVisualPose)
	if err != nil {
		return err
	}
	movingCfg, err := spatialmath.NewGeometryConfig(moving)
	if err != nil {
		return fmt.Errorf("converting the jaw mesh to a geometry config: %w", err)
	}

	// JointConfig Min/Max are DEGREES; frame_json.go converts them back to radians.
	cfg.Joints = append(cfg.Joints, referenceframe.JointConfig{
		ID:     gripperJawJointFrame,
		Type:   "revolute",
		Parent: gripperJawMountFrame,
		Axis:   spatialmath.AxisConfig{X: 0, Y: 0, Z: 1},
		Min:    utils.RadToDeg(GripperJointMin),
		Max:    utils.RadToDeg(GripperJointMax),
	})
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID:       gripperJawLinkFrame,
		Parent:   gripperJawJointFrame,
		Geometry: movingCfg,
	})
	return nil
}
```

Rewrite `BuildGripperModel`'s body so the static chain is built from `buildGripperBodyMeshes`, then it forks:

```go
func BuildGripperModel(gripperType, meshDetail, name string, jawDoF bool) (referenceframe.Model, error) {
	cfg := &referenceframe.ModelConfigJSON{Name: name, KinParamType: "SVA"}

	bodies, err := buildGripperBodyMeshes(gripperType, meshDetail)
	if err != nil {
		return nil, err
	}
	if !jawDoF {
		// Frozen closed, in the chain, exactly as before.
		frozen, err := buildGripperMovingMesh(gripperType, meshDetail,
			spatialmath.Compose(toolFromGripperLink, spatialmath.Compose(
				spatialmath.Compose(gripperJointPose,
					spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Yaw: GripperJointMin})),
				gripperMovingVisualPose)))
		if err != nil {
			return nil, err
		}
		bodies = append(bodies, frozen)
	}

	// Each mesh gets its own link because a link carries at most one geometry. All transforms
	// are identity, so the full mesh pose lives in the geometry offset.
	parent := referenceframe.World
	for _, mesh := range bodies {
		geomCfg, err := spatialmath.NewGeometryConfig(mesh)
		if err != nil {
			return nil, fmt.Errorf("converting gripper mesh %q to a geometry config: %w", mesh.Label(), err)
		}
		cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
			ID: mesh.Label(), Parent: parent, Geometry: geomCfg,
		})
		parent = mesh.Label()
	}

	tcp := GripperTCPPose
	if gripperType == LeaderGripper {
		tcp = spatialmath.NewZeroPose()
	}
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID: gripperTCPFrame, Parent: parent, Translation: tcp.Point(),
	})

	if jawDoF {
		// Two leaves (tcp and gripper_moving) are legal only with output_frames, which v1.0.0
		// accepts for exactly one entry.
		if err := appendJawBranch(cfg, gripperType, meshDetail, parent); err != nil {
			return nil, err
		}
		cfg.OutputFrames = []string{gripperTCPFrame}
	}

	jsonBytes, err := json.Marshal(cfg)
	if err != nil {
		return nil, fmt.Errorf("serializing the %s gripper model: %w", gripperType, err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
}
```

Add `"go.viam.com/rdk/utils"` to the imports.

Update `BuildGripperModel`'s doc comment: it is no longer unconditionally "zero-DoF". State that `jawDoF` selects a branching 1-DoF model whose TCP stays invariant, and keep the existing paragraphs about frame-system geometry and the `OriginalFile` round trip.

- [ ] **Step 4: Verify both shape tests pass**

Run: `go test ./internal/geometry/ -run 'TestArticulatedGripperModelShape|TestZeroDoFModelUnchanged' -v`
Expected: PASS

- [ ] **Step 5: Full suite**

Run: `go test ./... && gofmt -s -w . && go vet ./...`
Expected: PASS. `TestBuildGripperModelPlacesFrameAtTCP`, `TestBuildGripperModelCarriesMeshes` and `TestGripperFrameResolvesToTCPInFrameSystem` must all still pass untouched.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat(geometry): optional 1-DoF revolute jaw in the gripper model"
```

---

## Task 4: Prove the two posing paths agree

**Files:**
- Test: `internal/geometry/gripper_frame_test.go`

- [ ] **Step 1: Write the test**

```go
// TestModelGeometriesMatchBuiltMeshes closes the drift risk introduced by articulation: the jaw
// pose is now computed twice -- by the frame system from the jaw joint (collision geometry) and
// by hand in BuildGripperMeshes (the 3D viewer). The two must agree at every angle or the
// planner will avoid a jaw that is not where the user sees it.
func TestModelGeometriesMatchBuiltMeshes(t *testing.T) {
	for _, gt := range []string{FollowerGripper, LeaderGripper} {
		t.Run(gt, func(t *testing.T) {
			m, err := BuildGripperModel(gt, LowDetail, "gripper", true)
			require.NoError(t, err)

			steps := 9
			for i := 0; i <= steps; i++ {
				theta := GripperJointMin + (GripperJointMax-GripperJointMin)*float64(i)/float64(steps)

				gif, err := m.Geometries([]referenceframe.Input{theta})
				require.NoError(t, err)
				fromModel := gif.GeometryByName("gripper:gripper_moving")
				require.NotNilf(t, fromModel, "model has no jaw geometry at %.4f rad", theta)

				built, err := BuildGripperMeshes(gt, LowDetail, theta)
				require.NoError(t, err)
				fromMeshes := built[len(built)-1]
				require.Equal(t, "gripper_moving", fromMeshes.Label())

				a, b := fromModel.Pose(), fromMeshes.Pose()
				assert.Lessf(t, a.Point().Sub(b.Point()).Norm(), 1e-9,
					"jaw=%.4f rad: model places the mesh at %v, BuildGripperMeshes at %v", theta, a.Point(), b.Point())
				oriD := spatialmath.QuatToR3AA(
					spatialmath.OrientationBetween(a.Orientation(), b.Orientation()).Quaternion()).Norm()
				assert.Lessf(t, oriD, 1e-9,
					"jaw=%.4f rad: jaw mesh orientation differs by %.3e rad between the two paths", theta, oriD)
			}
		})
	}
}
```

- [ ] **Step 2: Run it**

Run: `go test ./internal/geometry/ -run TestModelGeometriesMatchBuiltMeshes -v`
Expected: PASS. **If it fails**, the jaw-branch composition in Task 3 is wrong — most likely `jaw_mount`'s orientation, or the geometry offset being measured from the wrong frame. Do not "fix" it by changing the test. (Reference: this was verified to hold to 7e-15 mm and 0 rad during plan review, so a real failure means a real bug.)

- [ ] **Step 3: Commit**

```bash
git add internal/geometry/gripper_frame_test.go
git commit -m "test(geometry): pin jaw pose equivalence between model and meshes"
```

---

## Task 5: Prove the joint survives the module boundary

**Files:**
- Test: `internal/geometry/serialization_test.go`

- [ ] **Step 1: Write the test**

Append to `internal/geometry/serialization_test.go`:

```go
// TestArticulatedModelSurvivesSerialization extends the module-boundary guard to the jaw joint.
// A component ships its kinematics as ModelConfig().OriginalFile.Bytes, so a joint that exists
// only in the in-memory model transmits as a 0-DoF gripper: viam-server would then never ask for
// jaw inputs, the planner would never see the DoF, and every in-process test here would still
// pass. Do NOT weaken this to an in-process DoF check.
func TestArticulatedModelSurvivesSerialization(t *testing.T) {
	m, err := BuildGripperModel(FollowerGripper, LowDetail, "sim-gripper", true)
	require.NoError(t, err)

	resp := referenceframe.KinematicModelToProtobuf(m)
	out, err := referenceframe.KinematicModelFromProtobuf("sim-gripper", resp)
	require.NoError(t, err, "server-side reconstruction from transmitted kinematics")

	dof := out.DoF()
	require.Len(t, dof, 1, "the jaw joint was lost across the gRPC boundary")
	assert.InDelta(t, GripperJointMin, dof[0].Min, 1e-9, "jaw lower limit did not survive")
	assert.InDelta(t, GripperJointMax, dof[0].Max, 1e-9, "jaw upper limit did not survive")

	// The TCP must still be the output frame, and still invariant, on the server's copy.
	for _, theta := range []float64{GripperJointMin, GripperJointMax} {
		p, err := out.Transform([]referenceframe.Input{theta})
		require.NoError(t, err)
		assert.Lessf(t, p.Point().Sub(GripperTCPPose.Point()).Norm(), 1e-6,
			"transmitted TCP at %v (jaw=%.3f), want %v", p.Point(), theta, GripperTCPPose.Point())
	}

	gif, err := out.Geometries([]referenceframe.Input{GripperJointMin})
	require.NoError(t, err)
	assert.Len(t, gif.Geometries(), 2, "transmitted model lost its gripper meshes")
}
```

Add `"github.com/stretchr/testify/assert"` to the imports if it is not already there.

- [ ] **Step 2: Run it**

Run: `go test ./internal/geometry/ -run TestArticulatedModelSurvivesSerialization -v`
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add internal/geometry/serialization_test.go
git commit -m "test(geometry): guard the jaw joint across the module boundary"
```

---

## Task 6: `articulated_jaw` config flag

**Files:**
- Modify: `components/simulated/simulated_gripper.go`
- Test: `components/simulated/simulated_gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestArticulatedJawConfig covers the flag end to end. The ErrUnsupported half matters as much
// as the enabled half: robot/framesystem's CurrentInputs walks every frame with DoF and hard-
// errors if the component is not InputEnabled, so a 1-DoF model with unwired inputs would break
// the WHOLE machine's frame system, arm moves included.
func TestArticulatedJawConfig(t *testing.T) {
	ctx := context.Background()

	newGripper := func(t *testing.T, articulated bool) gripper.Gripper {
		t.Helper()
		g, err := newSimulatedSO101Gripper(ctx, nil, resource.Config{
			Name: "sim-gripper",
			ConvertedAttributes: &SO101SimulatedGripperConfig{ArticulatedJaw: articulated},
		}, logging.NewTestLogger(t))
		require.NoError(t, err)
		t.Cleanup(func() { require.NoError(t, g.Close(ctx)) })
		return g
	}

	t.Run("default is static", func(t *testing.T) {
		g := newGripper(t, false)
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		assert.Empty(t, m.DoF(), "articulated_jaw must default to false")

		_, err = g.CurrentInputs(ctx)
		assert.ErrorIs(t, err, errors.ErrUnsupported)
		assert.ErrorIs(t, g.GoToInputs(ctx, []referenceframe.Input{0}), errors.ErrUnsupported)
	})

	t.Run("articulated", func(t *testing.T) {
		g := newGripper(t, true)
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		require.Len(t, m.DoF(), 1)

		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		require.Len(t, in, 1)
	})
}
```

`gripper.Gripper` embeds `framesystem.InputEnabled` (rdk `components/gripper/gripper.go:79-83`), so `CurrentInputs`/`GoToInputs`/`Kinematics` are all on the interface — no type assertion is needed here.

- [ ] **Step 2: Run it and watch it fail**

Run: `go test ./components/simulated/ -run TestArticulatedJawConfig`
Expected: FAIL — `unknown field ArticulatedJaw`

- [ ] **Step 3: Implement**

Add the field to `SO101SimulatedGripperConfig`:

```go
	// ArticulatedJaw gives the gripper a 1-DoF revolute jaw joint in its kinematic model and
	// makes it InputEnabled, so the motion planner sees the jaw as a variable it may drive.
	// Off by default: the jaw does not affect the TCP, so the planner gets a degree of freedom
	// that changes only collision geometry. See docs/simulated.md.
	ArticulatedJaw bool `json:"articulated_jaw,omitempty"`
```

Store `articulatedJaw bool` and `logger logging.Logger` on the struct, set both in the constructor, and pass `conf.ArticulatedJaw` as `BuildGripperModel`'s fourth argument.

- [ ] **Step 4: Verify**

Run: `go test ./components/simulated/ -run TestArticulatedJawConfig -v`
Expected: the "default is static" subtest passes; "articulated" fails on `CurrentInputs` still returning `ErrUnsupported`. That is expected — Task 8 wires it. Mark the second subtest `t.Skip("wired in Task 8")` temporarily, or write Task 6 and Task 8 as one commit.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(simulated): add the articulated_jaw config flag"
```

---

## Task 7: Split `moveTo` so `set_position` stays non-blocking

`services/teleop/teleop.go:187-193` issues a `set_position` DoCommand on the follower gripper on **every teleop tick** and depends on it returning immediately. Today `set_position` only sets `targetPct`. Making it block would stall the teleop loop.

**Files:**
- Modify: `components/simulated/simulated_gripper.go:167-190, 249-257`
- Test: `components/simulated/simulated_gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestSetPositionStaysNonBlocking is a regression guard for the teleop loop: teleop.go issues a
// set_position DoCommand every tick and depends on it returning before the jaw arrives. Open and
// Grab block; set_position must not.
func TestSetPositionStaysNonBlocking(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, geometry.FollowerGripper, geometry.LowDetail)

	_, err := g.Grab(ctx, nil) // start closed
	require.NoError(t, err)
	start := time.Now()
	_, err = g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position", "percentage": 100.0,
	})
	require.NoError(t, err)
	elapsed := time.Since(start)

	// A full 0->100 sweep takes ~500ms at gripperTravelPctPerSec. Returning in under 50ms proves
	// we did not wait for arrival.
	assert.Lessf(t, elapsed, 50*time.Millisecond,
		"set_position blocked for %v; the teleop loop ticks faster than that", elapsed)

	moving, err := g.IsMoving(ctx)
	require.NoError(t, err)
	assert.True(t, moving, "set_position should have started the jaw moving")
}
```

Uses the existing `newTestSimGripper` helper (`simulated_gripper_test.go:39`). While you are there: that helper never `Close`s the gripper, so its interpolator goroutine leaks across the whole package. Add `t.Cleanup(func() { require.NoError(t, g.Close(context.Background())) })` to it.

- [ ] **Step 2: Run it and watch it fail (or pass)**

Run: `go test ./components/simulated/ -run TestSetPositionStaysNonBlocking -v`
Expected: PASS today (this locks in current behaviour *before* Task 8 tempts a refactor into `moveTo`).

- [ ] **Step 3: Make the split explicit**

Replace `moveTo` with three methods:

```go
// setTarget sets the target opening without waiting. set_position uses this: the teleop loop
// issues one every tick and must not block.
func (g *simulatedSO101Gripper) setTarget(pct float64) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.targetPct = pct
}

// awaitArrival blocks until the jaw reaches its target, the gripper is closed, or ctx is done.
func (g *simulatedSO101Gripper) awaitArrival(ctx context.Context) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-g.cancelCtx.Done():
			return g.cancelCtx.Err()
		default:
			g.mu.Lock()
			done := g.currentPct == g.targetPct
			g.mu.Unlock()
			if done {
				return nil
			}
			time.Sleep(time.Millisecond)
		}
	}
}

// moveTo sets the target and blocks until the jaw reaches it. Open, Grab and GoToInputs use it.
func (g *simulatedSO101Gripper) moveTo(ctx context.Context, pct float64) error {
	g.setTarget(pct)
	return g.awaitArrival(ctx)
}
```

Change `set_position`'s DoCommand branch to call `g.setTarget(clamped)` instead of locking and assigning inline. `Stop` keeps its own inline lock (it reads `currentPct` and writes `targetPct` atomically).

- [ ] **Step 4: Verify**

Run: `go test ./components/simulated/ ./services/teleop/ -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor(simulated): split gripper moveTo into setTarget and awaitArrival"
```

---

## Task 8: Wire `CurrentInputs` and `GoToInputs`

**Files:**
- Modify: `components/simulated/simulated_gripper.go:277-283`
- Test: `components/simulated/simulated_gripper_test.go`

- [ ] **Step 1: Write the failing tests**

```go
// TestGripperCurrentInputsTracksJaw checks the input reported to the frame system follows the
// simulated jaw through the same bijection the meshes use.
func TestGripperCurrentInputsTracksJaw(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	require.NoError(t, g.Open(ctx, nil))
	in, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	require.Len(t, in, 1)
	assert.InDelta(t, geometry.GripperJointMax, in[0], 1e-9, "open jaw should report the upper limit")

	_, err = g.Grab(ctx, nil)
	require.NoError(t, err)
	in, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.InDelta(t, geometry.GripperJointMin, in[0], 1e-9, "closed jaw should report the lower limit")
}

// TestGoToInputsValidation covers what the motion service will actually send. execute() batches
// consecutive trajectory steps into one variadic GoToInputs call, so multi-step batches are the
// normal case, not the exception.
func TestGoToInputsValidation(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	t.Run("rejects wrong arity", func(t *testing.T) {
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{0, 0}))
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{}))
	})

	t.Run("rejects out-of-limit angles", func(t *testing.T) {
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax + 0.5}))
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMin - 0.5}))
	})

	t.Run("applies a multi-step batch in order, ending at the last", func(t *testing.T) {
		mid := (geometry.GripperJointMin + geometry.GripperJointMax) / 2
		require.NoError(t, g.GoToInputs(ctx,
			[]referenceframe.Input{mid},
			[]referenceframe.Input{geometry.GripperJointMax},
			[]referenceframe.Input{geometry.GripperJointMin},
		))
		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		assert.InDelta(t, geometry.GripperJointMin, in[0], 1e-9)
	})
}
```

Add the helper:

```go
func newArticulatedTestGripper(t *testing.T) *simulatedSO101Gripper {
	t.Helper()
	ctx := context.Background()
	g, err := newSimulatedSO101Gripper(ctx, nil, resource.Config{
		Name:                "sim-gripper",
		ConvertedAttributes: &SO101SimulatedGripperConfig{ArticulatedJaw: true},
	}, logging.NewTestLogger(t))
	require.NoError(t, err)
	t.Cleanup(func() { require.NoError(t, g.Close(ctx)) })
	return g.(*simulatedSO101Gripper)
}
```

- [ ] **Step 2: Run and watch fail**

Run: `go test ./components/simulated/ -run 'TestGripperCurrentInputsTracksJaw|TestGoToInputsValidation'`
Expected: FAIL — `ErrUnsupported`

- [ ] **Step 3: Implement**

```go
// CurrentInputs reports the jaw angle in radians. It is an error rather than a zero value when
// articulated_jaw is off: robot/framesystem only asks components whose model has DoF, so being
// asked at all while static means something is inconsistent.
func (g *simulatedSO101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	if !g.articulatedJaw {
		return nil, errors.ErrUnsupported
	}
	g.mu.Lock()
	pct := g.currentPct
	g.mu.Unlock()
	return []referenceframe.Input{geometry.JawRadiansFromPct(pct)}, nil
}

// GoToInputs drives the jaw through each step in turn. The motion service batches consecutive
// trajectory steps for one component into a single variadic call, so a batch is the normal case.
func (g *simulatedSO101Gripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	if !g.articulatedJaw {
		return errors.ErrUnsupported
	}
	limits := g.model.DoF()
	for i, step := range inputSteps {
		if len(step) != len(limits) {
			return fmt.Errorf("step %d: got %d inputs, the jaw takes %d", i, len(step), len(limits))
		}
		if step[0] < limits[0].Min || step[0] > limits[0].Max {
			return fmt.Errorf("step %d: jaw angle %.4f rad is outside [%.4f, %.4f]",
				i, step[0], limits[0].Min, limits[0].Max)
		}
	}
	g.recordJawTrajectory(inputSteps)  // added in Task 9; omit for now
	for _, step := range inputSteps {
		if err := g.moveTo(ctx, geometry.JawPctFromRadians(step[0])); err != nil {
			return err
		}
	}
	return nil
}
```

Validate **every** step before moving any of them — a batch that fails halfway leaves the jaw somewhere the planner did not intend.

- [ ] **Step 4: Verify**

Run: `go test ./components/simulated/ -v`
Expected: PASS, including the previously-skipped "articulated" subtest from Task 6 (remove the skip).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(simulated): wire gripper CurrentInputs and GoToInputs to the jaw"
```

---

## Task 9: Instrumentation

**Files:**
- Modify: `components/simulated/simulated_gripper.go`
- Test: `components/simulated/simulated_gripper_test.go`

- [ ] **Step 1: Write the failing test**

```go
// TestJawTrajectoryDoCommand covers the instrumentation used to watch the planner on a live
// machine. Offsets are monotonic since construction rather than wall-clock so this stays
// deterministic.
func TestJawTrajectoryDoCommand(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	mid := (geometry.GripperJointMin + geometry.GripperJointMax) / 2
	require.NoError(t, g.GoToInputs(ctx,
		[]referenceframe.Input{mid}, []referenceframe.Input{geometry.GripperJointMax}))

	res, err := g.DoCommand(ctx, map[string]interface{}{"command": "get_jaw_trajectory"})
	require.NoError(t, err)

	batches, ok := res["batches"].([]map[string]interface{})
	require.True(t, ok, "get_jaw_trajectory returned %T for batches", res["batches"])
	require.Len(t, batches, 1)
	assert.Equal(t, 2, batches[0]["step_count"])
	assert.Greater(t, batches[0]["total_travel_rad"], 0.0)

	// The ring buffer keeps only the most recent jawTrajectoryHistory batches.
	for i := 0; i < jawTrajectoryHistory+3; i++ {
		require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{mid}))
	}
	res, err = g.DoCommand(ctx, map[string]interface{}{"command": "get_jaw_trajectory"})
	require.NoError(t, err)
	assert.Len(t, res["batches"], jawTrajectoryHistory)
}

// TestGetPositionReportsJaw checks the viewer can read jaw state in one call.
func TestGetPositionReportsJaw(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)
	require.NoError(t, g.Open(ctx, nil))

	res, err := g.DoCommand(ctx, map[string]interface{}{"command": "get_position"})
	require.NoError(t, err)
	assert.Equal(t, 1, res["dof"])
	assert.InDelta(t, geometry.GripperJointMax, res["jaw_angle_rad"], 1e-9)
}
```

- [ ] **Step 2: Run and watch fail**

Run: `go test ./components/simulated/ -run 'TestJawTrajectoryDoCommand|TestGetPositionReportsJaw'`
Expected: FAIL — `undefined: jawTrajectoryHistory`, unknown command

- [ ] **Step 3: Implement**

```go
// jawTrajectoryHistory is how many GoToInputs batches the gripper remembers for
// get_jaw_trajectory. Small on purpose: this is a debugging window, not a log.
const jawTrajectoryHistory = 8

// jawBatch is one recorded GoToInputs call.
type jawBatch struct {
	steps          [][]referenceframe.Input
	totalTravelRad float64
	offset         time.Duration // since construction; monotonic so tests stay deterministic
}
```

Add `createdAt time.Time` and `jawBatches []jawBatch` to the struct (guarded by `g.mu`).

```go
// recordJawTrajectory logs a planner-issued batch and keeps it for get_jaw_trajectory. This is
// the instrument the jaw-DoF experiment reads: the planner is free to drive the jaw even though
// it does not affect the TCP, and this is where that shows up.
func (g *simulatedSO101Gripper) recordJawTrajectory(steps [][]referenceframe.Input) {
	if len(steps) == 0 {
		return
	}
	var travel float64
	g.mu.Lock()
	prev := geometry.JawRadiansFromPct(g.currentPct)
	g.mu.Unlock()
	first := prev
	for _, s := range steps {
		travel += math.Abs(s[0] - prev)
		prev = s[0]
	}

	g.logger.Infof(
		"jaw GoToInputs: %d step(s), theta %.4f -> %.4f rad, total travel %.4f rad, linf from current %.4f",
		len(steps), first, prev, travel, math.Abs(steps[0][0]-first))

	g.mu.Lock()
	defer g.mu.Unlock()
	g.jawBatches = append(g.jawBatches, jawBatch{
		steps: steps, totalTravelRad: travel, offset: time.Since(g.createdAt),
	})
	if len(g.jawBatches) > jawTrajectoryHistory {
		g.jawBatches = g.jawBatches[len(g.jawBatches)-jawTrajectoryHistory:]
	}
}
```

Add the `get_jaw_trajectory` DoCommand branch. Build the value as a **`[]map[string]interface{}`** (the test type-asserts exactly that, so an `[]interface{}` will fail):

```go
	case "get_jaw_trajectory":
		g.mu.Lock()
		defer g.mu.Unlock()
		out := make([]map[string]interface{}, 0, len(g.jawBatches))
		for _, b := range g.jawBatches {
			steps := make([][]float64, len(b.steps))
			for i, s := range b.steps {
				steps[i] = append([]float64(nil), s...)
			}
			out = append(out, map[string]interface{}{
				"step_count":       len(b.steps),
				"total_travel_rad": b.totalTravelRad,
				"offset_ms":        float64(b.offset.Milliseconds()),
				"steps":            steps,
			})
		}
		return map[string]interface{}{"batches": out}, nil
```

Extend `get_position` with `jaw_angle_rad` (float64) and `dof` (int, `len(g.model.DoF())`).

Remove the `// omit for now` note from Task 8's `recordJawTrajectory` call.

- [ ] **Step 4: Verify**

Run: `go test ./components/simulated/ -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(simulated): log and expose planner-issued jaw trajectories"
```

---

## Task 10: Document the execute-epsilon hazard in a test

**Files:**
- Test: `components/simulated/simulated_gripper_test.go`

- [ ] **Step 1: Write the test**

```go
// TestExecuteEpsilonTripsOnMovingJaw documents the sharpest practical consequence of making the
// gripper InputEnabled. services/motion/builtin/builtin.go:633 rejects a plan whose FIRST
// trajectory step differs from CurrentInputs by more than defaultExecuteEpsilon (0.01), and
// aborts the whole move -- the ARM's move -- when it does. A jaw still travelling from an
// earlier Open() is enough to trip it.
func TestExecuteEpsilonTripsOnMovingJaw(t *testing.T) {
	const defaultExecuteEpsilon = 0.01 // services/motion/builtin/builtin.go:77

	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	// Close, then start opening without waiting: the jaw is now mid-travel.
	_, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	g.setTarget(100)
	time.Sleep(50 * time.Millisecond)

	current, err := g.CurrentInputs(ctx)
	require.NoError(t, err)

	// A plan seeded a moment earlier would start from the closed jaw.
	planStart := []referenceframe.Input{geometry.GripperJointMin}
	assert.Greaterf(t,
		referenceframe.InputsLinfDistance(current, planStart), defaultExecuteEpsilon,
		"a mid-travel jaw (%v) should exceed the execute epsilon against the plan start %v",
		current, planStart)
}
```

- [ ] **Step 2: Run it**

Run: `go test ./components/simulated/ -run TestExecuteEpsilonTripsOnMovingJaw -v`
Expected: PASS. If it fails, `gripperTravelPctPerSec` may have made the sweep finish inside the sleep — shorten the sleep rather than loosening the assertion.

- [ ] **Step 3: Commit**

```bash
git add components/simulated/simulated_gripper_test.go
git commit -m "test(simulated): document the execute-epsilon hazard for a moving jaw"
```

---

## Task 11: The experiment

This is the deliverable. It **reports** rather than asserts magnitudes — the magnitude is the unknown.

Two things about this task were established empirically during plan review; do not re-derive them:

1. **The goal must be derived from forward kinematics.** Hand-picked poses do not work. `testGoal()`
   from `internal/planning` — `(300, 0, 200)` with `OZ: -1` — is **physically unreachable** by this
   arm and returns "zero IK solutions" for every seed. (It is used elsewhere only with an *injected*
   motion service that never plans, so nothing ever caught it.) At zero configuration the TCP sits at
   `(393, 0, 228)` with the tool pointing along **+Y**, not down. Taking the goal from FK of a real
   joint configuration makes it reachable by construction.
2. **Two scenarios are needed, because they measure different things.** With no obstacle the planner
   returns a **2-step** trajectory (start and goal only) — that measures whether *IK and seeding*
   choose a non-start jaw angle. With an obstacle in the path it returns **~300-420 steps** — that
   measures whether the *RRT* drives the jaw while searching. Only running the first would look like
   a result while exercising no sampling at all.

**Files:**
- Create: `internal/geometry/planner_jaw_test.go`
- Modify: `go.mod`, `go.sum`

- [ ] **Step 1: Tidy first — the new import is not in go.mod yet**

`internal/geometry` does not currently import `motionplan/armplanning`, so `go test` fails
*before compiling anything* with `go: updates to go.mod needed`. Run this first so a real
compile error is not mistaken for a module error:

Run: `go mod tidy`
Expected: `go.mod` gains ~8 lines and `go.sum` ~5 (new indirect deps: `go-nlopt/nlopt`,
`go-ole`, `lufia/plan9stats`, `yusufpapurcu/wmi`, …). This is normal.

- [ ] **Step 2: Write the test**

```go
package geometry

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// jawGoalConfig is the arm configuration whose forward kinematics defines the planning goal.
// Deriving the goal from FK guarantees it is reachable; hand-picked poses are not (see the task
// notes -- internal/planning's testGoal is unreachable by this arm).
var jawGoalConfig = []float64{0.8, -1.0, 1.0, 0.6, 0.3}

// TestPlannerJawTravel is the experiment this whole change exists to run.
//
// The jaw is a DoF that changes collision geometry but has NO effect on the TCP, so nothing in
// the goal metric constrains it. Reading armplanning predicts the planner hands it FULL range:
// computeJointSensitivities (linearized_frame_system.go:78) computes
//
//	thisRatio := startDistance / math.Abs(myDistance-startDistance)
//
// which is INVERSE sensitivity -- a joint that does not move the TCP divides by zero, yields
// +Inf, and clampSensitivities floors it at 1.0, the joint's entire range. RDK does this on
// purpose ("a small change has a small effect -> take bigger steps").
//
// What that means for the EMITTED trajectory, after RRT and smoothing, is not determinable by
// reading the code. That is what this measures. It asserts only structural facts and t.Logf's
// the magnitudes; once characterised, tighten the reported numbers into a real assertion.
func TestPlannerJawTravel(t *testing.T) {
	if testing.Short() {
		t.Skip("motion planning is slow")
	}
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	armModel, err := ArmModelJSON("arm")
	require.NoError(t, err)
	gripperModel, err := BuildGripperModel(FollowerGripper, LowDetail, "gripper", true)
	require.NoError(t, err)

	armLink, err := (&referenceframe.LinkConfig{ID: "arm", Parent: referenceframe.World}).ParseConfig()
	require.NoError(t, err)
	gripperLink, err := (&referenceframe.LinkConfig{ID: "gripper", Parent: "arm"}).ParseConfig()
	require.NoError(t, err)

	fs, err := referenceframe.NewFrameSystem("test", []*referenceframe.FrameSystemPart{
		{FrameConfig: armLink, ModelFrame: armModel},
		{FrameConfig: gripperLink, ModelFrame: gripperModel},
	}, nil)
	require.NoError(t, err)

	// Goal by forward kinematics, so it is reachable by construction.
	fkInputs := referenceframe.NewZeroInputs(fs)
	fkInputs["arm"] = jawGoalConfig
	tf, err := fs.Transform(fkInputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame("gripper", spatialmath.NewZeroPose()), referenceframe.World)
	require.NoError(t, err)
	goal := referenceframe.NewPoseInFrame(referenceframe.World, tf.(*referenceframe.PoseInFrame).Pose())
	t.Logf("FK-derived goal: %v", goal.Pose().Point())

	// A slab across the direct path, forcing the RRT to actually search.
	wall, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 290, Y: -78, Z: 169}),
		r3.Vector{X: 90, Y: 90, Z: 260}, "wall")
	require.NoError(t, err)

	fullRange := GripperJointMax - GripperJointMin

	for _, sc := range []struct {
		name      string
		obstacles *referenceframe.GeometriesInFrame
	}{
		{"direct", nil},
		{"obstructed", referenceframe.NewGeometriesInFrame(
			referenceframe.World, []spatialmath.Geometry{wall})},
	} {
		t.Run(sc.name, func(t *testing.T) {
			planned := 0
			for seed := 0; seed < 10; seed++ {
				opts := armplanning.NewBasicPlannerOptions()
				opts.RandomSeed = seed

				plan, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
					FrameSystem:           fs,
					StartState:            armplanning.NewPlanState(nil, referenceframe.NewZeroInputs(fs)),
					Goals:                 []*armplanning.PlanState{armplanning.NewPlanState(referenceframe.FrameSystemPoses{"gripper": goal}, nil)},
					ObstaclesInWorldFrame: sc.obstacles,
					PlannerOptions:        opts,
				})
				if err != nil {
					t.Logf("seed %d: planning failed: %v", seed, err)
					continue
				}
				planned++

				traj := plan.Trajectory()
				require.NotEmpty(t, traj)

				var travel, prev, minT, maxT float64
				for i, step := range traj {
					jaw, ok := step["gripper"]
					require.Truef(t, ok, "seed %d step %d: the trajectory has no gripper column", seed, i)
					require.Len(t, jaw, 1)

					// Structural assertion: the planner must respect the joint limits.
					assert.GreaterOrEqualf(t, jaw[0], GripperJointMin, "seed %d step %d below the jaw limit", seed, i)
					assert.LessOrEqualf(t, jaw[0], GripperJointMax, "seed %d step %d above the jaw limit", seed, i)

					if i == 0 {
						prev, minT, maxT = jaw[0], jaw[0], jaw[0]
						continue
					}
					travel += math.Abs(jaw[0] - prev)
					prev = jaw[0]
					minT, maxT = math.Min(minT, jaw[0]), math.Max(maxT, jaw[0])
				}

				t.Logf("seed %2d: %3d steps | jaw travel %.4f rad (%.1f%% of range) | span [%.4f, %.4f] | net %.4f",
					seed, len(traj), travel, 100*travel/fullRange, minT, maxT, traj[len(traj)-1]["gripper"][0]-traj[0]["gripper"][0])
			}

			// Without this the whole experiment can pass green having measured nothing.
			require.Positivef(t, planned, "no seed produced a trajectory in the %q scenario", sc.name)
		})
	}
}
```

- [ ] **Step 3: Run it and read the output**

Run: `go test ./internal/geometry/ -run TestPlannerJawTravel -v 2>&1 | grep -E "seed|goal|PASS|FAIL"`
Expected: PASS, with `seed NN:` lines under both `direct` and `obstructed`. Reference, measured during plan review
with the articulated model: `direct` plans in 2 steps with jaw travel 0.0000 on every seed;
`obstructed` plans in ~300-420 steps with jaw travel 0.083-0.383 rad (4.3%-19.9% of range),
spanning both signs. The obstructed subtest takes ~35s, so it lands on every later
`go test ./...` — that is expected, and why the `testing.Short()` skip is there.

If a scenario reports zero successful seeds the `require.Positivef` fails loudly — that is the
guard against a silently empty result. Re-derive `jawGoalConfig` from a configuration you have
confirmed with FK rather than loosening the assertion.

- [ ] **Step 4: Record the finding**

Add a `## Result` section to the spec with the log lines from both scenarios, and state plainly
whether the full-range prediction held — separately for IK/seeding (`direct`) and for RRT search
(`obstructed`). A null result is a real result; write it down either way.

- [ ] **Step 5: Commit**

```bash
git add internal/geometry/planner_jaw_test.go go.mod go.sum docs/superpowers/specs/2026-08-23-articulated-gripper-jaw-design.md
git commit -m "test(geometry): measure planner jaw travel across seeds"
```

---

## Task 12: Documentation

**Files:**
- Modify: `docs/simulated.md:76-79`
- Modify: `CLAUDE.md`

- [ ] **Step 1: `docs/simulated.md`**

Add to the `devrel:so101:simulated-gripper` attribute table:

| Name | Type | Inclusion | Description |
| `articulated_jaw` | bool | Optional | Give the gripper a 1-DoF revolute jaw joint in its kinematic model and make it `InputEnabled`, so the motion planner can drive the jaw. Default `false`. **Experimental.** The jaw does not affect the gripper's TCP, so the planner gains a degree of freedom that changes only collision geometry — and it may command jaw motion you did not ask for during an arm move. It also means a jaw still travelling from an earlier `Open`/`Grab` can abort an arm move: the motion service rejects a plan whose first step is more than 0.01 rad from the gripper's current input. |

- [ ] **Step 2: `CLAUDE.md`**

Amend the existing "The model is **0-DoF on purpose**" sentence in the gripper gotcha to read *0-DoF by default*, and note the opt-in `articulated_jaw` flag on the simulated gripper. Keep the existing reasoning — it is now the thing being measured, not a claim being retracted.

Add two gotchas:

```markdown
- **SVA `JointConfig.Min`/`Max` are DEGREES**, while `internal/geometry`'s `GripperJointMin`/`Max`
  and every `referenceframe.Limit` are radians. `referenceframe/frame_json.go:152` applies
  `DegToRad` for revolute joints (prismatic joints pass through unconverted). Assert *parsed*
  `DoF()` limits, never the written config values.
- **A joint that does not affect the goal gets the planner's FULL search range, not none.**
  `computeJointSensitivities` (`motionplan/armplanning/linearized_frame_system.go:78`) computes
  `startDistance / |myDistance - startDistance|` — *inverse* sensitivity, so a zero-effect joint
  divides by zero, yields `+Inf`, and `clampSensitivities` clamps it to `1.0`. This is deliberate
  in rdk. It is why `articulated_jaw` is opt-in and off by default.
```

- [ ] **Step 3: Final verification**

Run: `go test ./... && gofmt -s -w . && go vet ./...`
Expected: PASS, clean

- [ ] **Step 4: Commit**

```bash
git add docs/simulated.md CLAUDE.md
git commit -m "docs: document articulated_jaw and the sensitivity/units gotchas"
```

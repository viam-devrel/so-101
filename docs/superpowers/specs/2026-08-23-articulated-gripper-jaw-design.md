# Opt-in 1-DoF jaw on the simulated SO-101 gripper

**Date:** 2026-08-23
**Status:** Design, approved for planning
**Scope:** `components/simulated`, `internal/geometry`, `docs/simulated.md`, `CLAUDE.md`

## Purpose

Make the simulated SO-101 gripper `InputEnabled` behind a config flag, with a real
revolute jaw joint in its kinematic model, in order to **measure what viam-server's
motion planner does with a degree of freedom that changes collision geometry but has
no effect on the goal pose.**

This is an experiment. The deliverable is a characterisation of planner behaviour, not
a feature. Nothing in the default configuration changes, and the hardware gripper is
untouched.

## Background

`internal/geometry.BuildGripperModel` currently builds a **0-DoF** chain: one link per
mesh, all with identity transforms, ending at a `tcp` link carrying `GripperTCPPose`.
The jaw is frozen at `GripperJointMin`. `Geometries()` separately serves a live,
articulating jaw to the 3D viewer by re-posing the moving mesh through
`BuildGripperMeshes`.

That was deliberate — see the CLAUDE.md gotcha "The model is 0-DoF on purpose" — because
a jaw DoF becomes a variable the motion planner may drive. This design does not dispute
that reasoning. It makes the consequence observable and measurable.

Three facts from RDK v1.0.0 establish feasibility:

1. **Branching gripper models are first-class.** `referenceframe/testfiles/test_gripper.json`
   is a gripper with prismatic finger joints and `output_frames: ["tcp"]`, pinned by
   `TestGripperModelJSON`. The TCP link hangs off the static base and stays invariant
   while the finger geometries move with the joints.
2. **The gripper gRPC API already carries inputs.** `components/gripper/server.go:92-122`
   and `client.go:173-197` implement `GetCurrentInputs`/`GoToInputs`. They pass **raw
   float64 with no degree conversion**, unlike the arm API which round-trips through
   `Frame.ProtobufFromInput`. Gripper inputs are therefore radians end to end across the
   module boundary.
3. **Adding the DoF without wiring the inputs breaks the whole robot.**
   `robot/framesystem/framesystem.go:317` walks every frame with `len(DoF()) > 0` and
   hard-errors on `NotInputEnabledError` or any `CurrentInputs` error. Today the gripper
   is skipped because it has no DoF. The moment it has a joint, returning
   `errors.ErrUnsupported` fails `frameSystemService.CurrentInputs` for the entire
   machine — every arm motion request included. The model change and the input wiring
   must land together.

## Predicted planner behaviour

Traced through `motionplan/armplanning`. This prediction is what the experiment tests;
it is stated here so a surprising result is recognisable as a surprise.

1. `motion_chains.go:139-151` — `movingFS` walks the traceback for the frame **nearest
   world** that has DoF, then takes `FrameSystemSubset` of it. A move of the arm
   therefore pulls in the arm's entire subtree, gripper included. The jaw lands in the
   **moving** set, not the non-moving set.
2. `linearized_frame_system.go:22-70` — `computeJointSensitivities` perturbs each moving
   joint by 1% of its range and measures the resulting change in distance-to-goal. The
   jaw's perturbation moves the TCP not at all, so its raw ratio is approximately zero.
3. `linearized_frame_system.go:107` — `clampSensitivities` computes
   `min(1, max(minJog, raw*searchHeadroom))`. A zero-sensitivity joint does **not** get a
   zero search range; it is floored at `minJog`.

**Prediction:** small bounded jitter, not free roaming. Jaw travel of roughly
`minJog × range`, possibly sanded out by the smoothing pass. Falsifiable, and the
headline number the experiment reports.

**Secondary hazard, expected to be the more disruptive one:** `services/motion/builtin/builtin.go:634`
rejects a plan whose first trajectory step differs from `CurrentInputs` by more than
`defaultExecuteEpsilon = 0.01`. A jaw still mid-travel from an `Open()` when a plan begins
executing will trip that check and abort the **arm** move.

## Design

### 1. Model topology

`BuildGripperModel` gains a `jawDoF bool` parameter. When false it produces exactly
today's chain, byte for byte. When true:

```
mount ─ gripper_body_0 [mesh] ─┬─ tcp                        ← output_frames: ["tcp"]
                               │
                               └─ jaw_mount ─ jaw_joint ─ gripper_moving [mesh]
                                  (static)    (revolute)
```

(The leader gripper has two static body meshes, so its chain is
`gripper_body_0 ─ gripper_body_1 ─ {tcp, jaw_mount}`. The branch hangs off the last
body link either way.)

- **`jaw_mount`** — `Translation` and `Orientation` set from
  `Compose(toolFromGripperLink, gripperJointPose)`, computed in Go rather than
  hardcoded, so it cannot drift from `BuildGripperMeshes`.
- **`jaw_joint`** — revolute, `axis: {z: 1}`. `BuildGripperMeshes` applies
  `EulerAngles{Yaw: θ}` immediately after `gripperJointPose`, so local +Z is already the
  correct axis, and `jaw_mount` already carries the `toolFromGripperLink` correction.
- **`gripper_moving`** — parent `jaw_joint`, geometry offset `gripperMovingVisualPose`.
  Per `TestGripperModelJSON`, a link's geometry offset is measured from its **parent**
  frame, i.e. the joint's *output*. This is the same SVA convention already recorded in
  CLAUDE.md, and it is what makes the mesh swing with the joint.

Two leaves (`tcp` and `gripper_moving`) are legal only with `output_frames`, which
v1.0.0 accepts for exactly one entry (`model_json.go:135`). `Transform([θ])` therefore
remains the TCP and is invariant in θ, so `TestBuildGripperModelPlacesFrameAtTCP` and
`TestGripperFrameResolvesToTCPInFrameSystem` pass unchanged.

**Units trap.** `JointConfig.Min`/`Max` in the JSON are **degrees** —
`frame_json.go:152` applies `DegToRad` when building the `Limit`. `GripperJointMin`/`Max`
are radians. They must be converted on the way in. Given this repository's history with
unit and sentinel mix-ups, the parsed `DoF()[0]` limits are asserted in radians rather
than the written values being trusted.

### 2. Configuration

```go
type SO101SimulatedGripperConfig struct {
    GripperType    string `json:"gripper_type,omitempty"`
    MeshDetail     string `json:"mesh_detail,omitempty"`
    ArticulatedJaw bool   `json:"articulated_jaw,omitempty"`  // new, default false
}
```

Default false preserves current behaviour exactly. The flag applies to both gripper
types; the leader's trigger articulating under planner control is itself worth watching.
`components/gripper` always passes `false`.

### 3. State and unit wiring

`currentPct` / `targetPct` (0-100) remain the single source of truth, with the existing
background interpolator. Every command path — `Open`, `Grab`, `set_position`,
`GoToInputs` — funnels through `moveTo`.

The percent-to-angle bijection moves to `internal/geometry` so the model builder and the
component share one definition:

```go
func JawAngleFromPct(pct float64) float64  // GripperJointMin + pct/100*(Max-Min)
func JawPctFromAngle(rad float64) float64  // inverse, clamped to [0, 100]
```

It is currently inlined at `simulated_gripper.go:234-235`; that call site becomes the
helper.

- **`CurrentInputs`** — `[]Input{JawAngleFromPct(currentPct)}`, read under `g.mu`. Returns
  `errors.ErrUnsupported` when the flag is off.
- **`GoToInputs(steps...)`** — validates each step is length 1 and within
  `model.DoF()[0]`, converts to percent, and walks the steps sequentially through
  `moveTo`, mirroring the arm's `MoveThroughJointPositions`. Logs the whole batch before
  moving. Returns `errors.ErrUnsupported` when the flag is off.

### 4. Geometries: assert equivalence, do not unify

With an articulated model the jaw pose is computed twice — by the joint frame for
frame-system collision geometry, and by the `Compose` chain in `BuildGripperMeshes` for
`Geometries()`. That is a silent-drift risk.

The rejected fix was to serve `Geometries()` from `model.Geometries(inputs)`, as the
simulated arm does at `simulated.go:464-472`. It was rejected because it renames geometry
labels from `gripper_moving` to `<name>:gripper_moving`, which is observable in the 3D
viewer and churns existing tests for no experimental gain.

Instead `Geometries()` keeps calling `BuildGripperMeshes`, and a test sweeps θ across the
full range asserting the two paths agree to 1e-9. Same protection, no behaviour change.

### 5. Instrumentation

The constructor's `logger` parameter is currently discarded (`simulated_gripper.go:90`).
Store it.

- `GoToInputs` logs step count, θ first to last, summed `|Δθ|`, and the Linf distance
  from current inputs.
- A ring buffer of the last 8 batches, exposed as
  `DoCommand{"command": "get_jaw_trajectory"}`, returning the steps, `total_travel_rad`,
  and a **monotonic** receipt offset since construction — not wall-clock, so tests stay
  deterministic.
- `get_position` gains `jaw_angle_rad` and `dof`, so the viewer can read jaw state in one
  call.

### 6. Tests

Placed in `internal/geometry`, reusing the frame-system assembly from
`TestGripperFrameResolvesToTCPInFrameSystem` (`gripper_frame_test.go:169-205`).

| Test | Asserts |
|---|---|
| `TestArticulatedGripperModelShape` | `DoF() == 1`; limits equal `GripperJointMin`/`Max` **in radians** (the degrees-conversion guard); `Transform` invariant across θ; output frame is `tcp` |
| `TestArticulatedModelSurvivesSerialization` | DoF and joint limits survive the real gRPC round trip |
| `TestModelGeometriesMatchBuiltMeshes` | joint-frame pose and `Compose`-chain pose agree to 1e-9 across the θ range |
| `TestPlannerJawTravel` | the experiment — structural facts only, magnitudes reported |
| `TestExecuteEpsilonTripsOnMovingJaw` | a mid-travel jaw yields `InputsLinfDistance > 0.01` against a plan's start state |

`TestArticulatedModelSurvivesSerialization` earns its place specifically: a component
ships its kinematics as `ModelConfig().OriginalFile.Bytes`, so a joint that existed only
in the in-memory model would vanish server-side while every in-process test stayed green.
That is the exact failure CLAUDE.md documents for the arm's URDF, and an in-process
`Transform` check would not catch it.

`TestPlannerJawTravel` builds the arm plus articulated gripper frame system, plans a move
to `testGoal()` via `armplanning.PlanMotion`, and sweeps `PlannerOptions.RandomSeed` over
0-9. It **asserts** only that the trajectory contains a gripper column and that θ stays
within the joint limits. It **reports** per-step θ, total travel, and min/max via
`t.Logf` rather than asserting a threshold — the magnitude is the unknown the experiment
exists to determine. Once characterised, the reported numbers can be tightened into a
real assertion in follow-up work.

### 7. Documentation

- `docs/simulated.md` — add `articulated_jaw` to the `devrel:so101:simulated-gripper`
  attribute table (line 77), with the degrees-versus-radians joint-limit note and the
  execute-epsilon hazard. Both simulated models share this file; there is no separate
  `docs/simulated-gripper.md`.
- `CLAUDE.md` — the "0-DoF on purpose" gotcha becomes "0-DoF by default, opt-in 1-DoF via
  `articulated_jaw`". Its reasoning stays true and is now the thing being measured. Add
  the degrees-versus-radians `JointConfig` trap to the gotcha list.

## Out of scope

- The hardware gripper stays 0-DoF. It has a `jawAngle(ctx)` helper
  (`components/gripper/gripper.go:325`) that would make wiring cheap, but letting the
  planner command a physical jaw mid-move — with no grasp-retention logic — is a separate
  decision with a real dropped-object risk.
- Mimic joints (`buildMimicMappings`). The SO-101 has one moving jaw.
- Any grasp-retention guard on `GoToInputs`. Only relevant once hardware opts in.
- Tightening `TestPlannerJawTravel` into a threshold assertion. That needs the
  experiment's result first.

## Risks

| Risk | Mitigation |
|---|---|
| Adding DoF without input wiring breaks the whole machine's frame system | Model change and wiring land in one commit; flag defaults to false |
| Joint limits written in radians where degrees are expected | Test asserts parsed `DoF()[0]` limits in radians |
| Joint lost across the module gRPC boundary | `TestArticulatedModelSurvivesSerialization` |
| Jaw pose diverges between model and `Geometries()` | `TestModelGeometriesMatchBuiltMeshes` |
| Mid-travel jaw aborts arm moves via the execute-epsilon check | Reproduced and documented by `TestExecuteEpsilonTripsOnMovingJaw`; noted in `docs/simulated.md` |

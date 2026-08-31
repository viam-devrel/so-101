# Opt-in 1-DoF jaw on the simulated SO-101 gripper

**Date:** 2026-08-23
**Status:** Design, approved for planning
**Scope:** `components/simulated`, `components/gripper`, `internal/geometry`,
`docs/simulated.md`, `CLAUDE.md`

## Purpose

Make the simulated SO-101 gripper `InputEnabled` behind a config flag, with a real
revolute jaw joint in its kinematic model, in order to **measure what viam-server's
motion planner does with a degree of freedom that changes collision geometry but has
no effect on the goal pose.**

This is an experiment. The deliverable is a characterisation of planner behaviour, not
a feature. Nothing in the default configuration changes, and the hardware gripper's
kinematics are untouched.

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
   hard-errors on `NotInputEnabledError` (`:338`) or any `CurrentInputs` error. Today the
   gripper is skipped because it has no DoF. The moment it has a joint, returning
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
2. `linearized_frame_system.go:78` — `computeJointSensitivities` perturbs each moving
   joint by 1% of its range and computes

   ```go
   thisRatio := startDistance / math.Abs(myDistance-startDistance)
   ```

   This is **inverse** sensitivity: the *less* a joint affects distance-to-goal, the
   *larger* its ratio. RDK's own comment — "for cases where a small change has a small
   effect, we want to allow the walking algorithm to take bigger steps" — states the
   intent. The jaw does not move the TCP at all, so `myDistance == startDistance`, the
   denominator is zero, and `thisRatio` is `+Inf`. The code notes this is deliberate:
   "Go deals with the potential divide by 0. Representing `thisRatio` as infinite."
3. `linearized_frame_system.go:107` — `clampSensitivities` computes
   `min(1, max(minJog, raw*searchHeadroom))`, so the jaw's `+Inf` clamps to **1.0: its
   entire range.**
4. `node.go:193,198` seed the search at `minJog` of `.03` and `.25`; the full-freedom
   `minJog: 1` seed at `node.go:201` is gated on `len(rawRatios) > 6` and does **not**
   fire for the SO-101 (5 arm joints + 1 jaw = exactly 6). It does not need to — the jaw
   is already clamped to 1.0 in every seed.

**Prediction:** the jaw is handed **full-range search freedom** at the seeding stage.
This is the opposite of what a naive reading suggests, and it is the reason the
experiment is worth running.

**What remains genuinely unknown, and is what `TestPlannerJawTravel` measures:** seed
limits govern IK seeding, not the final trajectory. How much of that freedom survives
the RRT and the smoothing pass is not determinable by reading the code. The magnitude of
jaw travel in the emitted trajectory is the open question.

**Secondary hazard, expected to be the more disruptive one in practice:**
`services/motion/builtin/builtin.go:633-634` rejects a plan whose **first** trajectory
step (`i == 0` only) differs from `CurrentInputs` by more than
`defaultExecuteEpsilon = 0.01` (`builtin.go:77`). A jaw still mid-travel from an `Open()`
when a plan begins executing will trip that check and abort the **arm** move.

## Result

`TestPlannerJawTravel` (`internal/geometry/planner_jaw_test.go`), run against the arm +
articulated-gripper frame system, goal derived from FK of `jawGoalConfig = [0.8, -1.0,
1.0, 0.6, 0.3]`, 10 seeds per scenario:

```
FK-derived goal: (186.979256423666214459444745, -155.686818967522413004189730, 109.886552140576299052554532)

--- direct ---
seed  0:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  1:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  2:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  3:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  4:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  5:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  6:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  7:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  8:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000
seed  9:   2 steps | jaw travel 0.0000 rad (0.0% of range) | span [0.0000, 0.0000] | net 0.0000

--- obstructed ---
seed  0: 384 steps | jaw travel 0.1552 rad (8.1% of range)  | span [0.0000,  0.0776] | net 0.0000
seed  1: 346 steps | jaw travel 0.1728 rad (9.0% of range)  | span [0.0000,  0.1439] | net 0.1150
seed  2: 412 steps | jaw travel 0.2171 rad (11.3% of range) | span [0.0000,  0.1086] | net 0.0000
seed  3: 410 steps | jaw travel 0.3828 rad (19.9% of range) | span [0.0000,  0.1914] | net 0.0000
seed  4: 322 steps | jaw travel 0.2935 rad (15.3% of range) | span [0.0000,  0.2776] | net 0.2616
seed  5: 393 steps | jaw travel 0.2619 rad (13.6% of range) | span [0.0000,  0.1310] | net 0.0000
seed  6: 391 steps | jaw travel 0.1123 rad (5.8% of range)  | span [-0.0562, 0.0000] | net 0.0000
seed  7: 338 steps | jaw travel 0.0833 rad (4.3% of range)  | span [0.0000,  0.0417] | net 0.0000
seed  8: 326 steps | jaw travel 0.1707 rad (8.9% of range)  | span [-0.0853, 0.0000] | net 0.0000
seed  9: 305 steps | jaw travel 0.2044 rad (10.6% of range) | span [0.0000,  0.1022] | net 0.0000

--- PASS: TestPlannerJawTravel (37.39s)
    --- PASS: TestPlannerJawTravel/direct (0.83s)
    --- PASS: TestPlannerJawTravel/obstructed (36.56s)
```

**The full-range prediction did not hold, for either half of the pipeline it was checked
against.**

- **IK/seeding (`direct`):** zero jaw travel on all 10 seeds. With no obstacle the
  two-step trajectory is exactly start-and-goal, and IK never moves the jaw off its
  start value to reach the goal — unsurprising, since the goal is reachable at the jaw's
  starting angle and IK has no reason to pay for a change that costs nothing and buys
  nothing. This says nothing by itself about what the *search* does when it has to work;
  that is the second scenario.
- **RRT search (`obstructed`):** the jaw does move — every seed shows nonzero travel,
  from 4.3% of range up to 19.9% — but nowhere near the full range the sensitivity
  analysis licenses, and never in a consistent direction (`span` straddles zero across
  seeds; `net` is zero in 8 of 10, meaning the jaw returns to its start angle by the end
  of the trajectory even when it explored several tenths of a radian in the middle).
  Reading `clampSensitivities` alone would predict a joint free to swing anywhere in
  `[GripperJointMin, GripperJointMax]`; in practice the RRT samples the jaw somewhat more
  than the arm joints (matching the *ratio* the sensitivity code intends) but the smoothing
  pass pulls the trajectory back toward small, incidental jaw motion rather than large
  excursions. The naive reading of the prediction — "expect the jaw to swing wildly since
  it's unconstrained" — is contradicted by the measured trajectories.

Net effect for the design: the planner does not shove the jaw to a limit and leave it
there, and it does not leave the jaw pinned at its start value either. It wanders a few
percent to double digits of its range mid-trajectory in response to obstacles, then
mostly settles back near its start. This is a real, if modest, hazard for the mid-travel
`defaultExecuteEpsilon` check described above (a nonzero `net` value, seen in 2 of 10
obstructed seeds here, means the trajectory's *final* jaw angle differs from where it
started — relevant to any follow-up move that starts by reading `CurrentInputs`), but it
does not motivate clamping the jaw's range for planning purposes on this evidence alone.

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

The leader gripper has two static body meshes, so its chain is
`gripper_body_0 ─ gripper_body_1 ─ {tcp, jaw_mount}`. The branch hangs off the **last**
body link either way.

- **`jaw_mount`** — `Translation` and `Orientation` from
  `Compose(toolFromGripperLink, gripperJointPose)`, computed in Go rather than
  hardcoded.
- **`jaw_joint`** — revolute, `axis: {z: 1}`. `BuildGripperMeshes` applies
  `EulerAngles{Yaw: θ}` immediately after `gripperJointPose`, so local +Z is already the
  correct axis, and `jaw_mount` already carries the `toolFromGripperLink` correction.
- **`gripper_moving`** — parent `jaw_joint`, geometry offset `gripperMovingVisualPose`.
  Per `TestGripperModelJSON`, a link's geometry offset is measured from its **parent**
  frame, i.e. the joint's *output*. This is the same SVA convention already recorded in
  CLAUDE.md, and it is what makes the mesh swing with the joint.

**Sourcing the un-composed mesh.** `BuildGripperMeshes` returns the moving mesh already
posed in the tool frame (`movingPose` is the full compose), which is the wrong pose for a
joint-parented link. The articulated builder must **not** un-compose that result. It
calls `gripperMeshPLY(meshDetail, movingName)` directly — the existing unexported
accessor — and constructs the geometry at `gripperMovingVisualPose`. This keeps the two
posing paths genuinely independent, which is what makes §4's equivalence test meaningful
rather than tautological.

Two leaves (`tcp` and `gripper_moving`) are legal only with `output_frames`, which
v1.0.0 accepts for exactly one entry (`model_json.go:136`). `Transform([θ])` therefore
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
`components/gripper` always passes `false` — its only change is the new argument at the
`BuildGripperModel` call site (`gripper.go:142`) plus the shared-helper adoption in §3.

### 3. State and unit wiring

`currentPct` / `targetPct` (0-100) remain the single source of truth, with the existing
background interpolator.

**`set_position` must stay non-blocking.** `services/teleop/teleop.go:187-193` issues a
`set_position` DoCommand on the follower gripper on **every teleop tick** and depends on
it returning immediately; today it only sets `targetPct` (`simulated_gripper.go:249-257`).
Routing it through the blocking `moveTo` would stall the teleop loop. To make the two
behaviours explicit rather than incidental, split the existing `moveTo` in two:

```go
func (g *simulatedSO101Gripper) setTarget(pct float64)                  // non-blocking
func (g *simulatedSO101Gripper) awaitArrival(ctx context.Context) error // blocks
func (g *simulatedSO101Gripper) moveTo(ctx, pct) error                  // setTarget + awaitArrival
```

`Open`, `Grab`, and `GoToInputs` block via `moveTo`. `set_position` calls `setTarget`
only, preserving today's semantics and `docs/simulated.md`'s "the jaw interpolates
toward it".

The percent-to-angle bijection moves to `internal/geometry` so every call site shares one
definition:

```go
func JawRadiansFromPct(pct float64) float64  // GripperJointMin + pct/100*(Max-Min)
func JawPctFromRadians(rad float64) float64  // inverse, clamped to [0, 100]
```

It is currently inlined **twice** — `simulated_gripper.go:235` and the hardware gripper's
`jawAngle` at `components/gripper/gripper.go:318-319`. Both call sites adopt the helper.

- **`CurrentInputs`** — `[]Input{JawRadiansFromPct(currentPct)}`, read under `g.mu`. Returns
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
simulated arm does at `simulated.go:462-471`. It was rejected because it renames geometry
labels from `gripper_moving` to `<name>:gripper_moving`, which is observable in the 3D
viewer and churns existing tests for no experimental gain.

Instead `Geometries()` keeps calling `BuildGripperMeshes`, and a test sweeps θ across the
full range asserting the two independent paths agree to 1e-9. Same protection, no
behaviour change.

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

Split by package. `internal/geometry` tests are `package geometry`; component behaviour
cannot be tested there because `components/simulated` imports `internal/geometry` and the
reverse edge is forbidden by CLAUDE.md's dependency rule.

**`internal/geometry`** — reusing the frame-system assembly from
`TestGripperFrameResolvesToTCPInFrameSystem` (`gripper_frame_test.go:171-205`):

| Test | Asserts |
|---|---|
| `TestArticulatedGripperModelShape` | `DoF() == 1`; limits equal `GripperJointMin`/`Max` **in radians** (the degrees-conversion guard); `Transform` invariant across θ; output frame is `tcp`; both gripper types |
| `TestZeroDoFModelUnchanged` | `jawDoF=false` produces a model identical to today's — no DoF, same links, same TCP |
| `TestArticulatedModelSurvivesSerialization` | DoF and joint limits survive the real gRPC round trip |
| `TestModelGeometriesMatchBuiltMeshes` | joint-frame pose and `Compose`-chain pose agree to 1e-9 across the θ range |
| `TestJawAngleBijection` | `JawPctFromRadians(JawRadiansFromPct(p)) == p`; clamping at both ends; endpoints map to `GripperJointMin`/`Max` |
| `TestPlannerJawTravel` | the experiment — structural facts only, magnitudes reported |

**`components/simulated`**:

| Test | Asserts |
|---|---|
| `TestArticulatedJawConfig` | `articulated_jaw` defaults false; false ⇒ `CurrentInputs`/`GoToInputs` return `ErrUnsupported` and `Kinematics` is 0-DoF; true ⇒ both work and `Kinematics` is 1-DoF |
| `TestGripperCurrentInputsTracksJaw` | `CurrentInputs` reflects `currentPct` through the bijection after `Open`/`Grab` |
| `TestGoToInputsValidation` | wrong-length steps and out-of-limit θ are rejected; valid steps move the jaw and are applied in order |
| `TestSetPositionStaysNonBlocking` | `set_position` returns before the jaw arrives (the teleop-loop regression guard) |
| `TestJawTrajectoryDoCommand` | ring buffer records batches, reports `total_travel_rad`, caps at 8, monotonic offsets |
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
  attribute table (lines 76-79), with the degrees-versus-radians joint-limit note and the
  execute-epsilon hazard. Both simulated models share this file; there is no separate
  `docs/simulated-gripper.md`.
- `CLAUDE.md` — the "0-DoF on purpose" gotcha becomes "0-DoF by default, opt-in 1-DoF via
  `articulated_jaw`". Its reasoning stays true and is now the thing being measured. Add
  two new gotchas: the degrees-versus-radians `JointConfig` trap, and the inverse-
  sensitivity behaviour in `computeJointSensitivities` (a zero-effect joint gets *full*
  search range, not none).

## Out of scope

- The hardware gripper's kinematics stay 0-DoF. It has a `jawAngle(ctx)` helper
  (`components/gripper/gripper.go:315-320`) that would make wiring cheap, but letting the
  planner command a physical jaw mid-move — with no grasp-retention logic — is a separate
  decision with a real dropped-object risk. Its only changes here are the new
  `BuildGripperModel` argument and adopting the shared bijection.
- Mimic joints (`buildMimicMappings`). The SO-101 has one moving jaw.
- Any grasp-retention guard on `GoToInputs`. Only relevant once hardware opts in.
- Tightening `TestPlannerJawTravel` into a threshold assertion. That needs the
  experiment's result first.

## Risks

| Risk | Mitigation |
|---|---|
| Adding DoF without input wiring breaks the whole machine's frame system | Model change and wiring land together; flag defaults to false; `TestArticulatedJawConfig` |
| Joint limits written in radians where degrees are expected | `TestArticulatedGripperModelShape` asserts parsed `DoF()[0]` limits in radians |
| Joint lost across the module gRPC boundary | `TestArticulatedModelSurvivesSerialization` |
| Jaw pose diverges between model and `Geometries()` | `TestModelGeometriesMatchBuiltMeshes`, kept meaningful by the independent mesh-sourcing path in §1 |
| `set_position` made blocking, stalling the teleop loop | `TestSetPositionStaysNonBlocking` |
| Default-path behaviour changes | `TestZeroDoFModelUnchanged` |
| Mid-travel jaw aborts arm moves via the execute-epsilon check | Reproduced by `TestExecuteEpsilonTripsOnMovingJaw`; noted in `docs/simulated.md` |

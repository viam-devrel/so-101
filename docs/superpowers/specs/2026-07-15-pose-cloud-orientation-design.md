# Approach-axis orientation planning via PoseCloud

**Date:** 2026-07-15
**Status:** Approved design, pending implementation plan
**Affects:** `devrel:so101:arm`, `devrel:so101:simulated`

## Problem

The SO-101 is a 5-DOF arm. Six-DOF pose targets are reachable only on a 5-dimensional
submanifold of SE(3), so most combinations of target position and target orientation are
unreachable and produce `zero IK solutions produced`. Both arm models work around this by
defaulting the planner's goal metric to `position_only` (`arm.go:491`, `simulated.go:307`),
which matches the target point and accepts whatever orientation falls out.

That is a blunt instrument: it discards orientation entirely, including the orientation
the arm *could* have honored. We want the planner to respect the requested approach
direction where the geometry allows it.

## What this delivers (and does not)

This is **approach-axis planning with free roll**, not 6-DOF orientation planning.
The caller constrains where the tool *points* — a cone around the goal's orientation
axis — and the planner picks the roll about that axis.

Roll cannot be constrained simultaneously. This is forced by the orientation-vector
parameterization, not chosen; see "Theta encodes azimuth" below.

It remains a strict improvement over the status quo: `position_only` frees *both* the
axis and the roll; this pins the axis.

### Why an approach-axis cone suits this arm

Joints 2-4 pitch about parallel axes, so the tool axis is confined to the vertical plane
set by joint 1's pan, and joint 5 rolls about that axis. For a given target position, the
reachable orientations form a 2-parameter family (in-plane pitch + roll) out of 3. The
missing DOF is out-of-plane tilt. Since the direction of that shortfall is not knowable
in advance, the cone is isotropic.

## Verified planner semantics

Established empirically by running Go programs against `rdk v1.0.0`'s
`referenceframe.PoseCloud`, not from documentation. Each item below contradicts a
reasonable reading of the docs, and each is load-bearing.

### PoseCloud is a scoring shortcut, not a constraint

`armplanning.PlannerOptions.GetGoalMetric` (v1.0.0, `planner_options.go:177`) consults
`goal.GoalCloud.PoseInCloud(goal.Pose(), &dq)`. Inside the cloud the candidate scores
zero; outside, the full weighted metric applies with `orientScale = 100`. For a goal the
arm cannot reach exactly, landing in the cloud is the *only* path to success. A cloud that
never matches is therefore worse than no cloud at all: planning silently reverts to strict
6-DOF scoring and fails.

### Theta encodes tilt azimuth, not roll

`PoseInCloud` compares the *relative* pose, `PoseBetween(goal, candidate)`, where zero
deviation is the orientation vector `(0, 0, 1)`. For a pure tilt with no roll, the
relative `Theta` reports the **azimuth of the tilt**:

| tilt axis | tilt 5° | tilt 20° |
|---|---|---|
| X | `Theta = 90` | `Theta = 90` |
| Y | `Theta = 0` | `Theta = 0` |
| XY diagonal | `Theta = 45` | `Theta = 45` |

`Theta` measures roll only when tilt is zero. There is also a parameterization singularity
at identity: a 0.5° tilt about X reports `Theta = 0`, while 5° reports `Theta = 90`.

The relationship is exact: measured **`Theta = 90 - azimuth`, independent of the tilt's
magnitude**. Verified across azimuths 0/45/90/135/180/225/270 at tilts of 5°, 20° and 29° —
every reading matched the prediction to three decimals, and azimuth 270 reports exactly
`-180`.

Consequence: **`Theta` leeway must be exactly 180.** Azimuth sweeps `[0, 360)`, so the
*reported* `Theta` sweeps the full `[-180, 180]` — and only a leeway of 180 admits every
azimuth. Only 180 yields an *isotropic* cone.

The precise failure mode matters, because the tempting shorthand ("smaller Theta rejects
tilts") is **false** and a reader who tests it will falsify it. A smaller `Theta` does not
reject tilts outright; it silently carves out a wedge of tilt *directions*. Measured with a
30° cone against a 5° tilt:

| `Theta` leeway | azimuth 0 (about X) | azimuth 90 (about Y) | azimuth 270 |
|---|---|---|---|
| 0 | rejected | **accepted** | rejected |
| 90 | **accepted** | accepted | rejected |
| 170 | **accepted** | accepted | rejected |
| 180 | accepted | accepted | **accepted** |

So even `Theta: 0` accepts a 5° tilt about Y. Only the isotropy argument holds under
testing, which is why it — not the shorthand — is what `coneToPoseCloud`'s doc comment
carries, alongside a pointer to the azimuth-270 test that enforces it.

And because `Theta` mixes azimuth with roll, no value of it can bound roll while permitting
isotropic tilt. Hence: roll is free.

### OZ alone defines the cone, across the full 0-180 range

With `Theta: 180`, the tilt bound is enforced entirely by `OZ`, verified isotropic across
azimuths 0/45/90/135/180/270 (29° accepted, 31° rejected at a 30° cone). `OX`/`OY` set to
`sin(tolerance)` are redundant, so they are set to `1` (unconstrained). This is why the
Viam docs example reads `OX: 1, OY: 1, OZ: 0.1`.

The mapping stays correct **above 90°**, contrary to the intuition that `OZ = 1 - cos(a)`
reaching 1 makes it degenerate. Verified:

| cone | `OZ` | behavior |
|---|---|---|
| 90° | 1.000 | 89° accepted, 91° rejected |
| 120° | 1.500 | 119° accepted, 121° rejected |
| 177.40° | 1.998971 | 180° tilt still **rejected** |
| 177.44° | 1.999002 | saturated — every orientation accepted |

So `orientation_tolerance_deg` is meaningful across `[0, 180]`.

**Saturation begins at 177.4374°**, not 179. Acceptance is
`|1 - OZ_rel| <= OZ + epsilon` with a maximum possible deviation of 2.0, so everything is
accepted once `1 - cos(a) + 0.001 >= 2`, i.e. `cos(a) <= -0.999`, i.e.
`a >= acos(-0.999) = 177.4374°`. Verified: 177.40 still rejects a 180° tilt; 177.44
accepts it. At or above that, the cone legitimately means "orientation free" (comparable
to `position_only`).

### Epsilon is added to every leeway

`PoseInCloud` tests `math.Abs(between.Point().X) > pc.X + defaultEpsilon` with
`defaultEpsilon = 0.001`. It is **additive to every leeway**, not merely a floor on zeros.
Units are **millimeters** for `X`/`Y`/`Z` and **degrees** for `Theta`. So
`position_tolerance_mm: 1.0` yields a true bound of **1.001mm** — verified: 1.0009mm
accepted, 1.0015mm rejected.

At zero the same rule demands a match within 1 micron — verified: a 0.01mm offset is
rejected. Positional leeway is therefore mandatory, not optional.

The same additivity means the **effective cone is always slightly wider than requested**:
the orientation test reduces to `cos(tilt) >= cos(tol) - 0.001`, so

```
effective_cone = acos(cos(tol) - 0.001)
```

Verified against measured acceptance to four decimals: `0° → 2.5626°`, `1° → 2.7508°`,
`2.6° → 3.6509°`, `10° → 10.3247°`, `30° → 30.1144°`, `90° → 90.0573°`. The widening is
not confined to small values, and the effective cone is never narrower than ~2.5626°.

### The positional cloud is a per-axis box, not a radius

`X`/`Y`/`Z` are checked independently against `PoseBetween(goal, candidate)`, i.e. along
the goal frame's axes. `position_tolerance_mm: 1.0` therefore permits a worst-case corner
deviation of `sqrt(3) x 1.001 ≈ 1.73mm`, not 1mm — verified: a per-axis offset of 1.0005mm
(norm 1.7329mm) is accepted.

### ReferenceFrame does not exist post-bump

v0.123.0's `PoseCloud` had a `ReferenceFrame string` field (`pose_cloud.go:45`). **v1.0.0
removes it.** Post-bump the fields are exactly `X`, `Y`, `Z`, `OX`, `OY`, `OZ`, `Theta`.

The long doc comment about "evaluating in the reference frame of the object being asked to
move" survives in v1.0.0 as free-floating prose inside the struct, with no field attached —
which reliably reads as though the field still exists. It does not. Referencing it will not
compile, and it was never wire-serialized anyway: `commonpb.PoseCloud` carries only `x`,
`y`, `z`, `o_x`, `o_y`, `o_z`, `theta`.

Consequence for the `pose_cloud` escape hatch: `reference_frame` is rejected as an unknown
key because it is **not a `PoseCloud` field at all**, not because it is inert.

## The mapping

```go
&referenceframe.PoseCloud{
    X: posTolMM, Y: posTolMM, Z: posTolMM, // MUST be > 0; 0 means match-within-1-micron
    OX: 1, OY: 1,                          // deliberately unconstrained; OZ alone defines the cone
    OZ:    1 - math.Cos(rad(toleranceDeg)),
    Theta: 180,                            // MUST be 180: Theta encodes tilt AZIMUTH, not roll
}
```

These seven are the *complete* field set in v1.0.0; there is nothing else to set.

## Config surface

Both `SO101ArmConfig` (`arm.go:70`) and `SO101SimulatedArmConfig` (`simulated.go:42`)
gain the same two fields, so the simulated model remains a faithful stand-in for
motion-plan testing.

```go
// OrientationToleranceDeg is the half-angle, in degrees, of the cone of acceptable
// end-effector approach directions around the goal orientation. Roll about that axis
// is NOT constrained. Valid range [0, 180]. Zero or unset means the default, 30.
//
// The planner adds a 0.001 epsilon to the OZ leeway, so the cone actually enforced is
// acos(cos(tol) - 0.001) -- always slightly wider than requested (30 gives ~30.11) and
// never narrower than ~2.56. At or above 177.44 every orientation is accepted.
OrientationToleranceDeg float64 `json:"orientation_tolerance_deg,omitempty"`

// PositionToleranceMM is the per-axis positional leeway of the goal cloud, applied as a
// box along the goal frame's axes (worst-case corner deviation is sqrt(3) times this).
// It must be large enough for the IK solver to realistically land inside; a cloud only an
// exact match could satisfy means every move fails. Zero or unset means the default, 1.0.
PositionToleranceMM float64 `json:"position_tolerance_mm,omitempty"`
```

Defaults: `defaultOrientationToleranceDeg = 30.0`, `defaultPositionToleranceMM = 1.0`.

`Validate` rejects:
- `orientation_tolerance_deg < 0` or `> 180`
- `position_tolerance_mm < 0`

Zero and unset are indistinguishable in Go for a `float64` with `omitempty`, and both mean
"use the default". Consequence: **the friendly knob cannot express a near-exact cone** —
`0` is captured by defaulting. Near-exact orientation requires the raw `pose_cloud`
escape hatch. This is accepted; a near-exact cone on a 5-DOF arm fails almost everywhere
anyway (and the epsilon floors it at ~2.56° regardless).

No upper bound is enforced on `position_tolerance_mm`: large values are legitimate (the
Viam docs' own example uses 75mm). But because a large cloud makes the planner report
success far from the goal — a silent-wrong-answer, the failure mode this design otherwise
avoids — `resolveGoalCloudConfig` logs a warning above 10mm. It also warns when the cone is
saturated and accepts every orientation.

The saturation check must be implemented as `math.Cos(rad(tol)) <= -0.999`, **not** as a
literal `tol >= 177.44`: saturation actually begins at 177.4374°, so a literal 177.44
threshold would stay silent across `[177.4374, 177.44)`, which is saturated. The rounded
177.44 figure is for user-facing docs only, where "≥ 177.44 accepts any orientation" is
true as written.

`Validate` only ever returns errors; it has no logger, which is why these warnings live in
`resolveGoalCloudConfig` at construction instead.

### On the default of 1.0mm

`position_only` today drives the solver to converge as tightly as IK allows. A cloud lets
it stop as soon as it is inside, so positional accuracy loosens to the box above:
`sqrt(3) x 1.001 ≈ 1.73mm` worst case. That is *on the order of* the arm's repeatability
(estimated ±2mm; this figure is an unverified estimate, not a datasheet value), not
comfortably below it.

The default is therefore a **starting point to be tuned empirically during the
experiment**, not a derived value. The geometry is documented so the trade-off is
legible: smaller tolerance means better accuracy but the cloud matches less often, and a
cloud that never matches fails every move.

## Behavior

The cone is **on by default** (30°). This is a deliberate behavior change: a cone is
*stricter* than `position_only`, which accepts any orientation, so goals that plan today
may fail after this lands. That is accepted — the point is to plan orientation again.

There is **no fallback**. A failed cone plan fails the move. One code path, predictable,
and it forces explicit tuning rather than silently degrading.

### Precedence

| `extra` contains | Destination | Extras forwarded to planner |
|---|---|---|
| neither key | cone from config | all caller keys verbatim |
| `pose_cloud` only | raw cloud, replacing the cone | all caller keys **except** `pose_cloud` |
| `goal_metric_type` only | **no cloud** | all caller keys verbatim (incl. `goal_metric_type`) |
| **both keys** | — | **error** |

The `goal_metric_type` row avoids the trap where `position_only` sets `orientScale = 0`,
making any cloud meaningless. It also keeps today's behavior reachable per-call.

Both keys together is incoherent — a raw cloud plus a metric that ignores clouds — so it
errors rather than silently dropping one. Silent-drop is the exact failure mode this
design exists to eliminate.

`pose_cloud` is stripped from the forwarded extras because it is not a planner key.
`goal_metric_type` is forwarded because it is one. All other caller keys pass through,
preserving `arm.go`'s existing promise that callers "may override any key".

### The `pose_cloud` wire format

`extra` arrives via structpb, so every JSON number is a `float64`.

Accepted keys, matching `referenceframe.PoseCloud`'s own JSON tags exactly:
`x`, `y`, `z`, `ox`, `oy`, `oz`, `theta` — **lowercase**. All are optional; omitted fields
stay `0`, which per the epsilon rule means ~zero leeway. That is faithful passthrough and
the point of an escape hatch, but it is sharp, and is documented as expert-mode.

Note the casing is a genuine hazard: the Viam docs example renders these as `OX`/`OY`, and
the protobuf JSON spells them `o_x`/`oX`. Neither is accepted. Unknown keys are therefore
**rejected with an error** rather than ignored, so a typo or a copy-paste from the docs
fails loudly instead of silently producing a 1-micron cloud. `reference_frame` is rejected
under the same rule — it is not a `PoseCloud` field in v1.0.0.

## Components

New file `goal_cloud.go`:

```go
// goalCloudConfig is the resolved tolerance pair, letting both arm models share one
// code path despite separate config structs. Defaults are already applied.
type goalCloudConfig struct {
    OrientationToleranceDeg float64
    PositionToleranceMM     float64
}

// resolveGoalCloudConfig applies defaultOrientationToleranceDeg / defaultPositionToleranceMM
// to zero or unset values, and emits the two construction warnings (see "Config surface").
// It is the single owner of both defaulting and those warnings, so each constructor is one
// call and neither model re-implements the thresholds. Validate has no logger, which is why
// the warnings live here rather than there.
func resolveGoalCloudConfig(tolDeg, posTolMM float64, logger logging.Logger) goalCloudConfig

func coneToPoseCloud(cfg goalCloudConfig) *referenceframe.PoseCloud

// goalPath reports which precedence row buildMoveDestination took, so the caller can
// wrap failures appropriately without re-inspecting extra (which would duplicate the
// precedence logic in both models and let it drift).
type goalPath int

const (
    pathCone goalPath = iota // cone from config
    pathRawCloud             // caller-supplied pose_cloud
    pathMetricType           // caller-supplied goal_metric_type; no cloud sent
)

// buildMoveDestination applies the precedence table.
//
// originFrame is the FULLY-FORMED frame name (e.g. "myarm_origin"); this function does
// not append "_origin". It is passed through to NewPoseInFrame unmodified.
//
// On success returns a destination, a NEW extras map (the caller's map is never mutated),
// and the path taken. On error returns (nil, nil, 0, err).
func buildMoveDestination(originFrame string, pose spatialmath.Pose, cfg goalCloudConfig,
    extra map[string]interface{},
) (*referenceframe.PoseInFrame, map[string]interface{}, goalPath, error)
```

`buildMoveDestination` errors when:
- `extra` contains both `pose_cloud` and `goal_metric_type`
- `pose_cloud` is not a `map[string]interface{}`
- `pose_cloud` contains an unknown key (including `reference_frame`, `OX`, `o_x`, …)
- a `pose_cloud` value is not a `float64`, or is NaN or Inf
- a `pose_cloud` value is negative

Both models embed `resource.AlwaysRebuild` and have **no `Reconfigure`** — they are
rebuilt on config change. `goalCloudConfig` is therefore resolved once at construction and
never mutated, so it needs no mutex despite `so101` holding an `mu sync.RWMutex` for other
state.

`MoveToPosition` shrinks to a `buildMoveDestination` call plus the existing `motion.Move`,
collapsing the near-identical bodies currently duplicated across `arm.go` and
`simulated.go` rather than doubling them. The call site keeps today's frame expression:

```go
dest, planExtra, path, err := buildMoveDestination(
    fmt.Sprintf("%v_origin", s.Name().Name), pose, s.goalCloud, extra)
```

## Data flow

```
arm.MoveToPosition(pose, extra)
  └─ buildMoveDestination → (PoseInFrame{pose, GoalCloud}, planExtra)
  └─ motion.Move(MoveReq{Destination, Extra})
        └─ PoseInFrameToProtobuf serializes GoalCloud
   ═══════════════ module gRPC boundary ═══════════════
        └─ ProtobufToPoseInFrame decodes GoalCloud     ← needs viam-server >= 0.127.0
        └─ armplanning GetGoalMetric → PoseInCloud → score 0 inside the cloud
```

The module only ever *sends* the cloud; viam-server plans. The module's own RDK version
therefore has no bearing on whether the cloud is honored — that depends solely on the
server.

### Minimum viam-server version: 0.127.0

Established by bisecting RDK releases. Two independent pieces must both be present:

| RDK version | `armplanning` reads `GoalCloud` | `ProtobufToPoseInFrame` decodes `GoalCloud` |
|---|---|---|
| v0.123.0 – v0.124.0 | no | no |
| v0.125.0 – v0.126.1 | **yes** | no |
| **v0.127.0** and later | yes | **yes** |

The v0.125–v0.126 band is the trap: the planner consults a field that proto never
populates, so the cloud is silently inert. **v0.127.0** is the first release where the
full path works — verified by diffing `ProtobufToPoseInFrame`, which gains
`result.GoalCloud = PoseCloudFromProto(proto.GetGoalCloud())` in v0.127.0 and lacks it in
v0.126.1.

viam-server releases track RDK version numbers, so this maps to viam-server **0.127.0**.
The implementer should confirm that release exists before publishing the constraint.

Enforced by declaring the minimum Registry-side. Nothing lands in `meta.json`: its schema
(`cli/module.schema.json`) defines no version-constraint property, so this is **not** a
code or workflow change.

That makes it a **manual post-publish step**, not something `.github/workflows/deploy.yml`
or `viamrobotics/build-action` can carry: after the release uploads the version, set the
minimum viam-server version on the module's Registry page. The plan must include it as an
explicit release-checklist item, or the constraint silently never gets applied and users
land in exactly the silent-ignore failure this design is trying to make loud.

## RDK bump to v1.0.0

Bundled into this work, as its own commit ahead of the feature so a bisect can separate
"the upgrade broke it" from "the cone broke it".

Verified in a throwaway worktree: `go build ./cmd/module/ .`, `go test ./cmd/module/ .`
(`ok so_arm 2.721s`), and `go vet` all pass with **zero source changes**.

`go.mod` moves:

| dep | from | to |
|---|---|---|
| `go.viam.com/rdk` | v0.123.0 | v1.0.0 |
| `go.viam.com/api` | v0.1.539 | v0.1.566 |
| `go.viam.com/utils` | v0.4.19 | v0.6.6 |
| `go` directive | 1.25.1 | 1.25.9 |

Nothing in CI pins a Go version, and 1.25.1→1.25.9 is a patch-level move within 1.25.x
that modern toolchains auto-resolve. Low risk, but the plan must verify CI rather than
assume.

The bump's payoff is testing: v0.123.0 has the `PoseCloud` type but no `PoseInCloud`, and
its `ProtobufToPoseInFrame` silently drops `GoalCloud` on decode. v1.0.0 has both, so cone
semantics and the proto round-trip become directly assertable.

`CLAUDE.md`'s header pins `go.viam.com/rdk v0.123.0` and must be updated.

## Error handling

Fail-loudly makes the error message the entire UX. The confusing failure mode: on an
outdated viam-server the cloud is dropped, the planner reverts to strict 6-DOF scoring,
and essentially every move fails with no hint why.

The cone-specific wrapping applies **only on the cone path**, because on the other two
paths it would misdirect — telling a caller who just passed `position_only` to pass
`position_only`, or blaming a config tolerance that had no bearing on a raw-cloud failure.
The `goalPath` value returned by `buildMoveDestination` selects the wrapping, so the
precedence logic is never re-derived at the call site:

| `goalPath` | error |
|---|---|
| `pathCone` | wrapped with the tolerance and all three remedies (below) |
| `pathRawCloud` | wrapped noting the caller-supplied cloud; no tolerance cited |
| `pathMetricType` | returned unwrapped — today's behavior |

```go
// pathCone only
return fmt.Errorf(
    "move to position failed with orientation_tolerance_deg=%.1f (approach-axis cone): %w; "+
    "widen the tolerance, pass extra {\"goal_metric_type\": \"position_only\"} to ignore "+
    "orientation, or check that viam-server is >= 0.127.0 (older servers ignore goal clouds)",
    tolDeg, err)
```

The effective cloud is logged at debug on each move.

## Testing

`goal_cloud_test.go` — pure, no hardware, no motion service:

- **Cone semantics via `PoseInCloud`** (available post-bump), asserting real behavior
  rather than freezing `OZ == 0.13397`:
  - 30° cone: 29° accepted, 31° rejected, across azimuths 0/45/90/135/180/270.
  - Above 90°: cone 90 → 89 accepted / 91 rejected; cone 120 → 119 accepted / 121
    rejected.
  - Saturation boundary: cone 177.40 → 180° tilt **rejected**; cone 177.44 → 180°
    accepted. Guards the exact `acos(-0.999)` threshold against drift.
  - Effective cone widening: `acos(cos(tol) - 0.001)`, asserted against measured
    acceptance for 0 → 2.5626, 2.6 → 3.6509, 30 → 30.1144, 90 → 90.0573.
  - Roll free: rolls 0/45/90/179 accepted at zero tilt.
  - Tilt+roll combined (verified values): at a 30° cone, `tilt 20` and `tilt 29` accepted
    at rolls 0/70/170; `tilt 31` rejected at rolls 0/70/170. Roll does not shift the tilt
    bound.
  - Positional box at `posTol = 1.0`: 1.0009mm accepted, 1.0015mm rejected (the additive
    epsilon); per-axis 1.0005mm on all three axes accepted at norm 1.7329mm (box, not
    radius).
- **Regression guards** encoding *why* the mapping looks as it does, so a future
  "simplification" fails loudly:
  - `Theta: 0` makes a 30° cone reject a 5° tilt.
  - Zero positional leeway rejects a 0.01mm offset.
- `resolveGoalCloudConfig` defaulting when zero/unset, and leaving non-zero values alone.
- The full four-row precedence table, including that both keys together errors, that
  `pose_cloud` is stripped while `goal_metric_type` is forwarded, that the caller's
  `extra` map is not mutated, and that each row returns the expected `goalPath`.
- `pose_cloud` parsing: lowercase keys accepted; `OX`/`o_x`/`reference_frame`/unknown keys
  rejected; non-`float64`, NaN, Inf, and negative values rejected.
- `Validate` rejecting `orientation_tolerance_deg` `< 0` and `> 180`, and
  `position_tolerance_mm < 0`.

**Proto round-trip**: `PoseInFrameToProtobuf` → `ProtobufToPoseInFrame` preserves
`GoalCloud` field-for-field. Directly guards the CLAUDE.md module-boundary gotcha — that a
component ships over gRPC and in-memory state does not survive. In-process assertions
would miss it.

**Both models**: `MoveToPosition` sends a destination carrying the expected `GoalCloud`,
using `testutils/inject/motion.MotionService`'s
`MoveFunc func(ctx, req motion.MoveReq) (bool, error)` to capture the `MoveReq`. The repo
has no existing `MoveToPosition` tests or motion mocks, but the inject helper exists on
the pinned version, so this needs no new infrastructure.

## Docs

- **README**: `orientation_tolerance_deg` and `position_tolerance_mm` in both arm model
  sections, each stating that roll is unconstrained, that the positional tolerance is a
  per-axis box (corner ≈ `sqrt(3)` times the value), that `0` means the 30° default rather
  than exact, that the enforced cone is `acos(cos(tol) - 0.001)` and so always slightly
  wider than requested, that `>= 177.44` means orientation-free, and that goal clouds
  require **viam-server >= 0.127.0**. Document the `pose_cloud` escape hatch with its exact
  lowercase keys and the warning that docs-style `OX` casing is rejected.
- **CLAUDE.md**: update the RDK version, and add a gotcha covering `Theta`-encodes-azimuth
  (forcing `Theta: 180`), `OZ`-alone-defines-the-cone (valid across 0-180, saturating at
  177.44), the additive `0.001` epsilon and the ~2.56° effective-cone floor it implies,
  that `ReferenceFrame` was removed in v1.0.0 (v0.123.0 had it) leaving
  `X, Y, Z, OX, OY, OZ, Theta` as the complete field set, and that goal clouds need
  viam-server >= 0.127.0 (v0.125-v0.126 read the field but never decode it). Without it,
  the next reader will "simplify" `Theta: 180` to `0` and break every move.

## Out of scope

Deliberately excluded; all available later, none needed to try this:

- Version detection or a fallback ladder (cone → wider cone → `position_only`).
- Separate `x`/`y`/`z` leeway knobs on the friendly surface.
- A shared-config refactor unifying the two arm models beyond `goalCloudConfig`.
- Constraining roll via a separate `OrientationConstraint` in `Constraints`.

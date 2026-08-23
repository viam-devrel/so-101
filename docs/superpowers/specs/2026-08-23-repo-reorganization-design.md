# Repository Reorganization

**Date:** 2026-08-23
**Status:** Approved (revised after review)

## Problem

`so-101` has grown to 46 Go files / 13.4k lines in a single flat `package so_arm` at the
repository root (19 non-test files, 27 test files), plus a 1098-line `README.md` covering seven
models. Three concrete frictions motivated this work:

1. **Finding code by hand** — 46 undifferentiated filenames give no signal about where a
   concern lives.
2. **Giant files** — `arm.go` (1326 lines) and `calibration.go` (1205 lines) are 18.9% of the
   codebase. Finding the right file still leaves a long hunt.
3. **Docs sprawl** — a single 1098-line README makes it hard to point a user at the section
   they need.

Explicitly *not* problems: agent context bloat, test count, comment density. See
[Non-goals](#non-goals).

## Constraints discovered during analysis

These shaped the design and must survive it.

### `arm/` and `meshes/` are asset directories, not Go source

The two most natural package names are already taken. `arm/` holds the vendored URDF, its
per-link collision meshes, `gen_collision_meshes.py`, and the Apache-2.0 `SO-ARM100-LICENSE`;
`meshes/` holds the GLB/PLY assets and two generator scripts.

### `go:embed` forbids `..` traversal — and nothing else relevant here

Embed patterns cannot escape the embedding package's directory upward. They **can** descend
into subdirectories, including subdirectories that are themselves Go packages. Verified
empirically: a root-package `//go:embed internal/geometry/meshes/x.json` and
`//go:embed internal/geometry/meshes` both compile, even with `.go` files present in the target
tree (directory embeds silently skip them).

This is weaker than it first appears and it is what makes the sequencing below work: a
root-level `meshes.go` can keep embedding assets after they move under `internal/geometry/`,
before that package's Go code exists.

### `VIAM_MODULE_ROOT` resolves against the tarball root, not the Go package

`arm.go:334` reads the URDF via
`filepath.Join(os.Getenv("VIAM_MODULE_ROOT"), "arm", "so101.urdf")`. `VIAM_MODULE_ROOT` is set
by viam-server to the module's unpack root, so this path is stable relative to the *tarball* and
independent of Go package layout. Runtime assets must NOT move into a component package; they
belong at a fixed top-level path that the Makefile packs.

Note that `makeSO101ModelURDF` carries this `os.Getenv` call with it into `internal/geometry`.

### Tests assume the repository root is the working directory

Nine call sites across six files set `t.Setenv("VIAM_MODULE_ROOT", ".")` and expect to find
`./arm/so101.urdf`: `arm_frame_alignment_test.go:105,139`, `arm_collision_coverage_test.go:48,119`,
`model_frame_test.go:13,33`, `arm_serialization_test.go:22`, `arm_oob_test.go:24`,
`simulated_test.go:84`. Go runs each package's tests from that package's own directory, so `"."`
breaks the moment those tests move — latent breakage that exists today independent of this work.

### `_test.go` files are package-private, and the shared fakes reach into unexported state

Thirteen helpers are shared across test files that land in different packages. Critically, the
registry cluster cannot simply be relocated: `bus_session_test.go:23` assigns `r.busFactory`,
an **unexported field** (`registry.go:81`). Any helper outside `internal/controller` is barred
from it, and moving the helper into a package that `internal/controller`'s own in-package tests
import would create an import cycle.

Sorted by what each actually requires:

| Helpers | Requires | Resolution |
|---|---|---|
| `newFakeTransport`, `fakeTransport`, `encodeWordLE` | only `feetech` types | → `internal/testfake` |
| `fakeServoOps` | the `servoOps` interface, which is being exported anyway | → `internal/testfake` |
| `testRegistry`, `acquireTestHandle`, `testHandle`, `testRegistryAndHandle`, `cloneCalibration`, `shortTimeoutConfig` | unexported `ControllerRegistry.busFactory` | stay in `internal/controller`; `calibration_recording_test.go` reaches them through a new **exported** seam (below) |
| `newFakeIO` | unexported `manualIO` (`manual_mode.go:48`) | stays in `components/arm` — both consumers land there |
| `toInputs` | nothing | stays with its two consumers, both → `internal/geometry` |
| `newTestSimArm`, `testGoal` | unexported `simulatedSO101` / `goalCloudConfig` | resolved by the test split below |

**New requirement this creates:** `internal/controller` must expose an exported bus-factory
injection seam (a `WithBusFactory(...)` option on `AcquireController`, or an exported field),
because `calibration_recording_test.go:18` builds a `so101CalibrationSensor` — which lands in
`components/calibration` — using `testHandle(t, ft)` and `fakeTransport`. This is a production
API addition, not a test-only hack, and it must be designed rather than improvised.

### Two test files straddle three future packages

`arm_move_to_position_test.go` constructs `&so101{...}` (lines 41, 59, 81, 106, 128 →
`components/arm`) **and** `&simulatedSO101{...}` (line 142 → `components/simulated`), both with
unexported fields, and calls `resolveGoalCloudConfig` (→ `internal/planning`) at five sites. No
single package can hold it; it must be split, and `goalCloudConfig` must be exported with either
settable fields or a constructor so both halves can build one.

`model_frame_test.go` has the same problem smaller: `makeModelFrame` stays in `components/arm`
while `makeSO101Model` (line 41) moves to `internal/geometry`.

### Cross-file coupling is low, but the first-pass analysis produced a false positive

An automated pass found 88 apparent cross-file references to unexported symbols; most are false
positives on common identifiers (`loop`, `positions`, `running`, `run`, `so101`, `isMoving`).
One survived into an earlier draft of this spec and is corrected here: `jawAngle` is a **method**
on `*so101Gripper` (`gripper.go:480`), while `simulated_gripper.go:233` declares an unrelated
local variable of the same name. There is no cross-file reference.

The genuine `gripper.go` → `simulated_gripper.go` shared symbols are `buildGripperMeshes`,
`buildGripperModel`, `gripperJointMin`, `gripperJointMax`, `leaderGripper`, `followerGripper`,
`highDetail`, and `lowDetail` — all of which move to `internal/geometry`.

`arm.go` → `simulated.go` shares `computeOOBPosition` and `makeSO101Model`, likewise moving.

`manual_mode.go` and `arm.go` are mutually dependent (`arm.go:129` declares `ManualModeConfig`;
`manual_mode.go:348 resolveManualParams` consumes it; `arm.go:980-1007` drives `manualSession`;
`manual_mode.go:226 newControllerManualIO` takes a `*ControllerHandle`). They must stay in one
package.

### `Go`'s `_arm.go` suffix is reserved

Go treats `*_arm.go` as a `GOARCH=arm`-only build constraint — why the simulated arm lives in
`simulated.go` today. Package-internal filenames may use `arm` as a **prefix**, never a suffix.

## Design

### Package map

```
cmd/module/main.go              imports the six component/service packages

internal/servo/                 MotorCalibration, CalibratedServo, degree<->tick
                                conversion, coordinated-arrival profiles
                                (calibrated_servo + speed + motion_profile)     ~640 ln
internal/controller/            registry, manager, bus_session, calibration
                                file I/O, gripper_range + exported bus-factory
                                injection seam                                 ~1740 ln
internal/geometry/              so101.json + meshes/ (embedded assets), arm and
                                gripper model builders, gripper pose/detail
                                constants                                       ~640 ln
internal/planning/              goal_cloud (goalCloudConfig exported)           ~280 ln
internal/servocmd/              the servo_* DoCommand protocol (arm<->gripper)  ~190 ln
internal/testfake/              fakeTransport, fakeServoOps, RepoRoot()

components/arm/                 hardware arm + manual mode
components/gripper/             hardware gripper
components/calibration/         calibration sensor + motor setup
components/simulated/           simulated arm + simulated gripper
services/teleop/
services/discovery/             imports components/{arm,gripper,calibration}

assets/urdf/                    so101.urdf, meshes/*.stl, SO-ARM100-LICENSE
tools/                          gen_collision_meshes.py, gen_ee_frame.py,
                                gen_gripper_meshes.py
docs/                           per-model documentation
```

**`internal/servocmd` stays separate from `internal/controller` deliberately.** The gripper
imports the servo-command protocol but must *not* import the controller — owning no serial
connection, and driving its servo through the arm it depends on, is the entire point of the
gripper's design. Verified: `gripper.go` and `simulated_gripper.go` contain zero references to
`ControllerHandle`, `AcquireController`, or `globalRegistry`.

**`services/discovery` imports three component packages.** `discovery.go:186,198,215` reference
`SO101Model`, `SO101GripperModel`, and `SO101CalibrationSensorModel` in order to suggest
configs. This pulls in those packages' `init()` registrations as a side effect. Not a cycle, but
a real fan-in the map should show rather than hide.

**`internal/geometry` is buildable standalone** — every symbol below was traced and none
references controller or component state.

### What moves to `internal/geometry`

Larger than a first pass suggests. From `arm.go`: `makeSO101Model`, `makeSO101ModelURDF`,
`makeSO101ModelFrame` (:232), `makeSO101ModelFrameWithEEMarker` (:261), `computeOOBPosition`,
`eeMarkerGeometry`, `so101ModelJson` (:32), `numSO101CollisionMeshes`/`defaultMeshDecimation`
(:308-309). From `meshes.go`: `so101ArmModels` (:81), `so101ArmMeshParts` (:61),
`so101EEFrameMesh` (:73), `so101EEFrameMeshKey`, `gripperMeshPLY` (:46). From `gripper.go`:
`buildGripperMeshes`, `buildGripperModel`, `gripperTCPPose`, `gripperTCPFrame` (:412),
`toolFromGripperLink` (:337), `gripperBodyPose`/`gripperJointPose`/`gripperMovingVisualPose`
(:326-332), `gripperJointMin`/`gripperJointMax` (:356-357),
`leaderGripper`/`followerGripper` (:28-29), `highDetail`/`lowDetail` (:36-37).

Moving `gripperJointMin`/`gripperJointMax` is **required, not cosmetic**: `gripper_range.go:43`
reads them, and `gripper_range.go` lands in `internal/controller`. Leaving them in
`components/gripper` would make `internal/controller` import a component package — inverting the
layering this design exists to establish. `leaderGripper`, `followerGripper`, `highDetail`, and
`lowDetail` must be **exported** from geometry, since `gripper.go`'s config validation and
`simulated_gripper.go:233` both read them.

### Asset strategy

| Class | Assets | Constraint | Destination |
|---|---|---|---|
| **Embedded** | `so101.json`, `meshes/so101/*.glb` (six named directives, `meshes.go:14,17,20,23,26,32`), `meshes/gripper` (directory embed, `:41`) | must not traverse `..` | `internal/geometry/` (+ its `meshes/` subdir) |
| **Runtime-loaded** | `so101.urdf`, `meshes/*_collision.stl`, `SO-ARM100-LICENSE` | resolved from `VIAM_MODULE_ROOT` (tarball root) | `assets/urdf/` |

The URDF's internal mesh references are relative and directory-local — exactly five
`filename="meshes/*_collision.stl"` entries — and rdk resolves them against the URDF file's own
directory. So `so101.urdf` and its `meshes/` subdirectory move together with **no edit to the
URDF itself**.

**The Apache-2.0 license must ship.** `Makefile:34` currently tars the whole `arm/` directory,
so `arm/SO-ARM100-LICENSE` reaches users today. `SO-ARM100-LICENSE` therefore moves to
`assets/urdf/` alongside the meshes it covers — not to `tools/` with the generator. Dropping it
would ship vendored SO-ARM100 geometry without its attribution.

**Stop shipping `.DS_Store`.** `tar tzf module.tar.gz` shows `arm/.DS_Store` in the current
artifact. The new tar invocation excludes it.

Three places encode the old `arm/` name and all three change: `arm.go:334` (the `filepath.Join`),
`Makefile:28` (the `arm/so101.urdf arm/meshes/*.stl` prerequisite), and `Makefile:34` (the tar).
A further 22 Go comment references plus `README.md:70,515` and `CLAUDE.md` are prose.

### File splits

```
components/arm/          arm.go        config, constructor, resource lifecycle   ~400
                         motion.go     MoveToPosition / MoveToJointPositions /
                                       MoveThroughJointPositions, moveJoints,
                                       clampPositions                            ~450
                         manual.go     manual_mode.go + enter/exitManualLocked   ~430
                         status.go     Status, DoCommand, torque diagnostics     ~200

components/calibration/  sensor.go     config, constructor, Readings, dispatch   ~450
                         workflow.go   start / setHoming / record / save /
                                       abort / reset                             ~450
                         motorsetup.go SO101MotorConfigs, motorSetup*            ~300
```

`arm_move_to_position_test.go` and `model_frame_test.go` split along the same boundaries.

### Documentation split

```
README.md              overview, hardware setup, safety, quick start, links out   ~150 ln
docs/arm.md            devrel:so101:arm — config, calibration priority, motion,
                       approach-axis planning, manual mode, DoCommand, servo cmds
docs/simulated.md      devrel:so101:simulated + devrel:so101:simulated-gripper
docs/gripper.md        devrel:so101:gripper, including the TCP reference frame
docs/calibration.md    devrel:so101:calibration — state machine, file format, workflows
docs/teleop.md         devrel:so101:teleop
docs/discovery.md      devrel:so101:discovery
docs/setup-app.md      the so101-setup application
docs/troubleshooting.md
```

`docs/` already holds 17 design docs under `docs/plans/` (gitignored) and `docs/superpowers/`;
the new files sit alongside them.

`meta.json`'s seven per-model `markdown_link` values repoint from `README.md#model-devrelso101arm`
to `docs/arm.md`-style paths. The module schema (`https://dl.viam.dev/module.schema.json`) types
`markdown_link` as an unconstrained string, so non-README paths are structurally permitted.
`.viam-gen-info`'s `module_readme_link` stays `README.md` and is unaffected.

### Build and runtime changes

`build.sh`, `setup.sh`, `first_run.sh`, and `.github/workflows/deploy.yml` contain **no** path
references and are unchanged. `setup-app/` is likewise unaffected: it references only the model
string `devrel:so101:calibration` in prose and generic `DoCommand` plumbing, and no `DoCommand`
command names change.

**`Makefile`** — the root `*.go` prerequisite matches nothing after the split:

```make
GO_SRC         := $(shell find cmd internal components services -name '*.go' 2>/dev/null)
EMBEDS         := $(shell find internal/geometry -type f ! -name '*.go' 2>/dev/null)
RUNTIME_ASSETS := $(shell find assets -type f 2>/dev/null)
$(if $(GO_SRC),,$(error GO_SRC is empty — has the source layout moved?))
$(if $(RUNTIME_ASSETS),,$(error RUNTIME_ASSETS is empty — has assets/ moved?))

$(MODULE_BINARY): Makefile go.mod $(GO_SRC) $(EMBEDS)
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) \
	  go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

test:
	go test ./...          # works again once cmd/cli is deleted

module.tar.gz: meta.json $(MODULE_BINARY) first_run.sh build-app $(RUNTIME_ASSETS)
	...
	tar --exclude='.DS_Store' -czf $@ meta.json first_run.sh $(MODULE_BINARY) \
	  setup-app/build/ assets/
```

**The `$(if ...)` guards are load-bearing.** Verified on this platform: `$(shell find missingdir ...)`
prints to stderr and make **continues with exit 0**, yielding a silently empty prerequisite list.
Without the guards, a mid-migration state would stop triggering rebuilds on source and mesh
edits — and CLAUDE.md documents that rebuild trigger as a deliberate feature.

The build target also changes from `cmd/module/main.go` to `./cmd/module`; naming the file breaks
as soon as that package gains a second one.

**Runtime** — one literal: `filepath.Join(root, "arm", "so101.urdf")` →
`filepath.Join(root, "assets", "urdf", "so101.urdf")`.

**`cmd/module/main.go`** — one import becomes six; each component package keeps its own `init()`
registration, and `module.ModularMain` still receives the same seven `APIModel` entries. The
`resource.NewModel` triplets are unchanged, so no test assertion on model strings breaks.

**Generator scripts** move to `tools/` and take explicit paths; their current defaults (e.g.
`--so101-json` → `../so101.json`) break either way once `so101.json` moves.

**`CLAUDE.md`** cites roughly 30 file paths, nearly all invalidated. Rewritten as part of this
work, not deferred.

**`internal/` visibility is a non-issue.** The Go module path is the bare, unfetchable `so_arm`
(`go.mod:1`), so no external consumer exists or can exist.

## Non-goals

**Test pruning was considered and rejected.** 196 tests running in 3.3 seconds is cheap, and the
names encode specific contracts rather than padding (`TestURDFModelSurvivesSerialization`,
`TestConeBoundsTiltIsotropically`, `TestGripperTCPLiesBetweenTheJawTips`). `CLAUDE.md` documents
several bugs that *only* these tests caught, and notes that an in-process check would have missed
the serialization one. The real complaint — 196 tests across 27 undifferentiated files — is
addressed by moving tests alongside their packages.

**Comment pruning was considered and rejected.** Density runs 3–37%, and the dense files are
dense for reasons not recoverable from the code (`servo_commands.go` explaining why numeric
coercion is load-bearing rather than defensive: local dependencies deliver native ints, remote
ones deliver float64 through structpb). There is real tension with the standing preference for
terse comments, but a sweep is the wrong instrument.

Two targeted exceptions are in scope:

- **Delete `cmd/cli/`** — 1321 lines across nine files with nine conflicting `func main()`s,
  referencing config fields (`DefaultSpeed`, `DefaultAcceleration`) that no longer exist
  (`cmd/cli/debug_cli.go:25-26`). It breaks `go build ./...` and `go test ./...`, and therefore
  the Makefile's `test` target; the `CLAUDE.md` gotcha instructing `go test ./cmd/module/ .`
  exists solely to route around it.
- **Fix `TestEnumerateSerialPorts`** — `discovery.go:294-304` appends to a nil
  `var portPaths []string`, so a machine with no serial ports returns nil and
  `discovery_test.go:154`'s `NotNil` assertion fails. One-line fix.

## Sequencing

Each step is independently landable and ends with `go test ./...` green.

1. **Delete `cmd/cli/`.** Restores `go build ./...`, `go test ./...`, and `make test`; removes a
   `CLAUDE.md` gotcha. Zero risk.
2. **Documentation split** + `meta.json` repoint. Touches no Go code.
3. **Asset relocation.** `arm/` → `assets/urdf/` (with `SO-ARM100-LICENSE`), `meshes/` →
   `internal/geometry/meshes/`, `so101.json` → `internal/geometry/`, generators → `tools/`. Also
   rewrites the seven `go:embed` directives in `meshes.go` and the one in `arm.go` to the new
   *deeper* paths — legal because embeds may descend — plus the Makefile, the `VIAM_MODULE_ROOT`
   literal, and the nine `t.Setenv` sites. Those embed directives shorten again in step 4; that
   churn is the price of keeping step 3 independently landable.
4. **Extract the internal packages** — `servo`, `controller`, `geometry`, `planning`, `servocmd`,
   `testfake`. Includes exporting the controller's bus-factory seam and `goalCloudConfig`. The
   mechanical bulk of the work.
5. **Split components and services out of the root package**, including the `arm.go` /
   `calibration.go` file splits and the `arm_move_to_position_test.go` /
   `model_frame_test.go` test splits.
6. **Rewrite `CLAUDE.md`** against the final tree.

Step 3 is the only one that changes a runtime-resolved path and is the natural place for a
hardware smoke test.

## Verification

- `go build ./...` and `go test ./...` green after every step — and they must *run* at all,
  which they do not today.
- `gofmt -s -w .` and `go vet ./...` clean.
- `module.tar.gz` contents compared against a pre-reorganization baseline captured before step 1:
  same files at the same relative paths apart from the deliberate `arm/` → `assets/urdf/` move,
  **plus** the deliberate removal of `.DS_Store` and of `gen_collision_meshes.py` (now in
  `tools/`, not shipped). `SO-ARM100-LICENSE` must still be present.
- `use_urdf: true` exercised against real hardware after step 3 — the one path where a wrong
  `VIAM_MODULE_ROOT` join fails only at runtime.

# SO-101 Viam Module

`viam-devrel/so-101` — a Viam module for the LeRobot **SO-101 / SO-ARM101** robot arm
(TheRobotStudio open hardware: 6× Feetech STS3215 servos on one serial/USB bus @ 1 Mbaud).
Go module `so_arm`, Go 1.25, `go.viam.com/rdk v1.0.0`. It can drive either the leader or
the follower arm, or both as separate components for mirrored teleoperation.

## Models

| Model | API | File | Notes |
|---|---|---|---|
| `devrel:so101:arm` | arm | `arm.go` | Hardware arm — 5-DOF, servos 1-5 |
| `devrel:so101:simulated` | arm | `simulated.go` | Hardware-free arm; time-interpolated joint motion |
| `devrel:so101:gripper` | gripper | `gripper.go` | Hardware gripper — servo 6 |
| `devrel:so101:simulated-gripper` | gripper | `simulated_gripper.go` | Hardware-free gripper; time-interpolated open/close |
| `devrel:so101:calibration` | sensor | `calibration.go` | Servo-calibration workflow (state machine) |
| `devrel:so101:discovery` | service | `discovery.go` | Auto-detects connected SO-101 components |

All models register in `cmd/module/main.go`. Per-model config attributes are documented
in `README.md` (one `## Model …` section each).

## Hardware architecture

- The SO-101 has 6 servos on one serial bus: the **arm uses servos 1-5**, the **gripper
  uses servo 6**. The arm and gripper components therefore need the *same* serial port.
- The **shared controller** (`registry.go`, `manager.go`, `calibrated_servo.go`) is
  ref-counted — `GetSharedControllerWithCalibration` / `ReleaseSharedController` — so the
  arm and gripper on one SO-101 share a single serial connection instead of contending
  for the port. `SafeSoArmController` is the thread-safe wrapper with per-servo
  calibration applied.
- **Calibration** (`config.go`): priority is calibration file → servo registers →
  hardcoded defaults. The `devrel:so101:calibration` sensor runs a homing / range-recording
  workflow via `DoCommand` and persists calibration files.
- **Discovery** (`discovery.go`): enumerates serial ports, pings servos 1 and 6 to detect
  which components are present, and suggests configs.
- **Simulated models** (`simulated.go`, `simulated_gripper.go`) use none of the above —
  no controller, no serial port — and exist for developing, testing, and visualizing
  without a physical robot.

## Kinematics & meshes

- `so101.json` — SVA kinematics, `//go:embed`-ed, shared by both arm models. Derived from
  TheRobotStudio/SO-ARM100's URDF; its `tool` link is the end-effector (TCP) frame. This is
  the default kinematic source for both arm models.
- `arm/so101.urdf` — the alternate kinematic source, used when an arm model's `use_urdf`
  config attribute is set. Vendored from TheRobotStudio/SO-ARM100 (Apache-2.0, see
  `arm/SO-ARM100-LICENSE`), trimmed to the arm-only 5-DOF chain (servos 1-5). Each of the 5
  arm links carries **one merged collision mesh** (`arm/meshes/<link>_collision.stl`, ~88k
  tris / 4.2 MB total): rdk's URDF parser keeps only the *first* `<collision>` per link
  (`referenceframe/model_urdf.go`), but the upstream links are multi-part assemblies, so
  `arm/gen_collision_meshes.py` bakes every sub-part's `<collision>` origin into its vertices
  and concatenates them per link into a single mesh — otherwise a lone offset sub-part would
  survive per link (see `arm_collision_coverage_test.go`). Each mesh is generated relative to
  so101.json's per-link box pose, with a matching `<collision><origin>` in the URDF, so the
  geometry pose equals so101.json's — the 3D viewer places each Get3DModels GLB at the link's
  *geometry* pose, so without this the shared GLBs scatter by the box offsets under `use_urdf`
  (see `TestURDFGeometryPoseMatchesJSON`). Printable parts use the optimized `STL/SO101/Individual`
  print files (scaled mm→m; same local frame as the sim meshes); the servo body comes from
  `Simulation/SO101/assets` decimated to ~3k tris. The merged mesh is not decimated offline (that
  created sliver artifacts); instead `mesh_decimation_ratios` defaults to `0.9` per link (a light
  keep-~90% trim) when empty — `makeSO101ModelURDF` fills `numSO101CollisionMeshes` (5) copies of
  `defaultMeshDecimation` so no trailing mesh is left at full resolution (aggressive ratios reintroduced
  the slivers, hence 0.9). The URDF is authored to be a true drop-in for
  `so101.json` **in the file itself** — its links/joints use so101.json's names (`base`,
  `shoulder`, …; joints `1`-`5`) and it ends at a `tool` leaf at so101.json's TCP (joint-5
  output + 180° flip, via a `tool_mount`+`tool_joint` pair). This has to be baked into the file
  rather than patched in memory because the model ships to viam-server as the raw URDF bytes
  (see the module-boundary gotcha below); guarded by `TestURDFvsJSONFrameAlignment`,
  `TestURDFExposesJSONFrameNames`, and `TestURDFModelSurvivesSerialization`. So `use_urdf` is a
  true drop-in: identical kinematics/TCP **and** frame names as the JSON model — only the
  collision geometry changes (per-link meshes vs so101.json's primitives). `makeSO101ModelURDF`
  therefore just parses the URDF; it only re-serializes to SVA JSON for `visualize_ee_frame`
  (to carry the tool's placeholder marker box across the wire). Assets ship under `arm/` in
  `module.tar.gz` and are located at runtime via `VIAM_MODULE_ROOT`.
- `meshes/so101/*.glb` — arm-link meshes (Draco GLB) + `ee_frame.glb` (the colored EE
  coordinate-frame marker). Served via the arm's `Get3DModels`. `visualize_ee_frame` works
  in both JSON and URDF modes, since both end at a `tool` frame that gets the same EE-marker
  placeholder box either way.
- `meshes/gripper/*.ply` — gripper meshes (ASCII PLY — rdk's `spatialmath.Mesh` reads PLY
  only, not GLB). Served via the gripper's `Geometries()` as `spatialmath.Mesh`.
- Asset generators: `meshes/gen_ee_frame.py`, `meshes/gen_gripper_meshes.py`. Meshes are
  authored in **meters** (the viewer scales to the millimeter kinematics).

## setup-app

`setup-app/` is a SvelteKit web app — the `so101-setup` Viam application, a machine-picker
calibration wizard. It is bundled into `module.tar.gz` and needs **Node ≥ 20** to build.

## Build & release

- `make bin/arm` — builds just the Go module binary. Embedded assets (`so101.json`,
  `meshes/`) are listed in the target's prerequisites, so mesh-only edits trigger a rebuild.
- `make module.tar.gz` — the full module, including the `setup-app` build. `setup.sh`
  installs mise + Node 22 + pnpm for this.
- Release: publishing a GitHub release runs `.github/workflows/deploy.yml` →
  `viamrobotics/build-action`, which builds for darwin/arm64 + linux/{amd64,arm64} and
  uploads the version to the Viam registry.

## Tests & conventions

- Build/test with `go test ./cmd/module/ .` — **not** `./...`: `cmd/cli/` is a folder of
  standalone debug scripts (multiple `func main()` in one package) and won't compile as a
  package.
- Lint/format with `gofmt -s -w .` (the Makefile `lint` target); also run `go vet`.
- `TestEnumerateSerialPorts` (in `discovery_test.go`) fails on machines with no serial
  hardware — environment-dependent, not a regression.

## Gotchas

- A `*_arm.go` filename is treated by Go as a GOARCH=`arm`-only file — the simulated-arm
  model lives in `simulated.go` (not `simulated_arm.go`), its tests in `simulated_test.go`.
- The `arm/` directory holds vendored URDF assets (`so101.urdf`, its per-link merged
  collision meshes, `gen_collision_meshes.py`, the SO-ARM100 license) — not Go source,
  despite the name.
- rdk's URDF parser uses only the **first** `<collision>` element per link (falls back to
  `Collision[0]` after a capsule-pattern check; see `referenceframe/model_urdf.go`). A link
  authored as several `<collision>` sub-parts therefore keeps just one offset fragment. This
  is why `arm/so101.urdf` gives each arm link a single merged mesh (`arm/gen_collision_meshes.py`)
  — passing the raw upstream multi-part links renders incomplete/misaligned collision geometry.
- **A component ships its kinematics to viam-server as `ModelConfig().OriginalFile.Bytes`**
  (`referenceframe.KinematicModelToProtobuf`), NOT its in-memory `LinkConfig`s. So any in-memory
  edit to a parsed model is **lost across the module gRPC boundary** — viam-server rebuilds the
  model from the original file bytes. This is why the correct TCP/frame names are baked into
  `arm/so101.urdf` itself rather than patched in `makeSO101ModelURDF`: an in-memory graft/rename
  left `OriginalFile` as the raw (un-grafted) URDF, so the server used an EE ~98mm past
  so101.json's TCP and the gripper (parented to the arm) followed it — while every in-process
  unit test stayed green. `TestURDFModelSurvivesSerialization` guards this by round-tripping
  through the real gRPC (de)serialization (`KinematicModelToProtobuf`/`FromProtobuf`); an
  in-process `Transform` check would NOT have caught it. The one place we still rely on this: for
  `visualize_ee_frame` the tool needs an in-memory placeholder box, so that path re-serializes to
  **SVA JSON** (`spatialmath.GeometryConfig.MeshData` embeds the mesh bytes, so meshes survive) —
  a bigger payload (~5.9 MB vs ~4.4 MB URDF), hence only when the marker is enabled.
- The Viam `tool` frame in `so101.json` is rotated ~180° from the URDF `gripper_link`
  frame (the TCP convention). Gripper meshes are authored in `gripper_link` coords, so
  `gripper.go`'s `toolFromGripperLink` transform corrects them in `buildGripperMeshes`.
- Leader gripper meshes are printable STLs with no assembly transforms (the SO-ARM100
  repo ships only a *follower* URDF/sim model), so leader placement is best-effort. The
  follower gripper meshes come from the SO-ARM100 simulation assets + URDF and are accurate.
- pnpm 11 approves dependency install-scripts via `allowBuilds` (a map of dependency
  name to true/false) in `setup-app/pnpm-workspace.yaml` — not the older
  `onlyBuiltDependencies` list, and not `package.json`'s `pnpm` field. `setup.sh` pins
  `pnpm@11` (matching the `node@22` pin) so a future pnpm major can't silently break
  the build again.
- The **simulated arm** uses coordinated-arrival interpolation (all joints finish together),
  while the **hardware arm** uses independent-joint speed mode (each joint moves at the
  configured `speed_degs_per_sec`; joints with longer travel finish later). This is intentional:
  the RDK motion planner feeds dense waypoints that keep per-waypoint displacements small, so
  independent-joint motion stays smooth on planned paths. Also note: `acceleration_degs_per_sec_per_sec`
  is validated and stored on the hardware arm but not yet written to servo hardware (planned follow-up).
- **`referenceframe.PoseCloud` semantics are counterintuitive and were established
  empirically** (see `docs/superpowers/specs/2026-07-15-pose-cloud-orientation-design.md`;
  `goal_cloud.go` depends on all of it):
  - `PoseInCloud` compares the *relative* pose `PoseBetween(goal, candidate)`, where zero
    deviation is the orientation vector `(0,0,1)`.
  - **`Theta` encodes the AZIMUTH of a tilt, not roll.** Measured exactly:
    `Theta = 90 - azimuth`, independent of tilt magnitude (about X → 90, about Y → 0,
    azimuth 270 → -180). Since reported Theta sweeps the full `[-180, 180]`, **only a
    leeway of 180 admits every azimuth** — only 180 gives an isotropic cone. Beware the
    tempting shorthand "a smaller Theta rejects tilts": it is **false** (`Theta: 0` still
    accepts a 5° tilt about Y). Smaller values silently carve out a wedge of tilt
    *directions*. `TestConeBoundsTiltIsotropically`'s azimuth-270 case is what pins this.
    The cost: roll cannot be constrained alongside tilt, which is why this is
    approach-axis planning with **free roll**.
  - **`OZ` alone defines the cone**; `OX`/`OY` are set to `1` (unconstrained). Valid across
    `[0, 180]`, saturating at `acos(-0.999) = 177.4374412669`. Check saturation by
    comparing cosines (`cos(tol) <= -0.999`), never a rounded degree literal — every
    rounding is wrong in one direction or the other.
  - **`defaultEpsilon = 0.001` is ADDED to every leeway**, not just zeroed ones. So zero
    `X`/`Y`/`Z` demands a match within 1 micron (which no IK solution realistically
    achieves), and the effective cone is `acos(cos(tol) - 0.001)` — always slightly wider
    than requested, floored at ~2.56°.
  - The cloud only *zeroes the score* when a candidate is inside it. A cloud the solver
    cannot realistically land inside is **worse than no cloud**: planning reverts to strict
    six-DOF scoring and fails. `position_only` sets `orientScale=0`, so a cloud's
    orientation leeways stop mattering (its positional leeways still zero the positional
    residual, since `GetGoalMetric` consults `PoseInCloud` regardless of metric type).
    Passing both is rejected as incoherent rather than silently honoring one.
  - **There is no `ReferenceFrame` field.** v0.123.0 had one; v1.0.0 removed it while
    leaving the struct's reference-frame doc comment in place as free-floating prose, so it
    reads as though the field still exists. The complete field set is
    `X, Y, Z, OX, OY, OZ, Theta`.
- **Goal clouds require viam-server >= 0.127.0.** RDK v0.125.0–v0.126.1 read `GoalCloud` in
  `armplanning` but never decode it in `ProtobufToPoseInFrame`, so the cloud is silently
  inert there; v0.127.0 is the first release with both halves. The module only *sends* the
  cloud — its own RDK version has no bearing on whether the server honors it. The minimum is
  declared Registry-side; `meta.json` has no field for it.
- **`NewSO101`'s `goalCloud` wiring is untested** — it needs serial hardware, so no test
  constructs the hardware arm. Deleting `goalCloud: resolveGoalCloudConfig(...)` from
  `NewSO101` leaves the whole suite green, and the result is a zero-valued cloud that fails
  every move silently. `TestSimulatedConstructorWiresGoalCloud` covers the equivalent line
  in `newSimulatedSO101` and the shared `resolveGoalCloudConfig` contract, but the hardware
  constructor's line is guarded only by review.

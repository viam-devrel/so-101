# SO-101 Viam Module

`viam-devrel/so-101` — a Viam module for the LeRobot **SO-101 / SO-ARM101** robot arm
(TheRobotStudio open hardware: 6× Feetech STS3215 servos on one serial/USB bus @ 1 Mbaud).
Go module `so_arm`, Go 1.25, `go.viam.com/rdk v0.123.0`. It can drive either the leader or
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
  `arm/SO-ARM100-LICENSE`), trimmed to the arm-only 5-DOF chain (servos 1-5), with collision
  meshes pre-decimated offline (`arm/gen_decimated_meshes.py` — rdk's runtime mesh
  decimation is too slow to use routinely, hence the `mesh_decimation_ratios` config
  attribute normally stays empty). Its `tool` frame is grafted from `so101.json` so
  `use_urdf` is a true drop-in: identical kinematics/TCP to the JSON model, upgrading only
  the collision geometry from `so101.json`'s primitives to accurate per-link meshes (see
  `arm_frame_alignment_test.go`). Assets ship under `arm/` in `module.tar.gz` and are
  located at runtime via `VIAM_MODULE_ROOT`.
- `meshes/so101/*.glb` — arm-link meshes (Draco GLB) + `ee_frame.glb` (the colored EE
  coordinate-frame marker). Served via the arm's `Get3DModels`. `visualize_ee_frame` works
  in both JSON and URDF modes, since the grafted `tool` frame gets the same EE-marker
  placeholder either way.
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
- The `arm/` directory holds vendored URDF assets (`so101.urdf`, its meshes, the
  decimation script, the SO-ARM100 license) — not Go source, despite the name.
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

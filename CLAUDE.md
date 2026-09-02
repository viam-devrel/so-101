# SO-101 Viam Module

`viam-devrel/so-101` — a Viam module for the LeRobot **SO-101 / SO-ARM101** robot arm
(TheRobotStudio open hardware: 6× Feetech STS3215 servos on one serial/USB bus @ 1 Mbaud).
Go module `so_arm`, Go 1.25, `go.viam.com/rdk v1.0.0`. It can drive either the leader or
the follower arm, or both as separate components for mirrored teleoperation.

## Layout

```
cmd/module/            entrypoint; registers all seven models
components/arm/        devrel:so101:arm          — hardware arm, servos 1-5
components/gripper/    devrel:so101:gripper      — hardware gripper, servo 6
components/calibration/devrel:so101:calibration  — calibration sensor workflow
components/simulated/  devrel:so101:simulated + :simulated-gripper
services/teleop/       devrel:so101:teleop
services/discovery/    devrel:so101:discovery
internal/controller/   ref-counted shared serial bus (registry, manager, bus session,
                       calibration file I/O, gripper travel guard)
internal/servo/        servo primitives: MotorCalibration, CalibratedServo,
                       degree<->tick conversion, coordinated-arrival profiles
internal/geometry/     kinematic model builders + all embedded assets
internal/planning/     approach-axis goal clouds
internal/servocmd/     the servo_* DoCommand wire protocol
internal/testfake/     test doubles shared across package boundaries
assets/urdf/           runtime-loaded URDF + collision meshes + SO-ARM100 license
tools/                 mesh/URDF generator scripts
docs/                  one file per model; README.md is an index
```

**Dependency rule:** `internal/*` never imports `components/*` or `services/*`.
`components/gripper` does not import `components/arm` (it reaches the arm through the
`servocmd` DoCommand protocol, which may cross gRPC to a remote arm).
`components/simulated` imports neither hardware component. `internal/servocmd` imports
nothing else in the module — its `ServoOps` is deliberately a *structural* interface that
`*controller.ControllerHandle` satisfies implicitly, so no import edge exists.

Per-model config attributes are documented in `docs/<model>.md`, one file per model, linked
from `README.md` and from `meta.json`'s per-model `markdown_link` fields.

## Hardware architecture

- The SO-101 has 6 servos on one serial bus: the **arm uses servos 1-5**, the **gripper
  uses servo 6**. The arm and gripper components therefore need the *same* serial port.
- The **shared controller** (`internal/controller`) is ref-counted —
  `AcquireController` / `handle.Release()` — so every component on one SO-101 shares a
  single serial connection instead of contending for the port. A `ControllerEntry` owns the
  port: it holds the open bus in a `busSession`, the one calibration all servos normalize
  through, the reference count, and the mutex that serializes bus work. Components hold a
  `ControllerHandle`, which reaches the bus only through that entry. The last `Release()`
  closes the port and evicts the entry.
- **Calibration**: priority is calibration file → servo registers → hardcoded defaults.
  The `devrel:so101:calibration` sensor runs a homing / range-recording workflow via
  `DoCommand` and persists calibration files.
- **Discovery** enumerates serial ports, pings servos 1 and 6 to detect which components
  are present, and suggests configs. It imports the three hardware component packages for
  their model vars.
- **Simulated models** use none of the above — no controller, no serial port — and exist
  for developing, testing, and visualizing without a physical robot.

## Kinematics & meshes

All kinematic model construction and mesh handling lives in `internal/geometry`, shared by
both arm models and both gripper models.

- `internal/geometry/so101.json` — SVA kinematics, `//go:embed`-ed. Derived from
  TheRobotStudio/SO-ARM100's URDF; its `tool` link is the end-effector (TCP) frame. The
  default kinematic source for both arm models.
- `assets/urdf/so101.urdf` — the alternate kinematic source, used when an arm model's
  `use_urdf` config attribute is set. Vendored from TheRobotStudio/SO-ARM100 (Apache-2.0,
  see `assets/urdf/SO-ARM100-LICENSE`), trimmed to the arm-only 5-DOF chain (servos 1-5).
  Each of the 5 arm links carries **one merged collision mesh**
  (`assets/urdf/meshes/<link>_collision.stl`, ~88k tris / 4.2 MB total): rdk's URDF parser
  keeps only the *first* `<collision>` per link (`referenceframe/model_urdf.go`), but the
  upstream links are multi-part assemblies, so `tools/gen_collision_meshes.py` bakes every
  sub-part's `<collision>` origin into its vertices and concatenates them per link into a
  single mesh — otherwise a lone offset sub-part would survive per link (see
  `collision_coverage_test.go`). Each mesh is generated relative to so101.json's per-link
  box pose, with a matching `<collision><origin>` in the URDF, so the geometry pose equals
  so101.json's — the 3D viewer places each Get3DModels GLB at the link's *geometry* pose, so
  without this the shared GLBs scatter by the box offsets under `use_urdf` (see
  `TestURDFGeometryPoseMatchesJSON`). Printable parts use the optimized
  `STL/SO101/Individual` print files (scaled mm→m; same local frame as the sim meshes); the
  servo body comes from `Simulation/SO101/assets` decimated to ~3k tris. The merged mesh is
  not decimated offline (that created sliver artifacts); instead `mesh_decimation_ratios`
  defaults to `0.9` per link (a light keep-~90% trim) when empty — `armModelURDF` fills
  `numCollisionMeshes` (5) copies of `defaultMeshDecimation` so no trailing mesh is left at
  full resolution (aggressive ratios reintroduced the slivers, hence 0.9). The URDF is
  authored to be a true drop-in for so101.json **in the file itself** — its links/joints use
  so101.json's names (`base`, `shoulder`, …; joints `1`-`5`) and it ends at a `tool` leaf at
  so101.json's TCP (joint-5 output + 180° flip, via a `tool_mount`+`tool_joint` pair). This
  has to be baked into the file rather than patched in memory because the model ships to
  viam-server as the raw URDF bytes (see the module-boundary gotcha below); guarded by
  `TestURDFvsJSONFrameAlignment`, `TestURDFExposesJSONFrameNames`, and
  `TestURDFModelSurvivesSerialization`. So `use_urdf` is a true drop-in: identical
  kinematics/TCP **and** frame names as the JSON model — only the collision geometry changes
  (per-link meshes vs so101.json's primitives). `armModelURDF` therefore just parses the
  URDF; it only re-serializes to SVA JSON for `visualize_ee_frame` (to carry the tool's
  placeholder marker box across the wire).
- `internal/geometry/meshes/so101/*.glb` — arm-link meshes (Draco GLB) + `ee_frame.glb`
  (the colored EE coordinate-frame marker). Served via the arm's `Get3DModels`.
  `visualize_ee_frame` works in both JSON and URDF modes, since both end at a `tool` frame
  that gets the same EE-marker placeholder box either way.
- `internal/geometry/meshes/gripper/*.ply` — gripper meshes (ASCII PLY — rdk's
  `spatialmath.Mesh` reads PLY only, not GLB). Served via the gripper's `Geometries()`.
- Asset generators live in `tools/`. Meshes are authored in **meters** (the viewer scales
  to the millimeter kinematics).

**Two asset classes, opposite constraints.** Embedded assets (`so101.json`, `meshes/`) are
compiled in and must live inside `internal/geometry`, since a `//go:embed` pattern cannot
traverse `..` upward (it *can* descend into a subdirectory that is itself a package).
Runtime-loaded assets (`assets/urdf/`) are read via
`filepath.Join(os.Getenv("VIAM_MODULE_ROOT"), "assets", "urdf", ...)`; `VIAM_MODULE_ROOT` is
the *tarball* unpack root, so that path is stable relative to the tarball and independent of
Go package layout. **When an `//go:embed` directive changes, its `embed.FS.ReadFile` path
string must change in lockstep** — the FS's internal paths mirror the directive exactly, and
a mismatch compiles cleanly and fails only at runtime (`gripperMeshPLY` is the one such
call; `TestBuildGripperMeshes` covers it).

## setup-app

`setup-app/` is a SvelteKit web app — the `so101-setup` Viam application, a machine-picker
calibration wizard. It is bundled into `module.tar.gz` and needs **Node ≥ 20** to build.

## Build & release

- `make bin/arm` — builds just the Go module binary. `GO_SRC`, `EMBEDS`, and
  `RUNTIME_ASSETS` are computed with `$(shell find ...)`, guarded by `$(if ...,$(error ...))`
  — `find` on a missing directory prints to stderr and **exits 0**, so without the guards a
  moved directory would silently yield an empty prerequisite list and stop triggering
  rebuilds.
- `make module.tar.gz` — the full module, including the `setup-app` build. `setup.sh`
  installs mise + Node 22 + pnpm for this. The tar excludes `.DS_Store` and ships `assets/`.
- Release: publishing a GitHub release runs `.github/workflows/deploy.yml` →
  `viamrobotics/build-action`, which builds for darwin/arm64 + linux/{amd64,arm64} and
  uploads the version to the Viam registry.

## Tests & conventions

- Build/test with `go test ./...` (the Makefile `test` target).
- Lint/format with `gofmt -s -w .` (the Makefile `lint` target); also run `go vet`.
- Tests that need `VIAM_MODULE_ROOT` must use `testfake.RepoRoot()`, never `"."` — tests run
  from their own package directory, not the repo root.
- `go build ./...` drops a stray binary named `module` at the repo root (Go names it after
  `cmd/module`'s directory). It is gitignored.

## Gotchas

- **A component's `Close` really does close the serial port** once it was the last holder,
  and rdk closes a resource *before* constructing its replacement, so any reconfigure of a
  sole holder closes and reopens the port. Two components on one port (say an arm and a
  calibration sensor) keep it open for each other.
- **A servo goal write supersedes the previous one, so a waypoint stream must be paced by
  position, not just issued.** `SetGoals` overwrites the pending goal; an unpaced
  `MoveThroughJointPositions` puts its whole stream on the bus in tens of milliseconds and the
  servos execute only the final waypoint. That silently turned recorded-trajectory replay into
  a straight line from the first pose to the last (~3% of the recorded path on a 10 Hz nod)
  and made planned paths skip their obstacle avoidance, while every unit test stayed green —
  `components/simulated` loops over a *blocking* `MoveToJointPositions`, so playback worked in
  simulation. `dwellUntilNear` (`components/arm/motion.go`) fixes it by holding each
  intermediate waypoint until the arm is within `waypoint_lookahead_deg` of it. The
  lookahead is load-bearing in both directions: too tight and no waypoint is ever satisfied
  (servo steady-state error), too loose and the goal runs so far ahead the pacing stops
  gating. The dwell is deliberately *not* `WaitForServosToStop` — a stop at every waypoint is
  not a trajectory.
- **A servo stops short of its goal and reports `Moving = 0`, so the dwell needs a stall
  escape, not just a tolerance.** A position servo settles where error x P gain balances the
  load; measured droop is 4.6-5.2 deg at p_gain 16, 2.0-2.3 at 32, 1.26-1.49 at 48. Because
  `dwellUntilNear` gates on the **worst** joint, one joint parked past the lookahead makes
  every waypoint in the stream burn its full `DwellTimeoutMs`. Measured: a 1.5s trajectory
  took 31.3s; with the escape 6.5s, path deviation unchanged (3.8 -> 3.7 deg mean), so it
  removes dead time without cutting corners. The escape consults `AnyServoMoving` from the
  dwell's **second** poll onward -- Moving takes ~2ms to rise after a goal write, the same
  order as the position read before it, so a first-poll check can read the pre-write zero and
  abandon a waypoint before the arm started. A `Moving` read failure is deliberately NOT fatal
  (unlike the position read): it is auxiliary, and the deadline still bounds the wait. It is
  the backstop for any droop `DefaultLookaheadDeg` cannot cover -- notably a p_gain 16 arm,
  whose 4.8 deg droop would need a lookahead wider than a typical segment.
- **A fake bus has no gravity, so no unit test can observe the droop.** Every stall-escape
  test drives the `servosMoving` seam; `dwellTestArm` pins it to "always moving" so the
  lookahead and deadline tests keep pinning what they pin (the fake answers a `Moving` read
  with zero, which would otherwise send every dwell down the escape). Test residuals come from
  `shortOfGoal`, derived from `DefaultLookaheadDeg` -- they were once literals chosen against
  a 2.0 floor and silently stopped testing anything when it moved.
- **A waypoint whose dwell is provably satisfiable is SKIPPED without reading.** The dwell
  returns where the arm actually was, so if `|lastKnown - nextGoal|` is already inside the
  lookahead, its first read would return immediately and the read buys nothing. On a densified
  stream that is the common case -- 8x-interpolated 10 Hz frames give sub-degree segments
  against a multi-degree lookahead -- so this turns one read per waypoint into one per
  lookahead's worth of travel: ~8x fewer at the 25 deg/s derived profile, ~16x at an 80 deg/s
  arm. The bound must hold WITHOUT a fresh read, so it adds `speed * elapsed`, the furthest
  any joint could have travelled in any direction since that read (a joint may have reversed).
  That can only overstate the distance remaining, never understate it, so a skip is taken only
  where the real dwell would also have returned immediately -- and both terms grow as
  waypoints are skipped, so it stops skipping and reads again on its own.
  **It is only safe because of `MinExecutableSpeedDegsPerSec`.** Handing the commanded
  waypoint forward as the next travel reference is what CLAUDE.md previously warned against:
  a joint whose travel concentrated earlier in the path showed a near-zero delta and was
  floored to 1 step/s while its goal was still far away. The speed floor makes that
  unreachable, which is what unlocked this.
- **The dwell reads position and Moving in ONE transaction**
  (`GetJointPositionsAndMovingForServos`): the registers are 11 bytes apart, and every feetech
  transaction pays `enforceCommandGap`'s ~1ms while holding the bus lock, so payload size is
  nearly free but a second transaction is not. This DELETED a failure mode rather than making
  one fatal -- position could previously succeed while Moving failed, which the dwell had to
  treat as "assume still moving"; one read cannot land there. Moving is still only CONSULTED
  from the second poll (it takes ~2ms to rise after a goal write), just read for free earlier.
- **The dwell's poll wait is PREDICTED, not fixed, and that was worth 2.4s on a 6.9s
  recording.** A fixed tick quantises every waypoint to a multiple of itself: the arm needs
  some fraction of a tick to close the last of its gap and the dwell cannot notice until the
  next one. Measured on hardware by solving three replays as a system -- (poll 10ms, steps 7)
  11.9s, (10ms, 3) 10.4s, (4ms, 7) 9.5s -- the per-waypoint cost was 5.5ms at a 10ms tick and
  1.1ms at 4ms, against a base motion time of 8.9s that both fits agree on. So the fix is to
  sleep `(remaining - lookahead) / speed` capped at `dwellPollInterval` and floored at
  `minDwellPoll`, which beats shipping a shorter tick: it is more precise exactly when the arm
  is about to arrive and LESS chatty when it is far away. The floor is load-bearing -- the
  predicted wait goes to zero as the arm arrives, and CLAUDE.md's own note records a position
  read failing outright under sustained max-rate polling, which in this dwell is fatal.
  Over-estimating the speed only polls early (harmless); under-estimating degrades to the cap,
  i.e. the old behaviour.
- **8.9s of that 6.9s recording is still unexplained by any of this.** With waypoint overhead
  extrapolated to zero the base motion is 1.29x the recording, because arm-recorder's
  `derivedMaxVelRads` models total travel over total time and treats the deceleration and
  re-acceleration at every direction reversal as free. That is a separate, unfixed term.
- **The waypoint lookahead has TWO independent lower bounds, and a fixed value cannot clear
  both.** `LookaheadDegFor` derives it per move as `max(1.2 * v^2/2a, DefaultLookaheadDeg)`.
  The ramp term is the servo's deceleration distance: write the next goal after the servo is
  already inside that ramp and the stream decelerates and re-accelerates at every waypoint --
  on a 10 Hz recording that is ten velocity dips a second, and it reads as jerk. The floor
  term is the droop the arm parks with (4.8 deg at p_gain 16, 2.2 at 32, 1.5 at 48); below it
  the dwell is never satisfied at all. The old fixed `2.0` cleared only the second, and so was
  correct only below ~45 deg/s at the default acceleration -- which is why a recorded nod
  (25.2 deg/s, 0.64 deg ramp) replayed smoothly and a wave (58.8 deg/s, 3.45 deg ramp) felt
  jerky on hardware until the lookahead was raised to 4. Derived **per move**, not per
  component, because `arm.MoveOptions` can cap the speed well below the configured default.
  `waypoint_lookahead_deg` still overrides it outright.
- **Pacing is not interpolation, and this module does not interpolate.**
  `MoveThroughJointPositions` writes each waypoint as a servo goal and paces it; nothing
  inserts intermediate points. Smoothness therefore comes from the goal LEADING the arm --
  the lookahead -- not from step size. Do not "simplify" a caller's densification away on the
  assumption that the arm re-adds it: that was done to arm-recorder's 8x playback
  interpolation and the replay came back visibly jerky.
- **Linear interpolation does NOT disturb `CoordinatedProfiles`.** Two plausible-sounding
  objections to a caller densifying its waypoints are both false, and were checked against
  the real function rather than reasoned about: (1) sub-segments do NOT collapse onto
  `MinExecutableSpeedDegsPerSec` -- interpolation preserves each joint's share of the
  segment's travel, so `k = d_i/d_max` is invariant and the per-joint `SpeedSteps`/`AccUnits`
  are byte-identical between a parent segment and a 1/8 sub-segment; (2) sub-segments smaller
  than the lookahead do NOT reintroduce the pre-0.9.3 flythrough -- the goal leads by several
  sub-waypoints, but the lead is still bounded in absolute degrees by the lookahead, which is
  the property that matters. Densifying is a legitimate smoothness lever for a caller that
  knows its own tempo.
- **`Stop` must cancel the move, not just zero velocity.** A paced waypoint stream runs for
  seconds, so `arm.Stop` calls `opMgr.CancelRunning` *before* `controller.Stop`; the reverse
  order lets the dwell loop write its next goal after the zero and the arm resumes.
  `MoveToJointPositions` / `MoveThroughJointPositions` register via `opMgr.New`.
  `MoveToPosition` deliberately does **not**: it delegates to the motion service, which calls
  back into `MoveThroughJointPositions` over gRPC, and the op marker is a `context.Value` that
  cannot cross that boundary — the outer op would be cancelled by its own callback.
  `TestStopCancelsARunningWaypointStream` pins this.
- **The arm and the calibration sensor still share the bus without excluding each other.**
  One mutex serializes individual operations, so their transactions cannot interleave, but
  nothing stops an arm move during a calibration workflow. `Discover` holds that mutex for
  the whole ID sweep — seconds on a sparse bus — so motion waits during motor setup.
- **`controller.NewControllerRegistry` does not share the process-wide port ownership** the
  shared-bus design depends on; two registries can open one serial port and contend. It is
  exported only so a caller outside the package can inject a bus factory
  (`WithBusFactory`) — production goes through the package-level `AcquireController`.
- A `*_arm.go` filename is treated by Go as a GOARCH=`arm`-only file. Use `arm` as a
  filename *prefix*, never a suffix — this is why the simulated arm is `simulated.go`.
- rdk's URDF parser uses only the **first** `<collision>` element per link (falls back to
  `Collision[0]` after a capsule-pattern check; see `referenceframe/model_urdf.go`). A link
  authored as several `<collision>` sub-parts therefore keeps just one offset fragment. This
  is why `assets/urdf/so101.urdf` gives each arm link a single merged mesh.
- **A component ships its kinematics to viam-server as `ModelConfig().OriginalFile.Bytes`**
  (`referenceframe.KinematicModelToProtobuf`), NOT its in-memory `LinkConfig`s. So any
  in-memory edit to a parsed model is **lost across the module gRPC boundary** — viam-server
  rebuilds the model from the original file bytes. This is why the correct TCP/frame names
  are baked into `assets/urdf/so101.urdf` itself rather than patched in `armModelURDF`: an
  in-memory graft/rename left `OriginalFile` as the raw (un-grafted) URDF, so the server used
  an EE ~98mm past so101.json's TCP and the gripper (parented to the arm) followed it — while
  every in-process unit test stayed green. `TestURDFModelSurvivesSerialization` guards this
  by round-tripping through the real gRPC (de)serialization; an in-process `Transform` check
  would NOT have caught it. The one place we still rely on this: for `visualize_ee_frame` the
  tool needs an in-memory placeholder box, so that path re-serializes to **SVA JSON**
  (`spatialmath.GeometryConfig.MeshData` embeds the mesh bytes, so meshes survive) — a bigger
  payload (~5.9 MB vs ~4.4 MB URDF), hence only when the marker is enabled.
- The Viam `tool` frame in `so101.json` is rotated ~180° from the URDF `gripper_link`
  frame (the TCP convention). Gripper meshes are authored in `gripper_link` coords, so
  `internal/geometry`'s `toolFromGripperLink` transform corrects them in `BuildGripperMeshes`.
- Leader gripper meshes are printable STLs with no assembly transforms (the SO-ARM100
  repo ships only a *follower* URDF/sim model), so leader placement is best-effort. The
  follower gripper meshes come from the SO-ARM100 simulation assets + URDF and are accurate.
- **A gripper's `Geometries()` is NOT its frame-system geometry, and `Kinematics()` is not
  optional.** In `robot/impl/local_robot.go`'s `getLocalFrameSystemParts`, the
  `arm`/`gantry`/`gripper` subtypes take a dedicated branch that reads `Kinematics()` and then
  `continue`s — the `Geometries()` query below it is reached only by *other* subtypes. So a
  gripper's meshes become collision geometry only by riding on its kinematic model. Worse, on
  **viam-server >= 1.0.0 a `Kinematics()` error drops the gripper from the frame system
  entirely** (`continue`); on <= 0.13x it was added with a nil `ModelFrame`. Both grippers here
  therefore return a model from `geometry.BuildGripperModel` rather than `errors.ErrUnsupported`.
- `BuildGripperModel` builds that model as **SVA JSON round-tripped through
  `referenceframe.UnmarshalModelJSON`**, not as an in-memory `SimpleModel` — same
  module-boundary reason as the arm (a hand-assembled model has no `OriginalFile`, so it ships
  as `UNSPECIFIED` while every in-process test stays green).
  `TestGripperModelSurvivesSerialization` guards it. Shape constraints worth knowing: a
  `LinkConfig` carries **at most one geometry**, so each mesh needs its own link; and
  `ParseConfig` requires **exactly one leaf** unless `output_frames` is set, so the links form
  a chain. All mesh links have identity transforms (the pose lives in the geometry offset), and
  the leaf is the `tcp` link — which is what makes the model's `Transform` the TCP.
- The gripper's frame is its **TCP between the jaw tips** (`GripperTCPPose`, `(6.835, 0, 99.9)`
  mm from the arm's `tool` frame, pure translation so `+Z` stays the approach axis the goal
  cloud assumes). Measured off the follower meshes with the jaws closed;
  `TestGripperTCPLiesBetweenTheJawTips` re-derives the jaw bounds from the meshes so
  regenerating them can't strand the TCP inside a finger. The `leader` gripper has no jaws, so
  its leaf offset is zero. The model is **0-DoF on purpose**: a jaw DoF would become a variable
  the motion planner may drive, and it means the frame-system collision meshes are frozen
  closed while `Geometries()` keeps serving the live articulating jaw to the 3D viewer.
  `TestGripperFrameResolvesToTCPInFrameSystem` asserts the end-to-end contract, which is also
  why users must **not** add a compensating `translation` to the gripper's `frame`.
- **A part's `frame` config and its `kinematics` are two independent things**, and the 3D viewer
  reads frames from the kinematics. Verified against a live machine:
  `RobotService.FrameSystemConfig` reports `follower-gripper`'s `frame.poseInObserverFrame` as
  a *zero* pose relative to `follower-arm` (the mount, on the arm's `tool` frame) while the TCP
  lives only in `kinematics` as the `tcp` link's +99.9mm. The viewer draws a `<gripper>:tcp`
  node with axes at the grasp point from that link alone — no geometry required. (It did not
  always: an earlier viewer drew frames only where geometry existed, which briefly motivated a
  `visualize_tcp_frame` placeholder-box attribute. That was reverted once the viewer was fixed;
  don't re-add it.) The module cannot move the mount marker anyway — only the machine config's
  `frame.translation` can. Note also that `Get3DModels` is on the **arm API only**, so the
  gripper cannot serve the colored `ee_frame.glb` marker the arm uses.
- **In SVA, a link's `geometry` offset is relative to the link's *input* (parent) frame, not its
  output.** Empirically: a geometry on a link with a non-zero `translation` renders at the parent
  pose, not the link's own pose (this is also what `referenceframe`'s `tailGeometryStaticFrame`
  exists to flip for a *part's* `frame.geometry`). so101.json follows the same convention — the
  `base` link translates to `z=62.4` but its box offset `z=33.6` is measured from world. Hence
  every mesh link in `BuildGripperModel` carries an identity transform with the full mesh pose in
  the geometry offset; the `tcp` link is the only one with a real `translation`, and it carries
  no geometry at all.
- pnpm 11 approves dependency install-scripts via `allowBuilds` (a map of dependency name to
  true/false) in `setup-app/pnpm-workspace.yaml` — not the older `onlyBuiltDependencies` list,
  and not `package.json`'s `pnpm` field. `setup.sh` pins `pnpm@11` (matching the `node@22` pin)
  so a future pnpm major can't silently break the build again.
- Both the **simulated arm** and the **hardware arm** use coordinated-arrival interpolation:
  each joint's speed and acceleration are scaled by its share of the travel so all joints finish
  together. The hardware arm used to move each joint at the same configured speed independently,
  so longer-travel joints finished later (measured on hardware as a 260-520 ms spread).
- **Polling `AnyServoMoving` cannot corrupt or interleave with a concurrent position
  read/write, but it does queue behind (or ahead of) one.** `entry.busMu` in
  `internal/controller/bus_session.go` only arbitrates writes vs. reads (`withSessionRead`
  takes `RLock`, so two concurrent reads are NOT serialized by `busMu` itself) -- the real
  exclusion is one level down: every `*feetech.Bus` method takes the bus's own mutex for a
  transaction's whole request+response round trip, and `enforceCommandGap`'s sleep (default
  1ms, never overridden here -- see `registry.go`'s `feetech.BusConfig` literal) runs *while
  that lock is held*. So a queued caller pays the ~1ms gap of whatever transaction is ahead of
  it, not just its own -- both transactions are dominated by that fixed gap, not by payload
  size. Measured on hardware (300 samples @ 1Mbaud, 3 runs, since removed with the
  throwaway test that produced them): `SyncReadPositions(6)` 1.48-1.60ms mean,
  `AnyServoMoving(5)` 1.27-1.30ms, and the same 5 Moving reads issued one per servo (what
  `WaitForServosToStop` did before batching) ~5.4-5.7ms. A 50Hz poll costs a concurrent
  position read only ~0-100us mean -- but its p95 goes ~1.6ms -> ~2.7ms, because a read that
  *does* collide pays a whole extra transaction, so judge this by the tail and not the mean.
  An unthrottled poller costs +1.34-1.49ms mean, i.e. one full transaction: the degradation is
  linear and bounded, not pathological. Seen once in ~6 runs: under *sustained max-rate*
  polling a position read failed outright (nothing retries), the same transient that would
  abort a move from `WaitForServosToStop`'s own poll -- another reason not to poll `IsMoving`
  in a tight loop.
- **`Speed: 0` means MAXIMUM, not stopped**, and **`Acc: 0` means UNLIMITED, not zero** — both
  Feetech register sentinels are the opposite of what they look like, and both have already
  caused real bugs in this project. A short-travel joint scaled down by a small `k` can round
  into either floor if the scaling math doesn't guard against it (`servo.MinAccUnits = 1`,
  `servo.AccFloorDegsPerSecSq = 43.0`).
- In `internal/servo`, the package const `MaxAccelDegsPerSecSq` and the `JointLimits` field of
  the same name coexist, as do `maxSpeedDegsPerSec` (const) and `MaxSpeedDegsPerSec` (field).
  All field references are selector-qualified so they cannot be confused by the compiler — but
  in a package where speed/accel mix-ups have caused real bugs, read carefully before editing.
- Several locals in `internal/controller` are named `servo` and shadow the imported `servo`
  package within their scope (`config.go` uses `dev` where it needs `servo.X`). Harmless today;
  an edit inside one of those scopes that needs `servo.X` must rename the local first.
- **`referenceframe.PoseCloud` semantics are counterintuitive and were established
  empirically** against rdk's behavior. `internal/planning` depends on all of it, and the
  `TestCone*` / `TestRDKContract*` cases in `internal/planning/goal_cloud_test.go` pin each
  claim below:
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
    `[0, 180]`, saturating at `acos(-0.999) = 177.4374412669`. Check saturation by comparing
    cosines (`cos(tol) <= -0.999`), never a rounded degree literal — every rounding is wrong
    in one direction or the other.
  - **`defaultEpsilon = 0.001` is ADDED to every leeway**, not just zeroed ones. So zero
    `X`/`Y`/`Z` demands a match within 1 micron (which no IK solution realistically achieves),
    and the effective cone is `acos(cos(tol) - 0.001)` — always slightly wider than requested,
    floored at ~2.56°.
  - The cloud only *zeroes the score* when a candidate is inside it. A cloud the solver cannot
    realistically land inside is **worse than no cloud**: planning reverts to strict six-DOF
    scoring and fails. `position_only` sets `orientScale=0`, so a cloud's orientation leeways
    stop mattering (its positional leeways still zero the positional residual, since
    `GetGoalMetric` consults `PoseInCloud` regardless of metric type). Passing both is rejected
    as incoherent rather than silently honoring one.
  - **There is no `ReferenceFrame` field.** v0.123.0 had one; v1.0.0 removed it while leaving
    the struct's reference-frame doc comment in place as free-floating prose, so it reads as
    though the field still exists. The complete field set is `X, Y, Z, OX, OY, OZ, Theta`.
  - `planning.GoalCloudConfig`'s fields are exported, so a zero-valued literal is constructible
    — and a zero-valued cloud is the silently-fatal case above. Build one with
    `ResolveGoalCloudConfig`.
- **Goal clouds require viam-server >= 0.127.0.** RDK v0.125.0–v0.126.1 read `GoalCloud` in
  `armplanning` but never decode it in `ProtobufToPoseInFrame`, so the cloud is silently inert
  there; v0.127.0 is the first release with both halves. The module only *sends* the cloud — its
  own RDK version has no bearing on whether the server honors it. The minimum is declared
  Registry-side; `meta.json` has no field for it.
- **`NewSO101`'s `goalCloud` wiring is untested** — it needs serial hardware, so no test
  constructs the hardware arm. Deleting `goalCloud: planning.ResolveGoalCloudConfig(...)` from
  `NewSO101` leaves the whole suite green, and the result is a zero-valued cloud that fails
  every move silently. `TestSimulatedConstructorWiresGoalCloud` covers the equivalent line in
  the simulated constructor and the shared `ResolveGoalCloudConfig` contract, but the hardware
  constructor's line is guarded only by review.
- **The gripper's calibrated range is a scale factor; the arm's is only a center point.**
  `NormModeRange100` (servo 6) maps 0-100% linearly onto `[RangeMin, RangeMax]`, so a wrong
  range rescales every command. `NormModeDegrees` (servos 1-5) takes only
  `center = (min+max)/2` and then clamps, so the same wrong range costs ~4°. This is why an
  uncalibrated servo is survivable for the arm and drives the jaw into its stops. `HomingOffset`
  is stored but never used in `Normalize`/`Denormalize` — the servo firmware applies it itself
  (`Present_Position = Actual_Position - Homing_Offset`).
- **`0`/`4095` position limits mean "unset", not "calibrated".** `ReadCalibrationFromServos`
  accepts them (`min < max && max <= 4095`), so kits that ship with a homing offset but no
  limits read as calibrated. `guardGripperTravel` narrows the gripper's range when it is too
  wide to be real. The window is **anchored, not centered**: tick 2048
  (`GripperClosedStopTick`) is the jaw's *closed* stop on a vendor half-turn-homed kit, not its
  mid-travel, and the window opens `gripperSafeTravelTicks = 1200` ticks from there
  (`2048..3248`). `DefaultSO101FullCalibration`'s servo-6 entry is this same window, not the arm
  joints' `500/3500` — because `LoadFullCalibrationFromFile`'s `convertOrDefault` fills a
  *missing* `gripper` key with the default and returns `fromFile=true`, which makes the registry
  skip both the servo read and the guard, so an unsafe default there would never get caught. The
  guard itself runs inside `ReadCalibrationFromServos`, so a calibration file with a `gripper`
  entry bypasses it entirely. `resetCalibrationRegisters` does **not** run at calibration start —
  `startCalibration` only resets in-memory struct fields. It runs from `setHomingPosition`, so
  `0`/`4095` is reached only by abandoning *after* `set_homing`, and that same step also writes a
  fresh homing offset centered on wherever the jaw was told to sit ("move to the middle of its
  range of motion"). So this path does not land in the vendor's closed-stop state the anchor
  assumes — it lands in the mid-travel-homed state the anchor is *wrong* for (see the design
  doc's "conflict this design knowingly accepts"). Abandoning *before* `set_homing` leaves the
  registers untouched.
- **`Grab()` used to return `true` unconditionally.** Under the old `500/3500` default an empty
  jaw resting at its closed stop (tick 2048) normalized to 51.6%, clearing the `> 15.0` grasp
  threshold regardless of whether anything was held. Anyone who wrote code branching on
  `Grab()`'s return value before this change was reading a constant.
- **`feetech.BusConfig` takes an exported `Transport`**, so `ReadCalibrationFromServos`'s
  bus-dependent paths are unit-testable without serial hardware. Worth knowing generally — the
  repo has other paths written off as hardware-only that may not be.
- **`motor_setup_verify` returns `success` for "the call completed" and `all_ok` for the
  verdict.** It used to return `success: allGood`, alone among the commands in `motorsetup.go`
  in overloading that field — and since `setup-app`'s `sendCommand` throws on `success:false`,
  any single unhealthy motor threw and took the whole per-motor verification UI (grid, status
  badges, troubleshooting panel) down with it. The app still derives its own verdict from the
  per-motor `status` values rather than trusting `all_ok`, and only logs when the two disagree.
  Safe to change the wire format because the module binary and the built app ship in one
  `module.tar.gz`, so there is no version skew between them.
- **`motorSetupVerify` and `discoverOneMotor` deliberately disagree about an unrecognized servo
  model.** Both see the same fact — the servo answered but its model number is not in the
  registry, and `bus_session.go` drives it as `ModelSTS3215` either way — but verify treats it as
  a warning (`model_detection_failed`, leaves `all_ok` true) while
  discovery rejects it, because discovery is choosing which physical servo to write a bus ID to.
  Commented at both call sites; don't "fix" one to match the other. Mechanically they differ:
  verify classifies the model number `PingServo` already returns (one round trip) via
  `feetech.GetModelByNumber`, while discovery reads `FoundServo.Model` from `Discover`. Verify
  deliberately no longer calls `feetech.Servo.DetectModel` — that re-pinged, so a servo dropping
  out mid-detection was misreported as a model problem, and its side effect of repointing the
  session servo's register map (an SCS model would have moved `LockAddress` for the rest of the
  session) was a latent hazard. That left `ControllerHandle.DetectServoModel` with no caller, so
  it was deleted — a `PingServo` + `feetech.GetModelByNumber` pair is the replacement, and it has
  no register-map side effect. Note neither call site checks the model is one the SO-101 *ships*:
  a known-but-wrong model (say `scs0009`) still verifies as `ok`.
- **Enabling torque is a bare `SyncWrite`, so seed `Goal_Position` first.** `EnableAll` writes
  `RegTorqueEnable` with no goal sync, and these are closed-loop position servos — they resume
  driving toward whatever goal their register holds, which the calibration workflow never writes
  (so it is the arm component's last target, or a power-on leftover). Bounded only by the
  `min`/`max_angle_limit` pair just written, i.e. the joint's whole travel, at a `Goal_Velocity`
  where `0` means MAXIMUM. `saveCalibration` therefore calls `seedGoalPositions` (live
  `SyncReadPositions` → per-servo `goal_position` write) immediately before enabling, and leaves
  torque OFF if that seed fails. `abort`/`reset` perform no motion-capable operation at all: they
  are reached with registers half-written and hands on the arm.
- **`writeHomingOffset` really writes `position_offset` now**, sign-magnitude encoded at
  `feetech.RegPositionOffset.SignBit` (11) — it used to ignore its argument and write `128` to
  `torque_enable`. That was not inert: `128` is FEETECH's "calibrate middle position" command, so
  the firmware wrote the offset itself and the frame shift already existed. Every position the
  servo reports after `set_homing` is offset-shifted, and recorded ranges, the written limits, and
  the calibration file all live in that same post-offset frame — which is exactly why
  `HomingOffset` must stay unused in `Normalize`/`Denormalize`. Double-applying it is the bug.
- **A failed `saveCalibration` stays at `StateCompleted`, not `StateError`.** Its own guard
  requires `StateCompleted`, so the old `StateError` on failure made every retry return
  "calibration not completed (current state: error)" — the only recovery was a `reset` that
  discarded a completed recording session. Retry is safe because it rewrites every servo
  unconditionally (no per-servo "already done" tracking) from the untouched in-memory
  `cs.joints`. The calibration file is written *before* the servo registers, on purpose: it is
  the only durable record of the recording session, and nothing requires it to agree with the
  registers. `LoadCalibration` reads the registers only when no file exists, so a file present
  without the register writes is correct — whereas registers written without a file falls back
  to `ReadCalibrationFromServos`, where an unwritten servo's factory `0`/`4095` still reads as
  "calibrated" and the gripper lands on `guardGripperTravel`'s closed-stop window that this
  workflow's own homing offset invalidates. Don't "fix" the order.
- **`readings.last_save_error` is the durable signal for a failed `save_calibration`, not
  `readings.error`.** `setState` only populates `errorMsg` for `StateError`, and a save failure
  deliberately stays at `StateCompleted` so the command stays retryable — so `readings.error` is
  empty exactly when a save has failed. `sensor.go`'s `DoCommand` dispatch, not `workflow.go`,
  sets and clears `lastSaveError`: it already sees `saveCalibration`'s error return under
  `cs.mu`. Cleared by a successful save, `start`, `reset`, and a *successful* `abort` — a rejected
  abort must leave a prior save failure untouched, so the `abort` case only clears it once
  `abortCalibration` returns nil.
- **`available_commands` must match what `DoCommand` actually accepts, in both directions** —
  for the *workflow* commands. `get_current_positions` and the five `motor_setup_*` commands are
  accepted in every state and advertised in none, on purpose: they are not workflow steps.
  `resetCalibration` has no state guard at all — it is safe and idempotent from every state — so
  `Readings` advertises `reset` in every case, not just `error`. `abortCalibration` was the
  opposite risk: also unguarded, but `completed` holds a finished recording session waiting on
  `save_calibration`, so an abort there would silently discard minutes of hand-moved calibration.
  It now rejects `StateCompleted` and treats `StateIdle` as a no-op success, worded so it cannot
  be mistaken for having done anything. `abort`'s advertised states are unchanged — only the
  guard was added. One gap remains by choice: `error` still accepts `abort` without advertising
  it, which is harmless because `reset` is the escape hatch offered there.
- **Compliance zeroes `i_gain`, not just `p_gain` — and a failed READ falls through to the
  WRITE, not away from it.** The integral path is independent of P, so lowering P to 8 during
  hand-guided compliance used to leave a live integrator; inert until the tuned gains carry a
  non-zero I (`internal/servo/gains.go`), which they now do. Gated on `p_gain > 0`, the
  documented per-register opt-out; `d_gain` is deliberately left alone (reasoned drag, never
  measured). Both `applyCompliance`/`restoreCompliance` and `applyServoGains` route every gain
  write through one `writeGainIfChanged` (`components/arm/gains.go`) because these are EEPROM
  registers entered/exited (or reconfigured) repeatedly — and it treats a read it cannot trust
  as unable to prove a no-op, so it falls through to the write rather than skipping it. The
  asymmetry is deliberate: `applyCompliance` failing outright leaves the arm stiff, which is
  safe; `restoreCompliance` skipping a write on a flaky read would leave a joint at `p_gain
  8`/`i_gain 0` in EEPROM — soft, across power cycles — on a transient this hardware really
  produces, with nothing retrying it. Don't make either path bail out on a read error.
- **Tuned gains are written during `doServoInitialization`, in D→I→P order, and a failed WRITE
  is terminal for that servo.** `applyServoGains` BREAKS its per-servo loop on the first write
  failure rather than logging and continuing: without the break, {I,P}-without-D is reachable —
  48/32/4 on servo 3, the exact combination measured to oscillate on a gravity-loaded joint —
  and it would be stranded in EEPROM with nothing to retry it (`applyServoGains` returns no
  error, so `initializeServosWithRetry` never re-runs for a gain failure alone). The read-first
  skip is the EEPROM wear guard, and it runs on every `Reconfigure` as well as up to 3× within
  one bring-up. `servo.Gains` fields are `byte`, not `int`, so an out-of-range retune is a
  compile error rather than a silent truncation to less damping. Config is one
  `disable_servo_gains` bool; there are no per-joint overrides — `pidtune apply` is the
  escape hatch for a differently-tuned arm.
- **A streamed setpoint gets Speed 0 AND Acc 0 AND no coordination read; a profiled move gets
  none of the three.** The `"streamed": true` key in `MoveToJointPositions`'s `extra` map
  (`components/arm/motion.go`) branches to `moveJointsUniform` before the `currentJoints()` read
  and forces both registers to their unbounded sentinel — 0 is MAXIMUM speed and UNLIMITED
  acceleration, not stopped and not still. One principle covers all three: a streamed caller
  shapes the trajectory with its own update rate, so any profile the servo applies *between*
  setpoints is pure lag, and coordinated arrival is meaningless for a goal superseded in ~10ms.
  Both halves were confirmed on hardware, acceleration first (acc=32 quadrupled RMS error and
  took lag 40ms→160ms) and speed second — raising `speed_degs_per_sec` kept improving teleop
  tracking until the cap stopped binding, which is what motivated removing it. `minSpeedSteps`
  and `MinAccUnits` are both still 1, so the profiled path cannot reach either sentinel: a low
  *configured* value is not a substitute (Acc 1 delivers ~43 deg/s², worse than the acc=32 row).
  Measured in tests: one bus packet per call instead of two. Do NOT unify the two paths, and do
  not split `streamed` back into per-register flags — nothing wants them separately, and the
  read-skip belongs to the same decision. **It removes a bound:** teleop does not gate its first
  cycle on the follower being near the leader, so a large initial gap is now closed at full
  servo speed with no ramp.

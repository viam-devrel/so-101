# SO-101 Module Remediation Design

**Date:** 2026-04-27
**Status:** Draft
**Author:** Brainstorm with Nick Hehr

## Context

A full review of the `so_arm` module surfaced ~30 issues spanning critical bugs (silent calibration corruption, lifecycle leaks, a misleading speed/acceleration config), missing features (`IsHoldingSomething`, `MoveOptions`), structural problems (3-4 implementations of the same conversion math, type-explosion around calibration, `runtime.Caller`-based reference tracking), and gaps in testing/CI/docs.

This spec sequences the remediation work into 8 risk-ordered PRs, prioritizing arm and gripper fixes before calibration sensor and discovery, and explicitly defers a possible architectural shift (singleton shared-controller → arm-as-core with gripper using DoCommand) until the in-place fixes are complete.

## Goals

- Land the most user-visible correctness fixes (the silent calibration write bug, the speed/accel lie, the broken `Stop`) within the first two PRs.
- Make every PR independently reviewable: behavior changes and refactors do not share a PR.
- Add test infrastructure (mock-bus harness, conversion-math round-trip tests) inside the PRs that need it, rather than as a separate phase, so each PR ships covered.
- Establish PR-time CI so future regressions of this class are caught at review.
- Leave the codebase in a state where the singleton-vs-arm-as-core architectural question can be re-evaluated honestly post-remediation.

## Non-goals

- **No architectural shift in this spec.** The shared-controller singleton stays. Whether to flip to arm-as-core (gripper communicates with arm via DoCommand) is a deferred decision, re-evaluated after PR8 lands.
- **No package-layout reshuffle** (`internal/`, sub-packages). Discussed in the review, intentionally out of scope here.
- **No changes to the setup-app SvelteKit application.** Calibration sensor stays one Viam resource — the web app's UX depends on it.
- **No new features beyond filling already-promised contracts.** `IsHoldingSomething`/`MoveOptions`/`Get3DModels` are already-promised interfaces returning errors; we make them work. We do not add servo diagnostics, runtime joint limits, or park-on-close in this spec.

## Architecture decisions

### Defer arm-as-core refactor

The current shared-controller singleton has real boundary problems (gripper reaches across files into the controller's unexported state; calibration sensor reads `cs.controller.bus` directly), but the most painful manifestations — `runtime.Caller` PC tracking, `s.initCtx` reuse, the gripper-closes-first race — are independently fixable in place. After PR1-PR4 we will have a clean baseline against which the arm-as-core question is a more honest one. The arm-as-core option is not better than the singleton in the abstract; it is only better than the *broken* singleton.

### Same-pointer registry semantics

`getExistingController` currently returns a fresh `*SafeSoArmController` struct copied from the registry's cached one. The fix is to cache one `*SafeSoArmController` per port and return that exact pointer to all callers. When the bus closes, an internal `closed` flag is set; subsequent method calls return `ErrClosed`. This eliminates the gripper-closes-first race without introducing a new handle type.

### Per-move speed, init-time acceleration

Speed flows through `ServoGroup.SetPositionsWithSpeed` on every move, with the configured `speed_degs_per_sec` as the default and `MoveOptions.MaxVelocityRPS` as a per-call override. Acceleration is written to the `RegAcceleration` register at init/reconfigure (and re-written if reconfigure changes it). This matches Viam's expectation that `MoveOptions` overrides per call, and avoids the surprise of acceleration silently changing mid-move.

### Free conversion helpers, not methods

A new `conversion.go` provides `radiansFor(cal *MotorCalibration, raw int) (float64, error)` and `rawFor(cal *MotorCalibration, rad float64) (int, error)`. The gripper's "percent encoded as `[-π, π]` so the Viam gripper API can pretend it's a joint" convention lives in these helpers, not on `MotorCalibration` itself — that keeps `MotorCalibration` as a pure servo-math type. The DriveMode flip stays inside `Normalize`/`Denormalize` (single source of truth), eliminating the asymmetry bug where the gripper applied DriveMode at the radians layer while `MotorCalibration` applied it at the normalized layer.

### Strict opMgr semantics on the arm

`opMgr.New(ctx)` at the top of `MoveToJointPositions` cancels any in-flight move; `opMgr.CancelRunning(ctx)` from `Stop` actually stops; `IsMoving` becomes `opMgr.OpRunning()`. `moveLock` and `isMoving` are deleted. This is a user-observable behavior change: a second move now cancels the first instead of queueing behind a mutex. This is intentional — it is the correct arm semantics, matches RDK convention, and unblocks `Stop`.

## PR sequence

### PR1 — Lifecycle correctness

Refactor only; no user-visible behavior change.

- Delete `runtime.Caller`/`callerPorts` machinery in `registry.go:332-360`. Replace with explicit port tracking on each component struct (`s.controllerPort = config.Port`); `Close` calls `globalRegistry.ReleaseController(s.controllerPort)` directly.
- Stop reusing `s.initCtx` (`arm.go:111`). Pass per-call `ctx` through to `doServoInitialization`, `diagnoseConnection`, `verifyServoConfig`. Remove the `initCtx` field.
- Fix gripper-closes-first race: in `getExistingController`, cache one `*SafeSoArmController` per port in `ControllerEntry` and return that exact pointer (`registry.go:98-104`). Add `closed atomic.Bool` to `SafeSoArmController`; method calls return a sentinel `ErrControllerClosed` when set. `ReleaseController` at refcount zero sets the flag and closes the bus.
- Tests: introduce mock-bus harness based on `feetech/_examples/mock_transport`. Cover refcount-0 close, double-acquire returns same pointer, post-close method call returns sentinel error.

### PR2 — Move-path correctness

User-visible behavior change. Fixes the speed/accel lie, `Stop` actually stopping, and the silent calibration corruption bug.

- Wire `speed_degs_per_sec` through `ServoGroup.SetPositionsWithSpeed`. Default from config; `MoveOptions.MaxVelocityRPS` per-call override.
- Write `RegAcceleration` per-servo at init and on reconfigure (when value changes).
- Replace `time.Sleep(moveTimeSeconds)` block (`arm.go:380-389`) with `group.WaitForStop(ctx, timeoutMs)`. `timeoutMs` derived from the current speed estimate but used only as upper-bound safety net. Honors `ctx.Done()`.
- Drop the unused `speed, acc int` parameters from `MoveServosToPositions` and `MoveToJointPositions` controller methods (the latter is dead anyway, deleted in PR3).
- Fix `writeHomingOffset` (`calibration.go:872`): correct register name (`position_offset`), use the actual `homingOffset` argument, encode as 2-byte sign-magnitude using `feetech.RegPositionOffset.SignBit`.
- Tests: mock-bus test that `MoveToJointPositions` with non-default speed produces the expected sync-write packet; mock-bus test that `Stop` cancels an in-flight move (context observed); regression test for `writeHomingOffset` correct register + payload.

### PR3 — Conversion math consolidation + dead code removal

Refactor only; no user-visible behavior change. Protected by tests added in this PR.

- New `conversion.go` with `radiansFor(cal, raw)` and `rawFor(cal, rad)` covering both arm (degrees) and gripper (percent → `[-π, π]`) conventions.
- Replace inline math in: `MoveServosToPositions` (`manager.go:73-88`), `GetJointPositionsForServos` (`manager.go:150-153`), `gripper.percentToRadians` and `radiansToPercent` (`gripper.go:370-424`), `arm.calculateJointLimits` (`arm.go:130-167`), and the broken estimator at `calibration.go:838`.
- Delete dead methods: `SafeSoArmController.MoveToJointPositions` (`manager.go:31`), `GetJointPositions` (`manager.go:94`), `getCalibrationForServo` (`manager.go:253`), `CalibratedServo.SetPositionWithSpeed` (`calibrated_servo.go:213`), `GetCurrentCalibration` (`manager.go:356`).
- Delete dead config fields: `SoArm101Config.SpeedDegsPerSec`, `AccelerationDegsPerSec` (`config.go:23-24`).
- Consolidate the three calibration-update loops (`registry.go:78-90, 185-198`, `manager.go:228-244`) into one method on `SafeSoArmController`.
- Delete commented-out radians-conversion blocks at `calibration.go:427-457` and `:573-603`.
- Tests: round-trip property tests for `Normalize`/`Denormalize` over all four `NormMode` × DriveMode toggle × range edges. Round-trip for new `radiansFor`/`rawFor`. Unit tests for `calculateJointLimits` over hand-built calibrations.

### PR4 — Surface fixes + concurrency cleanup

Mostly behavior-correcting; the `opMgr` change is user-observable.

- `IsHoldingSomething` (`gripper.go:358`): extract the position-vs-threshold inference from `Grab` into a private helper, reuse from `IsHoldingSomething`. Cache last `Grab` outcome optionally.
- `MoveThroughJointPositions` (`arm.go:394`): honor `MoveOptions` per step — `MaxVelocityRPS` overrides default speed for that step's `SetPositionsWithSpeed` call.
- `Get3DModels` (`arm.go:444`): return `nil, errors.ErrUnsupported` instead of a hard error.
- `arm.DoCommand` `default:` branch (`arm.go:550-600`): replace with explicit `case "set_speed"`, `case "set_acceleration"`, `case "get_motion_params"`. The default case becomes a clean unknown-command error that lists valid commands.
- `opMgr` integration: `s.opMgr.New(ctx)` at top of `MoveToJointPositions`, `s.opMgr.CancelRunning(ctx)` from `Stop`, `IsMoving` returns `s.opMgr.OpRunning()`. Delete `moveLock` and `isMoving`.
- Cache `calculateJointLimits` result on `so101` struct; invalidate on `SetCalibration` and on successful `reload_calibration` DoCommand.
- Replace `CalibratedServo.mu` with `atomic.Pointer[MotorCalibration]`. The bus already serializes wire access; the per-servo lock was redundant.
- Tests: `IsHoldingSomething` returns true after a `Grab` that succeeded and false after one that didn't (using mock bus to set position); `MoveOptions.MaxVelocityRPS` override propagates to the sync-write packet; `Stop` cancels in-flight move via opMgr (verified via context cancellation in the mock).

### PR5 — Calibration sensor bug-fixes

Sensor stays one resource (web app constraint). No structural extraction.

- State-machine race teardown: consistent `if cs.recordingCancel != nil` checks in `Close`, `abortCalibration`, `resetCalibration`, `stopRangeRecording`. Audit for double-call.
- Replace `positionHistory []map[int]int` (`calibration.go:540-630`) with `var sampleCount atomic.Int64`. The history was only ever read for `len()`. Saves ~100KB churn during recording.
- Delete commented-out radians-conversion blocks (`:427-457`, `:573-603` — already covered in PR3 if we get there first; otherwise here).
- Replace emoji status strings (`calibration.go:1077-1080`) with plain text.
- Tests: state-machine progression test (idle → started → homing_position → range_recording → completed), backed by mock bus. Asserts on register writes and state transitions. Catches the writeHomingOffset class of regressions.

### PR6 — Discovery improvements + canonical joint mapping

- Worker-pool parallelize port scanning (`discovery.go:82-93`). Cap concurrency at 6 to avoid pathological hub behavior. Use `errgroup.WithContext` for cancellation propagation.
- Multi-baudrate discovery: either delegate to `feetech.Bus.Discover` (which sweeps internally) or explicit sweep `[1000000, 500000, 115200, 57600]`. Decision driven by what `Bus.Discover` actually does — verify in implementation.
- Extract canonical joint mapping into a shared file (likely `config.go` or new `joints.go`):
  ```go
  var SO101Joints = []struct{ ID int; Name string }{
      {1, "shoulder_pan"}, {2, "shoulder_lift"}, {3, "elbow_flex"},
      {4, "wrist_flex"}, {5, "wrist_roll"}, {6, "gripper"},
  }
  ```
- Replace `expectedMotors` literals at `calibration.go:1026-1033, :1105` and similar in discovery/arm with references to the canonical mapping.

### PR7 — Documentation

- Add `speed_degs_per_sec`, `acceleration_degs_per_sec_per_sec`, `motion` to the arm attributes table in README.
- Document gripper `calibrate_positions`, `set_motion_params`, `get_motion_params`.
- Document arm `set_speed`, `set_acceleration`, `get_motion_params`.
- Document the missing ~half of calibration sensor DoCommands.
- Fix the dead `MOTOR_SETUP.md` reference (line 432).
- Lift the duplicated "Communication" boilerplate (~25 lines × 3 sites) into one section; cross-link from each model.
- Add a table of contents.
- Add package-level godoc in a new `doc.go`. Backfill exported-symbol comments to ~80% coverage (today: ~30%).

### PR8 — CI + build hygiene

- New `.github/workflows/ci.yml` running on PRs: `go vet ./...`, `go test -race ./...`, `make`. Would have caught both the `writeHomingOffset` SA4006-class issue (unused parameter) and the `cmd/cli/` build break.
- Add `staticcheck` to `make lint`.
- Hardware build tag: `//go:build hardware` on tests currently using `t.Skip("hardware-dependent")`. Add `make test-hardware` target.
- Fix `cmd/cli/`: split the kept tools (`debug_cli`, `position_reader`, `read_servo`, `torque_disable`) into per-directory `cmd/<name>/main.go`. Delete throwaways (`simple_test_try`, `sync_test_again`, `gentle_move`, `raw_servo`). Add a `make tools` target that builds the kept ones.

## Test strategy

Test infrastructure is built incrementally inside PRs that need it:

- **PR1** introduces the mock-bus harness based on `feetech/_examples/mock_transport`. From this PR onward, lifecycle/move/calibration tests can use a real `feetech.Bus` over an in-process transport without hardware.
- **PR2** adds the first move-path tests using the harness.
- **PR3** adds pure-logic tests for conversion math (no harness needed).
- **PR4** adds surface tests using the harness (gripper inference, opMgr cancellation).
- **PR5** adds the first state-machine tests using the harness.

By PR8, the existing `t.Skip("hardware-dependent")` tests in `registry_test.go` should be re-evaluated. Many can be re-enabled with the harness; the rest are tagged `//go:build hardware` and excluded from the default run.

## Migration & compatibility

- **PR2** is the first user-observable behavior change: speed/accel actually take effect. Users who configured these expecting them to do nothing (unlikely but possible) will see slower motion. Document in release notes.
- **PR4** is the second: a second `MoveToJointPositions` request cancels the first instead of queueing. Document in release notes. Likely to surface in the motion service's corrective-move path; this is the *correct* behavior per RDK convention.
- All other PRs are refactors or surface fixes with no user-observable contract changes.

## Open questions

- `cmd/cli/` triage: the 4 "kept" tools listed in PR8 are a guess — Nick may want a different cut. Settled at implementation time by skimming each tool's actual utility.
- Whether to extract `internal/` packages comes after PR8 in the architecture revisit, not in this spec.
- `Reconfigure` support (today everything is `AlwaysRebuild`) is not in scope; revisit when the architecture decision is made.

## Architecture revisit (post-PR8)

After PR8, evaluate:

1. Are the singleton's remaining boundary issues (cross-file access from calibration sensor into `cs.controller.bus`, the controller still being a separate type from the arm) painful enough to justify the arm-as-core flip?
2. Would the gripper-as-DoCommand-client model actually solve those issues, or just relocate them?
3. What's the cost of the migration vs. the cost of living with the cleaned-up singleton?

Decision and ADR live in a separate spec written at that time.

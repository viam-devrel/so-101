# MoveThroughJointPositionsStreamed: time-scheduled setpoints

**Date:** 2026-09-04
**Branch:** `streamed-trajectory` (from `main` @ 7a33fae; rebase onto main after
`feetech-0.7-condition-reads` merges — only `go.mod`/`go.sum` overlap)
**Status:** approved design, pre-implementation. Experimental feature.

## Goal

Let a producer of dense, time-stamped joint trajectories (a TOTG like trajex sampling at
100 Hz, or arm-recorder replaying a recording) stream them to the SO-101 and have the arm
follow the producer's timing, instead of the module re-pacing each waypoint by position.
This exists to *measure* whether time-scheduled streaming tracks better than the paced
`MoveThroughJointPositions` path; it is not yet a replacement for it.

## Prerequisite: rdk v1.6.0

`MoveThroughJointPositionsStreamed` is a required method on `arm.Arm` from rdk v1.1.0.
Probe result on v1.6.0: the **only** compile errors are the missing method on
`components/arm` and `components/simulated`; all other packages' tests pass unchanged.
`go.mod` moves to `go 1.25.10`, `go.viam.com/api v0.1.577`, `go.viam.com/utils v0.10.1`
(plus grpc/otel indirect churn). CI uses `go-version-file: go.mod`; no workflow edits.

## The rdk contract (v1.6.0, `components/arm/arm.go:149-209`, `server.go:153-283`)

```go
MoveThroughJointPositionsStreamed(ctx, batches <-chan []arm.TrajectoryPoint,
    responses chan<- arm.Response, extra map[string]any) error
type TrajectoryPoint struct { Time time.Duration; Positions []referenceframe.Input;
    Constraints *KinematicConstraints }   // Time from motion start; 0 first, strictly increasing
type Response struct{}                     // empty ack today
```

- The framework owns both channels (unbuffered); the impl only reads `batches` and only
  writes `responses`, and never closes either. Back-pressure on `batches` is the flow
  control. A recv-side fault takes precedence over the impl's return.
- Server calls `operation.CancelOtherWithLabel` before invoking the impl, but that is a
  different manager from `so101.opMgr`, so the impl registers itself.
- Every in-tree rdk arm (fake, sim, wrapper) ignores `Time` and acks once per batch.

## Design

### Hardware arm (`components/arm/motion.go`)

Same envelope as `MoveThroughJointPositions`: `opMgr.New(ctx)`, `exitManualLocked`,
`moveLock`, `isMoving.Store(true)` / deferred `false`.

1. **First point gate.** Read `currentJoints()` once. If any joint is farther than
   `streamStartGapDeg = 5` (teleop's `maxStreamStartGapDeg`) from point 0, run a
   blocking profiled `moveJoints(from, p0, defaultSpeed, defaultAcc, nil, true)` to it.
   Reason: the streamed write path is `Speed 0 / Acc 0` (unbounded), so it must never be
   handed a large position error (CLAUDE.md, "It removes a bound"). Then `start = now()`.
2. **Every point** (including point 0 after the gate): `clampPositions`; validate
   `Time` strictly increasing (first must be 0) else return an error; sleep until
   `start + p.Time` via a `sleepUntil(ctx, t) error` seam (returns `ctx.Err()` on cancel);
   write with the existing `moveJointsUniform(ctx, clamped, speed, accel, streamed=true,
   wait=false)` — one `SetGoals` packet, no coordination read. A point whose time has
   already passed is written immediately, never dropped: a later goal supersedes it in
   ~1 ms and dropping would need a policy nothing has measured yet.
3. **Per batch:** after its last point, `select { case responses <- arm.Response{}:
   case <-ctx.Done(): return ctx.Err() }`.
4. **End:** when `batches` closes, `WaitForServosToStop(ctx, armServoIDs,
   servo.MaxMoveTimeoutMs)` so completion means "arrived", return nil.
5. `Constraints` are ignored (the streamed contract is that the producer shapes the
   trajectory; any servo-side profile is lag). `extra` is accepted and unused beyond what
   `moveJointsUniform` already reads.
6. `Stop` needs no change: `opMgr.CancelRunning` cancels ctx, the sleep/select returns,
   the loop exits before `controller.Stop` zeroes goals.

### Simulated arm (`components/simulated/simulated.go`)

Same loop shape. Per point: sleep until `start + p.Time`, then set `s.operation` to a new
`simOperation{targetInputs}` **without** waiting for `done`; the existing 10 ms tick loop
interpolates toward it at the configured speed. Ack per batch. After `batches` closes,
wait for the final operation's `done`/`stopped`. No `Time` validation beyond the hardware
arm's (shared helper in `internal/servo` or a small unexported func — decide in the plan;
do not create a package for it).

### Client for hardware experiments (`tools/stream_trajectory/`)

A small Go program on rdk v1.6.0: connects to a machine, loads an arm-recorder session
JSON (its `Frames` with timestamps), optionally linearly densifies to a target Hz
(default 100), and streams it in batches of N points via
`arm.Client.MoveThroughJointPositionsStreamed`, draining acks. arm-recorder itself is on
rdk v0.131 and cannot call the RPC, and trajex is a library, not an arm caller, so
nothing else can exercise this on hardware today. Kept out of the module binary.

## Tests (`components/arm`, `components/simulated`)

Use `accelTestArm`/`FakeTransport` and a `sleepUntil` seam pinned to "return
immediately, record the deadline". Pin:

- one bus packet per point after the gate (`PacketCount()` delta), zero reads inside the
  loop;
- deadlines recorded equal `start + p.Time` in order;
- gate: first point within 5° → no profiled move; outside → exactly one profiled
  `moveJoints` (its coordination read + write) before streaming;
- one `Response{}` per batch, none for an empty batch is *not* required (server never
  sends one);
- ctx cancel mid-stream returns `ctx.Err()` and writes no further goal;
- non-zero first `Time` or non-increasing `Time` → error before any write;
- simulated: `currInputs` converge to the last point; ack per batch.

`go test ./...`, `-race` on both arm packages, `go vet`, `gofmt -s`.

## Docs

- `docs/arm.md`: "Streaming trajectories (experimental)" — contract, 5° gate, late-point
  policy, Constraints ignored, how to run `tools/stream_trajectory`.
- `CLAUDE.md` gotcha: streamed path is time-scheduled and unbounded; the 5° gate is
  what makes that safe; late points are written not dropped; rdk ≥ v1.1.0 required.

## Alternatives rejected

- **Position-paced (ignore `Time`)**: what rdk's fake/sim do; defeats the experiment.
- **Hybrid that drops stale points by lookahead**: plausible improvement, but adds a
  policy before any measurement says late points are a problem. Add if hardware shows it.
- **Per-register `extra` flags or a fallback switch**: nothing wants them yet.

## Follow-ups (not this PR)

- Measure on hardware: tracking error vs the paced path on the same recording; whether
  late points accumulate at 100 Hz on a busy bus.
- Bump arm-recorder to rdk ≥ v1.1.0 and add a streamed replay mode there.
- Decide whether `Response` should carry executed-up-to time once rdk defines fields.

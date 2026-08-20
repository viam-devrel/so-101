# Servo motion-profile bench harness

**Date:** 2026-08-19
**Status:** Approved design, pending implementation plan
**Affects:** `cmd/cli/` only — no shipped module code changes

## Problem

The SO-101 hardware arm drives its servos in **independent-joint speed mode**: every joint
is commanded at the same goal velocity (steps/sec), so joints with shorter travel arrive
first. The executed path is therefore not the joint-space path the motion planner produced
— the TCP cuts a different curve. `CLAUDE.md` documents this as intentional, mitigated by
the planner emitting dense waypoints that keep per-waypoint displacements small.

Two consequences motivate this work:

1. `acceleration_degs_per_sec_per_sec` is validated, stored, and settable via the
   `set_acceleration` DoCommand, but **never written to servo hardware**
   (`README.md:104` calls it a planned follow-up). Both
   `SafeSoArmController.MoveToJointPositions` (`manager.go:47`) and
   `MoveServosToPositions` (`manager.go:75`) accept an `acc int` parameter and drop it on
   the floor — they delegate to `writePositions` (`manager.go:36`), which takes only
   `speed`. The arm's move path goes through the latter: `arm.go:626` calls
   `MoveServosToPositions(..., speedSteps, 0)`.
2. Recent RDK work adds trajectory generation to the builtin motion planner, also available
   as the `viam-modules/trajex` modular resource. trajex is a Kunz–Stilman time-optimal path
   parameterizer: given waypoints plus per-joint velocity/acceleration limits it returns
   `(sample_times_sec, configurations_rads, velocities_rads_per_sec)`. **The SO-101 module
   cannot consume that output at all today** — `MoveThroughJointPositions` (`arm.go:686`)
   reads only `options.MaxVelRads` and discards `MaxAccRads`, `MaxVelRadsJoints`,
   `MaxAccRadsJoints`, and `MaxTCPSpeedMPerSec`, all of which RDK v1.0.0 already plumbs
   through `arm.MoveOptions` (`components/arm/pb_helpers.go:10`).

Consuming a time-parameterized trajectory requires the arm to honor *timing*. That is a
prerequisite, not an add-on. Before designing it we need measured numbers, not estimates.

## What this delivers (and does not)

This delivers a **throwaway bench harness** that characterizes what the Feetech STS3215
servos and the USB-serial bus actually do. It changes no shipped module code.

It explicitly does **not** implement coordinated arrival, acceleration enforcement, or
trajectory execution. Those are follow-on work, scoped from this harness's output.

Streaming fidelity (pushing a dense timed setpoint sequence and measuring tracking error)
is deliberately deferred to a later round, once the primitives are characterized.

## Available hardware capability

`feetech-servo v0.6.0` exposes all three profile controls. Only goal velocity is used today.

| Register | Addr | `feetech` API | Module uses it? |
|---|---|---|---|
| `RegAcceleration` | 41 | `GoalRequest.Acc` (0–255, doc claims ~100 steps/s² per unit) | No |
| `RegGoalPosition` | 42 | `SetPositions` | Yes |
| `RegGoalTime` | 44 | `SetPositionsWithTime`, `GoalRequest.Time` (ms) | No |
| `RegGoalVelocity` | 46 | `SetPositionsWithSpeed`, `GoalRequest.Speed` (steps/s) | Yes — only path |

The key primitive is `ServoGroup.SetGoals` (`group.go:193`): one `SyncWrite` of a 7-byte
block `[acc, pos_lo, pos_hi, time_lo, time_hi, speed_lo, speed_hi]` per servo, starting at
the acceleration register. Firmware uses `Time` when non-zero, otherwise `Speed`; `Acc: 0`
means an unlimited ramp.

Wire cost is effectively unchanged from today: 7 bytes/servo vs 6. A 5-servo sync write is
~48 bytes ≈ **0.48 ms at 1 Mbaud**, with no response packet.

## Known constraints

**Software rate floor — ~95 Hz, not ~1000 Hz.** This section originally claimed a ~1 ms /
~1000 Hz floor from `MinCommandGap` (default **1 ms**, `bus.go:465`) plus the **100 µs**
half-duplex turnaround in `sendPacketLocked` (`bus.go:472`). That was wrong by ~10×, and the
real cause was found during Task 8 review.

`sendPacketLocked` calls `transport.Flush()` before **every** packet (`bus.go:476`).
`SerialTransport.Flush` (`transports/serial_os.go:82`) sets a 10 ms read timeout and loops on
`Read` until it returns 0 — and `go.bug.st/serial`'s `unixPort.Read` blocks in `Select`,
returning `(0, nil)` only once that deadline expires (`serial_unix.go:59`). On an idle bus
there is no early return, so **every transaction pays a full 10 ms**, swamping any
`MinCommandGap` below it. The practical floor is **~95 Hz**.

That is load-bearing for the follow-on work: 100 Hz setpoint streaming sits *at* this
ceiling, not comfortably beneath it. It is a property of `feetech v0.6.0`'s transport, not of
the bus — a 5-servo `SetGoals` is 48 bytes ≈ 480 µs of wire time, so the physical ceiling is
~2080 Hz. The harness therefore ships a `fastFlushTransport` using `ResetInputBuffer` (an
immediate ioctl) and sweeps **both** transports, so the run measures the gap between what the
library costs today and what the bus can actually do.

The 100 µs turnaround is **not additive** to the gap: `sendPacketLocked` stamps
`b.lastCmdTime = time.Now()` at `bus.go:486`, *before* the `time.Sleep(100 * time.Microsecond)`
at `bus.go:489`. The next call's `enforceCommandGap` therefore measures elapsed time that
already contains the sleep, so back-to-back transactions period at ~1 ms, not ~1.1 ms.

**`MinCommandGap: 0` is not settable.** `NewBus` coerces a zero value back to 1 ms
(`bus.go:63-64`), silently and with no error. Anywhere this design says "no gap" the
implementation MUST pass a smallest-non-zero sentinel — **`1 * time.Nanosecond`** — and never
the literal `0`. Passing `0` would make the sweep's fastest row a mislabeled duplicate of the
1 ms row, and would cap motion sampling at ~1 kHz rather than at the link rate, in both cases
producing a plausible-looking wrong number rather than a visible failure.

**Half-duplex contention.** Reads and writes share one bus and one mutex. Sampling position
to recover a velocity profile costs a round trip per sample and serializes against setpoint
writes. High-rate streaming and high-fidelity sampling cannot be measured simultaneously;
the harness measures them separately.

**Sampling costs the same 10 ms.** The flush floor above applies to `SyncRead` too, so
single-servo sampling runs at ~95 Hz on the stock transport rather than the ~200–300 Hz
originally assumed here — roughly 19 samples across a 200 ms trial. Workable for duration and
peak velocity, marginal for ramp timing, which is why `moveProfile` carries a
`rampBelowSamplingResolution` flag. The motion tests default to the fast-flush transport for
~10× the sampling rate; it changes what the harness can *observe*, not what the servo does.

**Adjacent registers make one read serve for two.** `RegPresentPosition` (56, 2 bytes) and `RegPresentVelocity`
(58, 2 bytes) are adjacent, so a single `SyncRead(ctx, 56, 4, ids)` returns position *and*
velocity per servo in one round trip. The harness reads the servo's own velocity rather
than numerically differentiating noisy position samples. Practical sampling rate is
~200–300 Hz for a single servo — ample for a 1–2 s move. Sampling all five costs
proportionally more, which is why the acceleration sweep is single-joint.

## Design

### Shape

One standalone `package main` file, `cmd/cli/profile_bench.go`, matching the existing
debug-script convention. Per `CLAUDE.md`, `cmd/cli/` is excluded from
`go test ./cmd/module/ .` and holds multiple `func main()` in one package; this file is
run directly and never compiles in CI.

It talks to `feetech` **directly** rather than through `SafeSoArmController`, for two
reasons: it needs to vary `MinCommandGap` (which `registry.go` does not expose), and it
works in **raw encoder ticks** rather than calibrated radians, so measurements are not
conflated with calibration state and the harness runs on an uncalibrated arm.

```
go run cmd/cli/profile_bench.go -port=/dev/tty.usbmodemXXXX -test=writerate
```

| Flag | Default | Purpose |
|---|---|---|
| `-port` | *(required)* | serial port |
| `-test` | `writerate` | `writerate` \| `goaltime` \| `accel` \| `all` |
| `-servo` | `1` | servo for single-joint tests |
| `-travel` | `20` | degrees of travel per trial. `accel` only — `goaltime` uses the fixed 10/20/40 sweep below |
| `-move` | `false` | **required** for any test that moves the arm |
| `-full-arm` | `false` | coordinated 5-joint `goaltime` run (needs `-move`) |
| `-assume-limits` | *(unset)* | `<min>:<max>` raw-tick bounds, for arms whose angle-limit registers read invalid |
| `-out` | `.` | directory for CSV output |

`-test=all` without `-move` runs `writerate` only and logs a line naming the tests it
skipped and why, rather than erroring. That keeps `all` useful as a default first run while
leaving no ambiguity about what did not execute.

`-servo 1` (base rotation) is the default because it is gravity-unloaded and moves in a
horizontal plane, making it the safest default. Gravity-loaded joints 2 and 3 will behave
differently under the same `Acc`; characterizing them is a matter of re-running with
`-servo 2`.

### Safety

- Nothing moves without `-move`.
- `writerate` never moves, and additionally runs with **torque disabled**, so a mis-encoded
  position cannot actuate anything.
- Before any motion, read each servo's `RegMinAngleLimit`/`RegMaxAngleLimit`, clamp every
  target into it, and **refuse to run** if the requested travel would clip — rather than
  silently shortening a trial and reporting a bad duration.
- Those limits are **not always valid**. They can legitimately read `0/0` on a multi-turn
  servo or one with limits disabled, which is why `config.go:424` guards them with
  `minLimit < maxLimit && maxLimit <= 4095` before trusting them. The harness applies the
  same guard. If the limits fail it, the harness **refuses to move** and reports the servo
  ID and the raw values read, rather than clamping every target to zero and silently
  refusing every trial. An explicit `-assume-limits=<min>:<max>` flag lets the operator
  supply bounds and proceed on such an arm.
- Travel is always relative to the position read at startup, so there is no move-to-home
  step.
- `Speed` is capped during the acceleration sweep. `Acc: 0` means an unlimited ramp, and
  pairing that with a high velocity target is the one genuinely jerky combination here.
- Torque disabled on exit via `defer` and on SIGINT.

### Sampling primitive

```go
data, err := bus.SyncRead(ctx, feetech.RegPresentPosition.Address, 4, ids)
t := time.Now()   // stamped immediately after the read returns
// data[id][0:2] = position (sign-magnitude, SignBit 15)
// data[id][2:4] = velocity (signed, SignBit 15)
```

Motion tests run with `MinCommandGap: 1 * time.Nanosecond` — the "no gap" sentinel, never a
literal `0`, per the coercion noted above — to sample as fast as the link allows. Every
sample carries its own measured timestamp, so sampling jitter appears **in the data** rather
than being assumed uniform.

### Test: `writerate` (no motion)

For each `MinCommandGap` in {1 ns, 250 µs, 500 µs, 1 ms, 2 ms}, open a fresh bus and issue 500
`SetGoals` writes commanding the *current* position. The 1 ns row is the "no gap" sentinel —
reported as such — because a literal `0` is coerced to 1 ms and would duplicate that row.

Reports achieved Hz and inter-write latency **p50 / p95 / p99 / max**. The tail matters more
than the mean: timed streaming breaks on jitter, not on average throughput.

### Test: `goaltime` (motion)

Single joint by default. Sweep commanded `Time` ∈ {200, 400, 800, 1600, 3000} ms × travel ∈
{10°, 20°, 40°}, sampling until motion stops, then returning to start.

Per trial, report commanded vs. measured duration (first-motion to last-motion timestamps),
overshoot, and **whether the servo silently clipped**. A 40° move in 200 ms may exceed max
speed, in which case `GoalTime` is not honored; the boundary is exactly what we need to know.

With `-full-arm`, command all five joints to *different* travels under the *same* `Time` and
report per-joint arrival spread. This is the direct test of whether coordinated arrival works.

### Test: `accel` (motion)

Single joint, fixed travel, fixed `Speed` (not `Time` — acceleration shapes the ramp toward
a velocity target). Sweep `Acc` descending ∈ {255, …, 1, 0} (gentlest first), sampling the
velocity profile each time, returning to start between trials.

Report peak velocity reached, time-to-peak, and implied steps/s² per `Acc` unit. This
validates or refutes the library's "~100 steps/s² per unit" doc claim and finds where it
saturates — which is what any real `acceleration_degs_per_sec_per_sec` → register mapping
must be built on.

### Output

Per-sample CSV per test written into `-out`, so profiles can be plotted, plus a derived
summary table to stdout. The CSV is the durable artifact; the summary is for deciding scope
on the spot.

## Testing

The harness is hardware-driving throwaway code in `cmd/cli/`, which CI neither compiles nor
tests. Its correctness is established by running it against a physical arm and sanity-checking
that the measured numbers are physically plausible. The `writerate` output must pass these
before any of it is trusted, and note they apply to the **fastflush** rows: `MinCommandGap:
2ms` must report ≈500 Hz and `1ms` ≈1000 Hz. The **stock** rows are expected to be flat at
~95 Hz regardless of gap — that is the 10 ms `Flush` floor, not a fault. Only if a *fastflush*
1 ns row matches its own 1 ms row has the zero coercion been reintroduced; a flat stock sweep
means the opposite of a bug.

This was a deliberate trade-off. The alternative considered — putting profile recovery,
latency statistics, and unit conversion in the root package with unit tests against synthetic
samples, leaving only hardware I/O in `cmd/cli/` — would make the analysis code both testable
and directly reusable by the eventual acceleration/`GoalTime` implementation. It was rejected
in favour of keeping the experiment fully throwaway and zero-impact on the shipped module.
The cost is that a statistics bug would surface as a plausible-looking wrong number rather
than a failing test, so measured results should be sanity-checked against the analytical
estimates recorded above before being used to scope follow-on work.

## Follow-on work (out of scope here)

Scoped from this harness's results, in dependency order:

1. **Coordinated arrival** — switch the hardware arm from shared-speed to per-joint
   `GoalTime` via `SetGoals`, so all joints reach each waypoint together and the executed
   path is the planned joint-space path. Also aligns hardware with the simulated arm's
   existing coordinated-arrival semantics.
2. **Acceleration enforcement** — write `RegAcceleration` via `SetGoals`, closing the
   documented `acceleration_degs_per_sec_per_sec` gap and honoring `MoveOptions.MaxAccRads`.
3. **Trajectory execution** — a clock-driven executor consuming time-parameterized
   trajectories from trajex or RDK TOTG. Gated on the `writerate` jitter numbers.

## Notes

Unrelated latent issue observed while reading the sampling path, recorded rather than fixed:
`calibration.go:463` passes `len(cs.cfg.ServoIDs)` as `SyncRead`'s `dataLen`, which is a
bytes-per-servo argument, not a servo count. It works by accident because `DecodeWord` reads
only the first 2 bytes, but it over-reads (6 bytes/servo on a 6-servo config, spanning
position + velocity + load).

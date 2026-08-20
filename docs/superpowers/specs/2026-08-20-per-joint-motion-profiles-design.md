# Per-joint motion profiles: coordinated arrival, acceleration, and MoveOptions

**Date:** 2026-08-20
**Status:** Approved design, pending implementation plan
**Affects:** `devrel:so101:arm` (hardware). Simulated arm unchanged.
**Grounded in:** `docs/superpowers/results/2026-08-19-bench-run.md`

## Problem

The SO-101 hardware arm commands every joint at the *same* goal velocity, so joints with
shorter travel arrive first and the executed path is not the joint-space path the motion
planner produced. Measured on hardware: five joints commanded different travels arrive
**260–520 ms apart**, varying only with distance.

Two capabilities were supposed to address this and neither is wired up:

1. `acceleration_degs_per_sec_per_sec` is validated, stored, and settable via
   `set_acceleration`, but **never written to servo hardware** (`README.md:104` calls it a
   planned follow-up). `SafeSoArmController.MoveToJointPositions` (`manager.go:47`) and
   `MoveServosToPositions` (`manager.go:75`) both accept an `acc int` and drop it —
   `writePositions` (`manager.go:36`) takes only `speed`.
2. `MoveThroughJointPositions` (`arm.go:686`) reads only `options.MaxVelRads`, discarding
   `MaxAccRads`, `MaxVelRadsJoints`, and `MaxAccRadsJoints` — all of which RDK v1.0.0
   already plumbs through `arm.MoveOptions` (`components/arm/pb_helpers.go:10`).

## What the bench run established

The 2026-08-19 hardware run settled three things this design depends on.

**Time-based goals are impossible and are out of scope.** `RegGoalTime` (44) is inert on
STS servos. FEETECH's own SDK defines `SMS_STS_GOAL_TIME_L 44` but references it nowhere in
`SMS_STS.cpp`, and `WritePosEx` has no time parameter — it writes a hardcoded 0 there. It is
an SCS-series feature. Measured confirmation: travel time depends only on distance, with a
10° move taking ~335 ms whether commanded at 200 ms or 3000 ms.

**Motion is a clean acceleration-limited profile.** Predicting travel from the independent
acceleration sweep via `t = 2√(d/a)` matched measurement within 2% at 20° and 40°.

**The acceleration register is linear from Acc 1 to ~50, then saturates.**

| servo | steps/s² per unit | offset | ceiling | knee |
|---|---|---|---|---|
| 1 (base, unloaded) | 127.6 | +533 | 7099 (624 °/s²) | Acc ≈ 52 |
| 2 (shoulder, loaded) | 89.3 | +329 | 5282 (464 °/s²) | Acc ≈ 55 |
| 3 (elbow, loaded) | 108.2 | +296 | 7140 (628 °/s²) | Acc ≈ 63 |

Servos 4 and 5 were not measured.

## Core insight: acceleration must be coordinated, not just speed

Motion is speed-limited (trapezoidal) only when `d > v²/a`. At the defaults below that
crossover is **~4.2°**, and motion-planner waypoints are typically shorter — so **most
planned motion is acceleration-limited**, where scaling per-joint *speed* accomplishes
nothing.

Scaling **both** `v` and `a` by each joint's share of the travel makes every joint's profile
a time-scaled copy of the same shape, so durations match in either regime:

- triangular: `2√(kd / ka) = 2√(d/a)`
- trapezoidal: `kd/kv + kv/ka = d/v + v/a`

This is the whole design in one line: `kᵢ = dᵢ / d_max`, then `vᵢ = v_max·kᵢ` and
`aᵢ = a_max·kᵢ`.

## Design

### Acceleration mapping — a single global constant

`Acc = (a_steps − 386) / 108`, clamped to `[1, 50]`, where 108 steps/s² per unit is the mean
measured slope and 386 the mean offset.

Per-joint calibration was considered and rejected: only three of five joints were measured,
and the 89–128 spread implies roughly ±20% per-joint error, i.e. ~10% arrival-timing error
between joints. Against a current spread of 50–100% of move duration, that is a large
improvement for none of the cost of a per-joint table that would be specific to one arm's
servos. The offset term is partly a fit artifact and should be treated as approximate.

The clamp at 50 matters: above the knee the register does nothing, so values beyond it would
silently stop scaling.

### Travel reference

`MoveToJointPositions` reads current position **once** (~0.6 ms on v0.6.1) and computes
travel from it.

`MoveThroughJointPositions` computes each segment's travel from the planner's own sequence,
`|q[k+1] − q[k]|`, with **no extra bus traffic**. This honors the planned path shape rather
than wherever the arm happens to be, and avoids adding a read between every waypoint — which
would risk reintroducing the stutter that back-to-back streaming currently avoids.

Accepted consequence: if the arm lags behind the stream, commanded deltas understate actual
travel. The *ratios* still reflect the planned path, which is what coordination needs.

### `MoveOptions` caps reduce the shared reference

Per-joint caps are constraints, not targets. Honoring one by slowing a single joint would
destroy the coordination this design exists to create. Instead they lower the shared
reference:

```
v_max = min over moving joints of (vcapᵢ / kᵢ)
a_max = min over moving joints of (acapᵢ / kᵢ)
```

Every joint's command derives from the reduced reference, so coordination stays exact and
every cap is honored — the most-constrained joint sets the pace for all. Conservative, and
that is what a cap means in a coordinated move.

`MaxVelRads`/`MaxAccRads` apply to all joints; the `*Joints` slices override per-joint.
Joints with zero travel are excluded from the `min` (they would divide by zero).

### Register traps this code must encode

Two values mean the opposite of what they look like, and both have already caused real bugs
in this project:

- **`Speed: 0` means maximum, not stopped.** `speed.go`'s `minSpeedSteps = 1` already floors
  this; reuse it rather than reimplementing.
- **`Acc: 0` means unlimited, not zero.** A profile that rounds to 0 becomes the jerkiest
  possible command instead of the gentlest.

A short-travel joint can round to either. Both floors are load-bearing.

### Components

| File | Change |
|---|---|
| `motion_profile.go` | **New.** Pure functions: `kᵢ` scaling, unit conversion, clamping, cap reduction. No I/O. |
| `manager.go` | **New** `MoveServosWithProfiles` using `ServoGroup.SetGoals`. `writePositions` untouched. |
| `arm.go` | `moveJoints` computes travels; `MoveToJointPositions` reads position once; `MoveThroughJointPositions` uses deltas and honors `MoveOptions`. |
| `config.go` | Acceleration default 100 → 600 °/s²; validated max 500 → 700. |
| `go.mod` | `feetech-servo` v0.6.0 → v0.6.1 (flush fix). |

`SetGoals` is the only primitive that can write acceleration — a 7-byte sync write carrying
acc+pos+time+speed, the same wire cost as today's 6-byte write. It writes `Time: 0` always,
matching FEETECH's own `WritePosEx`.

`writePositions` stays exactly as it is because two of its four callers are single-servo
gripper paths (`manager.go:134`, `:154`) where coordination is meaningless.

## Speed cost, stated honestly

Writing any finite acceleration adds ramp time `v/a` that unlimited ramp does not. Today the
register is never written, so the servos run at whatever they hold — almost certainly
`Acc: 0`, unlimited.

At the new 600 °/s² default, a 20° move goes from ~0.40 s to ~0.48 s: **about 21% longer.**
At the old documented 100 °/s² default it would have been ~0.90 s, **123% longer** — which is
why the default moves to 600 rather than honoring the number the config has been advertising.

This is a real regression on upgrade, accepted deliberately: it is the price of both smoother
motion and the ability to coordinate acceleration at all.

## Known limitation, deliberately accepted

`Acc: 1` has a floor of ~206–627 steps/s². A short-travel joint in a coordinated move can
require *less* acceleration than the register can express, and will arrive early. The profile
function clamps and logs at debug rather than pretending otherwise. Mitigation: such a joint
is usually also speed-limited, where scaling still works.

## Rollout

Coordination ships **on by default with no opt-out flag**. It is what the motion planner
already assumes, so this is a bug fix rather than a feature toggle; total move duration is
unchanged (the longest-travel joint still runs at the full reference); and a config
permutation that nobody exercises is worse than a patch release if something misbehaves.

## Testing

Unlike the bench harness, this is **shipped module code** and gets real unit tests under
`go test ./cmd/module/ .`.

`motion_profile.go` is pure by design so the math is directly testable. Cases:

- equal travels → equal profiles
- 4:1 travel ratio → 4:1 speed and acceleration
- zero travel on one joint, and on all joints
- the `Speed: 0` and `Acc: 0` floors, including a travel small enough to round into them
- a per-joint cap reducing the shared reference for everyone
- a cap on a *short*-travel joint correctly slowing the whole move (the case that
  distinguishes reference-reduction from per-joint clamping)
- `Acc` clamped at the knee of 50, and at the floor of 1

**Hardware verification** reuses `cmd/cli/profile_bench.go`. A `-full-arm` `goaltime` run
should show arrival spread collapse from the measured 260–520 ms to roughly 10% of move
duration. This is the gate on whether the approach is worth committing to.

## Documentation

`README.md:104`'s "validated and stored but not yet enforced" note is removed. `CLAUDE.md`'s
documented sim/hardware divergence becomes resolved rather than intentional — hardware gains
the coordinated arrival the simulated arm already had.

## Out of scope

- **Time-based goals.** Not deferred — impossible on this servo series.
- **Trajectory execution.** Now viable on v0.6.1 (~1750 Hz at 500 µs gap, p99 within 24 µs
  of p50) but needs the component to time-parameterize itself, and needs this work first.
- **Simulated arm changes.** It already coordinates arrival.
- **Per-joint acceleration calibration.** Rejected above; revisit only if the ~10% timing
  error proves to matter.

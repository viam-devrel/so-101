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

Motion is speed-limited (trapezoidal) only when `d > v²/a`. At the defaults (50 °/s,
500 °/s²) that crossover is **5.0°**, and motion-planner waypoints are typically shorter — so
**most planned motion is acceleration-limited**, where scaling per-joint *speed* accomplishes
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

**The default must land below the clamp, which is why it is 500 °/s² and not higher.**
600 °/s² maps to `Acc 59.6` and clamps to 50. Clamping the *reference* joint (`k = 1`) while
joints below `k ≈ 0.85` keep their exact scaled value breaks coordination in exactly the
regime this design targets: the reference arrives ~9% late in the triangular case, and every
joint above `k ≈ 0.85` collapses to the same register value, so acceleration scaling stops
working across the top 15% of the range on every move. 500 °/s² maps to `Acc 49.1` — the
largest default that leaves the whole `k` range unclamped — and is still 5× the 100 the
config has been advertising.

The validated range becomes **[50, 500] °/s²**, and the maximum stays at 500 rather than
rising. Two reasons: values above ~508 are unreachable (`Acc 50` delivers 508.5 °/s² under
the mean mapping, and less on servo 2, whose measured ceiling is 464), and the bench results
doc recommends lowering this bound, not raising it. The minimum rises from 10 because
`Acc: 1` delivers ~43 °/s² — anything below that silently clamps, so a configured 10 would
have been off by more than 4×.

Per-joint calibration was considered and rejected: only three of five joints were measured,
and the 89–128 spread implies roughly ±20% per-joint error, i.e. ~10% arrival-timing error
between joints. Against a current spread of 50–100% of move duration, that is a large
improvement for none of the cost of a per-joint table that would be specific to one arm's
servos. The offset term is partly a fit artifact and should be treated as approximate.

The clamp at 50 matters: above the knee the register does nothing, so values beyond it would
silently stop scaling.

### Travel reference

**One position read per call, never per waypoint.**

`MoveToJointPositions` reads current position once and computes travel from it.

`MoveThroughJointPositions` reads once for the **first** waypoint — `q[0]`'s travel is
measured from where the arm actually is, since there is no preceding waypoint to difference
against, and single-waypoint streams (which `GoToInputs` at `arm.go:762` emits routinely) have
no deltas at all. Every subsequent segment uses `|q[k+1] − q[k]|` from the planner's own
sequence, with no further bus traffic. This honors the planned path shape rather than wherever
the arm happens to be, and avoids a read between every waypoint, which would risk
reintroducing the stutter back-to-back streaming currently avoids.

Accepted consequence: if the arm lags behind the stream, commanded deltas understate actual
travel. The *ratios* still reflect the planned path, which is what coordination needs.

**The read becomes unconditional**, where today `moveJoints` reads only when `wait` is true
(`arm.go:615-623`). That condition exists because the read fed a completion-timeout estimate
needed only when waiting — and because on v0.6.0 a read cost ~12 ms, which is why teleop
passes `wait: false` to avoid it. On v0.6.1 the same read is roughly 1 ms, so coordinating the
`wait: false` teleop path costs a few percent of a 30–50 Hz setpoint budget rather than a
third of it. **This makes the v0.6.1 bump a correctness dependency, not just a performance
one.** If teleop latency proves to suffer in practice, the documented fallback is to skip
coordination when `wait` is false and retain uniform-speed behavior there.

Note the read latency above is estimated from the bench run's *write*-rate table; reads carry
a response packet and were not separately measured.

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

Following `arm.proto`: `MaxVelRads`/`MaxAccRads` apply to all joints, and are **ignored
entirely** when the corresponding `*Joints` slice is set — not merged with it. RDK does not
enforce this (`pb_helpers.go:26-28` explicitly leaves it to the implementer), so this driver
must.

Joints with zero travel are excluded from the `min` (they would divide by zero); their
commanded speed is `v_max·0`, floored to 1 step/s, which is harmless because goal equals
current position.

A `*Joints` slice whose length does not match the arm's DoF is **rejected with an error**.
RDK sizes the slice from whatever the client sent with no DoF check (`pb_helpers.go:40-51`),
and silently ignoring or truncating a mismatched slice would produce motion that violates a
cap the caller believes is in force.

### Degenerate cases

**All joints at zero travel:** no-op. `d_max = 0` makes every `kᵢ` undefined and `min` over
the empty set meaningless, so the move returns without writing anything — commanding an arm
to where it already is has no work to do.

**A single moving joint:** `k = 1` for it, so it moves at the full reference and the result
is identical to today's behavior. No special case needed.

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
| `arm.go:432,434` | Acceleration default 100 → 500 °/s²; validated range 10–500 → 50–500. **Not `config.go`**, which only declares the JSON tag (`config.go:24`). |
| `arm.go:1027` | The `set_acceleration` DoCommand carries a **second** copy of the bounds check that must change in lockstep. |
| `go.mod` | `feetech-servo` v0.6.0 → v0.6.1. Needed for the **flush fix** only — `SetGoals` already exists at v0.6.0. |

`SetGoals` is the only primitive that can write acceleration — a 7-byte sync write carrying
acc+pos+time+speed, against today's 6-byte write. That is +5 bytes on a ~48-byte 5-servo
packet, immaterial. It writes `Time: 0` always, matching FEETECH's own `WritePosEx`.

Scaled per-joint speeds must convert through **`degPerSecToStepsPerSec`** (floor 1 step/s),
**not** `resolveSpeedDegsPerSec` — the latter's `minSpeedDegsPerSec = 3.0` floor would destroy
scaling for any joint below `k ≈ 0.06`. Both live in `speed.go`.

`MoveServosToPositions` has one caller outside the arm path, `manual_mode.go:266` (speed 0,
acc 0). It must keep today's behavior.

`writePositions` stays exactly as it is because two of its four callers are single-servo
gripper paths (`manager.go:134`, `:154`) where coordination is meaningless.

## Speed cost, stated honestly

Writing any finite acceleration adds ramp time `v/a` that unlimited ramp does not. Today the
register is never written, so the servos run at whatever they hold — almost certainly
`Acc: 0`, unlimited.

At the new 500 °/s² default, a 20° move goes from 0.400 s to 0.500 s: **25% longer**
(trapezoidal, crossover 5.0°). At the old documented 100 °/s² it would have been 0.894 s,
**124% longer** (triangular, crossover 25°) — which is why the default moves rather than
honoring the number the config has been advertising.

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
- the shipped default maps to `Acc 49`, i.e. **below** the clamp, so the reference joint is
  not clamped — a regression here silently breaks coordination at the default
- a `*Joints` slice of the wrong length is rejected
- a `*Joints` slice set alongside the scalar ignores the scalar

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

# Gripper travel guard

**Date:** 2026-08-18 (revised 2026-08-20 with hardware measurements)
**Status:** Measured on hardware — the two open questions of the first draft are now answered,
and one of them came back the opposite of what was assumed
**Affects:** `devrel:so101:gripper` (behavior), `ReadCalibrationFromServos`,
`DefaultSO101FullCalibration`

## The problem

Some SO-101 kits ship "pre-calibrated": the vendor sets each servo's position offset so the
jaw's home reads ~2048, but leaves the min/max position-limit registers alone.
`ReadCalibrationFromServos` accepts that as a calibration — its check is
`minLimit < maxLimit && maxLimit <= 4095`, which the factory `0/4095` satisfies. So a servo
whose zero point is known but whose travel is not reads as fully calibrated.

That is survivable for the arm and not for the gripper, because the two normalization modes
use the recorded range differently:

| Mode | Servos | Uses the range as | Cost of a wrong range |
|---|---|---|---|
| `NormModeDegrees` | 1-5 | a center point, then clamps | center shifts ~47 ticks (≈4°) |
| `NormModeRange100` | 6 | a **linear scale factor** | 0-100% stretches across the full encoder |

`NormModeRange100` maps 0% → `RangeMin` and 100% → `RangeMax`, and `gripper.go` uses 0% as
*closed* (`closedPosition: 0.0`) and 95% as open (`openPosition: 95.0`). So both ends of the
recorded range are commanded positions, and both can land past a mechanical stop.

Measured through the real `Denormalize` path:

| Range | 0% (grab) | 50% | 100% | Span |
|---|---|---|---|---|
| Factory `0/4095` | tick 0 | 2048 | 4095 | 4095 |
| Module default `500/3500` | tick 500 | 2000 | 3500 | 3000 |
| Guarded (this design) | tick 2048 | 2648 | 3248 | 1200 |

A factory-limits servo is commanded well past its travel at both ends. It drives into the
mechanical stop, stalls, and latches its overload protection — the reported symptom of sync
reads failing until a power cycle.

Note the module's own placeholder default for servo 6 (`500/3500`, copied from the arm joints)
is 2.4× the jaw travel, so a user with *no* calibration hits a milder version of the same
thing. This revision fixes that default too; see "The default was the larger hole".

## Two ways users reach this state

1. **Vendor "pre-calibrated" kits**, as above.
2. **An abandoned calibration run.** `resetCalibrationRegisters` (`calibration.go:387`)
   writes `min_angle_limit = 0` and `max_angle_limit = 4095` at calibration *start*
   (`calibration.go:421`), deliberately widening the limits so the user can move each joint
   freely while recording. Anyone who begins the workflow and stops before recording ranges
   leaves their servos in exactly this state.

That second path also settles what `0/4095` means here: this codebase already treats it as
the factory/unset value, under a function named for resetting to factory defaults.

## What the hardware said

Probed with `cmd/cli/gripper_limits.go` across several kits. Two findings, both of which
change the first draft's design rather than confirming it.

**1. Encoder center is the jaw's *closed* stop, not its mid-travel.** A half-turn homing
offset puts 2048 at the closed end; the jaw opens from there toward higher ticks, on every
kit probed. The first draft assumed 2048 was mid-travel and substituted a window *centered*
on it — `1548..2548`. That puts 0% / grab at tick 1548, five hundred ticks past the closed
stop. The original guard therefore still drove the jaw into a stop; it had only moved the
collision from the open side to the closed side.

**2. Real travel exceeds the kinematic model.** A recorded calibration read `2030..3481` —
span 1451 against the URDF-derived 1251. The first draft worried the model would *overstate*
travel and sized its window at 0.8× to compensate. The error runs the other way.

## Design

Do **not** reject an implausible servo. The position offset is real information — it says
where the closed stop sits in encoder space — and only the *width* is missing. So keep the
offset's implied closed stop and substitute a conservative open travel, anchored to it.

```
  2048 ────────────────────► 3248
   0% (grab)            100% (open)
```

- **The window is anchored, not centered.** `RangeMin = 2048` is the measured closed stop;
  `RangeMax = 2048 + 1200`.
- **`gripperSafeTravelTicks = 1200` is a bench measurement**, held deliberately under the
  observed ~1450 so a kit with less travel under-opens rather than stalling. It is a separate
  constant from `gripperTravelTicks()` on purpose: the model demonstrably does not match the
  hardware (1251 vs 1451), so a kinematics edit must not silently move a hardware limit.
- **The closed end carries no margin, by decision.** 2048 is where the stop is; an empty-jaw
  `Grab()` pressing lightly into it is the same thing a properly calibrated kit does. All the
  slack (251 ticks) is on the open side. `openPosition` is 95%, not 100%, so `Open()` actually
  commands tick 3188 — a further 60-tick cushion that falls out of existing behavior.
- **Plausibility is judged by width**, not by matching specific factory values, so any
  too-wide range is caught rather than only `0/4095`. Threshold stays `1.25 ×` the modeled
  travel = 1564 ticks: the measured real calibration (1451) clears it, while `500/3500` (3000)
  and `0/4095` do not.
- **Applied inside `ReadCalibrationFromServos`**, which is the function that produces the
  untrustworthy calibration. Callers with a calibration file never reach it, so a recorded
  calibration is never second-guessed.
- **Arm servos are untouched.** Degrees mode takes only a center point from the range.

The substituted window is always strictly narrower than what it replaces, so the guard cannot
command anything the current code would not have.

## The default was the larger hole

`DefaultSO101FullCalibration` gave servo 6 `500/3500` — span 3000, 2.4× the jaw travel. That
default is used whenever no range is read: bus unavailable (`config.go:406`), servo 6's
registers unreadable (`config.go:318` via `FromFeetechCalibrationMap`), and
`GetSharedController` (`manager.go:513`). **None of those paths reach `guardGripperTravel`**,
so the guard as first written left the original stall fully reachable.

Fix: the default *is* the safe window. `RangeMin: gripperSafeRangeMin, RangeMax:
gripperSafeRangeMax`. One definition of "the conservative gripper range", shared by the guard
and the default, so the two cannot drift.

## Warnings: by cause, not by width

Making the default safe has a side effect that must be handled deliberately:
`guardGripperTravel(DefaultSO101FullCalibration, …)` now finds its input plausible and
returns `replaced=false`. The default paths would go **silent** — a gripper quietly running a
1200-tick window with no indication it is uncalibrated.

So "running on the safe window" has two distinct causes and only one of them is a too-wide
range. One helper, `warnGripperUsingSafeRange(logger, cause)`, three call sites:

| Cause | Site |
|---|---|
| Servo 6 reported a range too wide to be real | `guardGripperTravel` — names the reported range, and `0/4095` explicitly as the uncalibrated signature |
| Servo 6 was requested but produced no calibration | end of `ReadCalibrationFromServos` |
| Bus unavailable | `ReadCalibrationFromServos`'s nil-bus early return |

The second case requires that 6 was actually in `servoIDs`, so an arm-only machine configured
`servo_ids: [1,2,3,4,5]` does not warn about a gripper it does not have. (In practice
`arm.go:450` builds the controller with all six regardless of the arm's own `servo_ids`.)

All three end in the same sentence: the jaw will not open fully, run the calibration workflow.

## It does not break the workflow it recommends

- Calibration **disables torque** and has the user move joints by hand; positions are read
  raw via `bus.SyncRead(RegPresentPosition)` (`calibration.go:463`), bypassing the calibration
  mapping entirely. A narrowed range cannot truncate what gets recorded.
- On completion it writes the recorded limits back to the servo registers
  (`calibration.go:757-763`) **and** saves a file. So on the next start the guard either sees
  a plausible range or is skipped for `fromFile`. It disarms itself.

## Testing

The bug this revision fixes was invisible to the original tests because every one of them
asserted arithmetic about the window rather than what the window *means*. The load-bearing
addition is a test that drives `Denormalize` at 0% and 100% and asserts ticks 2048 and 3248 —
the only test that would have caught a window centered on the closed stop.

Alongside it: the plausibility table swaps its invented "a real calibration" case (`1900,
2900`, itself centered) for the measured `2030, 3481`; a test pins `config.go`'s default to
the shared constants; and a test with an observed logger covers the paths that would otherwise
go silent once the default is safe.

## Out of scope

- **Auto-discovering the range** by current-limited seek to both stops at construction. That
  is a real feature and effectively a mini-calibration; it needs hardware validation of its
  own.
- **Arm servos.** Degrees mode makes a wrong range cost a few degrees of center offset.
- **Tightening `ReadCalibrationFromServos`'s `min < max` check** for all servos. Rejecting
  `0/4095` outright would discard a usable offset, which is the thing this design
  deliberately avoids.

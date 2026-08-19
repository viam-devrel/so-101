# Gripper travel guard

**Date:** 2026-08-18
**Status:** Proof of concept — two assumptions unverified on hardware (see Open questions)
**Affects:** `devrel:so101:gripper` (behavior), `ReadCalibrationFromServos`

## The problem

Some SO-101 kits ship "pre-calibrated": the vendor sets each servo's position offset so
mid-travel reads ~2048, but leaves the min/max position-limit registers alone.
`ReadCalibrationFromServos` accepts that as a calibration — its check is
`minLimit < maxLimit && maxLimit <= 4095`, which the factory `0/4095` satisfies. So a servo
whose zero point is known but whose travel is not reads as fully calibrated.

That is survivable for the arm and not for the gripper, because the two normalization modes
use the recorded range differently:

| Mode | Servos | Uses the range as | Cost of a wrong range |
|---|---|---|---|
| `NormModeDegrees` | 1-5 | a center point, then clamps | center shifts ~47 ticks (≈4°) |
| `NormModeRange100` | 6 | a **linear scale factor** | 0-100% stretches across the full encoder |

Measured through the real `Denormalize` path:

| Range | 0% | 50% | 100% | Span |
|---|---|---|---|---|
| Factory `0/4095` | tick 0 | 2048 | 4095 | 4095 |
| Module default `500/3500` | tick 500 | 2000 | 3500 | 3000 |
| Guarded | tick 1548 | 2048 | 2548 | 1000 |

The jaw's real travel is ~1251 ticks (the URDF gripper joint spans −10°..100°, i.e. 110° of
4095). So a factory-limits servo is commanded 3.3× past its travel at full open. It drives
into the mechanical stop, stalls, and latches its overload protection — the reported symptom
of sync reads failing until a power cycle.

Note the module's own placeholder default for servo 6 (`500/3500`, copied from the arm
joints) is 2.4× the jaw travel, so a user with *no* calibration hits a milder version of the
same thing.

## Two ways users reach this state

1. **Vendor "pre-calibrated" kits**, as above.
2. **An abandoned calibration run.** `resetCalibrationRegisters` (`calibration.go:387`)
   writes `min_angle_limit = 0` and `max_angle_limit = 4095` at calibration *start*
   (`calibration.go:421`), deliberately widening the limits so the user can move each joint
   freely while recording. Anyone who begins the workflow and stops before recording ranges
   leaves their servos in exactly this state.

That second path also settles what `0/4095` means here: this codebase already treats it as
the factory/unset value, under a function named for resetting to factory defaults. The guard's
premise does not rest on a datasheet reading.

## Design

Do **not** reject such a servo. The position offset is real information — it says where
mid-travel sits in encoder space — and only the *width* is missing. So keep the offset's
implied center and substitute a conservative window around it, sized from the kinematic model
rather than the registers.

- **Plausibility** is judged by width, not by matching specific factory values, so any
  too-wide range is caught rather than only `0/4095`. Threshold: `1.25 ×` modeled travel,
  leaving headroom for real calibrations that legitimately overshoot the model (mechanical
  slop; the URDF limits are themselves approximate).
- **The substitute** is `2048 ± (0.8 × travel / 2)` = `1548..2548`. Under 1.0 on purpose: if
  the centering assumption is wrong, the failure is "the jaw does not open all the way", not
  "the servo drives into a stop".
- **Travel is derived** from `gripperJointMin/Max` rather than hand-copied, so it tracks the
  kinematics if those change.
- **Applied inside `ReadCalibrationFromServos`**, which is the function that produces the
  untrustworthy calibration. Callers with a calibration file never reach it, so a recorded
  calibration is never second-guessed.
- **Arm servos are untouched.** Degrees mode takes only a center point from the range.

The substituted window is always strictly narrower than what it replaces, so the guard cannot
command anything the current code would not have.

## It does not break the workflow it recommends

The warning tells the user to run calibration. Verified that still works:

- Calibration **disables torque** and has the user move joints by hand; positions are read
  raw via `bus.SyncRead(RegPresentPosition)` (`calibration.go:463`), bypassing the calibration
  mapping entirely. A narrowed range cannot truncate what gets recorded.
- On completion it writes the recorded limits back to the servo registers
  (`calibration.go:757-763`) **and** saves a file. So on the next start the guard either sees
  a plausible range or is skipped for `fromFile`. It disarms itself.

## Open questions

Neither is answerable without a servo. Both are in the hardware checklist below.

1. **Do vendors home the way LeRobot does?** The centering assumption rests entirely on
   mid-travel reading 2048, which is what `set_half_turn_homings` produces. If some kit sets
   the offset differently, the window is mis-centered — narrower than today, so not worse, but
   possibly unable to fully close.
2. **Is 0.8 conservative enough, and is 1.25 the right tolerance?** Both are invented. They
   should come off a real jaw.

## Hardware verification

1. On a pre-calibrated kit, read servo 6's `min_angle_limit`, `max_angle_limit`, and
   `position_offset`. Confirms whether kits really ship `0/4095`.
2. With torque disabled, hand-move the jaw to both stops and record the tick values. This
   gives the real center and the real travel — turning both open questions into measurements.
3. Compare that measured center against 2048. That is question 1, answered.
4. Compare the measured travel against `gripperTravelTicks()` (~1251). If the model overstates
   real travel, `gripperSafeTravelFraction` needs to come down.
5. With the guard active, drive the gripper to 0% and 100% and confirm no stall and no
   overload latch.

## Out of scope

- **Auto-discovering the range** by current-limited seek to both stops at construction. That
  is a real feature and effectively a mini-calibration; it needs hardware validation of its
  own.
- **Arm servos.** Degrees mode makes a wrong range cost a few degrees of center offset.
- **Tightening `ReadCalibrationFromServos`'s `min < max` check** for all servos. Rejecting
  `0/4095` outright would discard a usable offset, which is the thing this design
  deliberately avoids.

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

## The conflict this design knowingly accepts

The two paths above reach `0/4095` under **incompatible homing conventions**, and the guard
cannot tell them apart from the registers.

| Path | Homing convention | What tick 2048 is |
|---|---|---|
| Vendor pre-calibrated kit | LeRobot `set_half_turn_homings` | the jaw's **closed stop** |
| Abandoned module calibration | `calibration.go:375` — "move the robot to the **middle** of its range of motion, then use `set_homing`" | the jaw's **mid-travel** |

Anchoring at 2048 is correct for the first and wrong for the second: on a mid-travel-homed
servo, 0%/grab lands half-open (it cannot close on anything) and `Open()` at tick 3188 is
roughly 415 ticks past a real open stop near 2773 — a stall, on the side this design exists to
protect. Note the first draft's centered `1548..2548` had it exactly backwards: correct for the
abandoned-calibration path, wrong for the vendor kits.

**Decision: anchor at 2048 and accept the path-2 risk.** The reasoning:

- The measurements this design rests on were taken on vendor kits, which is the state a user
  arrives in by doing nothing. Path 2 requires starting a calibration, issuing `set_homing`,
  and then abandoning the run.
- Even on path 2 the anchored window is a large improvement on the status quo: ~415 ticks of
  overshoot instead of the ~1100 that raw `0/4095` commands today.
- The warning's remedy — finish the calibration workflow — is precisely what a path-2 user was
  already doing.

The alternative considered and rejected was shrinking the span to ~600 ticks, which clears both
conventions but halves the jaw's usable opening on the common case. Trading a real, permanent
usability cost for every vendor kit against a transient state on a rarer one is the wrong side
of the trade.

Anyone revisiting this should know the ambiguity is removable at the source: making
`calibration.go`'s homing prompt agree with the vendor convention would collapse the two paths
into one. That was left out of scope because it changes what `set_homing` means for all six
servos.

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
  travel (1563.75 ticks — keep the multiplication, not a rounded literal): the measured real calibration (1451) clears it, while `500/3500` (3000)
  and `0/4095` do not.
- **Applied inside `ReadCalibrationFromServos`**, which is the function that produces the
  untrustworthy calibration. Callers with a calibration file never reach it, so a recorded
  calibration is never second-guessed.
- **Arm servos are untouched.** Degrees mode takes only a center point from the range.

The substituted window is always narrower in *width* than what it replaces. It is not
necessarily a *subset* of it: a partially-calibrated `0/2500` is replaced by `2048..3248`,
whose max exceeds the recorded one. Containment holds for the two cases that actually occur
(`0/4095` and `500/3500`), and width is the property the guard is defending.

## Two assumptions the anchored window rests on

Both surfaced in review, neither is a regression, and neither is answerable without a servo.

**`DriveMode` must be 0 on servo 6.** `Denormalize` inverts `NormModeRange100` when
`DriveMode != 0` (`calibrated_servo.go:83-92`), so on an inverted servo 0%/grab commands
`RangeMax` — full open — and 100% commands the closed stop. `guardGripperTravel` copies
`DriveMode` through from the servo read untouched while substituting a window whose entire
premise is "min is the closed end". This guard is the first code in the module to assert a
*directional* meaning for the two endpoints, so it is the right place to notice the dependency.
The pre-existing `0/4095` behavior is equally inverted, so nothing gets worse; the hardware
pass should simply confirm servo 6 reads `DriveMode: 0` on real kits.

**The plausibility threshold has less headroom than it looks.** `1.25 × 1251 = 1563.75` against
a genuine calibration measured at 1451 is 7.7% of margin. A kit whose real travel measures
~1600 would have its *operator-recorded* calibration discarded and replaced by the 1200-tick
window — a worse outcome than trusting it, and the warning would claim the range is "wider than
the jaw can move" when it is not. The factor stays at 1.25 because that is what the bench work
supported; this is recorded so a second datapoint at a wider travel is recognised as a reason
to revisit it rather than as a mystery.

## The default: one bypass, and a DRY argument

`DefaultSO101FullCalibration` gave servo 6 `500/3500` — span 3000, 2.4× the jaw travel.

Most paths that hand out that default are already guarded, contrary to an earlier draft of
this section: `config.go:406` *is* the `guardGripperTravel` call, `FromFeetechCalibrationMap`
(`config.go:303-320`) is wrapped by the guard at `config.go:460`, and `GetSharedController`
(`manager.go:512`, which has no non-test callers) passes the raw default in but `registry.go:183` overwrites it with the
guarded servo read before the controller is built. In those paths the unguarded default is
transient and never commanded.

**One path does escape.** `LoadFullCalibrationFromFile`'s `convertOrDefault` (`config.go:201`)
fills a *missing* `gripper` key with `DefaultSO101FullCalibration.Gripper`, and
`LoadCalibration` returns `fromFile = true`. `registry.go:177`'s `if !fromFile` then skips
`ReadCalibrationFromServos` — and therefore the guard — entirely. A calibration file with no
gripper entry is not hypothetical: the calibration sensor's `servo_ids` is user-configurable
(defaulted at `calibration.go:93`, declared at `:77`) and `saveCalibration` (`calibration.go:710-743`)
only writes a `gripper` entry when servo 6 is among them, so a run over servos 1-5 writes exactly such a file. That machine
commands `500/3500` on the jaw with nothing to stop it.

Fix: the default *is* the safe window. `RangeMin: gripperSafeRangeMin, RangeMax:
gripperSafeRangeMax`. That closes the file path, and gives the guard and the default a single
shared definition of "the conservative gripper range" so the two cannot drift.

### A behavior fix that rides along

`Grab()` infers a successful grasp from position error: `currentPercent - closedPosition >
15.0` (`gripper.go:296-299`). Under `500/3500`, an **empty** jaw resting at its closed stop
(tick 2048) reports `(2048-500)/3000 = 51.6%`, clears the threshold, and `Grab()` returns
`true` — unconditionally, whether or not it is holding anything. Under `2048/3248` the same
jaw reports 0% and `Grab()` correctly returns `false`. This changes an API return value, so it
gets its own test rather than being left as a side effect.

## Warnings: by cause, not by width

Making the default safe has a side effect that must be handled deliberately:
`guardGripperTravel(DefaultSO101FullCalibration, …)` now finds its input plausible and returns
`replaced = false`. Every default path would go **silent** — a gripper quietly running a
1200-tick window with no indication it is uncalibrated. Today those paths warn only as an
accident of the default being implausible.

So "running on the safe window" has two distinct causes and only one of them is a too-wide
range. One helper, `warnGripperUsingSafeRange(logger, cause)`, three call sites:

| Cause | Site |
|---|---|
| Servo 6 reported a range too wide to be real | `guardGripperTravel` — names the reported range, and `0/4095` explicitly as the uncalibrated signature |
| Servo 6 was requested but produced no calibration | `finalizeCalibration` (below) |
| Bus unavailable | `finalizeCalibration`, reached from the nil-bus early return with an empty map |
| A calibration file has no `gripper` entry | `LoadFullCalibrationFromFile`, when `fileFormat.Gripper == nil` |

The fourth row is easy to miss and matters most: that path never calls
`ReadCalibrationFromServos` at all, so neither the guard nor `finalizeCalibration` runs, and
nothing else in `LoadFullCalibrationFromFile` or `registry.go` logs when `convertOrDefault`
substitutes. Without its own call site the change's *motivating* case would be the one case
nobody is told about.

Both non-guard causes are gated on servo 6 appearing in `servoIDs`, so an arm-only machine
configured `servo_ids: [1,2,3,4,5]` does not warn about a gripper it does not have — including
when its bus is down. (In practice `arm.go:450` builds the controller with all six regardless
of the arm's own `servo_ids`, so the gate rarely fires; it exists so the two sites cannot
disagree.)

All three end in the same sentence: the jaw will not open fully, run the calibration workflow.

### `finalizeCalibration`

`ReadCalibrationFromServos` has two return sites — the nil-bus early return and the end of the
loop — and both must compose the map, warn, and guard identically. Extract that into

```go
func finalizeCalibration(
    calibrations map[int]*MotorCalibration,
    servoIDs []int,
    missingCause string,
    logger logging.Logger,
) SO101FullCalibration
```

`missingCause` is required, not decorative: an empty map arrives both from the nil-bus early
return and from a live bus whose servo-6 read failed, and nothing else in the parameters tells
those apart. The caller names the cause; `finalizeCalibration` uses it only if the gripper was
in fact not read.

called from both. This is not only deduplication: it is what keeps the wiring testable.
`TestReadCalibrationFromServosGuardsTheGripper` currently proves the guard is applied by
driving the nil-bus path and watching the implausible default get replaced — a vehicle that
stops working the moment the default is plausible. A pure `finalizeCalibration` can be handed
a map with servo 6 at `0/4095` and asserted directly, with no bus.

One edge stays uncovered and should be understood as such: a test can prove the guard runs
*inside* `finalizeCalibration`, but nothing proves `ReadCalibrationFromServos`'s loop-end
return site still calls it — once the default is plausible, a nil-bus test cannot distinguish
"the guard ran" from "the default came back". That edge is review-guarded, the same category as
the `NewSO101` `goalCloud` wiring noted in CLAUDE.md.

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

**Existing tests that must change** (all four break otherwise; the first is a compile error, not a failing assertion):

| Test | Why it breaks | Resolution |
|---|---|---|
| `TestSafeGripperRangeStaysInsideTheModeledTravel` | asserts `(min+max)/2 == 2048`, the centering this revision removes (new center 2648) | rewrite as `TestSafeGripperRangeClosesAtTheClosedStop`, asserting `min == gripperEncoderCenter`; keep its width and self-plausibility assertions |
| `TestGuardGripperTravelDoesNotMutateTheSharedDefault` | `require.True(t, replaced, "the placeholder default is itself implausible")` — no longer true | keep the test (it covers pointer aliasing of the package-level `Gripper`, which still matters) but drive it with an explicitly implausible gripper instead of the default |
| `TestGuardGripperTravelReplacesAnImplausibleRange` | calls `safeGripperRange()`, which becomes dead — a *compile* failure, so it will not show up as a failing assertion alongside the others | switch to the constants; also check its `FilterMessageSnippet("wider than the jaw")` still matches after the warning rewrite |
| `TestReadCalibrationFromServosGuardsTheGripper` | asserts the "wider than the jaw" warning on the nil-bus path, which now neither fires the guard nor uses that wording | split: a `finalizeCalibration` test for the guard wiring, and a nil-bus test for the bus-unavailable warning |

**New tests:**

- `TestGripperClosesAtTheClosedStop` — `Denormalize` at 0% and 100% → 2048 and 3248.
- `TestDefaultGripperCalibrationUsesTheSafeWindow` — pins `config.go`'s default to the shared
  constants.
- `TestGripperRangeIsPlausible` gains the measured `2030, 3481` case (span 1451, clears
  1563.75 by 113) in place of the invented, centered `1900, 2900`.
- `TestFinalizeCalibrationWarnsWhenTheGripperIsUncalibrated` — observed logger, one case per
  cause.
- A `Grab()` test for the empty-jaw case above, since the default change flips its return value.

## Housekeeping this revision implies

- `gripperEncoderCenter`'s comment ("Where a half-turn homing offset puts mid-travel") is now
  wrong; 2048 is the closed stop. The name is kept because it is still literally the encoder
  centre and the CLI probe uses the same term.
- `gripperSafeTravelFraction` and `safeGripperRange()` become dead.
- `guardGripperTravel`'s warning text still says "centered on mid-travel".
- `cmd/cli/gripper_limits.go`, the probe these measurements came from, is still untracked and
  should be committed alongside.

## Out of scope

- **Auto-discovering the range** by current-limited seek to both stops at construction. That
  is a real feature and effectively a mini-calibration; it needs hardware validation of its
  own.
- **Arm servos.** Degrees mode makes a wrong range cost a few degrees of center offset.
- **Tightening `ReadCalibrationFromServos`'s `min < max` check** for all servos. Rejecting
  `0/4095` outright would discard a usable offset, which is the thing this design
  deliberately avoids.

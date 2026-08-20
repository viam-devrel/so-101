# Gripper Travel Guard: Anchoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-anchor the gripper's substituted travel window so 0%/grab lands on the jaw's measured closed stop (tick 2048) instead of 500 ticks past it, make the same window the module default, and warn whenever the jaw is running on it.

**Architecture:** Three independent changes to one package. (1) `gripper_range.go`'s window becomes two constants anchored at the closed stop rather than a fraction centered on it. (2) `DefaultSO101FullCalibration`'s servo-6 entry becomes that same window, closing a path where a calibration file with no `gripper` key reaches the hardware unguarded. (3) A `finalizeCalibration` extraction gives `ReadCalibrationFromServos`'s two return paths one place to warn and guard — necessary because making the default plausible silences the guard's own warning.

**Tech Stack:** Go 1.25, `go.viam.com/rdk v1.0.0`, testify, `logging.NewObservedTestLogger`.

**Spec:** `docs/superpowers/specs/2026-08-18-gripper-travel-guard-design.md`

**Build/test command for this repo:** `go test ./cmd/module/ .` — **never** `./...`, because `cmd/cli/` is a folder of standalone debug scripts with multiple `func main()` in one package and does not compile as a package. Format with `gofmt -s -w .` and run `go vet ./cmd/module/ .` before each commit.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `gripper_range.go` | the guard: plausibility test, the safe window, the warning | modify — window constants, warning helper, `finalizeCalibration` |
| `gripper_range_test.go` | guard unit tests | modify — 3 rewrites, 4 additions |
| `config.go` | calibration sources and the package defaults | modify — servo-6 default, both `ReadCalibrationFromServos` return sites |
| `gripper_test.go` | gripper behavior against a fake arm | modify — one `Grab()` test |
| `cmd/cli/gripper_limits.go` | hardware probe these measurements came from | commit (currently untracked) |
| `CLAUDE.md` | project notes | modify — the guard bullet says "keeping the offset's implied center", now wrong |

---

## Task 1: Anchor the safe window at the closed stop

**Files:**
- Modify: `gripper_range.go:14-51`
- Test: `gripper_range_test.go:40-48`, and 3 call sites of `safeGripperRange()`

- [ ] **Step 1: Write the failing test**

Add to `gripper_range_test.go`. Note the deliberate literals — 2048 and 3248 are hardware
measurements, so this test must not be expressible in terms of the constants it is guarding.

```go
// The bug the first draft shipped: a window centered on 2048 puts 0% (grab) at tick 1548,
// five hundred ticks past the jaw's closed stop. Every other test in this file asserts
// arithmetic about the window; this one asserts what the window means.
func TestGripperClosesAtTheClosedStop(t *testing.T) {
	min, max := safeGripperRange()
	cal := &MotorCalibration{ID: 6, RangeMin: min, RangeMax: max, NormMode: NormModeRange100}

	closedTick, err := cal.Denormalize(0)
	require.NoError(t, err)
	assert.Equal(t, 2048, closedTick,
		"0% is grab; it must land on the measured closed stop, not past it")

	openTick, err := cal.Denormalize(100)
	require.NoError(t, err)
	assert.Equal(t, 3248, openTick,
		"1200 ticks of open travel, held under the ~1450 measured on hardware")
}
```

- [ ] **Step 2: Run it and confirm it fails for the right reason**

Run: `go test -run TestGripperClosesAtTheClosedStop . -v`
Expected: FAIL — `expected: 2048, actual: 1548`. If it fails any other way, stop and re-read
`safeGripperRange()`.

- [ ] **Step 3: Replace the constants in `gripper_range.go`**

Replace the whole `const` block and delete `safeGripperRange()`:

```go
const (
	gripperEncoderTicks = 4095

	// The gripper is always servo 6: SO101FullCalibration.Gripper maps to that ID.
	gripperServoID = 6

	// Measured across kits: a half-turn homing offset puts the jaw's *closed* stop here,
	// not its mid-travel. This is 0% / grab.
	gripperEncoderCenter = 2048

	// Open travel from the closed stop, measured across kits and held under the observed
	// ~1450 so a kit with less travel under-opens rather than stalling. Deliberately not
	// derived from gripperTravelTicks(): the model does not match the hardware (1251 vs
	// 1451), so a kinematics edit must not move a hardware limit.
	gripperSafeTravelTicks = 1200

	gripperSafeRangeMin = gripperEncoderCenter
	gripperSafeRangeMax = gripperEncoderCenter + gripperSafeTravelTicks

	// How much wider than the model a recorded range may be before we stop believing it.
	// Real calibrations overshoot slightly (2030-3481 was measured); 0/4095 and the
	// 500/3500 default do not.
	gripperMaxPlausibleTravelFactor = 1.25
)
```

`gripperSafeTravelFraction` and `safeGripperRange()` are deleted. `gripperTravelTicks()` and
`gripperRangeIsPlausible()` are unchanged — the model stays the plausibility yardstick only.

- [ ] **Step 4: Update `guardGripperTravel` to the constants**

In `guardGripperTravel`, replace `safeMin, safeMax := safeGripperRange()` and the two
assignments with `guarded.RangeMin, guarded.RangeMax = gripperSafeRangeMin, gripperSafeRangeMax`.

**The `logger.Warnf` below it also passes `safeMin, safeMax` as its last two arguments** — swap
those for `gripperSafeRangeMin, gripperSafeRangeMax` too, or the package will not compile:

```
./gripper_range.go:65:69: undefined: safeMin
./gripper_range.go:65:78: undefined: safeMax
```

Leave the message *wording* alone for now; Task 3 rewrites it.

- [ ] **Step 5: Update the three test call sites**

`safeGripperRange()` no longer exists. In `gripper_range_test.go`:
- `TestGripperClosesAtTheClosedStop` (just written): `min, max := gripperSafeRangeMin, gripperSafeRangeMax`
- `TestGuardGripperTravelReplacesAnImplausibleRange`: replace `safeMin, safeMax := safeGripperRange()` with the constants
- `TestReadCalibrationFromServosGuardsTheGripper`: same (Task 3 rewrites this test entirely; just make it compile here)

And rewrite the centering test, whose middle assertion is now the bug:

```go
// Must stop short of the mechanical stops, and start exactly on the closed one.
func TestSafeGripperRangeClosesAtTheClosedStop(t *testing.T) {
	assert.Equal(t, gripperEncoderCenter, gripperSafeRangeMin,
		"0% is grab: the window starts on the jaw's closed stop")
	assert.Less(t, gripperSafeRangeMax-gripperSafeRangeMin, gripperTravelTicks(),
		"the safe window must be narrower than the jaw's modeled travel")
	assert.True(t, gripperRangeIsPlausible(gripperSafeRangeMin, gripperSafeRangeMax),
		"the guard must not reject its own substitute")
}
```

Delete `TestSafeGripperRangeStaysInsideTheModeledTravel`.

- [ ] **Step 6: Swap the invented plausibility case for the measured one**

In `TestGripperRangeIsPlausible`, replace `{"a real calibration", 1900, 2900, true}` — a
centered fiction — with the reading taken off a kit:

```go
		{"a real calibration, measured", 2030, 3481, true},
```

While here, rename the `"module placeholder default"` case (`gripper_range_test.go:25`) to
`"the arm joints' 500/3500"` — after Task 2 it is no longer the module's default, though it is
still a valid implausible range and the case should stay.

Span 1451 against a threshold of 1251 × 1.25 = 1563.75. Keep the multiplication in the
implementation; do not hardcode 1564, which is a tick wider.

- [ ] **Step 7: Run the full suite**

Run: `gofmt -s -w . && go vet ./cmd/module/ . && go test ./cmd/module/ .`
Expected: PASS, except `TestGuardGripperTravelDoesNotMutateTheSharedDefault` may still pass
here (the default is still 500/3500 until Task 2). `TestEnumerateSerialPorts` fails on
machines with no serial hardware — that is environmental, not a regression.

- [ ] **Step 8: Commit**

```bash
git add gripper_range.go gripper_range_test.go
git commit -m "fix(gripper): anchor the safe travel window at the jaw's closed stop

Encoder center 2048 is where the jaw closes, not its mid-travel. The centered
window 1548..2548 therefore commanded 0%/grab five hundred ticks past the
closed stop -- the guard had moved the stall from the open side to the closed
side rather than removing it. Anchor at 2048 with 1200 ticks of open travel,
measured across kits and held under the ~1450 observed."
```

---

## Task 2: Make the default the safe window

The default is reachable by hardware: `LoadFullCalibrationFromFile`'s `convertOrDefault`
(`config.go:201`) fills a missing `gripper` key with `DefaultSO101FullCalibration.Gripper` and
returns `fromFile = true`, so `registry.go:177`'s `if !fromFile` skips both the servo read and
the guard. A calibration run configured `servo_ids: [1,2,3,4,5]` (`calibration.go:93`) writes
exactly such a file.

**Files:**
- Modify: `config.go:67-71`
- Test: `gripper_range_test.go`, `gripper_test.go`

- [ ] **Step 1: Write the failing tests**

Add to `gripper_range_test.go`:

```go
// A calibration file with no gripper entry reaches the hardware with this default and
// without the guard (convertOrDefault + fromFile=true skips ReadCalibrationFromServos), so
// the default itself has to be safe.
func TestDefaultGripperCalibrationUsesTheSafeWindow(t *testing.T) {
	assert.Equal(t, gripperSafeRangeMin, DefaultSO101FullCalibration.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, DefaultSO101FullCalibration.Gripper.RangeMax)
	assert.True(t, gripperRangeIsPlausible(
		DefaultSO101FullCalibration.Gripper.RangeMin,
		DefaultSO101FullCalibration.Gripper.RangeMax))
}
```

Add to `gripper_test.go` — this one pins the API return value the default change fixes:

```go
// Grab infers a grasp from position error. Under the old 500/3500 default an *empty* jaw
// resting at its closed stop normalized to 51.6%, clearing the 15% threshold, so Grab
// returned true whether or not it held anything.
func TestGrabReportsNothingHeldWhenTheJawReachesItsClosedStop(t *testing.T) {
	restingPercent, err := DefaultSO101FullCalibration.Gripper.Normalize(gripperClosedStopTick)
	require.NoError(t, err)

	fake := newFakeServoArm()
	fake.percent = restingPercent
	g := newTestGripper(t, fake) // gripper_test.go:146, as the file's other tests do

	grabbed, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)
	assert.False(t, grabbed, "an empty jaw at its closed stop is not holding anything")
}
```

- [ ] **Step 2: Run them and confirm both fail**

Run: `go test -run 'TestDefaultGripperCalibrationUsesTheSafeWindow|TestGrabReportsNothingHeld' . -v`
Expected: both FAIL — the first on `500 != 2048`, the second on `true != false` (the empty jaw
normalizes to 51.6%).

- [ ] **Step 3: Change the default**

`config.go`, the `Gripper` entry of `DefaultSO101FullCalibration`:

```go
	Gripper: &MotorCalibration{
		ID: 6, DriveMode: 0, HomingOffset: 0,
		// The conservative window from gripper_range.go, not the arm joints' 500/3500:
		// Range100 mode scales 0-100% across this, and a calibration file with no gripper
		// entry reaches the servo with it and without the guard.
		RangeMin: gripperSafeRangeMin, RangeMax: gripperSafeRangeMax,
		NormMode: NormModeRange100, // 0-100% for gripper
	},
```

- [ ] **Step 4: Fix `TestGuardGripperTravelDoesNotMutateTheSharedDefault`**

Its `require.True(t, replaced, "the placeholder default is itself implausible")` is no longer
true. The test covers pointer aliasing of the package-level `Gripper`, which still matters, so
drive it with an explicitly implausible gripper instead of relying on the default being one:

```go
// DefaultSO101FullCalibration holds pointers; mutating in place would corrupt every later caller.
func TestGuardGripperTravelDoesNotMutateTheSharedDefault(t *testing.T) {
	original := *DefaultSO101FullCalibration.Gripper

	cal := DefaultSO101FullCalibration
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: NormModeRange100}
	_, replaced := guardGripperTravel(cal, nil)
	require.True(t, replaced)

	assert.Equal(t, original.RangeMin, DefaultSO101FullCalibration.Gripper.RangeMin,
		"the package-level default must be untouched")
	assert.Equal(t, original.RangeMax, DefaultSO101FullCalibration.Gripper.RangeMax)
}
```

- [ ] **Step 5: Run the full suite**

Run: `gofmt -s -w . && go vet ./cmd/module/ . && go test ./cmd/module/ .`
Expected: PASS except `TestReadCalibrationFromServosGuardsTheGripper`, which now fails on
`logs.FilterMessageSnippet("wider than the jaw").Len()` being 0 — the nil-bus path feeds the
guard a default that is now plausible, so the guard does not fire and does not warn. **This is
the silence the spec predicts; Task 3 fixes it.** Do not paper over it here. Check
`config_test.go` for any test asserting `500`/`3500` on the gripper and update it if so.

- [ ] **Step 6: Commit**

```bash
git add config.go gripper_range_test.go gripper_test.go
git commit -m "fix(gripper): default servo 6 to the safe window, not the arm's 500/3500

A calibration file with no gripper entry gets this default via convertOrDefault
and fromFile=true, which skips ReadCalibrationFromServos and therefore the
guard -- so the default itself reaches the jaw. 500/3500 is 2.4x the jaw's
travel.

Also fixes Grab(): under 500/3500 an empty jaw at its closed stop normalized to
51.6%, clearing the 15% grasp threshold, so Grab returned true unconditionally.

Known failing until the next commit: TestReadCalibrationFromServosGuardsTheGripper."
```

*(If a broken intermediate commit is unacceptable in this repo's history, fold Tasks 2 and 3
into one commit — but keep the steps and their separate red/green runs.)*

---

## Task 3: Warn by cause, via `finalizeCalibration`

**Files:**
- Modify: `gripper_range.go` (add helper + `finalizeCalibration`, rewrite the guard's warning)
- Modify: `config.go:405-407` and `config.go:459-461`
- Test: `gripper_range_test.go`

- [ ] **Step 1: Write the failing tests**

Replace `TestReadCalibrationFromServosGuardsTheGripper` with a pair — one for the wiring, one
for the nil-bus path. The first is the point of the extraction: the old test proved the guard
was wired in by watching an implausible default get replaced, and that vehicle no longer
exists now that the default is plausible.

```go
// The wiring: the composition step must apply the guard, not just define it. A pure function
// makes this testable without a bus -- the old vehicle (nil bus -> implausible default) died
// when the default became safe.
func TestFinalizeCalibrationGuardsTheGripper(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	read := map[int]*MotorCalibration{
		6: {ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: NormModeRange100},
	}
	// The implausible range matters: the old nil-bus vehicle asserted the returned range
	// equals the safe window, which the *default* now satisfies whether or not the guard
	// ran. Only an implausible input gives the wiring assertion teeth.

	cal := finalizeCalibration(read, []int{1, 2, 3, 4, 5, 6}, "unused", logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len())
	assert.Equal(t, DefaultSO101FullCalibration.ShoulderPan.RangeMin, cal.ShoulderPan.RangeMin,
		"arm servos pass through untouched")
}

// Making the default safe means the guard no longer fires on it, so the fallback has to
// announce itself or the jaw runs a 1200-tick window in silence.
func TestFinalizeCalibrationWarnsWhenTheGripperWasNotRead(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	cal := finalizeCalibration(nil, []int{1, 2, 3, 4, 5, 6}, "the servo bus is unavailable", logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, 1, logs.FilterMessageSnippet("the servo bus is unavailable").Len())
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

// An arm-only machine must not be warned about a gripper it does not have.
func TestFinalizeCalibrationStaysQuietWhenTheGripperWasNotRequested(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	finalizeCalibration(nil, []int{1, 2, 3, 4, 5}, "the servo bus is unavailable", logger)

	assert.Equal(t, 0, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

// The wiring one level up: ReadCalibrationFromServos's nil-bus path must go through it.
func TestReadCalibrationFromServosWarnsWhenTheBusIsUnavailable(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	cal := ReadCalibrationFromServos(context.Background(), nil, []int{1, 2, 3, 4, 5, 6}, logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}
```

- [ ] **Step 2: Run and confirm they fail to compile**

Run: `go test -run TestFinalizeCalibration . -v`
Expected: build failure, `undefined: finalizeCalibration`. That is the red state here.

- [ ] **Step 3: Add the warning helper and `finalizeCalibration` to `gripper_range.go`**

```go
// warnGripperUsingSafeRange reports that the jaw is running on the substituted window rather
// than a recorded range. cause names why; the remedy is the same for every cause, so it lives
// here rather than at the call sites.
func warnGripperUsingSafeRange(logger logging.Logger, cause string) {
	if logger == nil {
		return
	}
	logger.Warnf(
		"gripper servo %d: %s. Using the conservative range %d-%d (closed at the encoder "+
			"centre, opening %d ticks). The jaw will not open fully; run the calibration "+
			"workflow to record its real range.",
		gripperServoID, cause, gripperSafeRangeMin, gripperSafeRangeMax, gripperSafeTravelTicks)
}

// finalizeCalibration composes what was read into a full calibration, announces a gripper
// falling back to the safe window, and applies the travel guard. Both of
// ReadCalibrationFromServos's return paths go through here so they cannot drift, and so the
// guard's wiring is testable without a bus. missingCause explains why the gripper was not
// read, and is used only if it in fact was not.
func finalizeCalibration(
	calibrations map[int]*MotorCalibration,
	servoIDs []int,
	missingCause string,
	logger logging.Logger,
) SO101FullCalibration {
	if _, read := calibrations[gripperServoID]; !read && slices.Contains(servoIDs, gripperServoID) {
		warnGripperUsingSafeRange(logger, missingCause)
	}
	guarded, _ := guardGripperTravel(FromFeetechCalibrationMap(calibrations), logger)
	return guarded
}
```

Add `"slices"` to the imports. `FromFeetechCalibrationMap(nil)` is safe — its `getOrDefault`
returns the package default for every absent ID, which is the existing behavior for servos 1-5.

- [ ] **Step 4: Route the guard's own warning through the helper**

In `guardGripperTravel`, replace the `logger.Warnf(...)` block with:

```go
	warnGripperUsingSafeRange(logger, fmt.Sprintf(
		"reports a %d-tick range (%d-%d), wider than the jaw can move (~%d ticks) — it has a "+
			"homing offset but no real position limits, and the factory 0/4095 reads as calibrated",
		oldMax-oldMin, oldMin, oldMax, gripperTravelTicks()))
```

Keep the phrase "wider than the jaw" — two tests filter on it. Add `"fmt"` to the imports and
drop the now-unused `logger != nil` guard (the helper handles nil). Note `guarded.ID` is no
longer referenced; the helper uses `gripperServoID`.

- [ ] **Step 5: Route both `ReadCalibrationFromServos` return sites through it**

`config.go`, the nil-bus early return:

```go
	if bus == nil {
		if logger != nil {
			logger.Warn("Cannot read servo calibration: bus is nil")
		}
		return finalizeCalibration(nil, servoIDs, "the servo bus is unavailable", logger)
	}
```

and the end of the function:

```go
	// Registers can report a range the jaw cannot reach, and a servo may not answer at all.
	// Narrow and announce before anyone maps percentages onto it. Callers with a calibration
	// file never reach here.
	return finalizeCalibration(calibrations, servoIDs,
		"no calibration was read from its registers", logger)
```

- [ ] **Step 5b: Warn on the file path that motivates the whole change**

This is the one path that reaches the jaw with the default *without* calling
`ReadCalibrationFromServos`, so neither the guard nor `finalizeCalibration` runs and nothing
logs. Left alone, the motivating case is the one case nobody is told about.

First the test, in `config_test.go` (write a temp calibration JSON with the five arm entries
and no `gripper` key — copy the shape from any existing calibration-file test in that file):

```go
// A calibration file written by a servo_ids:[1,2,3,4,5] run has "gripper": null.
// convertOrDefault fills the default and fromFile=true skips the guard entirely, so this is
// the one fallback that has to announce itself here.
func TestLoadFullCalibrationWarnsWhenTheFileHasNoGripperEntry(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cal.json")
	require.NoError(t, os.WriteFile(path, []byte(`{"shoulder_pan":{"id":1,"range_min":500,"range_max":3500}}`), 0o600))

	logger, logs := logging.NewObservedTestLogger(t)
	cal, err := LoadFullCalibrationFromFile(path, logger)
	require.NoError(t, err)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}
```

Run it, confirm it fails on the log count being 0, then add to `LoadFullCalibrationFromFile`
immediately after the `calibration := SO101FullCalibration{...}` literal:

```go
	if fileFormat.Gripper == nil {
		warnGripperUsingSafeRange(logger, fmt.Sprintf("%s has no gripper entry", filePath))
	}
```

Adjust the JSON literal above if the real `CalibrationEntry` field names differ — check
`CalibrationFileFormat` and `CalibrationEntry` in `config.go` before writing the fixture.

- [ ] **Step 5c: Ride-along fixes from Task 2's review**

Four small items the Task 2 review raised, grouped here so they land with the code they touch.

**A. The guard's copy has no field-fidelity assertion (the one that matters).** Mutating
`guarded := *cal.Gripper` into a partial literal carrying only `ID`/`RangeMin`/`RangeMax`
survives the entire suite. On hardware that is not cosmetic: `NormMode`'s zero value is
`NormModeRaw`, so a guarded gripper silently downgraded to Raw would have `Denormalize` pass
the 0-100 percentage through as a raw tick and drive the jaw to ~0 — about 2000 ticks past its
closed stop. Add to `TestGuardGripperTravelReplacesAnImplausibleRange`:

```go
	assert.Equal(t, NormModeRange100, guarded.Gripper.NormMode,
		"the copy must carry the mode, not just the range")
	assert.Equal(t, 6, guarded.Gripper.ID)
```

**B. Trim `config.go`'s gripper comment to its first line** (ending in a period). Lines 2-3
restate `gripper_range.go`'s file header and duplicate the test's own comment.

**C. Drop the third assertion in `TestDefaultGripperCalibrationUsesTheSafeWindow`** — the
`gripperRangeIsPlausible` call is implied by the two above it plus
`TestGripperSafeWindowStaysInsideTheStops`.

**D. Give `Grab()` a positive case.** The empty-jaw test is currently the only `.Grab(` call in
the suite, so it would also pass against a `Grab` that always returned false:

```go
func TestGrabReportsHeldWhenTheJawStopsShortOfClosed(t *testing.T) {
	fake := newFakeServoArm()
	fake.percent = 50
	g := newTestGripper(t, fake)

	grabbed, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)
	assert.True(t, grabbed, "a jaw held open by an object is holding something")
}
```

- [ ] **Step 6: Run the full suite**

Run: `gofmt -s -w . && go vet ./cmd/module/ . && go test ./cmd/module/ .`
Expected: PASS (except the environmental `TestEnumerateSerialPorts`). Every test broken by
Task 2 is now green.

- [ ] **Step 7: Commit**

```bash
git add gripper_range.go gripper_range_test.go config.go config_test.go
git commit -m "feat(gripper): warn whenever the jaw runs on the substituted window

Making the default safe means guardGripperTravel finds it plausible and returns
replaced=false, so every default path would go silent -- today they warn only
as an accident of the default being implausible. Warn by cause instead: too-wide
range, not read, or bus unavailable, all ending in the same remedy.

finalizeCalibration gives ReadCalibrationFromServos's two return paths one place
to compose, warn, and guard. It also restores the coverage lost with the old
nil-bus wiring test, which worked only while the default was implausible."
```

---

## Task 4: Documentation and the probe

- [ ] **Step 1: Fix the CLAUDE.md gotcha**

Three things, all in the final bullet of the Gotchas section:

The last bullet says the guard narrows the range "keeping the offset's implied center". Replace
that clause: the window is **anchored** at tick 2048, the jaw's *closed* stop on a
half-turn-homed kit, and opens 1200 ticks from there. Add that `DefaultSO101FullCalibration`'s
servo-6 entry is the same window, and why (a calibration file with no `gripper` key gets the
default via `convertOrDefault` and `fromFile=true`, which skips `ReadCalibrationFromServos` and
therefore the guard).

Add two further notes worth having in the gotchas, both established during implementation:

- **`Grab()` used to return `true` unconditionally.** Under the old `500/3500` default an empty
  jaw at its closed stop normalized to 51.6%, clearing the `> 15.0` grasp threshold. Anyone
  who wrote code branching on `Grab()`'s return value before this change was reading a constant.
- **`feetech.BusConfig` takes an exported `Transport`**, so `ReadCalibrationFromServos`'s
  bus-dependent paths are unit-testable without serial hardware (see `deadTransport` in
  `gripper_range_test.go`). This is worth knowing generally — the repo has other paths written
  off as hardware-only that may not be.

- [ ] **Step 2: Update README's `#### Gripper travel guard` section**

Not optional — `README.md:92-108` documents this feature in detail and every specific in it is
now wrong. Four concrete edits:

1. **`README.md:86`** — "Hardcoded defaults - If servo reads fail, use default values (offset=0,
   range=500-3500)". Servo 6 is no longer 500-3500; say the gripper defaults to the conservative
   travel window instead.
2. **`README.md:98`** — "keeps the homing offset's mid-travel point and substitutes a
   conservative window around it (`1548`-`2548`)". Both halves are the bug being fixed: the
   window is *anchored* at the encoder centre, which is the jaw's closed stop on a vendor
   half-turn-homed kit, and opens 1200 ticks from there to `2048`-`3248`.
3. **`README.md:100-104`** — a fenced block quoting the old warning verbatim, including "Using
   1548-2548 instead, centered on mid-travel". Replace with the real messages below, captured
   from `logging.NewObservedTestLogger` during Task 3 (do not retype or reflow the wording; wrap
   the fenced block only if the README's existing blocks wrap):

   ```
   gripper servo 6: its registers report a 4095-tick range (0-4095), wider than the jaw can move (~1251 ticks): it has a homing offset but no real position limits. Using the conservative range 2048-3248 (closed at tick 2048, opening 1200 ticks). The jaw will not open fully; run the calibration workflow to record its real range.
   ```

   The other three causes substitute for the clause after `gripper servo 6:` — `its registers
   held no usable range`, `the servo bus is unavailable`, and `<path> has no gripper entry` —
   and are otherwise identical. Show one in full and describe the rest; four near-identical
   paragraphs would bury the point.
4. **Add** two things the section does not currently cover: that the module *default* is now
   this same window (so a calibration file with no `gripper` entry is safe), and that the
   warning also fires when servo 6 simply was not read or the bus was unavailable — not only
   when it reports a too-wide range.

Also add a short note reflecting the spec's "The conflict this design knowingly accepts": the
window assumes the vendor half-turn convention where the encoder centre is the closed stop, and
a gripper homed from mid-travel (this module's own calibration prompt) will not open fully and
may strain — finish the calibration workflow.

- [ ] **Step 3: Commit the hardware probe**

`cmd/cli/gripper_limits.go` is untracked and is the instrument these measurements came from.
Update its `modeledTravel = 1251` comment to also record the measured values (~1451 observed
travel, 1200 chosen), then:

```bash
git add cmd/cli/gripper_limits.go CLAUDE.md README.md
git commit -m "docs(gripper): record the anchored window, and commit the probe it came from"
```

- [ ] **Step 4: Final verification**

Run: `gofmt -l . && go vet ./cmd/module/ . && go test ./cmd/module/ .`
Expected: no gofmt output, no vet output, tests PASS apart from `TestEnumerateSerialPorts` on
machines without serial hardware.

---

## Hardware verification (cannot be done in software)

The spec's numbers came off real kits, but the *change* has not been run on one. Before merge,
on a kit with servo 6 at the factory `0/4095`:

1. `go run ./cmd/cli/gripper_limits.go -port ... ` — confirm the guard's preconditions.
2. Start the module and confirm exactly one warning naming `0/4095` and the `2048-3248` window.
3. Drive the gripper to 0% and 100%. Confirm no stall, no overload latch, and that `Grab()` on
   an empty jaw returns **false** — the behavior change from Task 2.
4. `-goto 3248` with `-force` and watch the load reading; confirm the open end is clear of the
   stop rather than merely close to it.

Then the accepted-risk path, which the spec's "The conflict this design knowingly accepts"
section explains. On a kit where homing was set from **mid-travel** (start the module's own
calibration, move the jaw to the middle, `set_homing`, then abandon the run):

5. Read servo 6's `DriveMode` and confirm it is 0. `Denormalize` inverts `NormModeRange100`
   when it is not, which would make 0%/grab command full open — see the spec's "Two assumptions
   the anchored window rests on".
6. Confirm the failure is what the spec predicts and no worse: grab lands half-open, and
   `Open()` strains near tick 2773. If it stalls harder than the ~415-tick overshoot predicted,
   the decision to anchor needs revisiting before merge.

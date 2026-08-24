package controller

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"

	"so_arm/internal/servo"
)

// The plausibility yardstick. The safe window is measured, not derived — see
// gripperSafeTravelTicks.
func TestGripperTravelTicksMatchesTheKinematicModel(t *testing.T) {
	assert.InDelta(t, 1251, gripperTravelTicks(), 2,
		"110 degrees of jaw travel at 4095 ticks/rev")
}

func TestGripperRangeIsPlausible(t *testing.T) {
	for _, tc := range []struct {
		name     string
		min, max int
		want     bool
	}{
		{"factory full encoder", 0, 4095, false},
		{"the arm joints' 500/3500", 500, 3500, false},
		{"a real calibration, measured", 2030, 3481, true},
		{"exactly the modeled travel", 2048 - 625, 2048 + 626, true},
		{"slightly over, within tolerance", 1500, 3000, true},
		{"well over tolerance", 1000, 3200, false},
		{"inverted", 2500, 2000, false},
		{"degenerate", 2048, 2048, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			assert.Equal(t, tc.want, gripperRangeIsPlausible(tc.min, tc.max))
		})
	}
}

// The bug the first draft shipped: a window centered on 2048 puts 0% (grab) at tick 1548,
// five hundred ticks past the jaw's closed stop. Every other test in this file asserts
// arithmetic about the window; this one asserts what the window means.
func TestGripperClosesAtTheClosedStop(t *testing.T) {
	min, max := gripperSafeRangeMin, gripperSafeRangeMax
	cal := &servo.MotorCalibration{ID: 6, RangeMin: min, RangeMax: max, NormMode: servo.NormModeRange100}

	closedTick, err := cal.Denormalize(0)
	require.NoError(t, err)
	assert.Equal(t, 2048, closedTick,
		"0% is grab; it must land on the measured closed stop, not past it")

	openTick, err := cal.Denormalize(100)
	require.NoError(t, err)
	assert.Equal(t, 3248, openTick,
		"1200 ticks of open travel, held under the ~1450 measured on hardware")
}

// Must stop short of the mechanical stops.
func TestGripperSafeWindowStaysInsideTheStops(t *testing.T) {
	assert.Less(t, gripperSafeTravelTicks, gripperObservedTravelTicks,
		"the safe window must stay inside the travel measured on hardware")
	assert.True(t, gripperRangeIsPlausible(gripperSafeRangeMin, gripperSafeRangeMax),
		"the guard must not reject its own substitute")
}

func TestGuardGripperTravelReplacesAnImplausibleRange(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	cal := DefaultSO101FullCalibration
	cal.Gripper = &servo.MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: servo.NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, logger)
	require.True(t, replaced)

	assert.Equal(t, gripperSafeRangeMin, guarded.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, guarded.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len(),
		"the degraded mode must be warned about, and named specifically enough to act on")
	assert.Equal(t, servo.NormModeRange100, guarded.Gripper.NormMode,
		"the copy must carry the mode, not just the range")
	assert.Equal(t, 6, guarded.Gripper.ID)
}

// Never override an operator who actually ran calibration.
func TestGuardGripperTravelLeavesACredibleRangeAlone(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = &servo.MotorCalibration{ID: 6, RangeMin: 2030, RangeMax: 3481, NormMode: servo.NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, nil)
	assert.False(t, replaced)
	assert.Equal(t, 2030, guarded.Gripper.RangeMin)
	assert.Equal(t, 3481, guarded.Gripper.RangeMax)
}

// Degrees mode only takes a center point from the range, so arm servos are out of scope.
func TestGuardGripperTravelDoesNotTouchArmServos(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = &servo.MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: servo.NormModeRange100}
	before := *cal.ShoulderPan

	guarded, replaced := guardGripperTravel(cal, nil)
	require.True(t, replaced)
	assert.Equal(t, before.RangeMin, guarded.ShoulderPan.RangeMin)
	assert.Equal(t, before.RangeMax, guarded.ShoulderPan.RangeMax)
}

// DefaultSO101FullCalibration holds pointers, and callers pass it straight through, so the
// guard must copy the gripper rather than mutate the caller's struct in place.
func TestGuardGripperTravelDoesNotMutateItsInput(t *testing.T) {
	in := &servo.MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: servo.NormModeRange100}
	cal := DefaultSO101FullCalibration
	cal.Gripper = in

	guarded, replaced := guardGripperTravel(cal, nil)
	require.True(t, replaced)

	assert.NotSame(t, in, guarded.Gripper, "the guard must copy, not alias")
	assert.Equal(t, 0, in.RangeMin, "the caller's calibration must be untouched")
	assert.Equal(t, 4095, in.RangeMax)
}

func TestGuardGripperTravelToleratesAMissingGripper(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = nil

	guarded, replaced := guardGripperTravel(cal, nil)
	assert.False(t, replaced)
	assert.Nil(t, guarded.Gripper)
}

// A calibration file with no gripper entry reaches the hardware with this default and
// without the guard (convertOrDefault + fromFile=true skips ReadCalibrationFromServos), so
// the default itself has to be safe.
func TestDefaultGripperCalibrationUsesTheSafeWindow(t *testing.T) {
	assert.Equal(t, gripperSafeRangeMin, DefaultSO101FullCalibration.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, DefaultSO101FullCalibration.Gripper.RangeMax)
}

// The wiring: the composition step must apply the guard, not just define it. A pure function
// makes this testable without a bus -- the old vehicle (nil bus -> implausible default) died
// when the default became safe. The implausible input matters: asserting the returned range
// equals the safe window proves nothing against a *default* that already equals it.
func TestFinalizeCalibrationFromServosGuardsTheGripper(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	read := map[int]*servo.MotorCalibration{
		6: {ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: servo.NormModeRange100},
	}

	cal := finalizeCalibrationFromServos(read, []int{1, 2, 3, 4, 5, 6}, "unused", logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len())
	assert.Equal(t, DefaultSO101FullCalibration.ShoulderPan.RangeMin, cal.ShoulderPan.RangeMin,
		"arm servos pass through untouched")
	assert.Equal(t, 0, logs.FilterMessageSnippet("unused").Len(),
		"missingCause must not surface when the gripper was read")
}

// FromFeetechCalibrationMap treats a present-but-nil entry as absent (exists && mc != nil);
// finalizeCalibrationFromServos's own "was it read" check has to agree, or a {6: nil} map
// guards silently -- present key, no warning -- which is exactly the bug this file exists to
// close. Not reachable today (nothing in this codebase populates a nil entry), but the two
// checks must not be free to drift apart.
func TestFinalizeCalibrationFromServosTreatsANilEntryAsUnread(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	read := map[int]*servo.MotorCalibration{6: nil}

	cal := finalizeCalibrationFromServos(read, []int{1, 2, 3, 4, 5, 6}, "unused", logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, 1, logs.FilterMessageSnippet("unused").Len())
}

// Making the default safe means the guard no longer fires on it, so the fallback has to
// announce itself or the jaw runs a 1200-tick window in silence.
func TestFinalizeCalibrationFromServosWarnsWhenTheGripperWasNotRead(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	cal := finalizeCalibrationFromServos(nil, []int{1, 2, 3, 4, 5, 6}, "the servo bus is unavailable", logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, 1, logs.FilterMessageSnippet("the servo bus is unavailable").Len())
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

// An arm-only machine must not be warned about a gripper it does not have.
func TestFinalizeCalibrationFromServosStaysQuietWhenTheGripperWasNotRequested(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	finalizeCalibrationFromServos(nil, []int{1, 2, 3, 4, 5}, "the servo bus is unavailable", logger)

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

// A bus whose every read fails: enough to reach ReadCalibrationFromServos's tail return
// without serial hardware. Without this, deleting the guard from that return -- the only
// path that guards a range read off a real servo -- breaks no test.
type deadTransport struct{}

func (deadTransport) Read([]byte) (int, error)           { return 0, errors.New("no bus") }
func (deadTransport) Write([]byte) (int, error)          { return 0, errors.New("no bus") }
func (deadTransport) Close() error                       { return nil }
func (deadTransport) SetReadTimeout(time.Duration) error { return nil }
func (deadTransport) Flush() error                       { return nil }

func TestReadCalibrationFromServosGuardsTheTailReturn(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: deadTransport{}})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	cal := ReadCalibrationFromServos(context.Background(), bus, []int{1, 2, 3, 4, 5, 6}, logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("its registers held no usable range").Len())
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

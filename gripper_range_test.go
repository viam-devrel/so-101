package so_arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
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
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, logger)
	require.True(t, replaced)

	assert.Equal(t, gripperSafeRangeMin, guarded.Gripper.RangeMin)
	assert.Equal(t, gripperSafeRangeMax, guarded.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len(),
		"the degraded mode must be warned about, and named specifically enough to act on")
}

// Never override an operator who actually ran calibration.
func TestGuardGripperTravelLeavesACredibleRangeAlone(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 2030, RangeMax: 3481, NormMode: NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, nil)
	assert.False(t, replaced)
	assert.Equal(t, 2030, guarded.Gripper.RangeMin)
	assert.Equal(t, 3481, guarded.Gripper.RangeMax)
}

// Degrees mode only takes a center point from the range, so arm servos are out of scope.
func TestGuardGripperTravelDoesNotTouchArmServos(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: NormModeRange100}
	before := *cal.ShoulderPan

	guarded, replaced := guardGripperTravel(cal, nil)
	require.True(t, replaced)
	assert.Equal(t, before.RangeMin, guarded.ShoulderPan.RangeMin)
	assert.Equal(t, before.RangeMax, guarded.ShoulderPan.RangeMax)
}

// DefaultSO101FullCalibration holds pointers; mutating in place would corrupt every later caller.
func TestGuardGripperTravelDoesNotMutateTheSharedDefault(t *testing.T) {
	original := *DefaultSO101FullCalibration.Gripper

	cal := DefaultSO101FullCalibration
	_, replaced := guardGripperTravel(cal, nil)
	require.True(t, replaced, "the placeholder default is itself implausible")

	assert.Equal(t, original.RangeMin, DefaultSO101FullCalibration.Gripper.RangeMin,
		"the package-level default must be untouched")
	assert.Equal(t, original.RangeMax, DefaultSO101FullCalibration.Gripper.RangeMax)
}

func TestGuardGripperTravelToleratesAMissingGripper(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = nil

	guarded, replaced := guardGripperTravel(cal, nil)
	assert.False(t, replaced)
	assert.Nil(t, guarded.Gripper)
}

// The wiring: ReadCalibrationFromServos must apply the guard, not just define it. The nil-bus
// path returns defaults without hardware, which is what makes this reachable in a test.
func TestReadCalibrationFromServosGuardsTheGripper(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)

	cal := ReadCalibrationFromServos(context.Background(), nil, []int{1, 2, 3, 4, 5, 6}, logger)

	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin, "the default gripper range must be guarded")
	assert.Equal(t, gripperSafeRangeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len())

	assert.Equal(t, DefaultSO101FullCalibration.ShoulderPan.RangeMin, cal.ShoulderPan.RangeMin,
		"arm servos pass through untouched")
}

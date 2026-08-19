package so_arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
)

// Every other number derives from this: the URDF jaw joint spans 110 degrees.
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
		{"module placeholder default", 500, 3500, false},
		{"a real calibration", 1900, 2900, true},
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

// Must stop short of the mechanical stops, not merely improve on what it replaced.
func TestSafeGripperRangeStaysInsideTheModeledTravel(t *testing.T) {
	min, max := safeGripperRange()

	assert.Less(t, max-min, gripperTravelTicks(),
		"the safe window must be narrower than the jaw's real travel")
	assert.Equal(t, gripperEncoderCenter, (min+max)/2,
		"centered where a half-turn homing offset puts mid-travel")
	assert.True(t, gripperRangeIsPlausible(min, max),
		"the guard must not reject its own substitute")
}

func TestGuardGripperTravelReplacesAnImplausibleRange(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	cal := DefaultSO101FullCalibration
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 0, RangeMax: 4095, NormMode: NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, logger)
	require.True(t, replaced)

	safeMin, safeMax := safeGripperRange()
	assert.Equal(t, safeMin, guarded.Gripper.RangeMin)
	assert.Equal(t, safeMax, guarded.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len(),
		"the degraded mode must be warned about, and named specifically enough to act on")
}

// Never override an operator who actually ran calibration.
func TestGuardGripperTravelLeavesACredibleRangeAlone(t *testing.T) {
	cal := DefaultSO101FullCalibration
	cal.Gripper = &MotorCalibration{ID: 6, RangeMin: 1900, RangeMax: 2900, NormMode: NormModeRange100}

	guarded, replaced := guardGripperTravel(cal, nil)
	assert.False(t, replaced)
	assert.Equal(t, 1900, guarded.Gripper.RangeMin)
	assert.Equal(t, 2900, guarded.Gripper.RangeMax)
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

	safeMin, safeMax := safeGripperRange()
	assert.Equal(t, safeMin, cal.Gripper.RangeMin, "the default gripper range must be guarded")
	assert.Equal(t, safeMax, cal.Gripper.RangeMax)
	assert.Equal(t, 1, logs.FilterMessageSnippet("wider than the jaw").Len())

	assert.Equal(t, DefaultSO101FullCalibration.ShoulderPan.RangeMin, cal.ShoulderPan.RangeMin,
		"arm servos pass through untouched")
}

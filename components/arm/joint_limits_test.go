package arm

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
	"so_arm/internal/servo"
)

const radPerTick = 360.0 / 4095.0 * math.Pi / 180

// The recorded range is asymmetric about the homing tick, so the limits must be too:
// the old (min+max)/2 math normalized every range to exactly [-pi, pi].
func TestJointLimitsFollowHomingTickZero(t *testing.T) {
	cal := controller.SO101FullCalibration{
		ElbowFlex: &servo.MotorCalibration{ID: 3, RangeMin: 866, RangeMax: 3073, NormMode: servo.NormModeDegrees},
	}
	// A partial servo_ids list: limits index by position, calibration by servo id.
	limits := jointLimitsFor(cal, []int{1, 3})

	require.Equal(t, [2]float64{-math.Pi, math.Pi}, limits[0], "missing calibration keeps the permissive default")
	require.InDelta(t, float64(866-servo.HomingTick)*radPerTick, limits[1][0], 1e-9)
	require.InDelta(t, float64(3073-servo.HomingTick)*radPerTick, limits[1][1], 1e-9)
}

// Normalize negates under drive-mode inversion, so the range ends swap sides of zero.
func TestJointLimitsMirrorUnderDriveModeInversion(t *testing.T) {
	cal := controller.SO101FullCalibration{
		ShoulderLift: &servo.MotorCalibration{ID: 2, RangeMin: 839, RangeMax: 3205, NormMode: servo.NormModeDegrees, DriveMode: 1},
	}
	limits := jointLimitsFor(cal, []int{2})
	require.InDelta(t, -float64(3205-servo.HomingTick)*radPerTick, limits[0][0], 1e-9)
	require.InDelta(t, -float64(839-servo.HomingTick)*radPerTick, limits[0][1], 1e-9)
}

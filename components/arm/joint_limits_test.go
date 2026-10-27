package arm

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
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

// NewSO101 publishes these as the model's DoF, so the keys must be the joint IDs and the values
// degrees; joints without a real calibration must fall through to the model's own limits.
func TestCalibratedJointLimitsDegFeedsTheModel(t *testing.T) {
	cal := controller.SO101FullCalibration{
		ShoulderPan:  controller.DefaultSO101FullCalibration.ShoulderPan,
		ShoulderLift: &servo.MotorCalibration{ID: 2, RangeMin: 0, RangeMax: 4095, NormMode: servo.NormModeDegrees},
		WristFlex:    &servo.MotorCalibration{ID: 4, RangeMin: 855, RangeMax: 3179, NormMode: servo.NormModeDegrees},
	}
	limits := calibratedJointLimitsDeg(cal, []int{1, 2, 3, 4})
	require.Len(t, limits, 1, "default, factory-unset and missing calibrations are skipped")
	require.InDelta(t, float64(855-servo.HomingTick)*360/4095, limits["4"][0], 1e-9)
	require.InDelta(t, float64(3179-servo.HomingTick)*360/4095, limits["4"][1], 1e-9)

	base, err := makeModelFrame(&SO101ArmConfig{}, "so101")
	require.NoError(t, err)
	m, err := geometry.WithJointLimits(base, limits, "so101")
	require.NoError(t, err)
	require.InDelta(t, jointLimitsFor(cal, []int{4})[0][0], m.DoF()[3].Min, 1e-9)
	require.Equal(t, base.DoF()[0], m.DoF()[0], "default-calibrated joint keeps so101.json's limit")
}

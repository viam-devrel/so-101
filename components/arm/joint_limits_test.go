package arm

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"so_arm/internal/servo"
)

// The recorded range is asymmetric about the homing tick, so the limits must be too:
// the old (min+max)/2 math normalized every range to exactly [-pi, pi].
func TestJointLimitsFollowHomingTickZero(t *testing.T) {
	elbow := &servo.MotorCalibration{ID: 3, RangeMin: 866, RangeMax: 3073, NormMode: servo.NormModeDegrees}
	limits := jointLimitsFromCalibration([]*servo.MotorCalibration{elbow, nil})

	degPerTick := 360.0 / 4095.0
	wantLo := float64(866-servo.HomingTick) * degPerTick * math.Pi / 180
	wantHi := float64(3073-servo.HomingTick) * degPerTick * math.Pi / 180
	require.InDelta(t, wantLo, limits[0][0], 1e-9)
	require.InDelta(t, wantHi, limits[0][1], 1e-9)
	require.Less(t, limits[0][0], 0.0)
	require.Greater(t, limits[0][1], 0.0)

	require.Equal(t, [2]float64{-math.Pi, math.Pi}, limits[1], "missing calibration keeps the permissive default")
}

func TestJointLimitsStayOrderedUnderDriveModeInversion(t *testing.T) {
	inv := &servo.MotorCalibration{ID: 2, RangeMin: 839, RangeMax: 3205, NormMode: servo.NormModeDegrees, DriveMode: 1}
	limits := jointLimitsFromCalibration([]*servo.MotorCalibration{inv})
	require.Less(t, limits[0][0], limits[0][1])
}

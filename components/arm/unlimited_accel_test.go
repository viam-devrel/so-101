package arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/servo"
	"so_arm/internal/testfake"
)

func accelTestArm(t *testing.T, ft *testfake.FakeTransport) *so101 {
	t.Helper()
	return &so101{
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		controller:   testArmHandle(t, ft),
		armServoIDs:  []int{1, 2, 3, 4, 5},
		defaultSpeed: 50,
		defaultAcc:   servo.DefaultAccelDegsPerSecSq,
		lookaheadDeg: servo.DefaultLookaheadDeg,
	}
}

// Teleop streams small setpoints; an acceleration ramp on each one quadruples tracking error
// (measured: acc=32 -> 160 ms lag vs 40 ms at acc=0). Acc 0 is the servo's "unlimited"
// sentinel, which the deg/s^2 conversion path deliberately cannot produce.
func TestMoveToJointPositionsHonoursUnlimitedAccel(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := accelTestArm(t, ft)
	goal := []referenceframe.Input{0.3, 0.3, 0.3, 0.3, 0.3}

	require.NoError(t, s.MoveToJointPositions(context.Background(), goal,
		map[string]interface{}{"wait": false, "unlimited_accel": true}))

	for _, id := range s.armServoIDs {
		got, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.Equal(t, byte(0), got[0], "servo %d must be commanded at unlimited acceleration", id)
	}
}

// The point-to-point default keeps its ramp: 500 deg/s^2 maps to Acc 49.
func TestMoveToJointPositionsRampsByDefault(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := accelTestArm(t, ft)
	goal := []referenceframe.Input{0.3, 0.3, 0.3, 0.3, 0.3}

	require.NoError(t, s.MoveToJointPositions(context.Background(), goal,
		map[string]interface{}{"wait": false}))

	for _, id := range s.armServoIDs {
		got, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.NotZero(t, got[0], "servo %d must keep its acceleration ramp", id)
	}
}

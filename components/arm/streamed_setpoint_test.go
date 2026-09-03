package arm

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"

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

// 0 in either register is the "unbounded" sentinel -- MAX SPEED and UNLIMITED acceleration --
// which neither conversion path can produce (minSpeedSteps and MinAccUnits are both 1).
func TestMoveToJointPositionsHonoursStreamed(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := accelTestArm(t, ft)
	// Seed NON-zero: an unset register reads back as zero, so without this the assertions
	// cannot tell "commanded 0" from "never wrote the register".
	for _, id := range s.armServoIDs {
		ft.SetRegister(id, feetech.RegAcceleration.Address, []byte{0x7F})
		ft.SetRegister(id, feetech.RegGoalVelocity.Address, testfake.EncodeWordLE(1234))
	}
	goal := []referenceframe.Input{0.3, 0.3, 0.3, 0.3, 0.3}

	before := ft.PacketCount()
	require.NoError(t, s.MoveToJointPositions(context.Background(), goal,
		map[string]interface{}{"wait": false, "streamed": true}))
	// Snapshot before the assertion loop below -- each of its reads is a packet too.
	moved := ft.PacketCount() - before

	for _, id := range s.armServoIDs {
		got, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.Equal(t, byte(0), got[0], "servo %d must be commanded at unlimited acceleration", id)
		vel, err := s.controller.ReadServoRegister(context.Background(), id, "goal_velocity")
		require.NoError(t, err)
		assert.Equal(t, []byte{0, 0}, vel, "servo %d must be commanded at max speed", id)
	}
	// Skipping the coordination read leaves no trace in the register store, so only the
	// packet count can pin it.
	assert.Equal(t, 1, moved, "the unlimited path must issue one packet: the goal write")
}

// The point-to-point default keeps both its ramp and its speed cap.
func TestMoveToJointPositionsProfilesByDefault(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := accelTestArm(t, ft)
	goal := []referenceframe.Input{0.3, 0.3, 0.3, 0.3, 0.3}

	before := ft.PacketCount()
	require.NoError(t, s.MoveToJointPositions(context.Background(), goal,
		map[string]interface{}{"wait": false}))
	moved := ft.PacketCount() - before

	for _, id := range s.armServoIDs {
		got, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.NotZero(t, got[0], "servo %d must keep its acceleration ramp", id)
		vel, err := s.controller.ReadServoRegister(context.Background(), id, "goal_velocity")
		require.NoError(t, err)
		assert.NotEqual(t, []byte{0, 0}, vel, "servo %d must keep its speed cap", id)
	}
	// SyncRead for the travel reference, then SyncWrite.
	assert.Equal(t, 2, moved, "the coordinated path reads before it writes")
}

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

// Teleop streams small setpoints; an acceleration ramp on each one quadruples tracking error
// (measured: acc=32 -> 160 ms lag vs 40 ms at acc=0). Acc 0 is the servo's "unlimited"
// sentinel, which the deg/s^2 conversion path deliberately cannot produce.
func TestMoveToJointPositionsHonoursUnlimitedAccel(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := accelTestArm(t, ft)
	// Seed a NON-zero acceleration first. An unset register reads back as zero, so without
	// this the assertion below cannot tell "commanded Acc 0" from "never wrote Acc at all".
	for _, id := range s.armServoIDs {
		ft.SetRegister(id, feetech.RegAcceleration.Address, []byte{0x7F})
	}
	goal := []referenceframe.Input{0.3, 0.3, 0.3, 0.3, 0.3}

	before := ft.PacketCount()
	require.NoError(t, s.MoveToJointPositions(context.Background(), goal,
		map[string]interface{}{"wait": false, "unlimited_accel": true}))
	// Snapshot before the assertion loop below -- each of its reads is a packet too.
	moved := ft.PacketCount() - before

	for _, id := range s.armServoIDs {
		got, err := s.controller.ReadServoRegister(context.Background(), id, "acceleration")
		require.NoError(t, err)
		assert.Equal(t, byte(0), got[0], "servo %d must be commanded at unlimited acceleration", id)
	}
	// One SyncWrite and nothing else. Skipping the coordination read is half the point of
	// the branch -- at 100 Hz the read plus the write is ~3 ms of a 10 ms budget -- and it
	// leaves no trace in the register store, so only the packet count can pin it.
	assert.Equal(t, 1, moved, "the unlimited path must issue one packet: the goal write")
}

// The point-to-point default keeps its ramp: 500 deg/s^2 maps to Acc 49.
func TestMoveToJointPositionsRampsByDefault(t *testing.T) {
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
	}
	// SyncRead for the travel reference, then SyncWrite. This is the traffic the unlimited
	// path exists to avoid.
	assert.Equal(t, 2, moved, "the coordinated path reads before it writes")
}

package arm

import (
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/testfake"
)

func releaseTestArm(t *testing.T, ft *testfake.FakeTransport) *so101 {
	t.Helper()
	for id := 1; id <= 6; id++ {
		ft.SetRegister(id, feetech.RegTorqueEnable.Address, []byte{1})
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(2047))
	}
	s := accelTestArm(t, ft)
	s.cfg = &SO101ArmConfig{ManualMode: &ManualModeConfig{ReleaseTorque: true}}
	return s
}

func torqueOn(t *testing.T, s *so101, id int) bool {
	t.Helper()
	b, err := s.controller.ReadServoRegister(context.Background(), id, "torque_enable")
	require.NoError(t, err)
	return b[0] == 1
}

func TestReleaseTorqueEnterFreesArmServosOnly(t *testing.T) {
	ctx := context.Background()
	s := releaseTestArm(t, testfake.NewFakeTransport())

	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))

	for id := 1; id <= 5; id++ {
		require.False(t, torqueOn(t, s, id), "servo %d", id)
	}
	require.True(t, torqueOn(t, s, 6), "gripper servo untouched")
	on, err := s.ManualMode(ctx, nil)
	require.NoError(t, err)
	require.True(t, on)
	st, err := s.Status(ctx)
	require.NoError(t, err)
	require.Equal(t, "manual", st["mode"])
	require.Equal(t, map[string]interface{}{"release_torque": true}, st["manual_mode"])
}

func TestReleaseTorqueExitHoldsWhereTheHandLeftIt(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
	for id := 1; id <= 5; id++ {
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(1000+id))
	}

	require.NoError(t, s.SetManualMode(ctx, false, 0, nil))

	for id := 1; id <= 5; id++ {
		b, err := s.controller.ReadServoRegister(ctx, id, "goal_position")
		require.NoError(t, err)
		require.Equal(t, 1000+id, int(b[0])|int(b[1])<<8)
		require.True(t, torqueOn(t, s, id))
	}
	on, _ := s.ManualMode(ctx, nil)
	require.False(t, on)
}

func TestReleaseTorqueFailedReHoldStaysManualAndBlocksMotion(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
	ft.SetRegister(1, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(5000))

	require.Error(t, s.SetManualMode(ctx, false, 0, nil))
	on, _ := s.ManualMode(ctx, nil)
	require.True(t, on, "a failed re-hold must not report the arm as holding")

	err := s.MoveToJointPositions(ctx, []referenceframe.Input{0, 0, 0, 0, 0}, nil)
	require.ErrorContains(t, err, "re-hold")
	require.Zero(t, ft.WriteCount(1, feetech.RegAcceleration.Address), "no goal written to a limp arm")
}

func TestReleaseTorqueMotionCommandExitsAndReHolds(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
	gripperTorqueWrites := ft.WriteCount(6, feetech.RegTorqueEnable.Address)

	_ = s.MoveToJointPositions(ctx, []referenceframe.Input{0, 0, 0, 0, 0}, nil)

	on, _ := s.ManualMode(ctx, nil)
	require.False(t, on)
	for id := 1; id <= 5; id++ {
		require.True(t, torqueOn(t, s, id), "servo %d", id)
	}
	require.Equal(t, gripperTorqueWrites, ft.WriteCount(6, feetech.RegTorqueEnable.Address), "gripper untouched on exit")
}

func TestSetTorqueExitsReleasedManualMode(t *testing.T) {
	for _, enable := range []bool{true, false} {
		ctx := context.Background()
		ft := testfake.NewFakeTransport()
		s := releaseTestArm(t, ft)
		require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
		goalWrites := ft.WriteCount(1, feetech.RegGoalPosition.Address)

		_, err := s.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": enable})
		require.NoError(t, err)

		on, _ := s.ManualMode(ctx, nil)
		require.False(t, on, "enable=%v", enable)
		if !enable {
			require.Equal(t, goalWrites, ft.WriteCount(1, feetech.RegGoalPosition.Address),
				"set_torque false must not re-hold (briefly stiffen) a released arm")
		}
	}
}

func TestSetTorqueFailureKeepsReleasedManualMode(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
	ft.Unplug()

	_, err := s.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": false})
	require.Error(t, err)

	on, _ := s.ManualMode(ctx, nil)
	require.True(t, on, "a failed set_torque must not report a released arm as auto")
}

func TestSetTorqueEnableFromReleaseSeedsOnce(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 0, nil))
	goalWrites := ft.WriteCount(1, feetech.RegGoalPosition.Address)

	_, err := s.DoCommand(ctx, map[string]interface{}{"command": "set_torque", "enable": true})
	require.NoError(t, err)

	require.Equal(t, goalWrites+1, ft.WriteCount(1, feetech.RegGoalPosition.Address))
}

func TestReleasedReenterCancelsPendingAutoExit(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	s := releaseTestArm(t, ft)
	require.NoError(t, s.SetManualMode(ctx, true, 50*time.Millisecond, nil))
	goalWrites := ft.WriteCount(1, feetech.RegGoalPosition.Address)

	_, err := s.DoCommand(ctx, map[string]interface{}{"command": "enter_manual_mode"})
	require.NoError(t, err)
	time.Sleep(200 * time.Millisecond)

	on, _ := s.ManualMode(ctx, nil)
	require.True(t, on)
	for id := 1; id <= 5; id++ {
		require.False(t, torqueOn(t, s, id), "servo %d", id)
	}
	require.Equal(t, goalWrites, ft.WriteCount(1, feetech.RegGoalPosition.Address))
}

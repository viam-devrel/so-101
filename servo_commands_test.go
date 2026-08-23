package so_arm

import (
	"context"
	"errors"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
	"so_arm/internal/servocmd"
	"so_arm/internal/testfake"
)

// armServos is the arm's own servo set for these tests: a standard 5-DOF SO-101.
var armServos = []int{1, 2, 3, 4, 5}

func TestServoCapabilitiesReportsNonArmServos(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	res, err := servocmd.HandleServoCommand(context.Background(),
		map[string]any{"command": servocmd.CmdServoCapabilities}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, true, res["servo_commands"])
	assert.Equal(t, []int{6}, res["servo_ids"])
}

func TestServoCapabilitiesRespectsFourDOFArm(t *testing.T) {
	// A 4-DOF variant leaves servos 5 and 6 available, so a gripper may sit on servo 5.
	ops := &testfake.FakeServoOps{}
	res, err := servocmd.HandleServoCommand(context.Background(),
		map[string]any{"command": servocmd.CmdServoCapabilities}, ops, []int{1, 2, 3, 4})
	require.NoError(t, err)
	assert.Equal(t, []int{5, 6}, res["servo_ids"])
}

func TestServoMoveByPercent(t *testing.T) {
	ops := &testfake.FakeServoOps{Percent: 95, Raw: 3000}
	res, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
		"command": servocmd.CmdServoMove, "servo_id": 6, "percent": 95.0,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.MovedID)
	assert.Equal(t, 95.0, ops.MovedPercent)
	assert.Equal(t, 95.0, res["percent"])
}

func TestServoMoveByRaw(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
		"command": servocmd.CmdServoMove, "servo_id": 6, "raw": 1834,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 1834, ops.MovedRaw)
	assert.Zero(t, ops.MovedPercent, "raw form must not also issue a percent move")
}

func TestServoMoveClampsPercent(t *testing.T) {
	for _, tc := range []struct{ in, want float64 }{{-10, 0}, {150, 100}, {42, 42}} {
		ops := &testfake.FakeServoOps{}
		_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
			"command": servocmd.CmdServoMove, "servo_id": 6, "percent": tc.in,
		}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, tc.want, ops.MovedPercent, "percent %v", tc.in)
	}
}

// The arm must refuse to drive its own joints on behalf of a gripper, or the two would
// fight over the same servo.
func TestServoCommandRejectsArmOwnedServo(t *testing.T) {
	for _, id := range armServos {
		ops := &testfake.FakeServoOps{}
		_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
			"command": servocmd.CmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo %d belongs to the arm", id)
		assert.Zero(t, ops.MovedID, "no move should be issued for servo %d", id)
	}
}

func TestServoCommandRejectsOutOfRangeServo(t *testing.T) {
	for _, id := range []int{0, 7, -1} {
		ops := &testfake.FakeServoOps{}
		_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
			"command": servocmd.CmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo id %d is out of range", id)
	}
}

func TestServoCommandRequiresServoID(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := servocmd.HandleServoCommand(context.Background(),
		map[string]any{"command": servocmd.CmdServoMove, "percent": 50.0}, ops, armServos)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "servo_id")
}

// A local dependency delivers Go-native ints; a remote arm delivers float64 through
// structpb. Both must work or the gripper breaks against a remote arm while every
// in-process test stays green.
func TestServoCommandCoercesNumericTypes(t *testing.T) {
	for name, id := range map[string]any{
		"int":     6,
		"int64":   int64(6),
		"float64": float64(6),
	} {
		t.Run(name, func(t *testing.T) {
			ops := &testfake.FakeServoOps{}
			_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
				"command": servocmd.CmdServoMove, "servo_id": id, "percent": 50.0,
			}, ops, armServos)
			require.NoError(t, err)
			assert.Equal(t, 6, ops.MovedID)
		})
	}

	t.Run("raw as float64", func(t *testing.T) {
		ops := &testfake.FakeServoOps{}
		_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
			"command": servocmd.CmdServoMove, "servo_id": float64(6), "raw": float64(1834),
		}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, 1834, ops.MovedRaw)
	})
}

func TestServoPositionReturnsPercentAndRaw(t *testing.T) {
	ops := &testfake.FakeServoOps{Percent: 42.5, Raw: 1834}
	res, err := servocmd.HandleServoCommand(context.Background(),
		map[string]any{"command": servocmd.CmdServoPosition, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 42.5, res["percent"])
	assert.Equal(t, 1834, res["raw"])
}

func TestServoStopIsScopedToOneServo(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := servocmd.HandleServoCommand(context.Background(),
		map[string]any{"command": servocmd.CmdServoStop, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.StoppedID)
}

func TestServoWaitStopIsScopedToOneServo(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
		"command": servocmd.CmdServoWaitStop, "servo_id": 6, "timeout_ms": 1500,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, []int{6}, ops.WaitedIDs)
	assert.Equal(t, 1500, ops.WaitedMs)
}

func TestServoCommandPropagatesOpsError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &testfake.FakeServoOps{MoveErr: boom}
	_, err := servocmd.HandleServoCommand(context.Background(), map[string]any{
		"command": servocmd.CmdServoMove, "servo_id": 6, "percent": 50.0,
	}, ops, armServos)
	require.ErrorIs(t, err, boom)
}

// servocmd.IsServoCommand lets the arm's DoCommand route only the servo_* family here and leave
// its own commands alone.
func TestIsServoCommand(t *testing.T) {
	assert.True(t, servocmd.IsServoCommand(servocmd.CmdServoMove))
	assert.True(t, servocmd.IsServoCommand(servocmd.CmdServoCapabilities))
	assert.False(t, servocmd.IsServoCommand("set_torque"))
	assert.False(t, servocmd.IsServoCommand(""))
}

// The real controller must satisfy servocmd.ServoOps, or the arm cannot serve the protocol that
// every test above exercises against fakes. This assertion is the seam between the tested
// dispatch logic and the hardware path, which needs a serial port and so cannot be unit
// tested directly.
var _ servocmd.ServoOps = (*controller.ControllerHandle)(nil)

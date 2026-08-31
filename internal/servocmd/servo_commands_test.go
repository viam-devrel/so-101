package servocmd

import (
	"context"
	"errors"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// armServos is the arm's own servo set for these tests: a standard 5-DOF SO-101.
var armServos = []int{1, 2, 3, 4, 5}

func TestServoCapabilitiesReportsNonArmServos(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	res, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoCapabilities}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, true, res["servo_commands"])
	assert.Equal(t, []int{6}, res["servo_ids"])
}

func TestServoCapabilitiesRespectsFourDOFArm(t *testing.T) {
	// A 4-DOF variant leaves servos 5 and 6 available, so a gripper may sit on servo 5.
	ops := &testfake.FakeServoOps{}
	res, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoCapabilities}, ops, []int{1, 2, 3, 4})
	require.NoError(t, err)
	assert.Equal(t, []int{5, 6}, res["servo_ids"])
}

func TestServoMoveByPercent(t *testing.T) {
	ops := &testfake.FakeServoOps{Percent: 95, Raw: 3000}
	res, err := HandleServoCommand(context.Background(), map[string]any{
		"command": CmdServoMove, "servo_id": 6, "percent": 95.0,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.MovedID)
	assert.Equal(t, 95.0, ops.MovedPercent)
	assert.Equal(t, 95.0, res["percent"])
}

func TestServoMoveByRaw(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := HandleServoCommand(context.Background(), map[string]any{
		"command": CmdServoMove, "servo_id": 6, "raw": 1834,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 1834, ops.MovedRaw)
	assert.Zero(t, ops.MovedPercent, "raw form must not also issue a percent move")
}

func TestServoMoveClampsPercent(t *testing.T) {
	for _, tc := range []struct{ in, want float64 }{{-10, 0}, {150, 100}, {42, 42}} {
		ops := &testfake.FakeServoOps{}
		_, err := HandleServoCommand(context.Background(), map[string]any{
			"command": CmdServoMove, "servo_id": 6, "percent": tc.in,
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
		_, err := HandleServoCommand(context.Background(), map[string]any{
			"command": CmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo %d belongs to the arm", id)
		assert.Zero(t, ops.MovedID, "no move should be issued for servo %d", id)
	}
}

func TestServoCommandRejectsOutOfRangeServo(t *testing.T) {
	for _, id := range []int{0, 7, -1} {
		ops := &testfake.FakeServoOps{}
		_, err := HandleServoCommand(context.Background(), map[string]any{
			"command": CmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo id %d is out of range", id)
	}
}

func TestServoCommandRequiresServoID(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoMove, "percent": 50.0}, ops, armServos)
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
			_, err := HandleServoCommand(context.Background(), map[string]any{
				"command": CmdServoMove, "servo_id": id, "percent": 50.0,
			}, ops, armServos)
			require.NoError(t, err)
			assert.Equal(t, 6, ops.MovedID)
		})
	}

	t.Run("raw as float64", func(t *testing.T) {
		ops := &testfake.FakeServoOps{}
		_, err := HandleServoCommand(context.Background(), map[string]any{
			"command": CmdServoMove, "servo_id": float64(6), "raw": float64(1834),
		}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, 1834, ops.MovedRaw)
	})
}

func TestServoPositionReturnsPercentAndRaw(t *testing.T) {
	ops := &testfake.FakeServoOps{Percent: 42.5, Raw: 1834}
	res, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoPosition, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 42.5, res["percent"])
	assert.Equal(t, 1834, res["raw"])
}

func TestServoLoadReturnsValue(t *testing.T) {
	ops := &testfake.FakeServoOps{Load: -37}
	res, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoLoad, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, -37, res["load"])
}

func TestServoLoadPropagatesError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &testfake.FakeServoOps{LoadErr: boom}
	_, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoLoad, "servo_id": 6}, ops, armServos)
	require.ErrorIs(t, err, boom)
}

func TestServoStopIsScopedToOneServo(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoStop, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.StoppedID)
}

func TestServoWaitStopIsScopedToOneServo(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := HandleServoCommand(context.Background(), map[string]any{
		"command": CmdServoWaitStop, "servo_id": 6, "timeout_ms": 1500,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, []int{6}, ops.WaitedIDs)
	assert.Equal(t, 1500, ops.WaitedMs)
}

func TestServoMovingReturnsUnderlyingState(t *testing.T) {
	for _, want := range []bool{true, false} {
		ops := &testfake.FakeServoOps{Moving: want}
		res, err := HandleServoCommand(context.Background(),
			map[string]any{"command": CmdServoMoving, "servo_id": 6}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, want, res["moving"])
		assert.Equal(t, []int{6}, ops.MovingIDs)
	}
}

func TestServoMovingRejectsArmOwnedServo(t *testing.T) {
	for _, id := range armServos {
		ops := &testfake.FakeServoOps{}
		_, err := HandleServoCommand(context.Background(),
			map[string]any{"command": CmdServoMoving, "servo_id": id}, ops, armServos)
		require.Error(t, err, "servo %d belongs to the arm", id)
	}
}

func TestServoMovingRequiresServoID(t *testing.T) {
	ops := &testfake.FakeServoOps{}
	_, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoMoving}, ops, armServos)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "servo_id")
}

func TestServoMovingCoercesFloat64ServoID(t *testing.T) {
	ops := &testfake.FakeServoOps{Moving: true}
	res, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoMoving, "servo_id": float64(6)}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, true, res["moving"])
	assert.Equal(t, []int{6}, ops.MovingIDs)
}

func TestServoMovingPropagatesOpsError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &testfake.FakeServoOps{MovingErr: boom}
	_, err := HandleServoCommand(context.Background(),
		map[string]any{"command": CmdServoMoving, "servo_id": 6}, ops, armServos)
	require.ErrorIs(t, err, boom)
}

func TestServoCommandPropagatesOpsError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &testfake.FakeServoOps{MoveErr: boom}
	_, err := HandleServoCommand(context.Background(), map[string]any{
		"command": CmdServoMove, "servo_id": 6, "percent": 50.0,
	}, ops, armServos)
	require.ErrorIs(t, err, boom)
}

// IsServoCommand lets the arm's DoCommand route only the servo_* family here and leave
// its own commands alone.
func TestIsServoCommand(t *testing.T) {
	assert.True(t, IsServoCommand(CmdServoMove))
	assert.True(t, IsServoCommand(CmdServoCapabilities))
	assert.False(t, IsServoCommand("set_torque"))
	assert.False(t, IsServoCommand(""))
}

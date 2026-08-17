package so_arm

import (
	"context"
	"errors"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeServoOps records the single-servo operations handleServoCommand dispatches, so the
// protocol can be tested without a serial bus.
type fakeServoOps struct {
	movedID      int
	movedPercent float64
	movedSpeed   int
	movedRaw     int
	stoppedID    int
	waitedIDs    []int
	waitedMs     int

	percent  float64
	raw      int
	moveErr  error
	posErr   error
	moveRaws []int
}

func (f *fakeServoOps) MoveServoPercent(_ context.Context, id int, percent float64, speed int) error {
	f.movedID, f.movedPercent, f.movedSpeed = id, percent, speed
	return f.moveErr
}

func (f *fakeServoOps) MoveServoRaw(_ context.Context, id, raw int) error {
	f.movedID, f.movedRaw = id, raw
	f.moveRaws = append(f.moveRaws, raw)
	return f.moveErr
}

func (f *fakeServoOps) ServoPositionPercent(_ context.Context, id int) (float64, int, error) {
	if f.posErr != nil {
		return 0, 0, f.posErr
	}
	return f.percent, f.raw, nil
}

func (f *fakeServoOps) StopServo(_ context.Context, id int) error {
	f.stoppedID = id
	return nil
}

func (f *fakeServoOps) WaitForServosToStop(_ context.Context, ids []int, timeoutMs int) error {
	f.waitedIDs, f.waitedMs = ids, timeoutMs
	return nil
}

// armServos is the arm's own servo set for these tests: a standard 5-DOF SO-101.
var armServos = []int{1, 2, 3, 4, 5}

func TestServoCapabilitiesReportsNonArmServos(t *testing.T) {
	ops := &fakeServoOps{}
	res, err := handleServoCommand(context.Background(),
		map[string]any{"command": cmdServoCapabilities}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, true, res["servo_commands"])
	assert.Equal(t, []int{6}, res["servo_ids"])
}

func TestServoCapabilitiesRespectsFourDOFArm(t *testing.T) {
	// A 4-DOF variant leaves servos 5 and 6 available, so a gripper may sit on servo 5.
	ops := &fakeServoOps{}
	res, err := handleServoCommand(context.Background(),
		map[string]any{"command": cmdServoCapabilities}, ops, []int{1, 2, 3, 4})
	require.NoError(t, err)
	assert.Equal(t, []int{5, 6}, res["servo_ids"])
}

func TestServoMoveByPercent(t *testing.T) {
	ops := &fakeServoOps{percent: 95, raw: 3000}
	res, err := handleServoCommand(context.Background(), map[string]any{
		"command": cmdServoMove, "servo_id": 6, "percent": 95.0,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.movedID)
	assert.Equal(t, 95.0, ops.movedPercent)
	assert.Equal(t, 95.0, res["percent"])
}

func TestServoMoveByRaw(t *testing.T) {
	ops := &fakeServoOps{}
	_, err := handleServoCommand(context.Background(), map[string]any{
		"command": cmdServoMove, "servo_id": 6, "raw": 1834,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 1834, ops.movedRaw)
	assert.Zero(t, ops.movedPercent, "raw form must not also issue a percent move")
}

func TestServoMoveClampsPercent(t *testing.T) {
	for _, tc := range []struct{ in, want float64 }{{-10, 0}, {150, 100}, {42, 42}} {
		ops := &fakeServoOps{}
		_, err := handleServoCommand(context.Background(), map[string]any{
			"command": cmdServoMove, "servo_id": 6, "percent": tc.in,
		}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, tc.want, ops.movedPercent, "percent %v", tc.in)
	}
}

// The arm must refuse to drive its own joints on behalf of a gripper, or the two would
// fight over the same servo.
func TestServoCommandRejectsArmOwnedServo(t *testing.T) {
	for _, id := range armServos {
		ops := &fakeServoOps{}
		_, err := handleServoCommand(context.Background(), map[string]any{
			"command": cmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo %d belongs to the arm", id)
		assert.Zero(t, ops.movedID, "no move should be issued for servo %d", id)
	}
}

func TestServoCommandRejectsOutOfRangeServo(t *testing.T) {
	for _, id := range []int{0, 7, -1} {
		ops := &fakeServoOps{}
		_, err := handleServoCommand(context.Background(), map[string]any{
			"command": cmdServoMove, "servo_id": id, "percent": 50.0,
		}, ops, armServos)
		require.Error(t, err, "servo id %d is out of range", id)
	}
}

func TestServoCommandRequiresServoID(t *testing.T) {
	ops := &fakeServoOps{}
	_, err := handleServoCommand(context.Background(),
		map[string]any{"command": cmdServoMove, "percent": 50.0}, ops, armServos)
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
			ops := &fakeServoOps{}
			_, err := handleServoCommand(context.Background(), map[string]any{
				"command": cmdServoMove, "servo_id": id, "percent": 50.0,
			}, ops, armServos)
			require.NoError(t, err)
			assert.Equal(t, 6, ops.movedID)
		})
	}

	t.Run("raw as float64", func(t *testing.T) {
		ops := &fakeServoOps{}
		_, err := handleServoCommand(context.Background(), map[string]any{
			"command": cmdServoMove, "servo_id": float64(6), "raw": float64(1834),
		}, ops, armServos)
		require.NoError(t, err)
		assert.Equal(t, 1834, ops.movedRaw)
	})
}

func TestServoPositionReturnsPercentAndRaw(t *testing.T) {
	ops := &fakeServoOps{percent: 42.5, raw: 1834}
	res, err := handleServoCommand(context.Background(),
		map[string]any{"command": cmdServoPosition, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 42.5, res["percent"])
	assert.Equal(t, 1834, res["raw"])
}

func TestServoStopIsScopedToOneServo(t *testing.T) {
	ops := &fakeServoOps{}
	_, err := handleServoCommand(context.Background(),
		map[string]any{"command": cmdServoStop, "servo_id": 6}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, 6, ops.stoppedID)
}

func TestServoWaitStopIsScopedToOneServo(t *testing.T) {
	ops := &fakeServoOps{}
	_, err := handleServoCommand(context.Background(), map[string]any{
		"command": cmdServoWaitStop, "servo_id": 6, "timeout_ms": 1500,
	}, ops, armServos)
	require.NoError(t, err)
	assert.Equal(t, []int{6}, ops.waitedIDs)
	assert.Equal(t, 1500, ops.waitedMs)
}

func TestServoCommandPropagatesOpsError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &fakeServoOps{moveErr: boom}
	_, err := handleServoCommand(context.Background(), map[string]any{
		"command": cmdServoMove, "servo_id": 6, "percent": 50.0,
	}, ops, armServos)
	require.ErrorIs(t, err, boom)
}

// isServoCommand lets the arm's DoCommand route only the servo_* family here and leave
// its own commands alone.
func TestIsServoCommand(t *testing.T) {
	assert.True(t, isServoCommand(cmdServoMove))
	assert.True(t, isServoCommand(cmdServoCapabilities))
	assert.False(t, isServoCommand("set_torque"))
	assert.False(t, isServoCommand(""))
}

// The real controller must satisfy servoOps, or the arm cannot serve the protocol that
// every test above exercises against fakes. This assertion is the seam between the tested
// dispatch logic and the hardware path, which needs a serial port and so cannot be unit
// tested directly.
var _ servoOps = (*SafeSoArmController)(nil)

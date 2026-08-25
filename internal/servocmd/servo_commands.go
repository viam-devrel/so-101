// Package servocmd implements the servo_* DoCommand protocol: the gripper owns no serial
// connection, so it drives its servo by sending these commands to the arm component it
// depends on.
package servocmd

import (
	"context"
	"fmt"
	"strings"
)

// The servo_* DoCommand family. The gripper owns no serial connection: it drives its servo
// by sending these to the arm component it depends on. Both sides live in this package, so
// the command names are shared constants rather than loose strings -- but they are still
// wire-compatible, so the arm may be a remote.
const (
	CmdServoCapabilities = "servo_capabilities"
	CmdServoMove         = "servo_move"
	CmdServoMoving       = "servo_moving"
	CmdServoPosition     = "servo_position"
	CmdServoStop         = "servo_stop"
	CmdServoWaitStop     = "servo_wait_stop"
)

// ServoOps is the single-servo surface HandleServoCommand needs. Keeping dispatch behind
// this interface (rather than calling the controller directly) is what lets the protocol be
// tested without serial hardware.
type ServoOps interface {
	MoveServoPercent(ctx context.Context, id int, percent float64, speed int) error
	MoveServoRaw(ctx context.Context, id, raw int) error
	ServoPositionPercent(ctx context.Context, id int) (percent float64, raw int, err error)
	StopServo(ctx context.Context, id int) error
	WaitForServosToStop(ctx context.Context, ids []int, timeoutMs int) error
	AnyServoMoving(ctx context.Context, ids []int) (bool, error)
}

// IsServoCommand reports whether a DoCommand belongs to this family, so the arm can route
// it here and leave its own commands untouched.
func IsServoCommand(command string) bool {
	return strings.HasPrefix(command, "servo_")
}

// NumArg coerces a DoCommand numeric argument to int.
//
// This coercion is load-bearing, not defensive. When the arm is a local dependency the
// module hands over the real Go object, so values arrive as native ints. When the arm is
// remote the call crosses gRPC and structpb renders every number as float64. Accepting only
// one shape would work in every in-process test and fail against a remote arm.
func NumArg(cmd map[string]any, key string) (int, bool) {
	switch v := cmd[key].(type) {
	case int:
		return v, true
	case int64:
		return int(v), true
	case float64:
		return int(v), true
	default:
		return 0, false
	}
}

// FloatArg coerces a DoCommand numeric argument to float64, accepting the same shapes.
func FloatArg(cmd map[string]any, key string) (float64, bool) {
	switch v := cmd[key].(type) {
	case int:
		return float64(v), true
	case int64:
		return float64(v), true
	case float64:
		return v, true
	default:
		return 0, false
	}
}

// ClampPercent clamps p into [0, 100].
func ClampPercent(p float64) float64 {
	if p < 0 {
		return 0
	}
	if p > 100 {
		return 100
	}
	return p
}

// WaitArg reads the optional "wait" bool from a command or extra map. Absent or
// non-bool values default to true, so every existing caller keeps blocking.
func WaitArg(cmd map[string]any) bool {
	if w, ok := cmd["wait"].(bool); ok {
		return w
	}
	return true
}

// availableServoIDs returns the servos on the bus that the arm does not drive itself, and
// which are therefore available to a gripper. Deriving this from the arm's own servo set
// (rather than hardcoding 6) is what lets a 4-DOF arm configured with servo_ids [1,2,3,4]
// host a gripper on servo 5.
func availableServoIDs(ownServos []int) []int {
	owned := make(map[int]bool, len(ownServos))
	for _, id := range ownServos {
		owned[id] = true
	}
	available := []int{}
	for id := 1; id <= 6; id++ {
		if !owned[id] {
			available = append(available, id)
		}
	}
	return available
}

// resolveServoID extracts and validates the target servo, refusing any servo the arm drives
// itself -- otherwise the arm and the gripper would fight over the same joint.
func resolveServoID(cmd map[string]any, ownServos []int) (int, error) {
	id, ok := NumArg(cmd, "servo_id")
	if !ok {
		return 0, fmt.Errorf("%s requires a numeric servo_id", cmd["command"])
	}
	if id < 1 || id > 6 {
		return 0, fmt.Errorf("servo_id must be between 1 and 6, got %d", id)
	}
	for _, own := range ownServos {
		if id == own {
			return 0, fmt.Errorf(
				"servo_id %d is driven by this arm (servo_ids %v) and cannot be commanded separately",
				id, ownServos)
		}
	}
	return id, nil
}

// HandleServoCommand dispatches the servo_* family against ops. ownServos is the arm's own
// servo set, used to reject commands that would collide with arm motion.
func HandleServoCommand(
	ctx context.Context, cmd map[string]any, ops ServoOps, ownServos []int,
) (map[string]any, error) {
	command, _ := cmd["command"].(string)

	if command == CmdServoCapabilities {
		return map[string]any{
			"servo_commands": true,
			"servo_ids":      availableServoIDs(ownServos),
		}, nil
	}

	id, err := resolveServoID(cmd, ownServos)
	if err != nil {
		return nil, err
	}

	switch command {
	case CmdServoMove:
		if raw, ok := NumArg(cmd, "raw"); ok {
			if err := ops.MoveServoRaw(ctx, id, raw); err != nil {
				return nil, err
			}
			return map[string]any{"raw": raw}, nil
		}
		percent, ok := FloatArg(cmd, "percent")
		if !ok {
			return nil, fmt.Errorf("%s requires a numeric percent or raw", CmdServoMove)
		}
		percent = ClampPercent(percent)
		speed, _ := NumArg(cmd, "speed")
		if err := ops.MoveServoPercent(ctx, id, percent, speed); err != nil {
			return nil, err
		}
		return map[string]any{"percent": percent}, nil

	case CmdServoPosition:
		percent, raw, err := ops.ServoPositionPercent(ctx, id)
		if err != nil {
			return nil, err
		}
		return map[string]any{"percent": percent, "raw": raw}, nil

	case CmdServoStop:
		if err := ops.StopServo(ctx, id); err != nil {
			return nil, err
		}
		return map[string]any{}, nil

	case CmdServoMoving:
		moving, err := ops.AnyServoMoving(ctx, []int{id})
		if err != nil {
			return nil, err
		}
		return map[string]any{"moving": moving}, nil

	case CmdServoWaitStop:
		timeoutMs, ok := NumArg(cmd, "timeout_ms")
		if !ok {
			timeoutMs = defaultServoWaitTimeoutMs
		}
		if err := ops.WaitForServosToStop(ctx, []int{id}, timeoutMs); err != nil {
			return nil, err
		}
		return map[string]any{}, nil

	default:
		return nil, fmt.Errorf("unknown servo command: %v", cmd["command"])
	}
}

// defaultServoWaitTimeoutMs bounds servo_wait_stop when the caller does not specify one.
const defaultServoWaitTimeoutMs = 3000

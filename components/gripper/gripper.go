// Package gripper implements devrel:so101:gripper, the hardware gripper (servo 6).
// It owns no serial connection: it drives its servo by sending servo_* DoCommands to
// the arm component it depends on, which may be remote.
package gripper

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
	"so_arm/internal/servocmd"
)

var (
	SO101GripperModel = resource.NewModel("devrel", "so101", "gripper")
)

type SO101GripperConfig struct {
	// Arm names the devrel:so101:arm component that owns this gripper's serial bus.
	// The gripper issues no serial traffic of its own -- it drives its servo through
	// the arm's servo_* DoCommand family -- so the arm is a required dependency.
	Arm string `json:"arm"`

	// Default to 6
	ServoID int `json:"servo_id,omitempty"`

	// GripperType selects which meshes Geometries() serves: "follower" (moving
	// jaw, the default) or "leader" (thumb-loop handle + trigger).
	GripperType string `json:"gripper_type,omitempty"`

	// MeshDetail selects the gripper mesh resolution Geometries() serves:
	// "low" (decimated, the default -- keeps motion planning fast) or "high"
	// (full resolution).
	MeshDetail string `json:"mesh_detail,omitempty"`
}

// Validate ensures all parts of the config are valid, and reports the arm as a required
// dependency so viam-server constructs it first and hands this module the real arm object.
func (cfg *SO101GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf(
			"must specify arm: the SO-101 gripper shares the arm's serial connection. " +
				"Set \"arm\" to your devrel:so101:arm component name (and remove \"port\")")
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	if cfg.ServoID < 1 || cfg.ServoID > 6 {
		return nil, nil, fmt.Errorf("servo_id must be between 1 and 6, got %d", cfg.ServoID)
	}

	if cfg.GripperType != "" && cfg.GripperType != geometry.LeaderGripper && cfg.GripperType != geometry.FollowerGripper {
		return nil, nil, fmt.Errorf("gripper_type must be %q or %q, got %q",
			geometry.LeaderGripper, geometry.FollowerGripper, cfg.GripperType)
	}

	if cfg.MeshDetail != "" && cfg.MeshDetail != geometry.HighDetail && cfg.MeshDetail != geometry.LowDetail {
		return nil, nil, fmt.Errorf("mesh_detail must be %q or %q, got %q",
			geometry.HighDetail, geometry.LowDetail, cfg.MeshDetail)
	}

	return []string{cfg.Arm}, nil, nil
}

type so101Gripper struct {
	resource.AlwaysRebuild

	name resource.Name
	// arm owns the serial bus. The gripper drives its servo exclusively through this
	// dependency's servo_* DoCommand family and holds no controller, port, or
	// calibration state of its own.
	arm         arm.Arm
	logger      logging.Logger
	gripperType string
	meshDetail  string
	servoID     int
	model       referenceframe.Model

	mu       sync.Mutex
	isMoving atomic.Bool

	// Gripper positions in percentage, 0-100%
	openPosition   float64
	closedPosition float64

	speed        float32
	acceleration float32
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		SO101GripperModel,
		resource.Registration[gripper.Gripper, *SO101GripperConfig]{
			Constructor: newSO101Gripper,
		},
	)
}

func newSO101Gripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*SO101GripperConfig](conf)
	if err != nil {
		return nil, err
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	armDep, err := arm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm %q for gripper: %w", cfg.Arm, err)
	}

	if err := probeServoSupport(ctx, armDep, cfg.Arm, cfg.ServoID); err != nil {
		return nil, err
	}

	gripperType := cfg.GripperType
	if gripperType == "" {
		gripperType = geometry.FollowerGripper
	}

	meshDetail := cfg.MeshDetail
	if meshDetail == "" {
		meshDetail = geometry.LowDetail
	}

	model, err := geometry.BuildGripperModel(gripperType, meshDetail, conf.ResourceName().ShortName())
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	g := &so101Gripper{
		name:           conf.ResourceName(),
		logger:         logger,
		arm:            armDep,
		gripperType:    gripperType,
		meshDetail:     meshDetail,
		servoID:        cfg.ServoID,
		model:          model,
		speed:          30,
		acceleration:   50,
		openPosition:   95.0,
		closedPosition: 0.0,
	}

	logger.Debugf("SO-101 gripper initialized with servo ID %d, open=%.1f%%, closed=%.1f%%",
		cfg.ServoID, g.openPosition, g.closedPosition)

	return g, nil
}

func (g *so101Gripper) Name() resource.Name {
	return g.name
}

// probeServoSupport verifies at construction time that the configured arm actually speaks
// the servo_* protocol and offers this gripper's servo. Without it, a gripper pointed at a
// non-SO-101 arm (or a stale arm name) would build cleanly and fail only on the first move.
func probeServoSupport(ctx context.Context, a arm.Arm, armName string, servoID int) error {
	res, err := a.DoCommand(ctx, map[string]interface{}{"command": servocmd.CmdServoCapabilities})
	if err != nil {
		return fmt.Errorf(
			"arm %q does not support the servo_* command family (is it a devrel:so101:arm?): %w",
			armName, err)
	}
	for _, id := range servoIDsFromCapabilities(res) {
		if id == servoID {
			return nil
		}
	}
	return fmt.Errorf("arm %q does not offer servo %d (it reports %v)",
		armName, servoID, servoIDsFromCapabilities(res))
}

// servoIDsFromCapabilities reads the servo_ids list out of a capabilities response,
// tolerating both the local ([]int) and remote ([]interface{} of float64) encodings.
func servoIDsFromCapabilities(res map[string]interface{}) []int {
	switch ids := res["servo_ids"].(type) {
	case []int:
		return ids
	case []interface{}:
		out := make([]int, 0, len(ids))
		for i := range ids {
			if n, ok := servocmd.NumArg(map[string]any{"v": ids[i]}, "v"); ok {
				out = append(out, n)
			}
		}
		return out
	default:
		return nil
	}
}

// servoDo sends one servo_* command for this gripper's servo.
func (g *so101Gripper) servoDo(
	ctx context.Context, command string, args map[string]interface{},
) (map[string]interface{}, error) {
	cmd := map[string]interface{}{"command": command, "servo_id": g.servoID}
	for k, v := range args {
		cmd[k] = v
	}
	return g.arm.DoCommand(ctx, cmd)
}

// moveToPercent commands the servo and waits for it to settle.
func (g *so101Gripper) moveToPercent(ctx context.Context, percent float64) error {
	if _, err := g.servoDo(ctx, servocmd.CmdServoMove,
		map[string]interface{}{"percent": servocmd.ClampPercent(percent)}); err != nil {
		return err
	}
	_, err := g.servoDo(ctx, servocmd.CmdServoWaitStop,
		map[string]interface{}{"timeout_ms": gripperSettleTimeoutMs})
	return err
}

// positionPercent reads the servo's current opening as a percentage.
func (g *so101Gripper) positionPercent(ctx context.Context) (float64, error) {
	res, err := g.servoDo(ctx, servocmd.CmdServoPosition, nil)
	if err != nil {
		return 0, err
	}
	pct, ok := servocmd.FloatArg(res, "percent")
	if !ok {
		return 0, fmt.Errorf("no position data available")
	}
	return pct, nil
}

// gripperSettleTimeoutMs bounds the wait for the gripper servo to stop moving. It replaces
// the fixed 500ms sleep the previous implementation used after every command.
const gripperSettleTimeoutMs = 2000

func (g *so101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Opening gripper")

	if err := g.moveToPercent(ctx, g.openPosition); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	g.logger.Debug("Gripper opened")
	return nil
}

func (g *so101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Attempting to grab with gripper")

	if err := g.moveToPercent(ctx, g.closedPosition); err != nil {
		return false, fmt.Errorf("failed to close gripper: %w", err)
	}

	currentPercent, err := g.positionPercent(ctx)
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		return true, nil
	}

	positionDifference := currentPercent - g.closedPosition
	threshold := 15.0

	grabbed := positionDifference > threshold

	if grabbed {
		g.logger.Debugf("Gripper successfully grabbed an object (position difference: %.1f%%)", positionDifference)
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop halts only this gripper's servo. The previous implementation called a controller
// Stop that zeroed velocity on every servo on the shared bus, so stopping the gripper also
// killed any in-flight arm motion.
func (g *so101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	_, err := g.servoDo(ctx, servocmd.CmdServoStop, nil)
	return err
}

func (g *so101Gripper) IsMoving(ctx context.Context) (bool, error) {
	// Fast path: mid-command is never wrong, and avoids bus traffic during a move.
	if g.isMoving.Load() {
		return true, nil
	}

	res, err := g.servoDo(ctx, servocmd.CmdServoMoving, nil)
	if err != nil {
		// A remote arm may run an older module build with no servo_moving; fall back to
		// the intent flag rather than fail a call teleop may poll repeatedly.
		g.logger.Debugf("servo_moving failed, falling back to intent flag: %v", err)
		return g.isMoving.Load(), nil
	}
	moving, ok := res["moving"].(bool)
	if !ok {
		g.logger.Debug("servo_moving response missing 'moving', falling back to intent flag")
		return g.isMoving.Load(), nil
	}
	return moving, nil
}

// jawAngle maps the gripper's current open percentage onto the URDF gripper-joint
// range. If the live position cannot be read it assumes the gripper is closed.
func (g *so101Gripper) jawAngle(ctx context.Context) float64 {
	percent, err := g.positionPercent(ctx)
	if err != nil {
		return geometry.GripperJointMin
	}
	pct := math.Max(0, math.Min(1, percent/100.0))
	return geometry.GripperJointMin + pct*(geometry.GripperJointMax-geometry.GripperJointMin)
}

// Geometries serves the gripper as meshes: a static body and a moving part posed by
// the live gripper opening, so the jaw/trigger articulates in the 3D viewer.
func (g *so101Gripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return geometry.BuildGripperMeshes(g.gripperType, g.meshDetail, g.jawAngle(ctx))
}

// Status returns the current status of the resource as a map of key-value pairs.
func (g *so101Gripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

// gripperSetPositionPercent resolves a set_position target percentage from either
// "percentage" (the key documented in the README and used by the simulated gripper
// and the teleop service) or "position_percentage" (the key get_position returns,
// kept so a single-key get/set round-trip — e.g. arm-recorder — still works).
// "percentage" takes precedence when both are supplied.
func gripperSetPositionPercent(cmd map[string]interface{}) (float64, bool) {
	if pct, ok := cmd["percentage"].(float64); ok {
		return pct, true
	}
	if pct, ok := cmd["position_percentage"].(float64); ok {
		return pct, true
	}
	return 0, false
}

func (g *so101Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["get"] == true {
		percentPos, err := g.positionPercent(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": percentPos}, nil
	}

	if percentPos, ok := cmd["set"].(float64); ok {
		percentPos = servocmd.ClampPercent(percentPos)

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		if err := g.moveToPercent(ctx, percentPos); err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": percentPos}, nil
	}

	switch cmd["command"] {
	case "get_position":
		percentPos, err := g.positionPercent(ctx)
		if err != nil {
			return nil, err
		}

		return map[string]interface{}{
			// position_radians is now derived from the percentage rather than read as a
			// radian value, so single-key get/set round-trips (arm-recorder) keep working.
			"position_radians":    (percentPos/100.0*2.0 - 1.0) * math.Pi,
			"position_percentage": percentPos,
			"open_position":       g.openPosition,
			"closed_position":     g.closedPosition,
		}, nil

	case "set_position":
		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		// The raw servo_position form goes to the arm as raw ticks; the arm owns the
		// calibration needed to interpret them, so the gripper no longer converts.
		if servoPos, ok := servocmd.FloatArg(cmd, "servo_position"); ok {
			_, err := g.servoDo(ctx, servocmd.CmdServoMove, map[string]interface{}{"raw": int(servoPos)})
			return map[string]interface{}{"success": err == nil}, err
		}

		targetPercent, ok := gripperSetPositionPercent(cmd)
		if !ok {
			return nil, fmt.Errorf("set_position command requires 'percentage', 'position_percentage', or 'servo_position' parameter")
		}

		err := g.moveToPercent(ctx, targetPercent)
		return map[string]interface{}{"success": err == nil}, err

	case "controller_status":
		// The arm owns the controller now, so forward and re-shape to this component's
		// documented keys.
		res, err := g.arm.DoCommand(ctx, map[string]interface{}{"command": "controller_status"})
		if err != nil {
			return nil, err
		}
		out := map[string]interface{}{"servo_id": g.servoID}
		for _, k := range []string{"ref_count", "has_controller", "config"} {
			if v, ok := res[k]; ok {
				out[k] = v
			}
		}
		return out, nil

	case "calibrate_positions":
		if openPos, ok := cmd["open_position"].(float64); ok {
			if openPos >= 0 && openPos <= 100 {
				g.openPosition = openPos
			}
		}
		if closedPos, ok := cmd["closed_position"].(float64); ok {
			if closedPos >= 0 && closedPos <= 100 {
				g.closedPosition = closedPos
			}
		}

		g.logger.Debugf("Gripper positions calibrated: open=%.1f%%, closed=%.1f%%", g.openPosition, g.closedPosition)

		return map[string]interface{}{
			"success":         true,
			"open_position":   g.openPosition,
			"closed_position": g.closedPosition,
		}, nil

	case "set_motion_params":
		if speed, ok := cmd["speed"].(float64); ok {
			if speed > 0 && speed <= 180 {
				g.speed = float32(speed)
			}
		}
		if acc, ok := cmd["acceleration"].(float64); ok {
			if acc > 0 && acc <= 500 {
				g.acceleration = float32(acc)
			}
		}

		return map[string]interface{}{
			"success":      true,
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	case "get_motion_params":
		return map[string]interface{}{
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// Close releases nothing: the gripper owns no serial connection. The arm it depends on
// owns the bus and is torn down on its own schedule.
func (g *so101Gripper) Close(ctx context.Context) error {
	return nil
}

func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *so101Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

// Kinematics returns the gripper's static model, whose leaf frame is the TCP between the jaw
// tips. See geometry.BuildGripperModel for why the gripper needs one at all.
func (g *so101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

func (g *so101Gripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, nil
}

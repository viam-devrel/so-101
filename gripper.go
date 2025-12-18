package so_arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	SO101GripperModel = resource.NewModel("devrel", "so101", "gripper")
)

type SO101GripperConfig struct {
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Default to 6
	ServoID int `json:"servo_id,omitempty"`

	Timeout time.Duration `json:"timeout,omitempty"`

	SpeedPercentPerSec        float32 `json:"speed_percent_per_sec,omitempty"`
	AccelerationPercentPerSec float32 `json:"acceleration_percent_per_sec_per_sec,omitempty"`

	// Shared with arm
	CalibrationFile string `json:"calibration_file,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	if cfg.ServoID < 1 || cfg.ServoID > 6 {
		return nil, nil, fmt.Errorf("servo_id must be between 1 and 6, got %d", cfg.ServoID)
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000
	}

	return nil, nil, nil
}

type so101Gripper struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	controller *SafeSoArmController
	geometries []spatialmath.Geometry
	servoID    int

	mu       sync.Mutex
	isMoving atomic.Bool

	// Gripper positions in percentage, 0-100%
	openPosition   float64
	closedPosition float64

	speed        float32
	acceleration float32

	// Load monitoring threshold for grip detection
	gripLoadThreshold int
}

// gripperMoveOptions holds movement parameters for gripper operations
type gripperMoveOptions struct {
	speedPercentPerSec        float32
	accelerationPercentPerSec float32
}

// buildMoveOptions constructs gripperMoveOptions from defaults and extra params
// Following the xarm module pattern for parameter precedence:
// 1. Start with configured defaults
// 2. Override with extra map parameters if provided
func (g *so101Gripper) buildMoveOptions(extra map[string]interface{}) gripperMoveOptions {
	speed := float32(g.speed)
	acc := float32(g.acceleration)

	// Apply extra map parameters
	if extra != nil {
		// Speed in percent/sec (preferred for gripper)
		if speedP, ok := extra["speed_percent"].(float32); ok && speedP > 0 {
			speed = speedP
		}
		// Backwards compatibility: Speed in degrees/sec (treat as percent)
		if speedD, ok := extra["speed_d"].(float32); ok && speedD > 0 {
			speed = speedD
		}
		// Acceleration in percent/sec² (preferred for gripper)
		if accP, ok := extra["acceleration_percent"].(float32); ok && accP > 0 {
			acc = accP
		}
		// Backwards compatibility: Acceleration in degrees/sec² (treat as percent)
		if accD, ok := extra["acceleration_d"].(float32); ok && accD > 0 {
			acc = accD
		}
	}

	// Clamp to valid ranges for gripper (percentage-based)
	if speed < 3 {
		speed = 3
	}
	if speed > 100 {
		speed = 100
	}
	if acc < 10 {
		acc = 10
	}
	if acc > 200 {
		acc = 200
	}

	return gripperMoveOptions{
		speedPercentPerSec:        speed,
		accelerationPercentPerSec: acc,
	}
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

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000
	}

	// Validate and set default motion parameters
	speedPercentPerSec := cfg.SpeedPercentPerSec
	if speedPercentPerSec == 0 {
		speedPercentPerSec = 30 // Default speed for gripper (30% per second)
	}
	if speedPercentPerSec < 0 || speedPercentPerSec > 100 {
		return nil, fmt.Errorf("speed_percent_per_sec must be between 0 and 100 percent/second, got %.1f", speedPercentPerSec)
	}

	accelerationPercentPerSec := cfg.AccelerationPercentPerSec
	if accelerationPercentPerSec == 0 {
		accelerationPercentPerSec = 50 // Default acceleration for gripper (50% per second²)
	}
	if accelerationPercentPerSec < 10 || accelerationPercentPerSec > 200 {
		return nil, fmt.Errorf("acceleration_percent_per_sec_per_sec must be between 10 and 200 percent/second^2, got %.1f", accelerationPercentPerSec)
	}

	controllerConfig := &SoArm101Config{
		Port:            cfg.Port,
		Baudrate:        cfg.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6},
		Timeout:         cfg.Timeout,
		CalibrationFile: cfg.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(cfg.CalibrationFile)

	fullCalibration, fromFile := controllerConfig.LoadCalibration(logger)

	if fullCalibration.Gripper.ID != cfg.ServoID {
		logger.Debugf("Updating gripper calibration servo ID from %d to %d (from config)",
			fullCalibration.Gripper.ID, cfg.ServoID)
		fullCalibration.Gripper.ID = cfg.ServoID
	}

	controller, err := GetSharedControllerWithCalibration(controllerConfig, fullCalibration, fromFile)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared controller for gripper: %w", err)
	}

	clawSize := r3.Vector{X: 67.0455, Y: 53.027, Z: 106.4}
	claws, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: clawSize.Z / 2}), clawSize, "claws")
	geometries := []spatialmath.Geometry{claws}

	g := &so101Gripper{
		name:              conf.ResourceName(),
		logger:            logger,
		controller:        controller,
		geometries:        geometries,
		servoID:           cfg.ServoID,
		speed:             speedPercentPerSec,
		acceleration:      accelerationPercentPerSec,
		openPosition:      95.0,
		closedPosition:    0.0,
		gripLoadThreshold: 1200,
	}

	logger.Debugf("SO-101 gripper initialized with servo ID %d, speed: %.1f %%/s, acceleration: %.1f %%/s², open=%.1f%%, closed=%.1f%%",
		cfg.ServoID, speedPercentPerSec, accelerationPercentPerSec, g.openPosition, g.closedPosition)

	return g, nil
}

func (g *so101Gripper) Name() resource.Name {
	return g.name
}

func (g *so101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Opening gripper")

	// Build move options from defaults and extra parameters
	opts := g.buildMoveOptions(extra)
	g.logger.Debugf("Gripper opts: %+v", opts)
	// Pass speed and acceleration to controller (in percent/sec for gripper)
	speed := int(opts.speedPercentPerSec)
	acc := int(opts.accelerationPercentPerSec)
	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.openPositionRadians()}, speed, acc); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	time.Sleep(500 * time.Millisecond)

	g.logger.Debug("Gripper opened")
	return nil
}

func (g *so101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Attempting to grab with gripper using load monitoring")

	// Build move options from defaults and extra parameters
	opts := g.buildMoveOptions(extra)
	g.logger.Debugf("Gripper opts: %+v", opts)
	// Start closing the gripper (non-blocking)
	speed := int(opts.speedPercentPerSec)
	acc := int(opts.accelerationPercentPerSec)
	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.closedPositionRadians()}, speed, acc); err != nil {
		return false, fmt.Errorf("failed to start gripper close: %w", err)
	}

	// Poll load and position to detect when gripper grabs object or reaches full close
	pollInterval := 10 * time.Millisecond

	// Calculate timeout based on speed: distance / speed * safety_factor
	// Distance to travel in percentage
	distance := g.openPosition - g.closedPosition
	// Time = distance / speed (in seconds), with 2x safety margin
	timeoutSeconds := (distance / float64(opts.speedPercentPerSec)) * 2.0
	// Clamp to reasonable bounds: minimum 1 second, maximum 10 seconds
	if timeoutSeconds < 1.0 {
		timeoutSeconds = 1.0
	}
	if timeoutSeconds > 10.0 {
		timeoutSeconds = 10.0
	}
	timeout := time.Duration(timeoutSeconds * float64(time.Second))
	start := time.Now()

	g.logger.Debugf("Grip timeout calculated: %.2f seconds (distance: %.1f%%, speed: %.1f%%/s)",
		timeoutSeconds, distance, opts.speedPercentPerSec)

	// Calculate position tolerance (2% of range)
	positionTolerance := (g.openPosition - g.closedPosition) * 0.02

	for {
		// Check timeout
		if time.Since(start) > timeout {
			g.logger.Warnf("Grip operation timed out after %.2f seconds", timeoutSeconds)
			return false, fmt.Errorf("grip operation timed out after %.2f seconds", timeoutSeconds)
		}

		// Read current load
		load, err := g.controller.GetServoLoad(ctx, g.servoID)
		if err != nil {
			g.logger.Warnf("Failed to read servo load: %v", err)
			time.Sleep(pollInterval)
			continue
		}

		// Check if load exceeds threshold (use absolute value)
		absLoad := load
		if absLoad < 0 {
			absLoad = -absLoad
		}

		if absLoad > g.gripLoadThreshold {
			g.logger.Debugf("Load threshold exceeded (load: %d, threshold: %d) - stopping gripper", absLoad, g.gripLoadThreshold)

			// Stop the gripper
			if err := g.controller.Stop(ctx); err != nil {
				g.logger.Warnf("Failed to stop gripper: %v", err)
			}

			// Read final position to determine if we grabbed something
			currentPositions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
			if err != nil {
				g.logger.Warnf("Failed to read final gripper position: %v", err)
				return true, nil // Assume grabbed since load was high
			}

			if len(currentPositions) == 0 {
				g.logger.Warn("No position data received from gripper")
				return true, nil // Assume grabbed since load was high
			}

			currentPercent := g.radiansToPercent(currentPositions[0])
			positionDiff := currentPercent - g.closedPosition

			// If stopped more than 5% before fully closed, assume we grabbed something
			grabbed := positionDiff > 5.0

			if grabbed {
				g.logger.Debugf("Gripper grabbed object at %.1f%% (%.1f%% from fully closed)", currentPercent, positionDiff)
			} else {
				g.logger.Debugf("Gripper closed to %.1f%% but may not have grabbed anything", currentPercent)
			}

			return grabbed, nil
		}

		// Read current position to check if we've reached the target
		currentPositions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
		if err != nil {
			g.logger.Warnf("Failed to read gripper position: %v", err)
			time.Sleep(pollInterval)
			continue
		}

		if len(currentPositions) > 0 {
			currentPercent := g.radiansToPercent(currentPositions[0])
			positionDiff := currentPercent - g.closedPosition

			// Check if we've reached the closed position (within tolerance)
			if positionDiff <= positionTolerance {
				g.logger.Debugf("Gripper reached fully closed position (%.1f%%) without high load - nothing grabbed", currentPercent)
				return false, nil
			}
		}

		// Wait before next poll
		time.Sleep(pollInterval)
	}
}

func (g *so101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	return g.controller.Stop(ctx)
}

func (g *so101Gripper) IsMoving(ctx context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *so101Gripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	return g.geometries, nil
}

func (g *so101Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_position":
		positions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
		if err != nil {
			return nil, err
		}
		if len(positions) == 0 {
			return nil, fmt.Errorf("no position data available")
		}

		percentPos := g.radiansToPercent(positions[0])

		return map[string]interface{}{
			"position_radians":    positions[0],
			"position_percentage": percentPos,
			"open_position":       g.openPosition,
			"closed_position":     g.closedPosition,
		}, nil

	case "set_position":
		var targetPercent float64

		if percentPos, ok := cmd["percentage"].(float64); ok {
			targetPercent = percentPos
		} else if servoPos, ok := cmd["servo_position"].(float64); ok {
			cal := g.controller.getCalibrationForServo(g.servoID)
			if cal != nil {
				normalizedPos := (servoPos - float64(cal.RangeMin)) / float64(cal.RangeMax-cal.RangeMin)
				targetPercent = normalizedPos * 100.0
			} else {
				targetPercent = (servoPos / 4095.0) * 100.0
			}
		} else {
			return nil, fmt.Errorf("set_position command requires 'percentage' or 'servo_position' parameter")
		}

		if targetPercent < 0 {
			targetPercent = 0
		}
		if targetPercent > 100 {
			targetPercent = 100
		}

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		// Build move options from cmd map
		opts := g.buildMoveOptions(cmd)
		speed := int(opts.speedPercentPerSec)
		acc := int(opts.accelerationPercentPerSec)

		targetRadians := g.percentToRadians(targetPercent)
		err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{targetRadians}, speed, acc)
		return map[string]interface{}{"success": err == nil}, err

	case "controller_status":
		refCount, hasController, configSummary := GetControllerStatus()
		return map[string]interface{}{
			"ref_count":      refCount,
			"has_controller": hasController,
			"config":         configSummary,
			"servo_id":       g.servoID,
		}, nil

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
			if speed > 0 && speed <= 100 {
				g.speed = float32(speed)
			}
		}
		if acc, ok := cmd["acceleration"].(float64); ok {
			if acc > 0 && acc <= 200 {
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

	case "get_load":
		load, err := g.controller.GetServoLoad(ctx, g.servoID)
		if err != nil {
			return nil, fmt.Errorf("failed to read servo load: %w", err)
		}
		return map[string]interface{}{
			"load":      load,
			"threshold": g.gripLoadThreshold,
		}, nil

	default:
		// Check for speed and acceleration setting (following xarm pattern)
		result := make(map[string]interface{})
		changed := false

		if speedVal, ok := cmd["set_speed"]; ok {
			if speed, ok := speedVal.(float64); ok {
				if speed < 3 || speed > 100 {
					return nil, fmt.Errorf("speed must be between 3 and 100 percent/second, got %.1f", speed)
				}
				g.mu.Lock()
				g.speed = float32(speed)
				g.mu.Unlock()
				result["speed_set"] = speed
				changed = true
			} else {
				return nil, fmt.Errorf("set_speed requires a number value")
			}
		}

		if accVal, ok := cmd["set_acceleration"]; ok {
			if acc, ok := accVal.(float64); ok {
				if acc < 10 || acc > 200 {
					return nil, fmt.Errorf("acceleration must be between 10 and 200 percent/second^2, got %.1f", acc)
				}
				g.mu.Lock()
				g.acceleration = float32(acc)
				g.mu.Unlock()
				result["acceleration_set"] = acc
				changed = true
			} else {
				return nil, fmt.Errorf("set_acceleration requires a number value")
			}
		}

		if getParams, ok := cmd["get_motion_params"]; ok && getParams.(bool) {
			g.mu.Lock()
			speedPercentPerSec := float32(g.speed)
			accPercentPerSec := float32(g.acceleration)
			g.mu.Unlock()

			result["current_speed_percent_per_sec"] = speedPercentPerSec
			result["current_acceleration_percent_per_sec_per_sec"] = accPercentPerSec
			changed = true
		}

		if changed {
			return result, nil
		}

		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (g *so101Gripper) Close(ctx context.Context) error {
	ReleaseSharedController()
	return nil
}

func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *so101Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *so101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.ErrUnsupported
}

func (g *so101Gripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, errors.ErrUnsupported
}

func (g *so101Gripper) openPositionRadians() float64 {
	return g.percentToRadians(g.openPosition)
}

func (g *so101Gripper) closedPositionRadians() float64 {
	return g.percentToRadians(g.closedPosition)
}

func (g *so101Gripper) percentToRadians(percent float64) float64 {
	// Since the gripper calibration uses NormModeRange100 (0-100%),
	// we can directly use the percentage value and let feetech-servo handle conversion
	// But we need to convert to the expected format for the controller

	// For now, we'll do a simple mapping based on the calibration range
	cal := g.controller.getCalibrationForServo(g.servoID)
	if cal == nil {
		// Fallback to default behavior
		return (percent - 50.0) / 50.0 * math.Pi
	}

	// Convert percentage to normalized position within the calibrated range
	normalizedPos := percent / 100.0

	// Convert to radians (assuming ±π range)
	radians := (normalizedPos*2.0 - 1.0) * math.Pi // Convert 0-1 to -π to +π

	// Apply drive mode if needed
	if cal.DriveMode != 0 {
		radians = -radians
	}

	return radians
}

func (g *so101Gripper) radiansToPercent(radians float64) float64 {
	cal := g.controller.getCalibrationForServo(g.servoID)
	if cal == nil {
		// Fallback to default behavior
		return (radians/math.Pi)*50.0 + 50.0
	}

	// Apply drive mode if needed
	adjustedRadians := radians
	if cal.DriveMode != 0 {
		adjustedRadians = -radians
	}

	// Convert radians to normalized position (-1 to 1)
	normalizedPos := adjustedRadians / math.Pi

	// Convert to percentage (0-100)
	percent := (normalizedPos + 1.0) / 2.0 * 100.0

	// Clamp to valid range
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}

	return percent
}

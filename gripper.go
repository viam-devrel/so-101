package so_arm

import (
	"context"
	"errors"
	"fmt"
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
		logger.Infof("Updating gripper calibration servo ID from %d to %d (from config)",
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
		name:           conf.ResourceName(),
		logger:         logger,
		controller:     controller,
		geometries:     geometries,
		servoID:        cfg.ServoID,
		speed:          30,
		acceleration:   50,
		openPosition:   85.0,
		closedPosition: 10.0,
	}

	logger.Infof("SO-101 gripper initialized with servo ID %d, open=%.1f%%, closed=%.1f%%",
		cfg.ServoID, g.openPosition, g.closedPosition)

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

	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.openPositionRadians()}, 0, 0); err != nil {
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

	g.logger.Debug("Attempting to grab with gripper")

	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.closedPositionRadians()}, 0, 0); err != nil {
		return false, fmt.Errorf("failed to close gripper: %w", err)
	}

	time.Sleep(500 * time.Millisecond)

	currentPositions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		return true, nil
	}

	if len(currentPositions) == 0 {
		g.logger.Warn("No position data received from gripper")
		return false, nil
	}

	currentPercent := g.radiansToPercent(currentPositions[0])

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

		targetRadians := g.percentToRadians(targetPercent)
		err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{targetRadians}, 0, 0)
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

		g.logger.Infof("Gripper positions calibrated: open=%.1f%%, closed=%.1f%%", g.openPosition, g.closedPosition)

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
		return (percent - 50.0) / 50.0 * 3.14159265359
	}

	// Convert percentage to normalized position within the calibrated range
	normalizedPos := percent / 100.0

	// Convert to radians (assuming ±π range)
	radians := (normalizedPos*2.0 - 1.0) * 3.14159265359 // Convert 0-1 to -π to +π

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
		return (radians/3.14159265359)*50.0 + 50.0
	}

	// Apply drive mode if needed
	adjustedRadians := radians
	if cal.DriveMode != 0 {
		adjustedRadians = -radians
	}

	// Convert radians to normalized position (-1 to 1)
	normalizedPos := adjustedRadians / 3.14159265359

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

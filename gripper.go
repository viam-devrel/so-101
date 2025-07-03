package so_arm

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	SO101GripperModel = resource.NewModel("devrel", "so101", "gripper")
)

// SO101GripperConfig represents the configuration for the SO-101 gripper
type SO101GripperConfig struct {
	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Gripper configuration
	ServoID int `json:"servo_id,omitempty"` // Default to 6

	// Common configuration
	Timeout time.Duration `json:"timeout,omitempty"`

	// Gripper calibration file (shared with arm)
	CalibrationFile string `json:"calibration_file,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6 // Default to servo ID 6
	}

	if cfg.ServoID < 1 || cfg.ServoID > 6 {
		return nil, nil, fmt.Errorf("servo_id must be between 1 and 6, got %d", cfg.ServoID)
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Default baudrate
	}

	return nil, nil, nil
}

// so101Gripper represents the SO-101 gripper component
type so101Gripper struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	controller *SafeSoArmController
	model      referenceframe.Model
	servoID    int

	// State management
	mu       sync.Mutex
	isMoving atomic.Bool

	// Gripper positions (in servo units, calculated from calibration)
	openPosition   int
	closedPosition int

	// Motion parameters (optimized for gripper)
	speed        int // Fixed speed for gripper operations
	acceleration int // Fixed acceleration for gripper operations
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
		cfg.ServoID = 6 // Default to servo ID 6
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Default baudrate
	}

	// Create controller configuration for gripper (shared with arm)
	controllerConfig := &SoArm101Config{
		Port:            cfg.Port,
		Baudrate:        cfg.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6}, // Controller handles all 6, gripper uses servo 6
		Timeout:         cfg.Timeout,
		CalibrationFile: cfg.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(cfg.CalibrationFile)

	// Load full calibration (includes all joints for shared controller)
	fullCalibration := controllerConfig.LoadCalibration(logger)

	// Validate that gripper calibration exists and matches config
	if fullCalibration.Gripper.ID != cfg.ServoID {
		logger.Infof("Updating gripper calibration servo ID from %d to %d (from config)",
			fullCalibration.Gripper.ID, cfg.ServoID)
		fullCalibration.Gripper.ID = cfg.ServoID
	}

	controller, err := GetSharedControllerWithCalibration(controllerConfig, fullCalibration)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared controller for gripper: %w", err)
	}

	g := &so101Gripper{
		name:         conf.ResourceName(),
		logger:       logger,
		controller:   controller,
		model:        referenceframe.NewSimpleModel("so101_gripper"),
		servoID:      cfg.ServoID,
		speed:        200, // Fixed speed optimized for gripper (slower for precision)
		acceleration: 50,  // Fixed acceleration optimized for gripper
	}

	// Calculate open and closed positions from calibration
	gripperCalibration := fullCalibration.Gripper

	// Open position: near the maximum of the range
	g.openPosition = gripperCalibration.RangeMax - 200 // Leave some margin

	// Closed position: near the minimum of the range
	g.closedPosition = gripperCalibration.RangeMin + 200 // Leave some margin

	logger.Infof("SO-101 gripper initialized with servo ID %d, open=%d, closed=%d",
		cfg.ServoID, g.openPosition, g.closedPosition)

	return g, nil
}

func (g *so101Gripper) Name() resource.Name {
	return g.name
}

// Open opens the gripper to the fully open position
func (g *so101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Opening gripper")

	// Convert servo position to radians using calibration
	openRadians := g.controller.SoArmController.servoPositionToRadiansCalibrated(g.openPosition, g.servoID)

	// Move to open position using specific servo
	if err := g.controller.MoveServosToPositions([]int{g.servoID}, []float64{openRadians}, g.speed, g.acceleration); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(500 * time.Millisecond)

	g.logger.Debug("Gripper opened")
	return nil
}

// Grab closes the gripper and returns true if an object was successfully gripped
func (g *so101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Attempting to grab with gripper")

	// Convert servo position to radians using calibration
	closedRadians := g.controller.SoArmController.servoPositionToRadiansCalibrated(g.closedPosition, g.servoID)

	// Move to closed position using specific servo
	if err := g.controller.MoveServosToPositions([]int{g.servoID}, []float64{closedRadians}, g.speed, g.acceleration); err != nil {
		return false, fmt.Errorf("failed to close gripper: %w", err)
	}

	// Wait for movement to complete
	time.Sleep(500 * time.Millisecond)

	// Check if something was gripped by reading the actual position
	currentPositions, err := g.controller.GetJointPositionsForServos([]int{g.servoID})
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		// Assume grab was successful if we can't read position
		return true, nil
	}

	if len(currentPositions) == 0 {
		g.logger.Warn("No position data received from gripper")
		return false, nil
	}

	// Convert current position back to servo units for comparison
	currentServoPos := g.controller.SoArmController.radiansToServoPositionCalibrated(currentPositions[0], g.servoID)

	// If the gripper couldn't close fully, something is blocking it (i.e., gripped)
	positionDifference := currentServoPos - g.closedPosition
	threshold := 100 // Servo units - if gripper is more than this distance from closed position

	grabbed := positionDifference > threshold

	if grabbed {
		g.logger.Debugf("Gripper successfully grabbed an object (position difference: %d)", positionDifference)
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

// Stop stops any current gripper movement
func (g *so101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	return g.controller.Stop()
}

// IsMoving returns whether the gripper is currently moving
func (g *so101Gripper) IsMoving(ctx context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

// ModelFrame returns the reference frame model for the gripper
func (g *so101Gripper) ModelFrame() referenceframe.Model {
	return g.model
}

// Geometries returns the gripper's geometric representation
func (g *so101Gripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, errors.New("geometries not implemented for SO-101 gripper")
}

// DoCommand handles custom gripper commands
func (g *so101Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_position":
		positions, err := g.controller.GetJointPositionsForServos([]int{g.servoID})
		if err != nil {
			return nil, err
		}
		if len(positions) == 0 {
			return nil, fmt.Errorf("no position data available")
		}

		// Convert to servo units for easier interpretation
		servoPos := g.controller.SoArmController.radiansToServoPositionCalibrated(positions[0], g.servoID)

		return map[string]interface{}{
			"position_radians":     positions[0],
			"position_servo_units": servoPos,
			"open_position":        g.openPosition,
			"closed_position":      g.closedPosition,
		}, nil

	case "set_position":
		servoPos, ok := cmd["servo_position"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_position command requires 'servo_position' parameter")
		}

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		// Convert servo position to radians
		radians := g.controller.SoArmController.servoPositionToRadiansCalibrated(int(servoPos), g.servoID)

		// Move to position using specific servo
		err := g.controller.MoveServosToPositions([]int{g.servoID}, []float64{radians}, g.speed, g.acceleration)
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
		// Allow manual calibration of open/closed positions
		if openPos, ok := cmd["open_position"].(float64); ok {
			g.openPosition = int(openPos)
		}
		if closedPos, ok := cmd["closed_position"].(float64); ok {
			g.closedPosition = int(closedPos)
		}

		g.logger.Infof("Gripper positions calibrated: open=%d, closed=%d", g.openPosition, g.closedPosition)

		return map[string]interface{}{
			"success":         true,
			"open_position":   g.openPosition,
			"closed_position": g.closedPosition,
		}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// Close releases the shared controller
func (g *so101Gripper) Close(ctx context.Context) error {
	ReleaseSharedController()
	return nil
}

// Additional interface methods for gripper

func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	positions, err := g.controller.GetJointPositionsForServos([]int{g.servoID})
	if err != nil {
		return nil, err
	}

	if len(positions) == 0 {
		return []referenceframe.Input{}, nil
	}

	return []referenceframe.Input{
		{Value: positions[0]},
	}, nil
}

func (g *so101Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	if len(inputs) == 0 {
		return nil
	}

	for _, inputSet := range inputs {
		if len(inputSet) != 1 {
			return fmt.Errorf("expected 1 input for gripper, got %d", len(inputSet))
		}

		g.mu.Lock()
		g.isMoving.Store(true)

		err := g.controller.MoveServosToPositions([]int{g.servoID}, []float64{inputSet[0].Value}, g.speed, g.acceleration)

		g.isMoving.Store(false)
		g.mu.Unlock()

		if err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}
	}

	return nil
}

func (g *so101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

func (g *so101Gripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, errors.ErrUnsupported
}

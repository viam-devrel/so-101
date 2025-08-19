package so_arm

import (
	"context"
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

var (
	SO101Model = resource.NewModel("devrel", "so101", "arm")
)

//go:embed so101.json
var so101ModelJson []byte

func init() {
	resource.RegisterComponent(arm.API, SO101Model,
		resource.Registration[arm.Arm, *SO101ArmConfig]{
			Constructor: NewSO101,
		},
	)
}

type SO101ArmConfig struct {
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Arm uses servos 1-5
	ServoIDs []int `json:"servo_ids,omitempty"`

	Timeout time.Duration `json:"timeout,omitempty"`

	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`

	CalibrationFile string `json:"calibration_file,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101ArmConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	// Default to arm servos (1-5) if not specified
	if len(cfg.ServoIDs) == 0 {
		cfg.ServoIDs = []int{1, 2, 3, 4, 5}
	}

	// Validate that only arm servos are specified
	for _, id := range cfg.ServoIDs {
		if id < 1 || id > 5 {
			return nil, nil, fmt.Errorf("arm servo IDs must be 1-5, got %d", id)
		}
	}

	return nil, nil, nil
}

type so101 struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *SO101ArmConfig
	opMgr      *operation.SingleOperationManager
	controller *SafeSoArmController

	mu       sync.RWMutex
	moveLock sync.Mutex
	isMoving atomic.Bool
	model    referenceframe.Model

	// Servo IDs controlled by this arm (1-5)
	armServoIDs []int

	defaultSpeed float32
	defaultAcc   float32

	cancelCtx  context.Context
	cancelFunc func()
}

func makeSO101ModelFrame() (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     so101ModelJson,
			Extension: "json",
		},
	}
	err := json.Unmarshal(so101ModelJson, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	return m.ParseConfig("soarm_101")
}

// calculateJointLimits dynamically calculates joint limits from calibration data
func (s *so101) calculateJointLimits() [][2]float64 {
	limits := make([][2]float64, len(s.armServoIDs))

	calibration := s.controller.GetCalibration()

	// Map servo IDs to calibration data
	jointCals := []*feetech.MotorCalibration{
		calibration.ShoulderPan,
		calibration.ShoulderLift,
		calibration.ElbowFlex,
		calibration.WristFlex,
		calibration.WristRoll,
	}

	for i, cal := range jointCals {
		if cal == nil {
			// Use default limits if calibration is missing
			limits[i] = [2]float64{-math.Pi, math.Pi}
			continue
		}

		// Convert calibration range to radians using the same logic as before
		center := float64(cal.RangeMin+cal.RangeMax) / 2
		halfRange := float64(cal.RangeMax-cal.RangeMin) / 2

		// Calculate min limit (RangeMin -> radians)
		minNormalized := (float64(cal.RangeMin) - center) / halfRange
		minRadians := minNormalized * math.Pi

		// Calculate max limit (RangeMax -> radians)
		maxNormalized := (float64(cal.RangeMax) - center) / halfRange
		maxRadians := maxNormalized * math.Pi

		limits[i] = [2]float64{minRadians, maxRadians}
	}

	return limits
}

func NewSO101(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*SO101ArmConfig](rawConf)
	if err != nil {
		return nil, err
	}

	// Validate and set default motion parameters
	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = 50 // Default speed in degrees per second
	}
	if speedDegsPerSec < 3 || speedDegsPerSec > 180 {
		return nil, fmt.Errorf("speed_degs_per_sec must be between 3 and 180 degrees/second, got %.1f", speedDegsPerSec)
	}

	accelerationDegsPerSec := conf.AccelerationDegsPerSec
	if accelerationDegsPerSec == 0 {
		accelerationDegsPerSec = 100 // Default acceleration in degrees per second^2
	}
	if accelerationDegsPerSec < 10 || accelerationDegsPerSec > 500 {
		return nil, fmt.Errorf("acceleration_degs_per_sec_per_sec must be between 10 and 500 degrees/second^2, got %.1f", accelerationDegsPerSec)
	}

	if conf.Baudrate == 0 {
		conf.Baudrate = 1000000
	}

	if len(conf.ServoIDs) == 0 {
		conf.ServoIDs = []int{1, 2, 3, 4, 5}
	}

	// Create controller configuration
	controllerConfig := &SoArm101Config{
		Port:            conf.Port,
		Baudrate:        conf.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6}, // Controller handles all 6, but arm only uses 1-5
		Timeout:         conf.Timeout,
		CalibrationFile: conf.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(conf.CalibrationFile)

	// Load full calibration (includes gripper for shared controller)
	calibration := controllerConfig.LoadCalibration(logger)
	if calibration.ShoulderPan != nil {
		logger.Infof("Using calibration for SO-101 with shoulder_pan homing_offset: %d", calibration.ShoulderPan.HomingOffset)
	} else {
		logger.Info("Using default calibration for SO-101")
	}

	controller, err := GetSharedControllerWithCalibration(controllerConfig, calibration)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared SO-ARM controller: %w", err)
	}

	model, err := makeSO101ModelFrame()
	if err != nil {
		ReleaseSharedController() // Clean up on error
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	arm := &so101{
		name:         rawConf.ResourceName(),
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
		logger:       logger,
		controller:   controller,
		model:        model,
		armServoIDs:  conf.ServoIDs, // Store which servos this arm controls
		defaultSpeed: speedDegsPerSec,
		defaultAcc:   accelerationDegsPerSec,
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}

	logger.Infof("SO-101 configured with speed: %.1f deg/s, acceleration: %.1f deg/s²",
		speedDegsPerSec, accelerationDegsPerSec)
	logger.Infof("Arm controlling servo IDs: %v", arm.armServoIDs)

	// Initialize and verify servo connections
	if err := arm.initializeServos(); err != nil {
		ReleaseSharedController() // Clean up on error
		return nil, fmt.Errorf("failed to initialize servos: %w", err)
	}

	return arm, nil
}

func (s *so101) Name() resource.Name {
	return s.name
}

func (s *so101) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (arm.Arm, error) {
	panic("not implemented")
}

func (s *so101) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := referenceframe.ComputeOOBPosition(s.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

func (s *so101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if err := motion.MoveArm(ctx, s.logger, s, pose); err != nil {
		return err
	}
	return nil
}

func (s *so101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	if len(positions) != len(s.armServoIDs) {
		return fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(positions))
	}

	values := make([]float64, len(positions))
	for i, input := range positions {
		values[i] = input.Value
	}

	// Calculate joint limits dynamically from calibration
	jointLimits := s.calculateJointLimits()

	// Validate input ranges and clamp positions for the arm joints
	clampedPositions := make([]float64, len(values))
	for i, pos := range values {
		min, max := jointLimits[i][0], jointLimits[i][1]

		// Validate and clamp the position
		if pos < min || pos > max {
			s.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				s.armServoIDs[i], pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clampedPositions[i] = math.Max(min, math.Min(max, pos))
	}

	if err := s.controller.MoveServosToPositions(s.armServoIDs, clampedPositions, 0, 0); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	currentPositions, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
	if err != nil {
		s.logger.Warnf("Failed to get current positions for timing calculation: %v", err)
		currentPositions = make([]float64, len(s.armServoIDs)) // Use zeros as fallback
	}

	maxMovement := 0.0
	for i, target := range clampedPositions {
		if i < len(currentPositions) {
			movement := math.Abs(target - currentPositions[i])
			if movement > maxMovement {
				maxMovement = movement
			}
		}
	}

	speedRadPerSec := float64(s.defaultSpeed) * math.Pi / 180.0
	moveTimeSeconds := maxMovement / speedRadPerSec
	if moveTimeSeconds < 0.1 {
		moveTimeSeconds = 0.1 // Minimum move time
	}
	if moveTimeSeconds > 10.0 {
		moveTimeSeconds = 10.0 // Maximum move time for safety
	}

	time.Sleep(time.Duration(moveTimeSeconds * float64(time.Second)))

	return nil
}

func (s *so101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	for _, jointPositions := range positions {
		if err := s.MoveToJointPositions(ctx, jointPositions, extra); err != nil {
			return err
		}

		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

func (s *so101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	radians, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
	if err != nil {
		s.logger.Warnf("Failed to read joint positions: %v", err)
		return nil, fmt.Errorf("failed to read joint positions: %w. Try running 'diagnose' command for more details", err)
	}

	if len(radians) != len(s.armServoIDs) {
		return nil, fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(radians))
	}

	positions := make([]referenceframe.Input, len(radians))
	for i, radian := range radians {
		positions[i] = referenceframe.Input{Value: radian}
	}

	return positions, nil
}

func (s *so101) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.isMoving.Store(false)
	return s.controller.Stop()
}

func (s *so101) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

func (s *so101) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *so101) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *so101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// Handle custom commands specific to SO-101
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := s.controller.SetTorqueEnable(enable)
		return map[string]interface{}{"success": err == nil}, err

	case "ping":
		err := s.controller.Ping()
		return map[string]interface{}{"success": err == nil}, err

	case "controller_status":
		refCount, hasController, configSummary := GetControllerStatus()
		return map[string]interface{}{
			"ref_count":      refCount,
			"has_controller": hasController,
			"config":         configSummary,
			"arm_servo_ids":  s.armServoIDs,
		}, nil

	case "diagnose":
		err := s.diagnoseConnection()
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
		}, nil

	case "verify_config":
		err := s.verifyServoConfig()
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
		}, nil

	case "reinitialize":
		retries := 3 // default
		if r, ok := cmd["retries"].(float64); ok {
			retries = int(r)
		}
		err := s.initializeServosWithRetry(retries)
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
			"retries": retries,
		}, nil

	case "test_servo_communication":
		positions, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
		result := map[string]interface{}{
			"success":       err == nil,
			"arm_servo_ids": s.armServoIDs,
		}
		if err != nil {
			result["error"] = fmt.Sprintf("%v", err)
		} else {
			result["positions"] = positions
		}
		return result, nil

	case "reload_calibration":
		if s.cfg.CalibrationFile == "" {
			return map[string]interface{}{
				"success": false,
				"error":   "No calibration file configured",
			}, nil
		}

		// Load the new calibration
		newCalibration, err := LoadFullCalibrationFromFile(s.cfg.CalibrationFile, s.logger)
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"error":   fmt.Sprintf("Failed to load calibration: %v", err),
			}, nil
		}

		// Update the controller with the new calibration
		if err := s.controller.SetCalibration(newCalibration); err != nil {
			return map[string]interface{}{
				"success": false,
				"error":   fmt.Sprintf("Failed to update calibration: %v", err),
			}, nil
		}

		s.logger.Infof("Successfully reloaded calibration from %s", s.cfg.CalibrationFile)
		return map[string]interface{}{
			"success":          true,
			"calibration_file": s.cfg.CalibrationFile,
			"message":          "Calibration reloaded successfully",
		}, nil

	case "get_calibration":
		calibration := s.controller.GetCalibration()
		return map[string]interface{}{
			"success":     true,
			"calibration": calibration,
		}, nil

	default:
		// Check for speed and acceleration setting
		result := make(map[string]interface{})
		changed := false

		if speedVal, ok := cmd["set_speed"]; ok {
			if speed, ok := speedVal.(float64); ok {
				if speed < 3 || speed > 180 {
					return nil, fmt.Errorf("speed must be between 3 and 180 degrees/second, got %.1f", speed)
				}
				s.mu.Lock()
				s.defaultSpeed = float32(speed)
				s.mu.Unlock()
				result["speed_set"] = speed
				changed = true
			} else {
				return nil, fmt.Errorf("set_speed requires a number value")
			}
		}

		if accVal, ok := cmd["set_acceleration"]; ok {
			if acc, ok := accVal.(float64); ok {
				if acc < 10 || acc > 500 {
					return nil, fmt.Errorf("acceleration must be between 10 and 500 degrees/second^2, got %.1f", acc)
				}
				s.mu.Lock()
				s.defaultAcc = float32(acc)
				s.mu.Unlock()
				result["acceleration_set"] = acc
				changed = true
			} else {
				return nil, fmt.Errorf("set_acceleration requires a number value")
			}
		}

		if getParams, ok := cmd["get_motion_params"]; ok && getParams.(bool) {
			s.mu.RLock()
			speedDegsPerSec := float64(s.defaultSpeed)
			accDegsPerSec := float64(s.defaultAcc)
			s.mu.RUnlock()

			result["current_speed_degs_per_sec"] = speedDegsPerSec
			result["current_acceleration_degs_per_sec_per_sec"] = accDegsPerSec
			changed = true
		}

		if changed {
			return result, nil
		}

		return nil, fmt.Errorf("unknown command: %v", cmd)
	}
}

func (s *so101) IsMoving(ctx context.Context) (bool, error) {
	return s.isMoving.Load(), nil
}

func (s *so101) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (s *so101) Close(context.Context) error {
	s.cancelFunc()
	ReleaseSharedController()
	return nil
}

// initializeServos pings each servo and enables torque to ensure proper communication
func (s *so101) initializeServos() error {
	return s.initializeServosWithRetry(3)
}

// initializeServosWithRetry attempts servo initialization with retries
func (s *so101) initializeServosWithRetry(maxRetries int) error {
	s.logger.Info("Initializing SO-101 arm servos...")

	var lastErr error
	for attempt := 1; attempt <= maxRetries; attempt++ {
		s.logger.Infof("Arm servo initialization attempt %d/%d", attempt, maxRetries)

		if err := s.doServoInitialization(); err != nil {
			lastErr = err
			s.logger.Warnf("Initialization attempt %d failed: %v", attempt, err)

			if attempt < maxRetries {
				waitTime := time.Duration(attempt) * 500 * time.Millisecond
				s.logger.Infof("Waiting %v before retry...", waitTime)
				time.Sleep(waitTime)
				continue
			}
		} else {
			s.logger.Infof("Arm servo initialization successful on attempt %d", attempt)
			return nil
		}
	}

	return fmt.Errorf("arm servo initialization failed after %d attempts, last error: %w", maxRetries, lastErr)
}

// doServoInitialization performs the actual initialization steps
func (s *so101) doServoInitialization() error {
	// Ping all servos to ensure they're responding
	s.logger.Debug("Pinging all servos...")
	if err := s.controller.Ping(); err != nil {
		return fmt.Errorf("servo ping failed: %w", err)
	}
	s.logger.Debug("All servos ping successful")

	// Enable torque for all servos (controller manages all 6)
	s.logger.Debug("Enabling torque for all servos...")
	if err := s.controller.SetTorqueEnable(true); err != nil {
		return fmt.Errorf("failed to enable torque: %w", err)
	}

	time.Sleep(100 * time.Millisecond)

	s.logger.Debug("Verifying position reading from arm servos...")
	positions, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to read initial joint positions: %w", err)
	}

	if len(positions) != len(s.armServoIDs) {
		return fmt.Errorf("expected %d joint positions, got %d", len(s.armServoIDs), len(positions))
	}

	s.logger.Infof("SO-101 arm servo initialization successful. Initial positions: %v", positions)
	return nil
}

// diagnoseConnection provides detailed diagnostics for troubleshooting
func (s *so101) diagnoseConnection() error {
	s.logger.Info("Starting SO-101 arm connection diagnosis...")

	// Test overall ping
	s.logger.Info("Testing overall servo communication...")
	if err := s.controller.Ping(); err != nil {
		s.logger.Errorf("Overall ping failed: %v", err)
		return err
	}
	s.logger.Info("Overall ping successful")

	positions, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
	if err != nil {
		s.logger.Errorf("Failed to read arm positions: %v", err)
		return err
	}

	for i, pos := range positions {
		s.logger.Infof("Arm servo %d position: %.3f rad", s.armServoIDs[i], pos)
	}

	return nil
}

// verifyServoConfig checks servo configuration
func (s *so101) verifyServoConfig() error {
	s.logger.Info("Verifying arm servo configuration...")

	positions, err := s.controller.GetJointPositionsForServos(s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to verify servo config: %w", err)
	}

	if len(positions) != len(s.armServoIDs) {
		return fmt.Errorf("config verification failed: expected %d servos, got %d", len(s.armServoIDs), len(positions))
	}

	s.logger.Infof("Arm servo configuration verified. Current positions: %v", positions)
	return nil
}

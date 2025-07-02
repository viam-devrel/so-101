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

//go:embed soarm_101.json
var so101ModelJson []byte

// SO-101 joint limits (5 joints for arm, excluding gripper)
var so101JointLimits = [][2]float64{
	{-math.Pi, math.Pi},               // Shoulder Pan: full rotation
	{-math.Pi * 0.75, math.Pi * 0.75}, // Shoulder Lift: ±135°
	{-math.Pi, math.Pi * 1.65},        // Elbow Flex: allow up to 297° (5.18 rad)
	{-math.Pi, math.Pi * 1.3},         // Wrist Flex: allow up to 234° (4.08 rad)
	{-math.Pi, math.Pi},               // Wrist Roll: full rotation
}

func init() {
	resource.RegisterComponent(arm.API, SO101Model,
		resource.Registration[arm.Arm, *SO101ArmConfig]{
			Constructor: newSO101,
		},
	)
}

// SO101ArmConfig represents the configuration for the SO-101 arm component
type SO101ArmConfig struct {
	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Common configuration
	Timeout time.Duration `json:"timeout,omitempty"`

	// Motion configuration
	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101ArmConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
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

	mu          sync.RWMutex
	moveLock    sync.Mutex
	isMoving    atomic.Bool
	model       referenceframe.Model
	jointLimits [][2]float64

	// Motion configuration
	defaultSpeed int
	defaultAcc   int

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

	return m.ParseConfig("so101_arm")
}

func newSO101(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
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

	// Convert degrees/sec to internal speed units (approximate conversion)
	defaultSpeed := int(speedDegsPerSec * 10)
	if defaultSpeed < 30 {
		defaultSpeed = 30
	}
	if defaultSpeed > 4096 {
		defaultSpeed = 4096
	}

	// Convert degrees/sec^2 to internal acceleration units
	defaultAcc := int(accelerationDegsPerSec * 0.5)
	if defaultAcc < 1 {
		defaultAcc = 1
	}
	if defaultAcc > 254 {
		defaultAcc = 254
	}

	if conf.Baudrate == 0 {
		conf.Baudrate = 1000000
	}

	// Create controller configuration
	controllerConfig := &SoArm101Config{
		Port:     conf.Port,
		Baudrate: conf.Baudrate,
		Timeout:  conf.Timeout,
		ServoIDs: []int{1, 2, 3, 4, 5}, // Only use first 5 servos for arm
		Logger:   logger,
	}

	controller, err := GetSharedController(controllerConfig)
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
		jointLimits:  so101JointLimits, // Only first 5 joints
		defaultSpeed: defaultSpeed,
		defaultAcc:   defaultAcc,
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}

	logger.Infof("SO-101 configured with speed: %.1f deg/s (internal: %d), acceleration: %.1f deg/s² (internal: %d)",
		speedDegsPerSec, defaultSpeed, accelerationDegsPerSec, defaultAcc)

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

	if len(positions) != 5 {
		return fmt.Errorf("expected 5 joint positions for SO-101 arm, got %d", len(positions))
	}

	values := make([]float64, len(positions))
	for i, input := range positions {
		values[i] = input.Value
	}

	// Validate input ranges and clamp positions for the 5 arm joints
	clampedPositions := make([]float64, len(values))
	for i, pos := range values {
		min, max := s.jointLimits[i][0], s.jointLimits[i][1]

		// Validate and clamp the position
		if pos < min || pos > max {
			s.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				i+1, pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clampedPositions[i] = math.Max(min, math.Min(max, pos))
	}

	// Use configured speed and acceleration
	speed := s.defaultSpeed
	acc := s.defaultAcc

	// Check for speed/acceleration overrides in extra parameters
	if extra != nil {
		if speedOverride, ok := extra["speed"]; ok {
			if speedVal, ok := speedOverride.(float64); ok {
				speed = int(speedVal * 10)
				if speed < 30 {
					speed = 30
				}
				if speed > 4096 {
					speed = 4096
				}
			}
		}
		if accOverride, ok := extra["acceleration"]; ok {
			if accVal, ok := accOverride.(float64); ok {
				acc = int(accVal * 0.5)
				if acc < 1 {
					acc = 1
				}
				if acc > 254 {
					acc = 254
				}
			}
		}
	}

	// Send command to controller with the 5 arm joints
	if err := s.controller.MoveToJointPositions(clampedPositions, speed, acc); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	// Calculate wait time based on movement distance and configured speed
	currentPositions, err := s.controller.GetJointPositions()
	if err != nil {
		s.logger.Warnf("Failed to get current positions for timing calculation: %v", err)
		currentPositions = make([]float64, 5) // Use zeros as fallback
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

	// Calculate move time based on configured speed (convert internal units back to rad/sec)
	speedRadPerSec := float64(speed) / 10.0 * math.Pi / 180.0 // Convert to rad/sec
	moveTimeSeconds := maxMovement / speedRadPerSec
	if moveTimeSeconds < 0.1 {
		moveTimeSeconds = 0.1 // Minimum move time
	}
	if moveTimeSeconds > 10.0 {
		moveTimeSeconds = 10.0 // Maximum move time for safety
	}

	// Wait for movement to complete
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

	// Get joint positions from controller (only 5 joints for arm)
	radians, err := s.controller.GetJointPositions()
	if err != nil {
		s.logger.Warnf("Failed to read joint positions: %v", err)

		return nil, fmt.Errorf("failed to read joint positions: %w. Try running 'diagnose' command for more details", err)
	}

	// Ensure we have exactly 5 joints for the arm
	if len(radians) != 5 {
		return nil, fmt.Errorf("expected 5 joint positions for SO-101 arm, got %d", len(radians))
	}

	// Convert to Viam input format
	positions := make([]referenceframe.Input, 5)
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
		servoID := 1 // default
		if id, ok := cmd["servo_id"].(float64); ok {
			servoID = int(id)
		}
		positions, err := s.controller.GetJointPositions()
		result := map[string]interface{}{
			"success":  err == nil,
			"servo_id": servoID,
		}
		if err != nil {
			result["error"] = fmt.Sprintf("%v", err)
		} else if servoID > 0 && servoID <= len(positions) {
			result["position"] = positions[servoID-1]
		} else {
			result["all_positions"] = positions
		}
		return result, nil

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
				s.defaultSpeed = int(speed * 10)
				if s.defaultSpeed < 30 {
					s.defaultSpeed = 30
				}
				if s.defaultSpeed > 4096 {
					s.defaultSpeed = 4096
				}
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
				s.defaultAcc = int(acc * 0.5)
				if s.defaultAcc < 1 {
					s.defaultAcc = 1
				}
				if s.defaultAcc > 254 {
					s.defaultAcc = 254
				}
				s.mu.Unlock()
				result["acceleration_set"] = acc
				changed = true
			} else {
				return nil, fmt.Errorf("set_acceleration requires a number value")
			}
		}

		if getParams, ok := cmd["get_motion_params"]; ok && getParams.(bool) {
			s.mu.RLock()
			speedDegsPerSec := float64(s.defaultSpeed) / 10.0
			accDegsPerSec := float64(s.defaultAcc) / 0.5
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
	s.logger.Info("Initializing SO-101 servos...")

	var lastErr error
	for attempt := 1; attempt <= maxRetries; attempt++ {
		s.logger.Infof("Servo initialization attempt %d/%d", attempt, maxRetries)

		if err := s.doServoInitialization(); err != nil {
			lastErr = err
			s.logger.Warnf("Initialization attempt %d failed: %v", attempt, err)

			if attempt < maxRetries {
				// Wait before retrying, with exponential backoff
				waitTime := time.Duration(attempt) * 500 * time.Millisecond
				s.logger.Infof("Waiting %v before retry...", waitTime)
				time.Sleep(waitTime)
				continue
			}
		} else {
			s.logger.Infof("Servo initialization successful on attempt %d", attempt)
			return nil
		}
	}

	return fmt.Errorf("servo initialization failed after %d attempts, last error: %w", maxRetries, lastErr)
}

// doServoInitialization performs the actual initialization steps
func (s *so101) doServoInitialization() error {
	// Ping each servo to ensure it's responding
	servoIDs := []int{1, 2, 3, 4, 5}
	for _, servoID := range servoIDs {
		s.logger.Debugf("Pinging servo %d...", servoID)
		if err := s.controller.Ping(); err != nil {
			return fmt.Errorf("servo %d ping failed: %w", servoID, err)
		}
		s.logger.Debugf("Servo %d ping successful", servoID)
	}

	// Enable torque for all servos
	s.logger.Debug("Enabling torque for all servos...")
	if err := s.controller.SetTorqueEnable(true); err != nil {
		return fmt.Errorf("failed to enable torque: %w", err)
	}

	// Brief delay to allow torque to stabilize
	time.Sleep(100 * time.Millisecond)

	// Verify we can read positions from all servos
	s.logger.Debug("Verifying position reading from all servos...")
	positions, err := s.controller.GetJointPositions()
	if err != nil {
		return fmt.Errorf("failed to read initial joint positions: %w", err)
	}

	if len(positions) != 5 {
		return fmt.Errorf("expected 5 joint positions, got %d", len(positions))
	}

	s.logger.Infof("SO-101 servo initialization successful. Initial positions: %v", positions)
	return nil
}

// diagnoseConnection provides detailed diagnostics for troubleshooting
func (s *so101) diagnoseConnection() error {
	s.logger.Info("Starting SO-101 connection diagnosis...")

	// Test each servo individually
	servoIDs := []int{1, 2, 3, 4, 5}
	for _, servoID := range servoIDs {
		s.logger.Infof("Testing servo %d...", servoID)

		// Try ping first
		if err := s.controller.Ping(); err != nil {
			s.logger.Errorf("Servo %d ping failed: %v", servoID, err)
			continue
		}
		s.logger.Infof("Servo %d ping successful", servoID)

		// Try reading current position
		positions, err := s.controller.GetJointPositions()
		if err != nil {
			s.logger.Errorf("Failed to read positions: %v", err)
			continue
		}

		if servoID-1 < len(positions) {
			s.logger.Infof("Servo %d position: %.3f rad", servoID, positions[servoID-1])
		}
	}

	return nil
}

// verifyServoConfig checks servo configuration
func (s *so101) verifyServoConfig() error {
	s.logger.Info("Verifying servo configuration...")

	// Try to read all positions to verify communication
	positions, err := s.controller.GetJointPositions()
	if err != nil {
		return fmt.Errorf("failed to verify servo config: %w", err)
	}

	if len(positions) != 5 {
		return fmt.Errorf("config verification failed: expected 5 servos, got %d", len(positions))
	}

	s.logger.Infof("Servo configuration verified. Current positions: %v", positions)
	return nil
}

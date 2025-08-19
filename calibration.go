// calibration_sensor.go - SO-101 Calibration Sensor Component
package so_arm

import (
	"context"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"time"

	"github.com/hipsterbrown/feetech-servo"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var (
	SO101CalibrationSensorModel = resource.NewModel("devrel", "so101", "calibration")
)

func init() {
	resource.RegisterComponent(sensor.API, SO101CalibrationSensorModel,
		resource.Registration[sensor.Sensor, *SO101CalibrationSensorConfig]{
			Constructor: NewSO101CalibrationSensor,
		},
	)
}

// CalibrationState represents the current state of the calibration workflow
type CalibrationState int

const (
	StateIdle CalibrationState = iota
	StateStarted
	StateHomingPosition
	StateRangeRecording
	StateCompleted
	StateError
)

func (s CalibrationState) String() string {
	switch s {
	case StateIdle:
		return "idle"
	case StateStarted:
		return "started"
	case StateHomingPosition:
		return "homing_position"
	case StateRangeRecording:
		return "range_recording"
	case StateCompleted:
		return "completed"
	case StateError:
		return "error"
	default:
		return "unknown"
	}
}

// JointCalibrationData holds calibration data for a single joint during the process
type JointCalibrationData struct {
	ID           int    `json:"id"`
	Name         string `json:"name"`
	HomingOffset int    `json:"homing_offset"`
	RangeMin     int    `json:"range_min"`
	RangeMax     int    `json:"range_max"`
	CurrentPos   int    `json:"current_position"`
	RecordedMin  int    `json:"recorded_min"`
	RecordedMax  int    `json:"recorded_max"`
	IsCompleted  bool   `json:"is_completed"`
}

// SO101CalibrationSensorConfig represents the configuration for the calibration sensor
type SO101CalibrationSensorConfig struct {
	// Servo configuration
	ServoIDs        []int  `json:"servo_ids,omitempty"`        // Default to all 6 servos
	CalibrationFile string `json:"calibration_file,omitempty"` // Where to save calibration

	// Controller configuration (shared with arm/gripper)
	Port     string        `json:"port,omitempty"`
	Baudrate int           `json:"baudrate,omitempty"`
	Timeout  time.Duration `json:"timeout,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101CalibrationSensorConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	// Default to all servos if not specified
	if len(cfg.ServoIDs) == 0 {
		cfg.ServoIDs = []int{1, 2, 3, 4, 5, 6} // All servos
	}

	// Validate servo IDs
	for _, id := range cfg.ServoIDs {
		if id < 1 || id > 6 {
			return nil, nil, fmt.Errorf("servo IDs must be 1-6, got %d", id)
		}
	}

	return nil, nil, nil
}

// so101CalibrationSensor implements the calibration workflow as a sensor component
type so101CalibrationSensor struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *SO101CalibrationSensorConfig
	controller *SafeSoArmController

	// Calibration state
	mu               sync.RWMutex
	state            CalibrationState
	errorMsg         string
	joints           map[int]*JointCalibrationData
	servoNames       map[int]string
	recordingStarted time.Time
	lastInstruction  string

	// Range recording state
	recordingActive bool
	recordingCtx    context.Context
	recordingCancel context.CancelFunc
	positionHistory []map[int]int // History of all servo positions during recording

	// Motor setup state (separate from calibration workflow)
	setupInProgress  bool
	currentSetupStep int
	setupStatus      string
}

// NewSO101CalibrationSensor creates a new SO-101 calibration sensor
func NewSO101CalibrationSensor(
	ctx context.Context,
	deps resource.Dependencies,
	rawConf resource.Config,
	logger logging.Logger,
) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*SO101CalibrationSensorConfig](rawConf)
	if err != nil {
		return nil, err
	}

	if conf.Baudrate == 0 {
		conf.Baudrate = 1000000
	}

	if conf.CalibrationFile == "" {
		conf.CalibrationFile = "so101_calibration.json"
	}

	// Handle relative paths using VIAM_MODULE_DATA
	if !filepath.IsAbs(conf.CalibrationFile) {
		moduleDataDir := os.Getenv("VIAM_MODULE_DATA")
		if moduleDataDir == "" {
			moduleDataDir = "/tmp" // Fallback if VIAM_MODULE_DATA not set
		}
		conf.CalibrationFile = filepath.Join(moduleDataDir, conf.CalibrationFile)
	}

	// Create controller configuration
	controllerConfig := &SoArm101Config{
		Port:            conf.Port,
		Baudrate:        conf.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6}, // Controller handles all 6
		Timeout:         conf.Timeout,
		CalibrationFile: conf.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(conf.CalibrationFile)

	// Load existing calibration for baseline
	calibration := controllerConfig.LoadCalibration(logger)

	controller, err := GetSharedControllerWithCalibration(controllerConfig, calibration)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared SO-ARM controller: %w", err)
	}

	// Define servo names
	servoNames := map[int]string{
		1: "shoulder_pan",
		2: "shoulder_lift",
		3: "elbow_flex",
		4: "wrist_flex",
		5: "wrist_roll",
		6: "gripper",
	}

	// Default to all servos if not specified
	if len(conf.ServoIDs) == 0 {
		conf.ServoIDs = []int{1, 2, 3, 4, 5, 6} // All servos
	}

	// Initialize joint calibration data
	joints := make(map[int]*JointCalibrationData)
	for _, servoID := range conf.ServoIDs {
		joints[servoID] = &JointCalibrationData{
			ID:          servoID,
			Name:        servoNames[servoID],
			RecordedMin: math.MaxInt32,
			RecordedMax: math.MinInt32,
		}
	}

	cs := &so101CalibrationSensor{
		name:            rawConf.ResourceName(),
		logger:          logger,
		cfg:             conf,
		controller:      controller,
		state:           StateIdle,
		joints:          joints,
		servoNames:      servoNames,
		lastInstruction: "Ready to start calibration. Use DoCommand with 'start' to begin.",
	}

	logger.Infof("SO-101 calibration sensor initialized for servos: %v", conf.ServoIDs)
	return cs, nil
}

// Name returns the sensor's name
func (cs *so101CalibrationSensor) Name() resource.Name {
	return cs.name
}

// Readings returns the current calibration status and instructions
func (cs *so101CalibrationSensor) Readings(ctx context.Context, extra map[string]any) (map[string]any, error) {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	readings := map[string]any{
		"calibration_state": cs.state.String(),
		"instruction":       cs.lastInstruction,
		"servo_count":       len(cs.cfg.ServoIDs),
	}

	if cs.state == StateError {
		readings["error"] = cs.errorMsg
	}

	// Add joint-specific information
	jointInfo := make(map[string]any)
	for _, joint := range cs.joints {
		jointInfo[joint.Name] = map[string]any{
			"id":               joint.ID,
			"current_position": joint.CurrentPos,
			"homing_offset":    joint.HomingOffset,
			"range_min":        joint.RangeMin,
			"range_max":        joint.RangeMax,
			"recorded_min":     joint.RecordedMin,
			"recorded_max":     joint.RecordedMax,
			"is_completed":     joint.IsCompleted,
		}
	}
	readings["joints"] = jointInfo

	// Add progress information
	if cs.state == StateRangeRecording && cs.recordingActive {
		elapsed := time.Since(cs.recordingStarted)
		readings["recording_time_seconds"] = elapsed.Seconds()
		readings["position_samples"] = len(cs.positionHistory)
	}

	// Add available commands based on state
	availableCommands := []any{}
	switch cs.state {
	case StateIdle:
		availableCommands = []any{"start"}
	case StateStarted:
		availableCommands = []any{"set_homing", "abort"}
	case StateHomingPosition:
		availableCommands = []any{"start_range_recording", "abort"}
	case StateRangeRecording:
		availableCommands = []any{"stop_range_recording", "abort"}
	case StateCompleted:
		availableCommands = []any{"save_calibration", "start"} // Allow restart
	case StateError:
		availableCommands = []any{"reset", "start"}
	}
	readings["available_commands"] = availableCommands

	// Add motor setup status
	readings["motor_setup"] = map[string]any{
		"in_progress": cs.setupInProgress,
		"step":        cs.currentSetupStep,
		"status":      cs.setupStatus,
	}

	return readings, nil
}

// DoCommand handles calibration workflow commands
func (cs *so101CalibrationSensor) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	command, ok := cmd["command"].(string)
	if !ok {
		return nil, fmt.Errorf("command must be a string")
	}

	cs.mu.Lock()
	defer cs.mu.Unlock()

	switch command {
	case "start":
		return cs.startCalibration(ctx)

	case "set_homing":
		return cs.setHomingPosition(ctx)

	case "start_range_recording":
		return cs.startRangeRecording(ctx)

	case "stop_range_recording":
		return cs.stopRangeRecording(ctx)

	case "save_calibration":
		return cs.saveCalibration(ctx)

	case "abort":
		return cs.abortCalibration(ctx)

	case "reset":
		return cs.resetCalibration(ctx)

	case "get_current_positions":
		return cs.getCurrentPositions(ctx)

	// Motor setup commands (separate workflow from calibration)
	case "motor_setup_discover":
		return cs.motorSetupDiscover(ctx, cmd)

	case "motor_setup_assign_id":
		return cs.motorSetupAssignID(ctx, cmd)

	case "motor_setup_verify":
		return cs.motorSetupVerify(ctx)

	case "motor_setup_scan_bus":
		return cs.motorSetupScanBus(ctx)

	case "motor_setup_reset_status":
		return cs.motorSetupResetStatus(ctx)

	default:
		return nil, fmt.Errorf("unknown command: %s", command)
	}
}

// startCalibration begins the calibration workflow
func (cs *so101CalibrationSensor) startCalibration(_ context.Context) (map[string]any, error) {
	if cs.state != StateIdle && cs.state != StateCompleted && cs.state != StateError {
		return map[string]any{"success": false},
			fmt.Errorf("calibration already in progress (state: %s)", cs.state.String())
	}

	cs.logger.Info("Starting SO-101 calibration workflow")

	// Disable torque to allow manual movement
	if err := cs.controller.SetTorqueEnable(false); err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to disable torque: %v", err))
		return map[string]any{"success": false}, err
	}

	// Reset joint data
	for _, joint := range cs.joints {
		joint.HomingOffset = 0
		joint.RangeMin = 0
		joint.RangeMax = 4095
		joint.RecordedMin = math.MaxInt32
		joint.RecordedMax = math.MinInt32
		joint.IsCompleted = false
	}

	cs.setState(StateStarted,
		"Calibration started. Manually move the robot to the middle of its range of motion, then use 'set_homing' command.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// Add new method to reset calibration registers to factory defaults
// resetCalibrationRegisters resets servo calibration registers to factory defaults
func (cs *so101CalibrationSensor) resetCalibrationRegisters(servoID int) error {
	// Reset homing offset to 0
	homingData := []byte{0x00, 0x00}
	if err := cs.controller.WriteServoRegister(servoID, "homing_offset", homingData); err != nil {
		return fmt.Errorf("failed to reset homing offset: %w", err)
	}

	// Reset min position limit to 0
	minData := []byte{0x00, 0x00}
	if err := cs.controller.WriteServoRegister(servoID, "min_position_limit", minData); err != nil {
		return fmt.Errorf("failed to reset min position limit: %w", err)
	}

	// Reset max position limit to 4095 (0x0FFF)
	maxData := []byte{0xFF, 0x0F}
	if err := cs.controller.WriteServoRegister(servoID, "max_position_limit", maxData); err != nil {
		return fmt.Errorf("failed to reset max position limit: %w", err)
	}

	return nil
}

// setHomingPosition sets the homing offsets based on current positions
func (cs *so101CalibrationSensor) setHomingPosition(_ context.Context) (map[string]any, error) {
	if cs.state != StateStarted {
		return map[string]any{"success": false},
			fmt.Errorf("must start calibration first (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Setting homing positions...")

	// First, reset all calibration registers to factory defaults
	cs.logger.Info("Resetting calibration registers to factory defaults...")
	for _, servoID := range cs.cfg.ServoIDs {
		if err := cs.resetCalibrationRegisters(servoID); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to reset calibration registers for servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}
		cs.logger.Debugf("Reset calibration registers for servo %d", servoID)
	}

	// Brief delay to ensure register writes are complete before reading positions
	time.Sleep(100 * time.Millisecond)

	// Read current positions
	var servos []*feetech.Servo
	for _, id := range cs.cfg.ServoIDs {
		if servo, exists := cs.controller.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return nil, fmt.Errorf("servo %d not available", id)
		}
	}

	positions, err := cs.controller.bus.SyncReadPositions(servos, false)
	if err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to read servo positions: %v", err))
		return map[string]any{"success": false}, err
	}

	// Calculate homing offsets to center the range
	homingOffsets := make(map[string]any)
	for _, servoID := range cs.cfg.ServoIDs {
		currentRawPos := int(positions[servoID])

		// Calculate offset to make current position the center (2047.5 for 12-bit encoder)
		targetCenter := 2047
		homingOffset := currentRawPos - targetCenter

		homingOffsets[strconv.Itoa(servoID)] = homingOffset
		cs.joints[servoID].HomingOffset = homingOffset
		cs.joints[servoID].CurrentPos = currentRawPos

		cs.logger.Infof("Servo %d (%s): raw_position=%d, homing_offset=%d",
			servoID, cs.joints[servoID].Name, currentRawPos, homingOffset)
	}

	// Write homing offsets to servo registers
	cs.logger.Info("Writing homing offsets to servo registers...")
	for _, servoID := range cs.cfg.ServoIDs {
		homingOffset := homingOffsets[strconv.Itoa(servoID)]
		if err := cs.writeHomingOffset(servoID, homingOffset.(int)); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write homing offset to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}
		cs.logger.Debugf("Successfully wrote homing offset %d to servo %d", homingOffset, servoID)
	}

	cs.setState(StateHomingPosition,
		"Homing positions set. Now use 'start_range_recording' command, then move all joints through their entire ranges of motion.")

	return map[string]any{
		"success":        true,
		"state":          cs.state.String(),
		"homing_offsets": homingOffsets,
		"message":        cs.lastInstruction,
	}, nil
}

// startRangeRecording begins recording min/max positions
func (cs *so101CalibrationSensor) startRangeRecording(_ context.Context) (map[string]any, error) {
	if cs.state != StateHomingPosition {
		return map[string]any{"success": false},
			fmt.Errorf("must set homing position first (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Starting range of motion recording...")

	// Create a dedicated context for recording that won't be cancelled when DoCommand returns
	cs.recordingCtx, cs.recordingCancel = context.WithCancel(context.Background())
	cs.recordingActive = true
	cs.recordingStarted = time.Now()
	cs.positionHistory = []map[int]int{}

	cs.setState(StateRangeRecording,
		"Recording range of motion. Move all joints through their full ranges. Use 'stop_range_recording' when complete.")

	// Start background recording goroutine with dedicated context
	go cs.recordPositions(cs.recordingCtx)

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// recordPositions continuously records servo positions in the background
func (cs *so101CalibrationSensor) recordPositions(recordingCtx context.Context) {
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()

	cs.logger.Debug("Position recording goroutine started")

	for {
		select {
		case <-recordingCtx.Done():
			cs.logger.Debug("Position recording goroutine stopped - context cancelled")
			return
		case <-ticker.C:
			cs.mu.RLock()
			if !cs.recordingActive || cs.state != StateRangeRecording {
				cs.mu.RUnlock()
				cs.logger.Debug("Position recording goroutine stopped - recording not active")
				return
			}
			cs.mu.RUnlock()

			// Read current positions
			var servos []*feetech.Servo
			for _, id := range cs.cfg.ServoIDs {
				if servo, exists := cs.controller.servos[id]; exists {
					servos = append(servos, servo)
				} else {
					return
				}
			}

			positions, err := cs.controller.bus.SyncReadPositions(servos, false)
			if err != nil {
				cs.logger.Errorf("Failed to read positions during recording: %v", err)
				continue
			}

			cs.mu.Lock()
			if cs.recordingActive {
				// Convert to raw positions and update min/max
				rawPositions := make(map[int]int)
				for _, servoID := range cs.cfg.ServoIDs {
					rawPos := int(positions[servoID])
					rawPositions[servoID] = rawPos

					joint := cs.joints[servoID]
					joint.CurrentPos = rawPos

					if rawPos < joint.RecordedMin {
						joint.RecordedMin = rawPos
					}
					if rawPos > joint.RecordedMax {
						joint.RecordedMax = rawPos
					}
				}

				cs.positionHistory = append(cs.positionHistory, rawPositions)

				// Limit history to last 1000 samples to prevent memory issues
				if len(cs.positionHistory) > 1000 {
					cs.positionHistory = cs.positionHistory[len(cs.positionHistory)-1000:]
				}
			}
			cs.mu.Unlock()
		}
	}
}

// stopRangeRecording completes the range recording process
func (cs *so101CalibrationSensor) stopRangeRecording(_ context.Context) (map[string]any, error) {
	if cs.state != StateRangeRecording {
		return map[string]any{"success": false},
			fmt.Errorf("range recording not active (current state: %s)", cs.state.String())
	}

	// Stop the recording goroutine
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}

	cs.recordingActive = false
	recordingDuration := time.Since(cs.recordingStarted)

	cs.logger.Infof("Range recording stopped after %.1f seconds, %d samples collected",
		recordingDuration.Seconds(), len(cs.positionHistory))

	// Validate recorded ranges
	rangeData := make(map[string]any)
	allValid := true

	for servoID, joint := range cs.joints {
		if joint.RecordedMin >= joint.RecordedMax {
			cs.logger.Errorf("Invalid range for servo %d (%s): min=%d, max=%d",
				servoID, joint.Name, joint.RecordedMin, joint.RecordedMax)
			allValid = false
			continue
		}

		joint.RangeMin = joint.RecordedMin
		joint.RangeMax = joint.RecordedMax
		joint.IsCompleted = true

		rangeData[joint.Name] = map[string]any{
			"min":   joint.RangeMin,
			"max":   joint.RangeMax,
			"range": joint.RangeMax - joint.RangeMin,
		}

		cs.logger.Infof("Servo %d (%s): range [%d, %d] (span: %d)",
			servoID, joint.Name, joint.RangeMin, joint.RangeMax, joint.RangeMax-joint.RangeMin)
	}

	if !allValid {
		cs.setState(StateError, "Invalid ranges detected. Some joints may not have been moved through their full range.")
		return map[string]any{"success": false}, fmt.Errorf("invalid ranges detected")
	}

	cs.setState(StateCompleted,
		"Range recording completed. Use 'save_calibration' to write calibration to servos and save to file.")

	return map[string]any{
		"success":            true,
		"state":              cs.state.String(),
		"recording_duration": recordingDuration.Seconds(),
		"samples_collected":  len(cs.positionHistory),
		"ranges":             rangeData,
		"message":            cs.lastInstruction,
	}, nil
}

// saveCalibration writes calibration to servos and saves to file
func (cs *so101CalibrationSensor) saveCalibration(_ context.Context) (map[string]any, error) {
	if cs.state != StateCompleted {
		return map[string]any{"success": false},
			fmt.Errorf("calibration not completed (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Saving calibration to servos and file...")

	// Create calibration structure
	fullCalibration := SO101FullCalibration{}

	for servoID, joint := range cs.joints {
		motorCal := &feetech.MotorCalibration{
			ID:           servoID,
			DriveMode:    0, // Normal direction
			HomingOffset: joint.HomingOffset,
			RangeMin:     joint.RangeMin,
			RangeMax:     joint.RangeMax,
			NormMode:     feetech.NormModeDegrees, // Default to degrees
		}

		// Special case for gripper - use percentage mode
		if servoID == 6 {
			motorCal.NormMode = feetech.NormModeRange100
		}

		// Assign to appropriate field in full calibration
		switch servoID {
		case 1:
			fullCalibration.ShoulderPan = motorCal
		case 2:
			fullCalibration.ShoulderLift = motorCal
		case 3:
			fullCalibration.ElbowFlex = motorCal
		case 4:
			fullCalibration.WristFlex = motorCal
		case 5:
			fullCalibration.WristRoll = motorCal
		case 6:
			fullCalibration.Gripper = motorCal
		}
	}

	// Save calibration to file
	if err := SaveFullCalibrationToFile(cs.cfg.CalibrationFile, fullCalibration); err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to save calibration file: %v", err))
		return map[string]any{"success": false}, err
	}

	// Apply calibration to servos (write to registers)
	cs.logger.Info("Writing calibration data to servo registers...")
	for servoID, joint := range cs.joints {
		cs.logger.Infof("Writing to servo %d (%s): min_limit=%d, max_limit=%d",
			servoID, joint.Name, joint.RangeMin, joint.RangeMax)

		// Write min position limit
		if err := cs.writeMinPositionLimit(servoID, joint.RangeMin); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write min position limit to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}

		// Write max position limit
		if err := cs.writeMaxPositionLimit(servoID, joint.RangeMax); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write max position limit to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}

		cs.logger.Debugf("Successfully wrote position limits to servo %d", servoID)
	}

	cs.setState(StateIdle, "Calibration completed and saved successfully. Ready for new calibration.")

	return map[string]any{
		"success":           true,
		"state":             cs.state.String(),
		"calibration_file":  cs.cfg.CalibrationFile,
		"joints_calibrated": len(cs.joints),
		"message":           cs.lastInstruction,
	}, nil
}

// abortCalibration cancels the current calibration process
func (cs *so101CalibrationSensor) abortCalibration(_ context.Context) (map[string]any, error) {
	cs.logger.Info("Aborting calibration...")

	// Stop any active recording
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}
	cs.recordingActive = false

	cs.setState(StateIdle, "Calibration aborted. Ready to start new calibration.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// resetCalibration resets the sensor to initial state
func (cs *so101CalibrationSensor) resetCalibration(_ context.Context) (map[string]any, error) {
	cs.logger.Info("Resetting calibration sensor...")

	// Stop any active recording
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}
	cs.recordingActive = false
	cs.errorMsg = ""
	cs.positionHistory = []map[int]int{}

	// Reset all joint data
	for _, joint := range cs.joints {
		joint.HomingOffset = 0
		joint.RangeMin = 0
		joint.RangeMax = 4095
		joint.RecordedMin = math.MaxInt32
		joint.RecordedMax = math.MinInt32
		joint.IsCompleted = false
	}

	cs.setState(StateIdle, "Calibration sensor reset. Ready to start calibration.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// getCurrentPositions returns current servo positions
func (cs *so101CalibrationSensor) getCurrentPositions(_ context.Context) (map[string]any, error) {
	positions, err := cs.controller.GetJointPositionsForServos(cs.cfg.ServoIDs)
	if err != nil {
		return nil, fmt.Errorf("failed to read positions: %w", err)
	}

	positionData := make(map[string]any)
	for i, servoID := range cs.cfg.ServoIDs {
		rawPos := int(positions[i] * 4095 / (2 * math.Pi))

		joint := cs.joints[servoID]
		joint.CurrentPos = rawPos

		positionData[joint.Name] = map[string]any{
			"servo_id":     servoID,
			"raw_position": rawPos,
			"radians":      positions[i],
			"degrees":      positions[i] * 180 / math.Pi,
		}
	}

	return map[string]any{
		"success":   true,
		"positions": positionData,
	}, nil
}

// setState updates the calibration state and instruction message
func (cs *so101CalibrationSensor) setState(state CalibrationState, instruction string) {
	cs.state = state
	cs.lastInstruction = instruction

	if state == StateError {
		cs.errorMsg = instruction
		cs.logger.Errorf("Calibration error: %s", instruction)
	} else {
		cs.errorMsg = ""
		cs.logger.Infof("Calibration state: %s - %s", state.String(), instruction)
	}
}

// writeHomingOffset writes the homing offset to a servo's register
func (cs *so101CalibrationSensor) writeHomingOffset(servoID, homingOffset int) error {
	data := []byte{
		byte(homingOffset & 0xFF),
		byte((homingOffset >> 8) & 0xFF),
	}

	return cs.controller.WriteServoRegister(servoID, "homing_offset", data)
}

// writeMinPositionLimit writes the minimum position limit to a servo's register
func (cs *so101CalibrationSensor) writeMinPositionLimit(servoID, minLimit int) error {
	data := []byte{
		byte(minLimit & 0xFF),
		byte((minLimit >> 8) & 0xFF),
	}

	return cs.controller.WriteServoRegister(servoID, "min_position_limit", data)
}

// writeMaxPositionLimit writes the maximum position limit to a servo's register
func (cs *so101CalibrationSensor) writeMaxPositionLimit(servoID, maxLimit int) error {
	data := []byte{
		byte(maxLimit & 0xFF),
		byte((maxLimit >> 8) & 0xFF),
	}

	return cs.controller.WriteServoRegister(servoID, "max_position_limit", data)
}

// Motor Setup Functions - separate from calibration workflow
// These implement the systematic motor setup process described in MOTOR_SETUP.md

// MotorSetupConfig represents the target configuration for SO-101 motors
type MotorSetupConfig struct {
	Name     string `json:"name"`
	TargetID int    `json:"target_id"`
	Model    string `json:"model"`
}

// SO101MotorConfigs defines the standard SO-101 motor configuration
// Processed in reverse order to avoid ID conflicts during assignment
var SO101MotorConfigs = []MotorSetupConfig{
	{"gripper", 6, "sts3215"},
	{"wrist_roll", 5, "sts3215"},
	{"wrist_flex", 4, "sts3215"},
	{"elbow_flex", 3, "sts3215"},
	{"shoulder_lift", 2, "sts3215"},
	{"shoulder_pan", 1, "sts3215"},
}

// motorSetupDiscover discovers a single motor connected to the bus
// Parameters: motor_name (string) - name of motor to discover
func (cs *so101CalibrationSensor) motorSetupDiscover(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	motorName, ok := cmd["motor_name"].(string)
	if !ok {
		return nil, fmt.Errorf("motor_name parameter required")
	}

	// Find motor config
	var motorConfig *MotorSetupConfig
	for _, config := range SO101MotorConfigs {
		if config.Name == motorName {
			motorConfig = &config
			break
		}
	}
	if motorConfig == nil {
		return nil, fmt.Errorf("unknown motor name: %s", motorName)
	}

	cs.setupStatus = fmt.Sprintf("Discovering %s motor...", motorName)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Try to discover the servo using controller's bus
	discoveredServo, foundBaudrate, err := cs.discoverOneMotor(motorConfig.Model)
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Failed to discover %s: %v", motorName, err)
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	cs.setupStatus = fmt.Sprintf("Found %s: ID %d, Model %s, Baudrate %d",
		motorName, discoveredServo.ID, discoveredServo.ModelName, foundBaudrate)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success":        true,
		"motor_name":     motorName,
		"current_id":     discoveredServo.ID,
		"target_id":      motorConfig.TargetID,
		"model":          discoveredServo.ModelName,
		"found_baudrate": foundBaudrate,
		"status":         cs.setupStatus,
	}, nil
}

// motorSetupAssignID assigns the target ID to a discovered motor
// Parameters: motor_name (string), current_id (int), target_id (int), current_baudrate (int)
func (cs *so101CalibrationSensor) motorSetupAssignID(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	motorName, ok := cmd["motor_name"].(string)
	if !ok {
		return nil, fmt.Errorf("motor_name parameter required")
	}

	currentID, ok := cmd["current_id"].(float64) // JSON numbers are float64
	if !ok {
		return nil, fmt.Errorf("current_id parameter required")
	}

	targetID, ok := cmd["target_id"].(float64)
	if !ok {
		return nil, fmt.Errorf("target_id parameter required")
	}

	currentBaudrate, ok := cmd["current_baudrate"].(float64)
	if !ok {
		return nil, fmt.Errorf("current_baudrate parameter required")
	}

	cs.setupInProgress = true
	cs.setupStatus = fmt.Sprintf("Configuring %s motor...", motorName)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Create a temporary connection at the current baudrate for configuration
	err := cs.assignMotorIDAndBaudrate(int(currentID), int(targetID), int(currentBaudrate), 1000000)
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Failed to configure %s: %v", motorName, err)
		cs.setupInProgress = false
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	cs.setupStatus = fmt.Sprintf("Successfully configured %s (ID: %d)", motorName, int(targetID))
	cs.setupInProgress = false
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success":      true,
		"motor_name":   motorName,
		"old_id":       int(currentID),
		"new_id":       int(targetID),
		"new_baudrate": 1000000,
		"status":       cs.setupStatus,
	}, nil
}

// motorSetupVerify verifies that all SO-101 motors are properly configured
func (cs *so101CalibrationSensor) motorSetupVerify(ctx context.Context) (map[string]any, error) {
	cs.setupStatus = "Verifying SO-101 motor configuration..."
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Expected motor configuration
	expectedMotors := map[int]string{
		1: "shoulder_pan",
		2: "shoulder_lift",
		3: "elbow_flex",
		4: "wrist_flex",
		5: "wrist_roll",
		6: "gripper",
	}

	results := make(map[string]any)
	allGood := true

	// Check each expected motor
	for id, name := range expectedMotors {
		if servo, exists := cs.controller.servos[id]; exists {
			// Try to ping the servo
			_, err := servo.Ping()
			if err != nil {
				results[name] = map[string]any{
					"id":     id,
					"status": "not_responding",
					"error":  err.Error(),
				}
				allGood = false
			} else {
				// Auto-detect model
				if err := servo.DetectModel(); err != nil {
					results[name] = map[string]any{
						"id":     id,
						"status": "model_detection_failed",
						"error":  err.Error(),
					}
				} else {
					results[name] = map[string]any{
						"id":     id,
						"status": "ok",
						"model":  servo.Model,
					}
				}
			}
		} else {
			results[name] = map[string]any{
				"id":     id,
				"status": "not_found",
				"error":  "servo not in controller",
			}
			allGood = false
		}
	}

	if allGood {
		cs.setupStatus = "✅ All SO-101 motors verified successfully"
	} else {
		cs.setupStatus = "⚠️ Some motors failed verification"
	}

	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success": allGood,
		"motors":  results,
		"status":  cs.setupStatus,
	}, nil
}

// motorSetupScanBus scans the entire bus for connected servos
func (cs *so101CalibrationSensor) motorSetupScanBus(ctx context.Context) (map[string]any, error) {
	cs.setupStatus = "Scanning servo bus for connected motors..."
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Use the controller's bus DiscoverServos method for more efficient discovery
	discovered, err := cs.controller.bus.DiscoverServos()
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Bus scan failed: %v", err)
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	// Process results
	foundServos := make([]map[string]any, 0)
	expectedMotors := map[int]string{1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
	unexpectedCount := 0

	for _, servo := range discovered {
		servoInfo := map[string]any{
			"id":    servo.ID,
			"model": servo.ModelName,
		}

		if expectedName, isExpected := expectedMotors[servo.ID]; isExpected {
			servoInfo["expected_name"] = expectedName
			servoInfo["status"] = "expected"
		} else {
			servoInfo["status"] = "unexpected"
			unexpectedCount++
		}

		foundServos = append(foundServos, servoInfo)
	}

	cs.setupStatus = fmt.Sprintf("Found %d servos (%d unexpected)", len(discovered), unexpectedCount)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success":          true,
		"servos_found":     len(discovered),
		"unexpected_count": unexpectedCount,
		"servos":           foundServos,
		"status":           cs.setupStatus,
	}, nil
}

// motorSetupResetStatus resets the motor setup status
func (cs *so101CalibrationSensor) motorSetupResetStatus(ctx context.Context) (map[string]any, error) {
	cs.setupInProgress = false
	cs.currentSetupStep = 0
	cs.setupStatus = "Motor setup status reset"

	return map[string]any{
		"success": true,
		"status":  cs.setupStatus,
	}, nil
}

// Helper function to discover a single motor using the bus DiscoverServos method
func (cs *so101CalibrationSensor) discoverOneMotor(expectedModel string) (*feetech.DiscoveredServo, int, error) {
	// Since we're using a shared controller, we need to work with the existing bus
	// Use the more efficient DiscoverServos method instead of scanning all IDs

	discovered, err := cs.controller.bus.DiscoverServos()
	if err != nil {
		return nil, 0, fmt.Errorf("discovery failed: %w", err)
	}

	if len(discovered) == 0 {
		return nil, 0, fmt.Errorf("no servos found")
	}

	if len(discovered) > 1 {
		return nil, 0, fmt.Errorf("multiple servos found (%d) - connect only one motor", len(discovered))
	}

	servo := discovered[0]
	if servo.ModelName != expectedModel {
		return nil, 0, fmt.Errorf("model mismatch: expected %s, found %s", expectedModel, servo.ModelName)
	}

	return &servo, cs.cfg.Baudrate, nil // Return current baudrate as "found" baudrate
}

// Helper function to assign motor ID and baudrate
func (cs *so101CalibrationSensor) assignMotorIDAndBaudrate(currentID, targetID, currentBaudrate, targetBaudrate int) error {
	// Get the servo instance
	servo, exists := cs.controller.servos[currentID]
	if !exists {
		return fmt.Errorf("servo with ID %d not found in controller", currentID)
	}

	// Ping to verify communication
	_, err := servo.Ping()
	if err != nil {
		return fmt.Errorf("failed to ping servo: %w", err)
	}

	// Set target ID if different from current
	if currentID != targetID {
		if err := servo.SetServoID(targetID); err != nil {
			return fmt.Errorf("failed to set servo ID: %w", err)
		}
		cs.logger.Infof("Updated servo ID from %d to %d", currentID, targetID)
	}

	// Set target baudrate if different from current
	if currentBaudrate != targetBaudrate {
		if err := servo.SetBaudrate(targetBaudrate); err != nil {
			return fmt.Errorf("failed to set baudrate: %w", err)
		}
		cs.logger.Infof("Updated servo baudrate to %d", targetBaudrate)
	}

	return nil
}

// Close cleans up the sensor
func (cs *so101CalibrationSensor) Close(ctx context.Context) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	// Stop any active recording
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}
	cs.recordingActive = false

	if cs.controller != nil {
		ReleaseSharedController()
	}

	return nil
}

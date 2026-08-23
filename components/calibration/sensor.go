// calibration_sensor.go - SO-101 Calibration Sensor Component
package calibration

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"

	"so_arm/internal/controller"
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
	controller *controller.ControllerHandle

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
	// recordingDone is closed by recordPositions on its way out, so Close can wait for it.
	// Joining matters because Close is followed by controller.Release(), which closes the
	// shared bus -- without the join that lands under a live SyncReadPositions.
	recordingDone   chan struct{}
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

	// Create controller configuration
	controllerConfig := &controller.SoArm101Config{
		Port:            conf.Port,
		Baudrate:        conf.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6}, // Controller handles all 6
		Timeout:         conf.Timeout,
		CalibrationFile: conf.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(conf.CalibrationFile)

	// Load existing calibration for baseline
	calibration, fromFile := controllerConfig.LoadCalibration(logger)

	ctrl, err := controller.AcquireController(rawConf.ResourceName(), controllerConfig, calibration, fromFile)
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
		controller:      ctrl,
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

// Status returns the current status of the resource as a map of key-value pairs.
func (cs *so101CalibrationSensor) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
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

// Close cleans up the sensor
func (cs *so101CalibrationSensor) Close(ctx context.Context) error {
	// Take cs.mu only long enough to snapshot the recording state. The join below MUST
	// happen with the lock released: recordPositions takes cs.mu every 10ms tick, so
	// waiting on its done channel while holding the lock deadlocks whenever the goroutine
	// is mid-cycle -- and cancelling does not help, it is already past its select.
	cs.mu.Lock()
	cancel := cs.recordingCancel
	done := cs.recordingDone
	ctrl := cs.controller
	cs.recordingCancel = nil
	cs.recordingDone = nil
	cs.recordingActive = false
	cs.mu.Unlock()

	if cancel != nil {
		cancel()
	}
	if done != nil {
		select {
		case <-done:
		case <-time.After(2 * time.Second):
			// Bounded: a hung read must not hang a reconfigure forever. Releasing anyway
			// is the lesser evil, and the bus close below is what the goroutine would
			// then fail against.
			cs.logger.Warn("recording goroutine did not exit; releasing the bus anyway")
		}
	}

	// Only now, with no reader left on the bus, is it safe to release -- Release closes
	// the shared bus once this is the last holder.
	if ctrl != nil {
		ctrl.Release()
	}

	return nil
}

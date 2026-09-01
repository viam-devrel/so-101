// Package arm implements the devrel:so101:arm model: the hardware SO-101 arm (5-DOF,
// servos 1-5).
package arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/utils/rpc"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
	"so_arm/internal/planning"
	"so_arm/internal/servo"
)

var (
	SO101Model = resource.NewModel("devrel", "so101", "arm")
)

func init() {
	resource.RegisterComponent(arm.API, SO101Model,
		resource.Registration[arm.Arm, *SO101ArmConfig]{
			Constructor: newso101,
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

	Motion string `json:"motion,omitempty"`

	CalibrationFile string `json:"calibration_file,omitempty"`

	// VisualizeEEFrame, when true, makes Get3DModels serve a colored XYZ
	// coordinate-frame marker at the end-effector. Defaults to false.
	VisualizeEEFrame bool `json:"visualize_ee_frame,omitempty"`

	// UseURDF sources kinematics + mesh collision geometry from the bundled
	// assets/urdf/so101.urdf instead of the embedded so101.json. Requires VIAM_MODULE_ROOT
	// (set by viam-server). Default false.
	UseURDF bool `json:"use_urdf,omitempty"`
	// MeshDecimationRatios is the per-collision-mesh simplification ratio in [0,1],
	// applied in URDF document order (one per arm link: base/shoulder/upper_arm/
	// lower_arm/wrist). Only values strictly in (0,1) actually decimate; lower = more
	// aggressive. Defaults to 0.9 for each of the 5 meshes when empty. Ignored unless
	// UseURDF is set.
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`

	// ManualMode tunes hand-guided ("manual") mode. Optional; nil disables
	// the feature / uses built-in defaults throughout.
	ManualMode *ManualModeConfig `json:"manual_mode,omitempty"`

	// OrientationToleranceDeg is the half-angle, in degrees, of the cone of acceptable
	// end-effector approach directions around the goal orientation. Roll about that axis
	// is NOT constrained. Valid range [0, 180]. Zero or unset means
	// defaultOrientationToleranceDeg (30).
	//
	// The planner adds a 0.001 epsilon to the OZ leeway, so the cone enforced is
	// acos(cos(tol) - 0.001): always slightly wider than requested (30 gives ~30.11), never
	// narrower than ~2.56. At or above 177.44 every orientation is accepted.
	//
	// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds.
	OrientationToleranceDeg float64 `json:"orientation_tolerance_deg,omitempty"`

	// PositionToleranceMM is the per-axis positional leeway of the goal cloud, applied as
	// a box along the goal frame's axes (worst-case corner deviation is sqrt(3) times this
	// value). It must be large enough for the IK solver to realistically land inside; a
	// cloud only an exact match could satisfy means every move fails. Zero or unset means
	// defaultPositionToleranceMM (1.0).
	PositionToleranceMM float64 `json:"position_tolerance_mm,omitempty"`

	// WaypointLookaheadDeg is how far ahead of the arm the commanded goal may run during a
	// MoveThroughJointPositions / GoToInputs stream, in degrees: waypoint k+1 is not written
	// until every joint is this close to waypoint k. It is the path-fidelity/speed trade --
	// larger is faster but cuts more corner, smaller tracks the path more exactly, down to a
	// near-stop at every waypoint.
	//
	// Zero or unset DERIVES it per move from the speed and acceleration actually in force
	// (servo.LookaheadDegFor) rather than using a fixed value -- a fast move needs a longer
	// lookahead than a slow one, because the servo's deceleration distance grows with the
	// square of its speed. Set this only to override that. Valid range 0.1-45.
	WaypointLookaheadDeg float64 `json:"waypoint_lookahead_deg,omitempty"`
}

// ManualModeConfig tunes hand-guided ("manual") mode. All fields optional;
// zero means "use the built-in default". See manualDefaults(), defaultManualPGain,
// and defaultManualTorqueLimit — the defaults are the bench-validated tuning, so
// manual mode is usable with no config at all.
type ManualModeConfig struct {
	PosDeadbandDeg float64 `json:"pos_deadband_deg,omitempty"` // backdrive distance (deg) before the goal follows the hand (default 0.5)
	LoopHz         float64 `json:"loop_hz,omitempty"`          // control loop rate (Hz) (default 50)
	PGain          int     `json:"p_gain,omitempty"`           // reduced servo P-gain during manual mode (default 8; 0-255)
	TorqueLimit    int     `json:"torque_limit,omitempty"`     // servo torque cap during manual mode for backdrive compliance (default 50; 0-1000; lower = easier to move but must stay above gravity-hold load)
}

// Validate ensures the manual mode config's fields are within acceptable ranges.
func (m *ManualModeConfig) Validate() error {
	if m == nil {
		return nil
	}
	if m.PosDeadbandDeg < 0 || m.PosDeadbandDeg > 45 {
		return fmt.Errorf("manual_mode.pos_deadband_deg must be in [0,45], got %v", m.PosDeadbandDeg)
	}
	if m.LoopHz < 0 || m.LoopHz > 200 {
		return fmt.Errorf("manual_mode.loop_hz must be in [0,200], got %v", m.LoopHz)
	}
	if m.PGain < 0 || m.PGain > 255 {
		return fmt.Errorf("manual_mode.p_gain must be in [0,255], got %v", m.PGain)
	}
	if m.TorqueLimit < 0 || m.TorqueLimit > 1000 {
		return fmt.Errorf("manual_mode.torque_limit must be in [0,1000], got %v", m.TorqueLimit)
	}
	return nil
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

	for i, r := range cfg.MeshDecimationRatios {
		if math.IsNaN(r) || r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
	}

	if err := cfg.ManualMode.Validate(); err != nil {
		return nil, nil, err
	}

	if err := planning.ValidateGoalCloudTolerances(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM); err != nil {
		return nil, nil, err
	}

	// NaN is checked explicitly: every comparison against it is false, so a NaN would pass a
	// range check and then make the dwell's `remaining <= lookahead` test false forever --
	// every waypoint would burn its full timeout.
	if t := cfg.WaypointLookaheadDeg; t != 0 {
		if math.IsNaN(t) || t < servo.MinLookaheadDeg || t > servo.MaxLookaheadDeg {
			return nil, nil, fmt.Errorf("waypoint_lookahead_deg must be in [%g,%g], got %v",
				servo.MinLookaheadDeg, servo.MaxLookaheadDeg, t)
		}
	}

	deps := []string{}

	if cfg.Motion != "" {
		deps = append(deps, motion.Named(cfg.Motion).String())
	} else {
		// use builtin motion service
		deps = append(deps, motion.Named("builtin").String())
	}

	return deps, nil, nil
}

type so101 struct {
	resource.AlwaysRebuild

	name       resource.Name
	logger     logging.Logger
	cfg        *SO101ArmConfig
	opMgr      *operation.SingleOperationManager
	controller *controller.ControllerHandle
	manual     *manualSession

	// goalCloud is the resolved approach-axis tolerance pair. Set once in NewSO101 and
	// never mutated (the model is resource.AlwaysRebuild), so it needs no mutex.
	goalCloud planning.GoalCloudConfig

	// lookaheadDeg is the CONFIGURED waypoint lookahead override, or 0 to derive it per
	// move from that move's own speed and acceleration. Set once in NewSO101 and never
	// mutated, same as goalCloud.
	lookaheadDeg float64

	// readJoints, when non-nil, replaces the bus read behind currentJoints. Tests only.
	readJoints func(context.Context) ([]float64, error)

	// servosMoving, when non-nil, replaces the bus read behind anyServoMoving. Tests only.
	servosMoving func(context.Context) (bool, error)

	mu       sync.RWMutex
	moveLock sync.Mutex
	isMoving atomic.Bool
	model    referenceframe.Model

	// Servo IDs controlled by this arm (1-5)
	armServoIDs []int

	defaultSpeed float32
	defaultAcc   float32

	motion motion.Service

	cancelCtx  context.Context
	cancelFunc func()
	initCtx    context.Context // Context for initialization operations
}

// makeModelFrame builds the SO-101 kinematic model from either the embedded so101.json
// (default) or the bundled assets/urdf/so101.urdf (when cfg.UseURDF). On the JSON path, when
// visualize_ee_frame is set the "tool" link gets a placeholder geometry so the viewer
// draws the EE marker. VisualizeEEFrame's placeholder handling under URDF mode is deferred to
// a later task (frame-alignment), so it is not applied here.
func makeModelFrame(cfg *SO101ArmConfig, name string) (referenceframe.Model, error) {
	return geometry.ArmModel(cfg.UseURDF, cfg.MeshDecimationRatios, cfg.VisualizeEEFrame, name)
}

func newso101(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	newConf, err := resource.NativeConfig[*SO101ArmConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewSO101(ctx, deps, rawConf.ResourceName(), newConf, logger)
}

func NewSO101(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SO101ArmConfig, logger logging.Logger) (arm.Arm, error) {
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
		accelerationDegsPerSec = servo.DefaultAccelDegsPerSecSq
	}
	if accelerationDegsPerSec < servo.MinAccelDegsPerSecSq || accelerationDegsPerSec > servo.MaxAccelDegsPerSecSq {
		return nil, fmt.Errorf(
			"acceleration_degs_per_sec_per_sec must be between %.0f and %.0f degrees/second^2, got %.1f",
			servo.MinAccelDegsPerSecSq, servo.MaxAccelDegsPerSecSq, accelerationDegsPerSec)
	}

	if conf.Baudrate == 0 {
		conf.Baudrate = 1000000
	}

	if len(conf.ServoIDs) == 0 {
		conf.ServoIDs = []int{1, 2, 3, 4, 5}
	}

	// Create controller configuration
	controllerConfig := &controller.SoArm101Config{
		Port:            conf.Port,
		Baudrate:        conf.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6}, // Controller handles all 6, but arm only uses 1-5
		Timeout:         conf.Timeout,
		CalibrationFile: conf.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(conf.CalibrationFile)

	// Load full calibration (includes gripper for shared controller)
	calibration, fromFile := controllerConfig.LoadCalibration(logger)
	if calibration.ShoulderPan != nil {
		logger.Debugf("Using calibration for SO-101 with shoulder_pan homing_offset: %d", calibration.ShoulderPan.HomingOffset)
	} else {
		logger.Debug("Using default calibration for SO-101")
	}

	ctrl, err := controller.AcquireController(name, controllerConfig, calibration, fromFile)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared SO-ARM controller: %w", err)
	}

	model, err := makeModelFrame(conf, name.Name)
	if err != nil {
		ctrl.Release() // Clean up on error
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	var ms motion.Service
	if conf.Motion != "" {
		if deps == nil {
			return nil, fmt.Errorf("no deps")
		}
		ms, err = motion.FromProvider(deps, conf.Motion)
		if err != nil {
			return nil, err
		}
	} else {
		ms, err = motion.FromProvider(deps, "builtin")
		if err != nil {
			return nil, err
		}
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	arm := &so101{
		name:         name,
		cfg:          conf,
		opMgr:        operation.NewSingleOperationManager(),
		logger:       logger,
		controller:   ctrl,
		model:        model,
		armServoIDs:  conf.ServoIDs, // Store which servos this arm controls
		defaultSpeed: speedDegsPerSec,
		defaultAcc:   accelerationDegsPerSec,
		motion:       ms,
		goalCloud:    planning.ResolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
		initCtx:      ctx, // Store initialization context

		lookaheadDeg: conf.WaypointLookaheadDeg,
	}

	logger.Debugf("SO-101 configured with speed: %.1f deg/s, acceleration: %.1f deg/s², "+
		"waypoint lookahead: %.2f deg (0 = derived per move)",
		speedDegsPerSec, accelerationDegsPerSec, conf.WaypointLookaheadDeg)
	logger.Debugf("Arm controlling servo IDs: %v", arm.armServoIDs)

	// Initialize and verify servo connections
	if err := arm.initializeServos(); err != nil {
		ctrl.Release() // Clean up on error
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

func (s *so101) Close(context.Context) error {
	s.mu.Lock()
	s.exitManualLocked("arm closing")
	s.mu.Unlock()

	s.cancelFunc()
	s.controller.Release()
	return nil
}

// initializeServos pings each servo and enables torque to ensure proper communication
func (s *so101) initializeServos() error {
	return s.initializeServosWithRetry(3)
}

// initializeServosWithRetry attempts servo initialization with retries
func (s *so101) initializeServosWithRetry(maxRetries int) error {
	s.logger.Debug("Initializing SO-101 arm servos...")

	var lastErr error
	for attempt := 1; attempt <= maxRetries; attempt++ {
		s.logger.Debugf("Arm servo initialization attempt %d/%d", attempt, maxRetries)

		if err := s.doServoInitialization(); err != nil {
			lastErr = err
			s.logger.Warnf("Initialization attempt %d failed: %v", attempt, err)

			if attempt < maxRetries {
				waitTime := time.Duration(attempt) * 500 * time.Millisecond
				s.logger.Debugf("Waiting %v before retry...", waitTime)
				time.Sleep(waitTime)
				continue
			}
		} else {
			s.logger.Debugf("Arm servo initialization successful on attempt %d", attempt)
			return nil
		}
	}

	return fmt.Errorf("arm servo initialization failed after %d attempts, last error: %w", maxRetries, lastErr)
}

// doServoInitialization performs the actual initialization steps
func (s *so101) doServoInitialization() error {
	// Use stored initialization context instead of creating new one
	ctx := s.initCtx

	// Ping all servos to ensure they're responding
	s.logger.Debug("Pinging all servos...")
	if err := s.controller.Ping(ctx); err != nil {
		return fmt.Errorf("servo ping failed: %w", err)
	}
	s.logger.Debug("All servos ping successful")

	// Enable torque for all servos (controller manages all 6)
	s.logger.Debug("Enabling torque for all servos...")
	if err := s.controller.SetTorqueEnable(ctx, true); err != nil {
		return fmt.Errorf("failed to enable torque: %w", err)
	}

	time.Sleep(100 * time.Millisecond)

	s.logger.Debug("Verifying position reading from arm servos...")
	positions, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to read initial joint positions: %w", err)
	}

	if len(positions) != len(s.armServoIDs) {
		return fmt.Errorf("expected %d joint positions, got %d", len(s.armServoIDs), len(positions))
	}

	s.logger.Debugf("SO-101 arm servo initialization successful. Initial positions: %v", positions)
	return nil
}

// diagnoseConnection provides detailed diagnostics for troubleshooting
func (s *so101) diagnoseConnection() error {
	// Use stored initialization context instead of creating new one
	ctx := s.initCtx

	s.logger.Debug("Starting SO-101 arm connection diagnosis...")

	// Test overall ping
	s.logger.Debug("Testing overall servo communication...")
	if err := s.controller.Ping(ctx); err != nil {
		s.logger.Errorf("Overall ping failed: %v", err)
		return err
	}
	s.logger.Debug("Overall ping successful")

	positions, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Errorf("Failed to read arm positions: %v", err)
		return err
	}

	for i, pos := range positions {
		s.logger.Debugf("Arm servo %d position: %.3f rad", s.armServoIDs[i], pos)
	}

	return nil
}

// verifyServoConfig checks servo configuration
func (s *so101) verifyServoConfig() error {
	// Use stored initialization context instead of creating new one
	ctx := s.initCtx

	s.logger.Debug("Verifying arm servo configuration...")

	positions, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		return fmt.Errorf("failed to verify servo config: %w", err)
	}

	if len(positions) != len(s.armServoIDs) {
		return fmt.Errorf("config verification failed: expected %d servos, got %d", len(s.armServoIDs), len(positions))
	}

	s.logger.Debugf("Arm servo configuration verified. Current positions: %v", positions)
	return nil
}

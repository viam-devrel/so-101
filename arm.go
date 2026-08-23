package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"

	"so_arm/internal/geometry"
	"so_arm/internal/planning"
	"so_arm/internal/servocmd"
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
	// The planner adds a 0.001 epsilon to the OZ leeway, so the cone actually enforced is
	// acos(cos(tol) - 0.001): always slightly wider than requested (30 gives ~30.11) and
	// never narrower than ~2.56. At or above 177.44 every orientation is accepted.
	//
	// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds.
	OrientationToleranceDeg float64 `json:"orientation_tolerance_deg,omitempty"`

	// PositionToleranceMM is the per-axis positional leeway of the goal cloud, applied as
	// a box along the goal frame's axes (worst-case corner deviation is sqrt(3) times this
	// value). It must be large enough for the IK solver to realistically land inside; a
	// cloud only an exact match could satisfy means every move fails. Zero or unset means
	// defaultPositionToleranceMM (1.0).
	PositionToleranceMM float64 `json:"position_tolerance_mm,omitempty"`
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
	controller *ControllerHandle
	manual     *manualSession

	// goalCloud is the resolved approach-axis tolerance pair. Set once in NewSO101 and
	// never mutated (the model is resource.AlwaysRebuild), so it needs no mutex.
	goalCloud planning.GoalCloudConfig

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

// calculateJointLimits dynamically calculates joint limits from calibration data
func (s *so101) calculateJointLimits() [][2]float64 {
	limits := make([][2]float64, len(s.armServoIDs))

	calibration := s.controller.GetCalibration()

	// Map servo IDs to calibration data
	jointCals := []*MotorCalibration{
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
		accelerationDegsPerSec = defaultAccelDegsPerSecSq
	}
	if accelerationDegsPerSec < minAccelDegsPerSecSq || accelerationDegsPerSec > maxAccelDegsPerSecSq {
		return nil, fmt.Errorf(
			"acceleration_degs_per_sec_per_sec must be between %.0f and %.0f degrees/second^2, got %.1f",
			minAccelDegsPerSecSq, maxAccelDegsPerSecSq, accelerationDegsPerSec)
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
	calibration, fromFile := controllerConfig.LoadCalibration(logger)
	if calibration.ShoulderPan != nil {
		logger.Debugf("Using calibration for SO-101 with shoulder_pan homing_offset: %d", calibration.ShoulderPan.HomingOffset)
	} else {
		logger.Debug("Using default calibration for SO-101")
	}

	controller, err := AcquireController(name, controllerConfig, calibration, fromFile)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared SO-ARM controller: %w", err)
	}

	model, err := makeModelFrame(conf, name.Name)
	if err != nil {
		controller.Release() // Clean up on error
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
		controller:   controller,
		model:        model,
		armServoIDs:  conf.ServoIDs, // Store which servos this arm controls
		defaultSpeed: speedDegsPerSec,
		defaultAcc:   accelerationDegsPerSec,
		motion:       ms,
		goalCloud:    planning.ResolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
		initCtx:      ctx, // Store initialization context
	}

	logger.Debugf("SO-101 configured with speed: %.1f deg/s, acceleration: %.1f deg/s²",
		speedDegsPerSec, accelerationDegsPerSec)
	logger.Debugf("Arm controlling servo IDs: %v", arm.armServoIDs)

	// Initialize and verify servo connections
	if err := arm.initializeServos(); err != nil {
		controller.Release() // Clean up on error
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

	pose, err := geometry.ComputeOOBPosition(s.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

// MoveToPosition moves the arm's end-effector to the target pose.
//
// The SO-101 is a 5-DOF arm, so most six-DOF pose targets are unreachable. Rather than
// discard orientation wholesale, the planner is given an approach-axis cone (a
// referenceframe.PoseCloud) around the goal orientation: the tool's pointing direction is
// constrained to within orientation_tolerance_deg, while roll about that axis is free.
//
// Callers may bypass the cone via extra: "goal_metric_type" restores the old
// orientation-agnostic behavior, and "pose_cloud" supplies a raw cloud. See internal/planning.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds, which makes
// planning revert to strict six-DOF scoring and fail.
func (s *so101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	// A motion command takes the arm out of hand-guided ("manual") mode.
	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	dest, planExtra, path, err := planning.BuildMoveDestination(
		fmt.Sprintf("%v_origin", s.Name().Name), pose, s.goalCloud, extra)
	if err != nil {
		// Return the build error UNWRAPPED: path is the zero GoalPath, and wrapping a pose_cloud
		// parse error as a cone-planning failure would tell the caller to widen tolerances
		// they never set.
		return err
	}
	s.logger.Debugf("MoveToPosition goal cloud: %+v", dest.GoalCloud)

	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().Name,
		Destination:   dest,
		Extra:         planExtra,
	})
	return planning.WrapMoveErr(err, path, s.goalCloud)
}

// clampPositions clamps joint inputs (already in radians) to each joint's calibrated
// limits, warning on out-of-range values.
func (s *so101) clampPositions(positions []referenceframe.Input) ([]float64, error) {
	if len(positions) != len(s.armServoIDs) {
		return nil, fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(positions))
	}

	values := make([]float64, len(positions))
	copy(values, positions) // referenceframe.Input is an alias for float64

	jointLimits := s.calculateJointLimits()

	clamped := make([]float64, len(values))
	for i, pos := range values {
		min, max := jointLimits[i][0], jointLimits[i][1]
		if pos < min || pos > max {
			s.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				s.armServoIDs[i], pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clamped[i] = math.Max(min, math.Min(max, pos))
	}
	return clamped, nil
}

// moveJoints commands a coordinated move from `from` to `to` (both radians, per arm servo),
// scaling every joint's speed and acceleration by its share of the longest travel so all
// joints arrive together. When wait is true it blocks until the arm stops.
//
// The caller must hold s.moveLock and manage s.isMoving.
func (s *so101) moveJoints(
	ctx context.Context,
	from, to []float64,
	speedDegsPerSec, accelDegsPerSecSq float64,
	caps []jointLimits,
	wait bool,
) error {
	if len(from) != len(to) {
		return fmt.Errorf("coordinated move needs matching position counts, got %d current and %d target",
			len(from), len(to))
	}
	travelsDeg, maxTravelDeg := jointTravelsDeg(from, to)

	profiles := coordinatedProfiles(travelsDeg, speedDegsPerSec, accelDegsPerSecSq, caps)
	if profiles == nil {
		// Every joint is already at its target. Note this is a behavior change: the old
		// path still issued a write, so a caller re-asserting its current pose used to
		// reach the bus and now does not.
		s.logger.Debug("moveJoints: all joints already at target, nothing to command")
		return nil
	}

	servoProfiles := make([]ServoProfile, len(profiles))
	for i, p := range profiles {
		servoProfiles[i] = ServoProfile{SpeedSteps: p.speedSteps, AccUnits: p.accUnits}
	}

	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, servoProfiles); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	if !wait {
		return nil
	}

	// Count moving joints pinned at the acceleration floor. Acc 1 delivers ~43 deg/s^2, so
	// a short-travel joint can need less than the register can express and will arrive
	// early. This is the diagnostic for that documented limitation.
	floored := 0
	for i, d := range travelsDeg {
		if d > 0 && profiles[i].accUnits == minAccUnits {
			floored++
		}
	}

	// Time against the speed ACTUALLY COMMANDED, not the reference the caller requested.
	// A MoveOptions cap can reduce the shared reference well below the request -- the cap
	// tests in motion_profile_test.go reduce 50 deg/s to 20, a 2.5x slowdown against
	// moveTimeoutFactor = 2.0 -- so timing against the request would abort a move that is
	// proceeding perfectly correctly.
	effectiveSpeed := speedDegsPerSec
	maxSteps := 0
	for _, p := range profiles {
		if p.speedSteps > maxSteps {
			maxSteps = p.speedSteps
		}
	}
	if maxSteps > 0 {
		effectiveSpeed = float64(maxSteps) / stepsPerDegree
	}

	// Use the acceleration actually achievable: the Acc register floors at ~43 deg/s^2, so
	// a cap-reduced reference below that is silently raised by the hardware and timing
	// against the lower value would over-estimate the duration.
	effectiveAccel := math.Max(accelDegsPerSecSq, accFloorDegsPerSecSq)
	timeout := moveTimeoutMs(maxTravelDeg, effectiveSpeed, effectiveAccel)
	s.logger.Debugf("moveJoints: ref %.1f deg/s (effective %.1f), %.0f deg/s^2, max travel "+
		"%.1f deg, %d joint(s) at the acceleration floor, wait timeout %d ms",
		speedDegsPerSec, effectiveSpeed, accelDegsPerSecSq, maxTravelDeg, floored, timeout)
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, timeout)
}

// moveJointsUniform is the pre-coordination path, retained only as the fallback for when
// the travel-reference read fails. It commands every joint at one shared speed, which is
// why joints arrive at different times.
func (s *so101) moveJointsUniform(ctx context.Context, to []float64, speedDegsPerSec, accelDegsPerSecSq float64, wait bool) error {
	// Writes an explicit UNIFORM profile rather than a bare position write. A coordinated
	// move leaves each servo's Speed and Acc registers at its own scaled value -- as low as
	// 1 step/s and Acc 1 -- and a position-only write does not touch either. So without
	// this the "uniform" fallback would inherit whatever the previous coordinated move left
	// behind, and would be uniform in neither speed nor acceleration.
	profile := ServoProfile{
		SpeedSteps: degPerSecToStepsPerSec(speedDegsPerSec),
		AccUnits:   degPerSecSqToAccUnits(accelDegsPerSecSq),
	}
	profiles := make([]ServoProfile, len(s.armServoIDs))
	for i := range profiles {
		profiles[i] = profile
	}
	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, profiles); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}
	if !wait {
		return nil
	}
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, maxMoveTimeoutMs)
}

// parseWaitExtra reads the optional "wait" bool from a DoCommand/extra map.
// Absent or non-bool values default to true so the motion planner and existing
// callers keep the blocking behavior; teleop passes wait=false to stream setpoints.
func parseWaitExtra(extra map[string]interface{}) bool {
	if extra != nil {
		if w, ok := extra["wait"].(bool); ok {
			return w
		}
	}
	return true
}

func (s *so101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	clamped, err := s.clampPositions(positions)
	if err != nil {
		return err
	}

	s.mu.RLock()
	speed := float64(s.defaultSpeed)
	accel := float64(s.defaultAcc)
	s.mu.RUnlock()

	// The position read is now unconditional, where it used to happen only when waiting.
	// Coordination needs a travel reference, and on v0.6.1 this read costs about 1ms
	// against roughly 12ms on v0.6.0 -- which is why the version bump is a correctness
	// dependency, not just a performance one.
	//
	// A read failure degrades to the old uniform-speed path rather than failing the move.
	// Before this change the read happened only when waiting and a failure was a warning;
	// making it fatal would mean a transient bus error aborts a teleop setpoint that used
	// to go through, at 30-50 Hz on the wait:false path.
	current, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("failed to read positions for coordinated move, falling back to "+
			"uniform speed: %v", err)
		// No arm.MoveOptions here, so caps is always nil -- kept for symmetry with
		// MoveThroughJointPositions' fallback.
		return s.moveJointsUniform(ctx, clamped, uniformSpeedUnderCaps(speed, nil), accel, parseWaitExtra(extra))
	}

	return s.moveJoints(ctx, current, clamped, speed, accel, nil, parseWaitExtra(extra))
}

func (s *so101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	s.mu.RLock()
	defaultSpeed := float64(s.defaultSpeed)
	defaultAccel := float64(s.defaultAcc)
	s.mu.RUnlock()
	accel := defaultAccel

	caps, err := jointLimitsFromMoveOptions(options, len(s.armServoIDs))
	if err != nil {
		return err
	}

	// MaxVelRads also becomes the reference speed, not just a per-joint cap. That is
	// deliberate double application: as the reference it sets the pace, and as a cap it
	// bypasses resolveSpeedDegsPerSec's 3 deg/s floor for very small values. Per arm.proto
	// the scalar is ignored entirely when the per-joint slice is set.
	speed := defaultSpeed
	if options != nil && len(options.MaxVelRadsJoints) == 0 && options.MaxVelRads > 0 {
		speed = resolveSpeedDegsPerSec(options.MaxVelRads, defaultSpeed)
	}

	// One read per CALL, not per waypoint. The first waypoint has no predecessor to
	// difference against, and GoToInputs routinely emits single-waypoint streams which
	// would otherwise have no travel reference at all.
	from, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("failed to read positions for coordinated move, falling back to "+
			"uniform speed: %v", err)
		from = nil
	}

	for idx, jointPositions := range positions {
		clamped, err := s.clampPositions(jointPositions)
		if err != nil {
			return err
		}
		isLast := idx == len(positions)-1

		// The hardware executes ONLY the final write. Each SetGoals overwrites the previous
		// goal, and the whole stream is issued in a few tens of milliseconds, so every
		// intermediate waypoint is superseded before the arm can act on it. The last
		// waypoint's profile therefore has to be computed against where the arm ACTUALLY
		// is, not against the previous commanded waypoint: a joint whose travel is
		// concentrated earlier in the path has a near-zero delta in the final segment,
		// which floors it to 1 step/s while its goal is still far away, and collapses the
		// completion timeout to the 1s floor so the call returns mid-motion.
		if isLast && from != nil {
			if cur, rerr := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs); rerr == nil {
				from = cur
			} else {
				s.logger.Warnf("failed to re-read positions before the final waypoint; "+
					"coordination and timeout will use the commanded predecessor: %v", rerr)
			}
		}

		if from == nil {
			if err := s.moveJointsUniform(ctx, clamped, uniformSpeedUnderCaps(speed, caps), accel, isLast); err != nil {
				return err
			}
		} else if err := s.moveJoints(ctx, from, clamped, speed, accel, caps, isLast); err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
		// Later segments difference against the previous COMMANDED waypoint.
		if from != nil {
			from = clamped
		}
	}
	return nil
}

func (s *so101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	radians, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("Failed to read joint positions: %v", err)
		return nil, fmt.Errorf("failed to read joint positions: %w. Try running 'diagnose' command for more details", err)
	}

	if len(radians) != len(s.armServoIDs) {
		return nil, fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(radians))
	}

	positions := make([]referenceframe.Input, len(radians))
	copy(positions, radians)

	return positions, nil
}

func (s *so101) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.mu.Lock()
	s.exitManualLocked("stop command received")
	s.mu.Unlock()

	s.isMoving.Store(false)
	return s.controller.Stop(ctx)
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

// Get3DModels serves the SO-101 link meshes for the 3D scene viewer, plus a colored XYZ
// coordinate-frame marker at the end-effector when the visualize_ee_frame attribute is
// enabled.
func (s *so101) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return geometry.ArmMeshes(s.cfg.VisualizeEEFrame), nil
}

// Status returns the current status of the resource as a map of key-value pairs.
func (s *so101) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if s.manual != nil && s.manual.running() {
		// GetStatus serializes this via structpb.NewStruct, which only accepts
		// structpb-native types — a typed []manualJointStatus slice fails. Emit the
		// joints as []interface{} of map[string]interface{} so the wire format holds.
		js := s.manual.statusJoints()
		joints := make([]interface{}, 0, len(js))
		for _, j := range js {
			joints = append(joints, map[string]interface{}{
				"servo":      j.Servo,
				"actual_deg": j.ActualDeg,
				"goal_deg":   j.GoalDeg,
				"dev_deg":    j.DevDeg,
				"load":       j.Load,
			})
		}
		return map[string]interface{}{
			"mode":        "manual",
			"manual_mode": map[string]interface{}{"joints": joints},
		}, nil
	}
	return map[string]interface{}{"mode": "auto"}, nil
}

// regDecodeLE decodes a little-endian unsigned register value from its raw bytes
// (STS3215 registers are little-endian; low byte first).
func regDecodeLE(data []byte) int {
	v := 0
	for i, b := range data {
		v |= int(b) << (8 * i)
	}
	return v
}

// setTorqueLimitDiag is a diagnostic that writes the torque_limit register directly on every
// arm servo, in isolation from manual mode, so you can confirm the register actually governs
// holding torque on this firmware (e.g. value 10 should make the arm droop; 1000 restores full
// torque). Requires a "value" (0-1000). Run with manual mode OFF; it does not auto-restore —
// set it back to 1000 (or reinitialize) when done. Refuses while manual mode is active so it
// can't fight the control loop.
func (s *so101) setTorqueLimitDiag(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.RLock()
	active := s.manual != nil && s.manual.running()
	s.mu.RUnlock()
	if active {
		return nil, fmt.Errorf("exit manual mode before set_torque_limit (it would fight the control loop)")
	}
	v, ok := cmd["value"].(float64)
	if !ok {
		return nil, fmt.Errorf("set_torque_limit requires a 'value' number (0-1000)")
	}
	iv := int(v)
	if iv < 0 || iv > 1000 {
		return nil, fmt.Errorf("value must be in [0,1000], got %d", iv)
	}
	data := []byte{byte(iv & 0xFF), byte((iv >> 8) & 0xFF)}
	results := make([]interface{}, 0, len(s.armServoIDs))
	for _, id := range s.armServoIDs {
		err := s.controller.WriteServoRegister(ctx, id, "torque_limit", data)
		row := map[string]interface{}{"servo": id, "ok": err == nil}
		if err != nil {
			row["error"] = err.Error()
		}
		// Read back to confirm the write stuck.
		if rb, rerr := s.controller.ReadServoRegister(ctx, id, "torque_limit"); rerr == nil {
			row["readback"] = regDecodeLE(rb)
		}
		results = append(results, row)
	}
	return map[string]interface{}{
		"set_torque_limit": iv,
		"servos":           results,
		"note":             "diagnostic: restore full torque with value 1000 (or reinitialize) when done",
	}, nil
}

// enterManualLocked starts a manual-mode session. Caller holds s.mu.
func (s *so101) enterManualLocked(overrides map[string]interface{}) map[string]interface{} {
	// If already active, tear down first so the new parameters take effect (live re-tuning).
	// exitManualLocked restores the prior compliance registers before the fresh session
	// re-applies with the new values.
	if s.manual != nil && s.manual.running() {
		s.exitManualLocked("re-enter with new parameters")
	}
	var mm *ManualModeConfig
	if s.cfg != nil {
		mm = s.cfg.ManualMode
	}
	params, pGain, torqueLimit := resolveManualParams(mm, overrides)
	io := newControllerManualIO(s.controller, s.armServoIDs, s.calculateJointLimits())
	s.manual = newManualSession(s.cancelCtx, io, params, pGain, torqueLimit, s.logger)
	s.manual.start()
	return map[string]interface{}{"mode": "manual", "servos": s.armServoIDs}
}

// exitManualLocked tears down manual mode if active and holds the current pose.
// Caller holds s.mu. Safe to call when not active.
func (s *so101) exitManualLocked(reason string) {
	if s.manual == nil || !s.manual.running() {
		return
	}
	s.logger.Infof("manual mode exiting: %s", reason)
	s.manual.stop()
	s.manual = nil
}

func (s *so101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// The servo_* family lets a gripper on this bus drive its own servo without holding a
	// serial connection. Routed before the arm's own commands; servocmd.HandleServoCommand
	// refuses any servo this arm drives itself, so the two cannot collide.
	if command, ok := cmd["command"].(string); ok && servocmd.IsServoCommand(command) {
		return servocmd.HandleServoCommand(ctx, cmd, s.controller, s.armServoIDs)
	}

	// Handle custom commands specific to SO-101
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := s.controller.SetTorqueEnable(ctx, enable)
		return map[string]interface{}{"success": err == nil}, err

	case "ping":
		err := s.controller.Ping(ctx)
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
		positions, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
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

		s.logger.Debugf("Successfully reloaded calibration from %s", s.cfg.CalibrationFile)
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

	case "enter_manual_mode":
		s.mu.Lock()
		defer s.mu.Unlock()
		return s.enterManualLocked(cmd), nil

	case "exit_manual_mode":
		s.mu.Lock()
		defer s.mu.Unlock()
		s.exitManualLocked("exit_manual_mode command")
		return map[string]interface{}{"mode": "auto"}, nil

	case "set_torque_limit":
		return s.setTorqueLimitDiag(ctx, cmd)

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
				if acc < minAccelDegsPerSecSq || acc > maxAccelDegsPerSecSq {
					return nil, fmt.Errorf(
						"acceleration must be between %.0f and %.0f degrees/second^2, got %.1f",
						minAccelDegsPerSecSq, maxAccelDegsPerSecSq, acc)
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

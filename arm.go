package so_arm

import (
	"context"
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	commonpb "go.viam.com/api/common/v1"
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

// computeOOBPosition takes a frame and a slice of Inputs and returns the cartesian position of
// the frame after transforming it by the given inputs even if the inputs given would violate the
// Limits of the frame. This is performed statelessly without changing any data.
//
// This replaces referenceframe.ComputeOOBPosition, which was removed upstream. Callers rely on
// being able to compute a pose for out-of-bounds joint positions (e.g. reporting EndPosition
// while a joint is slightly past its calibrated limit). rdk v0.123's Frame.Transform early-returns
// at the first out-of-bounds joint, yielding a pose truncated at that joint (and everything
// downstream), so inputs are clamped into the joint limits first to always compose the full chain.
func computeOOBPosition(frame referenceframe.Frame, inputs []referenceframe.Input) (spatialmath.Pose, error) {
	if inputs == nil {
		return nil, errors.New("cannot compute position for nil joints")
	}
	if frame == nil {
		return nil, errors.New("cannot compute position for nil frame")
	}

	limits := frame.DoF()
	safe := make([]referenceframe.Input, len(inputs))
	for i, v := range inputs {
		if i < len(limits) {
			v = math.Max(limits[i].Min, math.Min(limits[i].Max, v))
		}
		safe[i] = v
	}
	return frame.Transform(safe)
}

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
	// arm/so101.urdf instead of the embedded so101.json. Requires VIAM_MODULE_ROOT
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

	if err := validateGoalCloudTolerances(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM); err != nil {
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
	goalCloud goalCloudConfig

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

func makeSO101ModelFrame(resourceName string) (referenceframe.Model, error) {
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

	return m.ParseConfig(resourceName)
}

// makeSO101ModelFrameWithEEMarker builds the SO-101 kinematics with a small placeholder
// geometry on the otherwise-empty "tool" link. so101.json's "tool" link has no geometry,
// so the 3D viewer never draws it -- and a Get3DModels mesh keyed to a frame the viewer
// does not draw is dropped. Giving "tool" a geometry makes the viewer draw the link, so
// the colored EE-frame mesh served by Get3DModels renders there. so101.json is untouched;
// this is used only when an arm's visualize_ee_frame attribute is enabled.
// eeMarkerGeometry returns the small placeholder box given to the "tool" link so the 3D
// viewer draws that frame (a Get3DModels mesh keyed to a geometry-less frame is dropped).
// The colored EE-frame mesh from Get3DModels replaces it visually. Shared by the JSON and
// URDF model builders so the two visualize_ee_frame paths use the identical placeholder.
func eeMarkerGeometry() *spatialmath.GeometryConfig {
	return &spatialmath.GeometryConfig{Type: spatialmath.BoxType, X: 10, Y: 10, Z: 10}
}

func makeSO101ModelFrameWithEEMarker(resourceName string) (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     so101ModelJson,
			Extension: "json",
		},
	}
	if err := json.Unmarshal(so101ModelJson, m); err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}
	for i := range m.Links {
		if m.Links[i].ID == so101EEFrameMeshKey {
			m.Links[i].Geometry = eeMarkerGeometry()
		}
	}
	return m.ParseConfig(resourceName)
}

// makeModelFrame builds the SO-101 kinematic model from either the embedded so101.json
// (default) or the bundled arm/so101.urdf (when cfg.UseURDF). On the JSON path, when
// visualize_ee_frame is set the "tool" link gets a placeholder geometry so the viewer
// draws the EE marker (see makeSO101ModelFrameWithEEMarker). VisualizeEEFrame's placeholder
// handling under URDF mode is deferred to a later task (frame-alignment), so it is not
// applied here.
func makeModelFrame(cfg *SO101ArmConfig, name string) (referenceframe.Model, error) {
	return makeSO101Model(cfg.UseURDF, cfg.MeshDecimationRatios, cfg.VisualizeEEFrame, name)
}

// makeSO101Model builds the SO-101 kinematic model from either the embedded so101.json
// (default) or the bundled arm/so101.urdf (when useURDF). Shared by the hardware and
// simulated arms. See makeModelFrame for the JSON/URDF and VisualizeEEFrame semantics.
func makeSO101Model(useURDF bool, ratios []float64, visualizeEE bool, name string) (referenceframe.Model, error) {
	if !useURDF {
		if visualizeEE {
			return makeSO101ModelFrameWithEEMarker(name)
		}
		return makeSO101ModelFrame(name)
	}
	return makeSO101ModelURDF(ratios, visualizeEE, name)
}

// so101.urdf carries one merged collision mesh per arm link (base/shoulder/upper_arm/lower_arm/
// wrist, 5 total, in document order). The RDK URDF parser assigns mesh_decimation_ratios per mesh
// in that order, so the default fills one ratio per mesh. 0.9 is a light default (keep ~90% of
// triangles) that trims the dense merged meshes without the sliver artifacts more aggressive
// ratios produced (see arm/gen_collision_meshes.py).
const (
	numSO101CollisionMeshes = 5
	defaultMeshDecimation   = 0.9
)

// makeSO101ModelURDF builds the SO-101 model from arm/so101.urdf. The URDF is authored so its
// raw form is already a drop-in for so101.json: so101.json's frame names (base, shoulder,
// upper_arm, lower_arm, wrist; joints 1-5) and its "tool" TCP (joint-5 output + 180deg flip),
// with only the collision geometry differing (per-link meshes vs so101.json's primitives). This
// correctness has to live in the file itself, not in an in-memory patch, because a component
// ships its kinematics to viam-server as ModelConfig().OriginalFile.Bytes -- this raw URDF --
// and in-memory edits would be lost across the module gRPC boundary (see the arm/so101.urdf
// header and TestURDFModelSurvivesSerialization). Returning the parsed URDF lets it transmit AS
// URDF, with meshes sent separately, which is a much smaller kinematics payload than embedding
// them in SVA JSON.
//
// visualize_ee_frame is the one exception: it needs a placeholder geometry on the "tool" link so
// the 3D viewer draws that frame and Get3DModels' EE-marker mesh renders there. Baking that box
// statically into the URDF would add a spurious collision volume for everyone, so instead it is
// added in memory and the model re-serialized to SVA JSON -- which, unlike the URDF path, carries
// the in-memory geometry (and the embedded meshes) across the gRPC boundary. That makes the
// transmitted kinematics larger, so it is done only when the marker is enabled.
func makeSO101ModelURDF(ratios []float64, visualizeEE bool, name string) (referenceframe.Model, error) {
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		return nil, errors.New("use_urdf is set but VIAM_MODULE_ROOT is empty")
	}
	path := filepath.Join(root, "arm", "so101.urdf")
	// The RDK URDF parser assigns mesh_decimation_ratios per collision mesh in document order
	// (base/shoulder/upper_arm/lower_arm/wrist), so an empty or short slice would leave trailing
	// meshes undecimated. When the user supplies none, default every mesh to defaultMeshDecimation.
	if len(ratios) == 0 {
		ratios = make([]float64, numSO101CollisionMeshes)
		for i := range ratios {
			ratios[i] = defaultMeshDecimation
		}
	}
	model, err := referenceframe.ParseModelXMLFile(path, name, ratios)
	if err != nil {
		return nil, fmt.Errorf("parsing URDF %q: %w", path, err)
	}
	if !visualizeEE {
		return model, nil
	}

	// visualize_ee_frame: give "tool" a placeholder box and rebuild from SVA JSON so the
	// geometry survives the gRPC boundary (a raw-URDF transmission would drop it).
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("parsing URDF %q: expected *referenceframe.SimpleModel, got %T", path, model)
	}
	mc := sm.ModelConfig()
	for i := range mc.Links {
		if mc.Links[i].ID == so101EEFrameMeshKey {
			mc.Links[i].Geometry = eeMarkerGeometry()
		}
	}
	mc.OriginalFile = nil
	jsonBytes, err := json.Marshal(mc)
	if err != nil {
		return nil, fmt.Errorf("serializing URDF model %q for visualize_ee_frame: %w", path, err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
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
	calibration, fromFile := controllerConfig.LoadCalibration(logger)
	if calibration.ShoulderPan != nil {
		logger.Debugf("Using calibration for SO-101 with shoulder_pan homing_offset: %d", calibration.ShoulderPan.HomingOffset)
	} else {
		logger.Debug("Using default calibration for SO-101")
	}

	controller, err := GetSharedControllerWithCalibration(controllerConfig, calibration, fromFile)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared SO-ARM controller: %w", err)
	}

	model, err := makeModelFrame(conf, name.Name)
	if err != nil {
		ReleaseSharedController() // Clean up on error
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
		goalCloud:    resolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
		initCtx:      ctx, // Store initialization context
	}

	logger.Debugf("SO-101 configured with speed: %.1f deg/s, acceleration: %.1f deg/s²",
		speedDegsPerSec, accelerationDegsPerSec)
	logger.Debugf("Arm controlling servo IDs: %v", arm.armServoIDs)

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

	pose, err := computeOOBPosition(s.model, inputs)
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
// orientation-agnostic behavior, and "pose_cloud" supplies a raw cloud. See goal_cloud.go.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds, which makes
// planning revert to strict six-DOF scoring and fail.
func (s *so101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	// A motion command takes the arm out of hand-guided ("manual") mode.
	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	dest, planExtra, path, err := buildMoveDestination(
		fmt.Sprintf("%v_origin", s.Name().Name), pose, s.goalCloud, extra)
	if err != nil {
		// Return the build error UNWRAPPED: path is pathInvalid, and wrapping a pose_cloud
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
	return wrapMoveErr(err, path, s.goalCloud)
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

// moveJoints commands the arm joints at speedDegsPerSec (independent-joint speed mode) and,
// when wait is true, blocks until the arm servos stop (bounded by a safety timeout).
// The caller must hold s.moveLock and manage s.isMoving.
func (s *so101) moveJoints(ctx context.Context, clampedPositions []float64, speedDegsPerSec float64, wait bool) error {
	// Read current positions BEFORE commanding so the travel/timeout estimate is accurate.
	var currentPositions []float64
	if wait {
		cp, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
		if err != nil {
			s.logger.Warnf("Failed to read positions for timeout calculation: %v", err)
		} else {
			currentPositions = cp
		}
	}

	speedSteps := degPerSecToStepsPerSec(speedDegsPerSec)
	if err := s.controller.MoveServosToPositions(ctx, s.armServoIDs, clampedPositions, speedSteps, 0); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	if !wait {
		return nil
	}

	timeout := maxMoveTimeoutMs
	if currentPositions != nil {
		maxTravelDeg := 0.0
		for i, target := range clampedPositions {
			if i < len(currentPositions) {
				d := math.Abs(target-currentPositions[i]) * 180.0 / math.Pi
				if d > maxTravelDeg {
					maxTravelDeg = d
				}
			}
		}
		timeout = moveTimeoutMs(maxTravelDeg, speedDegsPerSec)
	}
	s.logger.Debugf("moveJoints: %.1f deg/s -> %d steps/s, wait timeout %d ms", speedDegsPerSec, speedSteps, timeout)
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, timeout)
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
	s.mu.RUnlock()

	return s.moveJoints(ctx, clamped, speed, parseWaitExtra(extra))
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
	s.mu.RUnlock()

	maxVelRads := 0.0
	if options != nil {
		maxVelRads = options.MaxVelRads // MaxAccRads is intentionally ignored (speed-only)
	}
	speed := resolveSpeedDegsPerSec(maxVelRads, defaultSpeed)

	// Stream waypoints back-to-back; only wait for the arm to settle after the last one so
	// intermediate waypoints don't stutter.
	for idx, jointPositions := range positions {
		clamped, err := s.clampPositions(jointPositions)
		if err != nil {
			return err
		}
		isLast := idx == len(positions)-1
		if err := s.moveJoints(ctx, clamped, speed, isLast); err != nil {
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
	return so101ArmModels(s.cfg.VisualizeEEFrame), nil
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
	// serial connection. Routed before the arm's own commands; handleServoCommand refuses
	// any servo this arm drives itself, so the two cannot collide.
	if command, ok := cmd["command"].(string); ok && isServoCommand(command) {
		return handleServoCommand(ctx, cmd, s.controller, s.armServoIDs)
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
	s.mu.Lock()
	s.exitManualLocked("arm closing")
	s.mu.Unlock()

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

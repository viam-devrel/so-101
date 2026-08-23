package so_arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// SO101SimulatedModel is the model triplet for the hardware-free simulated SO-101 arm.
var SO101SimulatedModel = resource.NewModel("devrel", "so101", "simulated")

// defaultSimSpeedDegsPerSec is the joint travel speed used when speed_degs_per_sec is unset.
const defaultSimSpeedDegsPerSec = 90.0

// timeSimulationInterval is how often the background goroutine advances the arm's position.
const timeSimulationInterval = 10 * time.Millisecond

func init() {
	resource.RegisterComponent(arm.API, SO101SimulatedModel,
		resource.Registration[arm.Arm, *SO101SimulatedArmConfig]{
			Constructor: newSimulatedSO101,
		},
	)
}

// SO101SimulatedArmConfig configures a simulated SO-101 arm. The simulated arm needs no
// hardware: it shares the SO-101 kinematics with the devrel:so101:arm model and emulates
// joint motion in software, which makes it useful for testing configs, motion plans, and
// the 3D scene viewer without a physical robot.
type SO101SimulatedArmConfig struct {
	// SpeedDegsPerSec is how fast each joint travels toward its target, in degrees per
	// second. Defaults to defaultSimSpeedDegsPerSec when unset.
	SpeedDegsPerSec float64 `json:"speed_degs_per_sec,omitempty"`

	// Motion is the name of the motion service used to plan MoveToPosition requests.
	// Defaults to "builtin".
	Motion string `json:"motion,omitempty"`

	// SimulateTime controls whether a background goroutine advances the arm's position in
	// real time. Defaults to true. Tests set it false to drive the simulated clock
	// deterministically via updateForTime.
	SimulateTime *bool `json:"simulate_time,omitempty"`

	// VisualizeEEFrame, when true, makes Get3DModels also serve a colored XYZ
	// coordinate-frame marker at the end-effector. Defaults to false.
	VisualizeEEFrame bool `json:"visualize_ee_frame,omitempty"`

	// UseURDF sources kinematics + mesh collision geometry from the bundled
	// assets/urdf/so101.urdf instead of the embedded so101.json. Requires VIAM_MODULE_ROOT
	// (set by viam-server). Lets the simulated arm visualize/test the URDF collision
	// model and 3D meshes without a physical robot. Default false.
	UseURDF bool `json:"use_urdf,omitempty"`
	// MeshDecimationRatios is the per-collision-mesh simplification ratio in [0,1],
	// applied in URDF document order (one per arm link: base/shoulder/upper_arm/
	// lower_arm/wrist). Only values strictly in (0,1) actually decimate; lower = more
	// aggressive. Defaults to 0.9 for each of the 5 meshes when empty. Ignored unless
	// UseURDF is set.
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`

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

// Validate ensures all parts of the config are valid and declares the motion service
// dependency consumed by MoveToPosition.
func (cfg *SO101SimulatedArmConfig) Validate(path string) ([]string, []string, error) {
	if cfg.SpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("speed_degs_per_sec must not be negative, got %.1f", cfg.SpeedDegsPerSec)
	}

	for i, r := range cfg.MeshDecimationRatios {
		if math.IsNaN(r) || r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
	}

	if err := validateGoalCloudTolerances(cfg.OrientationToleranceDeg, cfg.PositionToleranceMM); err != nil {
		return nil, nil, err
	}

	motionName := cfg.Motion
	if motionName == "" {
		motionName = "builtin"
	}
	return []string{motion.Named(motionName).String()}, nil, nil
}

// simOperation tracks the state of an in-flight MoveToJointPositions request.
//
// Logical states/invariants:
//  1. Default constructed -- no operation in flight.
//  2. Operation started -> targetInputs != nil, done == false, stopped == false.
//  3. Operation successful -> done == true.
//  4. Operation stopped -> stopped == true.
type simOperation struct {
	// targetInputs is the goal joint configuration in radians.
	targetInputs []float64
	done         bool
	stopped      bool
}

func (op simOperation) isMoving() bool {
	return op.targetInputs != nil && !op.done && !op.stopped
}

// simulatedSO101 is a hardware-free SO-101 arm. It shares the SO-101 kinematics with the
// devrel:so101:arm model and interpolates joint motion over time, mirroring the behavior
// of rdk's builtin "simulated" arm.
type simulatedSO101 struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	model  referenceframe.Model

	// motion plans MoveToPosition requests. It is nil when the arm was constructed
	// without dependencies (e.g. in unit tests); MoveToPosition is then unavailable but
	// joint-level simulation still works.
	motion motion.Service

	// speed is the joint travel speed in radians per second.
	speed float64

	// visualizeEEFrame adds the colored EE coordinate-frame marker to Get3DModels.
	visualizeEEFrame bool

	// goalCloud is the resolved approach-axis tolerance pair. Set once in
	// newSimulatedSO101 and never mutated (resource.AlwaysRebuild), so it needs no mutex.
	goalCloud goalCloudConfig

	// lifetime management
	closed     atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
	workers    sync.WaitGroup

	// mu guards the fields below.
	mu          sync.Mutex
	currInputs  []float64 // current joint positions in radians, length == model DoF
	lastUpdated time.Time
	operation   simOperation
}

func newSimulatedSO101(
	ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger,
) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*SO101SimulatedArmConfig](rawConf)
	if err != nil {
		return nil, err
	}

	model, err := makeSO101Model(conf.UseURDF, conf.MeshDecimationRatios, conf.VisualizeEEFrame, rawConf.Name)
	if err != nil {
		return nil, fmt.Errorf("failed to create kinematic model: %w", err)
	}

	speedDegsPerSec := conf.SpeedDegsPerSec
	if speedDegsPerSec == 0 {
		speedDegsPerSec = defaultSimSpeedDegsPerSec
	}

	// Resolve the motion service for MoveToPosition. deps is nil in unit tests; the
	// joint-level simulation does not need it, so resolution is skipped in that case.
	var ms motion.Service
	if deps != nil {
		motionName := conf.Motion
		if motionName == "" {
			motionName = "builtin"
		}
		ms, err = motion.FromProvider(deps, motionName)
		if err != nil {
			return nil, err
		}
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	sim := &simulatedSO101{
		name:             rawConf.ResourceName(),
		logger:           logger,
		model:            model,
		motion:           ms,
		speed:            speedDegsPerSec * math.Pi / 180.0,
		visualizeEEFrame: conf.VisualizeEEFrame,
		goalCloud:        resolveGoalCloudConfig(conf.OrientationToleranceDeg, conf.PositionToleranceMM, logger),
		cancelCtx:        cancelCtx,
		cancelFunc:       cancelFunc,
		currInputs:       make([]float64, len(model.DoF())),
	}

	// SimulateTime defaults to true so a deployed arm advances on its own.
	if conf.SimulateTime == nil || *conf.SimulateTime {
		// Avoid ever letting the zero value of lastUpdated be visible, lest the first
		// movement be unpredictable.
		sim.lastUpdated = time.Now()
		sim.startTimeSimulation()
	}

	logger.Debugf("simulated SO-101 configured with speed: %.1f deg/s", speedDegsPerSec)
	return sim, nil
}

// startTimeSimulation launches a background goroutine that advances the arm's position
// against a realtime clock until the arm is closed.
func (s *simulatedSO101) startTimeSimulation() {
	s.workers.Add(1)
	go func() {
		defer s.workers.Done()
		ticker := time.NewTicker(timeSimulationInterval)
		defer ticker.Stop()
		for {
			select {
			case <-s.cancelCtx.Done():
				return
			case <-ticker.C:
				s.updateForTime(time.Now())
			}
		}
	}()
}

// updateForTime advances the simulated joint positions to the given wall-clock time. It
// is called by the background goroutine when simulate_time is true, and directly by
// tests for a deterministic clock when it is false.
func (s *simulatedSO101) updateForTime(now time.Time) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.operation.isMoving() {
		s.lastUpdated = now
		return
	}

	timeSinceLastUpdate := now.Sub(s.lastUpdated)
	s.lastUpdated = now

	// Find the maximum joint travel distance. Because all joints move at the same top
	// speed, this maps to how long the whole movement takes.
	var maxDist float64
	for jointIdx, currJointInp := range s.currInputs {
		maxDist = math.Max(maxDist, math.Abs(s.operation.targetInputs[jointIdx]-currJointInp))
	}

	const epsilon = 1e-9
	if maxDist < epsilon {
		s.operation.done = true
		return
	}

	// Scale each joint's speed so that every joint finishes its travel at the same time.
	// This matches rdk's motion-planning interpolation.
	modifiedSpeeds := make([]float64, len(s.currInputs))
	for jointIdx, currJointInp := range s.currInputs {
		diffRads := math.Abs(s.operation.targetInputs[jointIdx] - currJointInp)
		modifiedSpeeds[jointIdx] = (diffRads / maxDist) * s.speed
	}

	// anyJointStillMoving stays false only when every joint has reached its target.
	anyJointStillMoving := false
	for jointIdx, currJointInp := range s.currInputs {
		// Signed remaining distance to the target.
		diffRads := s.operation.targetInputs[jointIdx] - currJointInp

		// How far this joint could travel since the last update, capped at diffRads.
		toTravelRads := timeSinceLastUpdate.Seconds() * modifiedSpeeds[jointIdx]
		if toTravelRads > math.Abs(diffRads)-epsilon {
			// We can travel at least as far as we need to; snap to the target.
			s.currInputs[jointIdx] = s.operation.targetInputs[jointIdx]
		} else {
			if diffRads < 0 {
				// toTravelRads is always positive; flip it to travel the other way.
				toTravelRads = -toTravelRads
			}
			s.currInputs[jointIdx] = currJointInp + toTravelRads
			anyJointStillMoving = true
		}
	}

	if !anyJointStillMoving {
		s.operation.done = true
	}
}

func (s *simulatedSO101) Name() resource.Name {
	return s.name
}

// EndPosition returns the pose of the end effector at the current joint positions.
func (s *simulatedSO101) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return computeOOBPosition(s.model, inputs)
}

// MoveToPosition moves the arm's end effector to the target pose using the motion service.
//
// As with the devrel:so101:arm model, the planner is given an approach-axis cone around
// the goal orientation rather than discarding orientation via "position_only": the tool's
// pointing direction is constrained to within orientation_tolerance_deg, while roll about
// that axis is free. Callers may bypass the cone via extra; see goal_cloud.go.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds.
func (s *simulatedSO101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if s.motion == nil {
		return errors.New("MoveToPosition requires a motion service, which was not available at construction")
	}

	dest, planExtra, path, err := buildMoveDestination(
		fmt.Sprintf("%v_origin", s.name.Name), pose, s.goalCloud, extra)
	if err != nil {
		// Return the build error UNWRAPPED: path is pathInvalid, and wrapping a pose_cloud
		// parse error as a cone-planning failure would tell the caller to widen tolerances
		// they never set.
		return err
	}
	s.logger.Debugf("MoveToPosition goal cloud: %+v", dest.GoalCloud)

	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.name.Name,
		Destination:   dest,
		Extra:         planExtra,
	})
	return wrapMoveErr(err, path, s.goalCloud)
}

// MoveToJointPositions starts a move to the given joint configuration and blocks until it
// completes, the arm is stopped, or the context is canceled.
func (s *simulatedSO101) MoveToJointPositions(
	ctx context.Context, positions []referenceframe.Input, extra map[string]interface{},
) error {
	if len(positions) != len(s.model.DoF()) {
		return fmt.Errorf("expected %d joint positions for the SO-101 arm, got %d",
			len(s.model.DoF()), len(positions))
	}
	if err := arm.CheckDesiredJointPositions(ctx, s, positions); err != nil {
		return err
	}

	target := make([]float64, len(positions))
	copy(target, positions)

	s.mu.Lock()
	s.operation = simOperation{targetInputs: target}
	s.mu.Unlock()

	// MoveToJointPositions blocks until the movement completes or is canceled.
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-s.cancelCtx.Done():
			return s.cancelCtx.Err()
		default:
			s.mu.Lock()
			done, stopped := s.operation.done, s.operation.stopped
			s.mu.Unlock()

			if done {
				return nil
			}
			if stopped {
				return errors.New("stopped before reaching target")
			}
			time.Sleep(time.Millisecond)
		}
	}
}

// MoveThroughJointPositions moves the arm through each joint configuration in order.
func (s *simulatedSO101) MoveThroughJointPositions(
	ctx context.Context, positions [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{},
) error {
	for _, goal := range positions {
		if err := s.MoveToJointPositions(ctx, goal, nil); err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
	}
	return nil
}

// JointPositions returns the current joint positions in radians.
func (s *simulatedSO101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	ret := make([]referenceframe.Input, len(s.currInputs))
	copy(ret, s.currInputs)
	return ret, nil
}

func (s *simulatedSO101) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *simulatedSO101) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *simulatedSO101) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

// Stop ends any in-flight movement. The arm holds its current position.
func (s *simulatedSO101) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Only set stopped while moving, otherwise the distinction between "reached the
	// goal" and "was stopped" is lost.
	if s.operation.isMoving() {
		s.operation.stopped = true
	}
	return nil
}

func (s *simulatedSO101) IsMoving(ctx context.Context) (bool, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	return s.operation.isMoving(), nil
}

// Geometries returns the arm's geometries at the current joint positions.
func (s *simulatedSO101) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
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

// Get3DModels returns the SO-101 link meshes for the 3D scene viewer, plus a colored
// end-effector coordinate-frame marker when visualize_ee_frame is enabled.
func (s *simulatedSO101) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return so101ArmModels(s.visualizeEEFrame), nil
}

// Status returns the current status of the resource as a map of key-value pairs.
func (s *simulatedSO101) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *simulatedSO101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "get_motion_params":
		return map[string]interface{}{
			"speed_degs_per_sec": s.speed * 180.0 / math.Pi,
		}, nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd)
	}
}

func (s *simulatedSO101) Close(ctx context.Context) error {
	if s.closed.Swap(true) {
		return nil
	}
	s.cancelFunc()
	s.workers.Wait()
	return nil
}

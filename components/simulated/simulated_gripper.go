package simulated

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
)

// SO101SimulatedGripperModel is the model triplet for the hardware-free simulated SO-101 gripper.
var SO101SimulatedGripperModel = resource.NewModel("devrel", "so101", "simulated-gripper")

const (
	// gripperTravelPctPerSec is how fast the simulated jaw opens/closes.
	gripperTravelPctPerSec = 200.0
	// gripperUpdateInterval is how often the background goroutine advances the jaw.
	gripperUpdateInterval = 10 * time.Millisecond
	// jawTrajectoryHistory is how many GoToInputs batches the gripper remembers for
	// get_jaw_trajectory. Small on purpose: this is a debugging window, not a log.
	jawTrajectoryHistory = 8
)

// jawBatch is one recorded GoToInputs call.
type jawBatch struct {
	steps          [][]referenceframe.Input
	totalTravelRad float64
	offset         time.Duration // since construction; monotonic so tests stay deterministic
}

func init() {
	resource.RegisterComponent(gripper.API, SO101SimulatedGripperModel,
		resource.Registration[gripper.Gripper, *SO101SimulatedGripperConfig]{
			Constructor: newSimulatedSO101Gripper,
		},
	)
}

// SO101SimulatedGripperConfig configures a hardware-free simulated SO-101 gripper. It
// needs no serial port: it emulates open/close in software and serves the gripper
// meshes for the 3D viewer, useful for testing without a physical robot.
type SO101SimulatedGripperConfig struct {
	// GripperType selects which meshes are served: "follower" (moving jaw, the
	// default) or "leader" (thumb-loop handle + trigger).
	GripperType string `json:"gripper_type,omitempty"`

	// MeshDetail selects the gripper mesh resolution: "low" (decimated, the
	// default) or "high" (full resolution).
	MeshDetail string `json:"mesh_detail,omitempty"`

	// ArticulatedJaw gives the gripper a 1-DoF revolute jaw joint in its kinematic model and
	// makes it InputEnabled, so the motion planner sees the jaw as a variable it may drive.
	// Off by default: the jaw does not affect the TCP, so the planner gets a degree of freedom
	// that changes only collision geometry. See docs/simulated.md.
	ArticulatedJaw bool `json:"articulated_jaw,omitempty"`
}

// Validate ensures the config is valid.
func (cfg *SO101SimulatedGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.GripperType != "" && cfg.GripperType != geometry.LeaderGripper && cfg.GripperType != geometry.FollowerGripper {
		return nil, nil, fmt.Errorf("gripper_type must be %q or %q, got %q",
			geometry.LeaderGripper, geometry.FollowerGripper, cfg.GripperType)
	}
	if cfg.MeshDetail != "" && cfg.MeshDetail != geometry.HighDetail && cfg.MeshDetail != geometry.LowDetail {
		return nil, nil, fmt.Errorf("mesh_detail must be %q or %q, got %q",
			geometry.HighDetail, geometry.LowDetail, cfg.MeshDetail)
	}
	return nil, nil, nil
}

// simulatedSO101Gripper is a hardware-free SO-101 gripper. Open/Grab move the jaw toward
// a target opening; a background goroutine interpolates the opening over time so the
// jaw/trigger visibly animates in the 3D viewer, mirroring the simulated arm.
type simulatedSO101Gripper struct {
	resource.AlwaysRebuild

	name           resource.Name
	gripperType    string
	meshDetail     string
	articulatedJaw bool
	model          referenceframe.Model
	logger         logging.Logger

	// lifetime management
	closed     atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
	workers    sync.WaitGroup

	// mu guards the fields below. currentPct/targetPct are 0 (closed) to 100 (open).
	mu          sync.Mutex
	currentPct  float64
	targetPct   float64
	lastUpdated time.Time
	// stopped mirrors the arm's simOperation.stopped (simulated.go). Without it, Stop()'s
	// targetPct = currentPct makes an in-flight awaitArrival see "arrived" rather than
	// "stopped", so GoToInputs would silently carry on to the next batch step.
	stopped bool

	// createdAt anchors jawBatch.offset to a monotonic clock so tests stay deterministic.
	createdAt time.Time
	// jawBatches is a ring of the most recent GoToInputs calls, for get_jaw_trajectory.
	jawBatches []jawBatch
}

func newSimulatedSO101Gripper(
	ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger,
) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*SO101SimulatedGripperConfig](rawConf)
	if err != nil {
		return nil, err
	}

	gripperType := conf.GripperType
	if gripperType == "" {
		gripperType = geometry.FollowerGripper
	}

	meshDetail := conf.MeshDetail
	if meshDetail == "" {
		meshDetail = geometry.LowDetail
	}

	model, err := geometry.BuildGripperModel(gripperType, meshDetail, rawConf.ResourceName().ShortName(), conf.ArticulatedJaw)
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	g := &simulatedSO101Gripper{
		name:           rawConf.ResourceName(),
		gripperType:    gripperType,
		meshDetail:     meshDetail,
		articulatedJaw: conf.ArticulatedJaw,
		model:          model,
		logger:         logger,
		cancelCtx:      cancelCtx,
		cancelFunc:     cancelFunc,
		lastUpdated:    time.Now(),
		createdAt:      time.Now(),
	}
	g.startMotionSimulation()
	return g, nil
}

// startMotionSimulation launches the background goroutine that interpolates the jaw.
func (g *simulatedSO101Gripper) startMotionSimulation() {
	g.workers.Add(1)
	go func() {
		defer g.workers.Done()
		ticker := time.NewTicker(gripperUpdateInterval)
		defer ticker.Stop()
		for {
			select {
			case <-g.cancelCtx.Done():
				return
			case <-ticker.C:
				g.updateForTime(time.Now())
			}
		}
	}()
}

// updateForTime advances the current opening toward the target at gripperTravelPctPerSec.
func (g *simulatedSO101Gripper) updateForTime(now time.Time) {
	g.mu.Lock()
	defer g.mu.Unlock()

	elapsed := now.Sub(g.lastUpdated).Seconds()
	g.lastUpdated = now
	if g.currentPct == g.targetPct {
		return
	}

	step := gripperTravelPctPerSec * elapsed
	if math.Abs(g.targetPct-g.currentPct) <= step {
		g.currentPct = g.targetPct
	} else if g.targetPct > g.currentPct {
		g.currentPct += step
	} else {
		g.currentPct -= step
	}
}

// setTarget sets the target opening without waiting. set_position uses this: the teleop loop
// issues one every tick and must not block.
func (g *simulatedSO101Gripper) setTarget(pct float64) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.targetPct = pct
}

// awaitArrival blocks until the jaw reaches its target, is stopped, or ctx is done. stopped is
// checked before the arrival comparison: Stop() sets targetPct = currentPct, which would
// otherwise make a stop indistinguishable from arrival.
func (g *simulatedSO101Gripper) awaitArrival(ctx context.Context) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-g.cancelCtx.Done():
			return g.cancelCtx.Err()
		default:
			g.mu.Lock()
			stopped := g.stopped
			done := g.currentPct == g.targetPct
			g.mu.Unlock()
			if stopped {
				return errors.New("stopped before reaching target")
			}
			if done {
				return nil
			}
			time.Sleep(time.Millisecond)
		}
	}
}

// clearStopped resets the stop flag at the start of a newly commanded move (Open, Grab, or a
// GoToInputs batch), so a Stop() from a previous move does not leak into this one.
func (g *simulatedSO101Gripper) clearStopped() {
	g.mu.Lock()
	g.stopped = false
	g.mu.Unlock()
}

// moveTo sets the target and blocks until the jaw reaches it. Open and Grab use it directly;
// GoToInputs clears stopped itself once per batch and calls setTarget/awaitArrival separately
// (see GoToInputs) so intermediate steps aren't each treated as a fresh move.
func (g *simulatedSO101Gripper) moveTo(ctx context.Context, pct float64) error {
	g.clearStopped()
	g.setTarget(pct)
	return g.awaitArrival(ctx)
}

func (g *simulatedSO101Gripper) Name() resource.Name {
	return g.name
}

// Open opens the simulated gripper, interpolating over time.
func (g *simulatedSO101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	return g.moveTo(ctx, 100)
}

// Grab closes the simulated gripper, interpolating over time. It always reports false: a
// simulated gripper never actually grasps an object.
func (g *simulatedSO101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, g.moveTo(ctx, 0)
}

// Stop halts the jaw where it currently is.
func (g *simulatedSO101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	// Only set stopped while moving, otherwise the distinction between "reached the target"
	// and "was stopped" is lost for whoever is awaiting arrival (mirrors simOperation in
	// simulated.go).
	if g.currentPct != g.targetPct {
		g.stopped = true
	}
	g.targetPct = g.currentPct
	return nil
}

// IsMoving reports whether the jaw is mid-travel.
func (g *simulatedSO101Gripper) IsMoving(ctx context.Context) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.currentPct != g.targetPct, nil
}

// IsHoldingSomething always reports not-holding: a simulated gripper grasps nothing.
func (g *simulatedSO101Gripper) IsHoldingSomething(
	ctx context.Context, extra map[string]interface{},
) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, nil
}

// Geometries serves the gripper meshes, with the moving part posed by the current
// opening so the jaw/trigger animates in the 3D viewer.
func (g *simulatedSO101Gripper) Geometries(
	ctx context.Context, extra map[string]interface{},
) ([]spatialmath.Geometry, error) {
	g.mu.Lock()
	pct := g.currentPct
	g.mu.Unlock()
	return geometry.BuildGripperMeshes(g.gripperType, g.meshDetail, geometry.JawRadiansFromPct(pct))
}

// Status returns the current status of the resource as a map of key-value pairs.
func (g *simulatedSO101Gripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

// DoCommand supports set_position (0-100%, interpolated) and get_position.
func (g *simulatedSO101Gripper) DoCommand(
	ctx context.Context, cmd map[string]interface{},
) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "set_position":
		pct, ok := cmd["percentage"].(float64)
		if !ok {
			return nil, fmt.Errorf("set_position requires a numeric 'percentage'")
		}
		clamped := math.Max(0, math.Min(100, pct))
		g.setTarget(clamped)
		return map[string]interface{}{"target_percentage": clamped}, nil
	case "get_position":
		g.mu.Lock()
		defer g.mu.Unlock()
		return map[string]interface{}{
			"position_percentage": g.currentPct,
			"target_percentage":   g.targetPct,
			"jaw_angle_rad":       geometry.JawRadiansFromPct(g.currentPct),
			"dof":                 len(g.model.DoF()),
		}, nil
	case "get_jaw_trajectory":
		g.mu.Lock()
		defer g.mu.Unlock()
		out := make([]map[string]interface{}, 0, len(g.jawBatches))
		for _, b := range g.jawBatches {
			steps := make([][]float64, len(b.steps))
			for i, s := range b.steps {
				steps[i] = append([]float64(nil), s...)
			}
			out = append(out, map[string]interface{}{
				"step_count":       len(b.steps),
				"total_travel_rad": b.totalTravelRad,
				"offset_ms":        float64(b.offset.Milliseconds()),
				"steps":            steps,
			})
		}
		return map[string]interface{}{"batches": out}, nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// Kinematics returns the gripper's static model, whose leaf frame is the TCP between the jaw
// tips. See geometry.BuildGripperModel for why the gripper needs one at all.
func (g *simulatedSO101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

// jawLimitEpsilon tolerates a planner-issued input that lands just outside a joint limit due to
// float rounding (e.g. left-associative (range*i)/steps in internal/geometry's angle sweep,
// which overshoots by ~2.22e-16 -- one ULP -- at i==steps). rdk's frame system rejects an input
// outside its limit by even that 1 ULP.
//
// 1e-6 is deliberately far wider than a ULP (about 4.5e9 ULP at GripperJointMax) -- it exists to
// absorb ordinary float rounding, not to approximate machine precision, so do not "tighten" it
// toward a literal ULP. What actually bounds the jaw is geometry.JawPctFromRadians's saturation
// to 0/100 on conversion, not this epsilon's width: even a value at the edge of the tolerated
// band converts to a fully-saturated percentage.
const jawLimitEpsilon = 1e-6

// CurrentInputs reports the jaw angle in radians. It is an error rather than a zero value when
// articulated_jaw is off: robot/framesystem only asks components whose model has DoF, so being
// asked at all while static means something is inconsistent.
func (g *simulatedSO101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	if !g.articulatedJaw {
		return nil, errors.ErrUnsupported
	}
	g.mu.Lock()
	pct := g.currentPct
	g.mu.Unlock()
	return []referenceframe.Input{geometry.JawRadiansFromPct(pct)}, nil
}

// recordJawTrajectory logs and remembers one GoToInputs batch, for get_jaw_trajectory. Called
// after validation passes but before any movement, so a rejected batch is never recorded.
func (g *simulatedSO101Gripper) recordJawTrajectory(steps [][]referenceframe.Input) {
	if len(steps) == 0 {
		return
	}

	g.mu.Lock()
	currentRad := geometry.JawRadiansFromPct(g.currentPct)
	g.mu.Unlock()
	current := []referenceframe.Input{currentRad}

	totalTravel := 0.0
	prev := currentRad
	for _, step := range steps {
		totalTravel += math.Abs(step[0] - prev)
		prev = step[0]
	}
	linf := referenceframe.InputsLinfDistance(current, steps[len(steps)-1])

	g.logger.Infof(
		"jaw trajectory: %d steps, theta %.4f -> %.4f rad, total travel %.4f rad, Linf from current %.4f rad",
		len(steps), steps[0][0], steps[len(steps)-1][0], totalTravel, linf)

	g.mu.Lock()
	g.jawBatches = append(g.jawBatches, jawBatch{
		steps:          steps,
		totalTravelRad: totalTravel,
		offset:         time.Since(g.createdAt),
	})
	if len(g.jawBatches) > jawTrajectoryHistory {
		g.jawBatches = g.jawBatches[len(g.jawBatches)-jawTrajectoryHistory:]
	}
	g.mu.Unlock()
}

// GoToInputs drives the jaw through each step in turn. The motion service batches consecutive
// trajectory steps for one component into a single variadic call, so a batch is the normal case.
// Every step is validated before any of them is applied: a batch that fails halfway would
// otherwise leave the jaw somewhere the planner did not intend.
//
// Only the final step's arrival is awaited. Intermediate targets are waypoints the interpolator
// passes through on the way to the last one; awaiting each of them individually would cost a
// full gripperUpdateInterval tick per step for no benefit (a 100-step trajectory of negligible
// per-step motion would otherwise cost >=1s of wall time).
func (g *simulatedSO101Gripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	if !g.articulatedJaw {
		return errors.ErrUnsupported
	}
	limits := g.model.DoF()
	if len(limits) != 1 {
		// Unreachable while articulatedJaw implies exactly 1 DoF, but the len(step) arity check
		// below only protects against a mismatched step -- it does not protect indexing
		// limits[0] when limits itself is empty (0 != 0 passes the arity check).
		return fmt.Errorf("gripper jaw model has %d DoF, want exactly 1", len(limits))
	}
	jawLimits := limits[0]
	for i, step := range inputSteps {
		if len(step) != 1 {
			return fmt.Errorf("step %d: got %d inputs, the jaw takes 1", i, len(step))
		}
		if step[0] < jawLimits.Min-jawLimitEpsilon || step[0] > jawLimits.Max+jawLimitEpsilon {
			return fmt.Errorf("step %d: jaw angle %.6f rad is outside [%.6f, %.6f]",
				i, step[0], jawLimits.Min, jawLimits.Max)
		}
	}

	g.recordJawTrajectory(inputSteps)

	g.clearStopped()
	for i, step := range inputSteps {
		g.setTarget(geometry.JawPctFromRadians(step[0]))
		if i == len(inputSteps)-1 {
			return g.awaitArrival(ctx)
		}
		g.mu.Lock()
		stopped := g.stopped
		g.mu.Unlock()
		if stopped {
			return errors.New("stopped before reaching target")
		}
	}
	return nil
}

func (g *simulatedSO101Gripper) Close(ctx context.Context) error {
	if g.closed.Swap(true) {
		return nil
	}
	g.cancelFunc()
	g.workers.Wait()
	return nil
}

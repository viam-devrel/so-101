package so_arm

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
)

// SO101SimulatedGripperModel is the model triplet for the hardware-free simulated SO-101 gripper.
var SO101SimulatedGripperModel = resource.NewModel("devrel", "so101", "simulated-gripper")

const (
	// gripperTravelPctPerSec is how fast the simulated jaw opens/closes.
	gripperTravelPctPerSec = 200.0
	// gripperUpdateInterval is how often the background goroutine advances the jaw.
	gripperUpdateInterval = 10 * time.Millisecond
)

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
}

// Validate ensures the config is valid.
func (cfg *SO101SimulatedGripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.GripperType != "" && cfg.GripperType != leaderGripper && cfg.GripperType != followerGripper {
		return nil, nil, fmt.Errorf("gripper_type must be %q or %q, got %q",
			leaderGripper, followerGripper, cfg.GripperType)
	}
	return nil, nil, nil
}

// simulatedSO101Gripper is a hardware-free SO-101 gripper. Open/Grab move the jaw toward
// a target opening; a background goroutine interpolates the opening over time so the
// jaw/trigger visibly animates in the 3D viewer, mirroring the simulated arm.
type simulatedSO101Gripper struct {
	resource.AlwaysRebuild

	name        resource.Name
	gripperType string

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
		gripperType = followerGripper
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	g := &simulatedSO101Gripper{
		name:        rawConf.ResourceName(),
		gripperType: gripperType,
		cancelCtx:   cancelCtx,
		cancelFunc:  cancelFunc,
		lastUpdated: time.Now(),
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

// moveTo sets the target opening and blocks until the jaw reaches it, the arm is closed,
// or the context is canceled.
func (g *simulatedSO101Gripper) moveTo(ctx context.Context, pct float64) error {
	g.mu.Lock()
	g.targetPct = pct
	g.mu.Unlock()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-g.cancelCtx.Done():
			return g.cancelCtx.Err()
		default:
			g.mu.Lock()
			done := g.currentPct == g.targetPct
			g.mu.Unlock()
			if done {
				return nil
			}
			time.Sleep(time.Millisecond)
		}
	}
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
	pct := g.currentPct / 100.0
	g.mu.Unlock()
	jawAngle := gripperJointMin + pct*(gripperJointMax-gripperJointMin)
	return buildGripperMeshes(g.gripperType, jawAngle)
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
		g.mu.Lock()
		g.targetPct = clamped
		g.mu.Unlock()
		return map[string]interface{}{"target_percentage": clamped}, nil
	case "get_position":
		g.mu.Lock()
		defer g.mu.Unlock()
		return map[string]interface{}{
			"position_percentage": g.currentPct,
			"target_percentage":   g.targetPct,
		}, nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// Kinematics is unsupported: the gripper has no kinematic model.
func (g *simulatedSO101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.ErrUnsupported
}

func (g *simulatedSO101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *simulatedSO101Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *simulatedSO101Gripper) Close(ctx context.Context) error {
	if g.closed.Swap(true) {
		return nil
	}
	g.cancelFunc()
	g.workers.Wait()
	return nil
}

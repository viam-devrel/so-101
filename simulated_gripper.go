package so_arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// SO101SimulatedGripperModel is the model triplet for the hardware-free simulated SO-101 gripper.
var SO101SimulatedGripperModel = resource.NewModel("devrel", "so101", "simulated-gripper")

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

// simulatedSO101Gripper is a hardware-free SO-101 gripper. Open/Grab move an in-memory
// opening percentage; Geometries serves the gripper meshes with the moving part posed
// by that percentage so the jaw/trigger articulates in the 3D viewer.
type simulatedSO101Gripper struct {
	resource.AlwaysRebuild

	name        resource.Name
	gripperType string

	// mu guards openPct: 0 = fully closed, 100 = fully open.
	mu      sync.Mutex
	openPct float64
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

	return &simulatedSO101Gripper{
		name:        rawConf.ResourceName(),
		gripperType: gripperType,
	}, nil
}

func (g *simulatedSO101Gripper) Name() resource.Name {
	return g.name
}

// Open fully opens the simulated gripper.
func (g *simulatedSO101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.openPct = 100
	return nil
}

// Grab fully closes the simulated gripper. It always reports false: a simulated gripper
// never actually grasps an object.
func (g *simulatedSO101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.openPct = 0
	return false, nil
}

// Stop is a no-op: Open/Grab apply instantly, so there is never a move to interrupt.
func (g *simulatedSO101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	return nil
}

// IsMoving always reports false: Open/Grab apply instantly.
func (g *simulatedSO101Gripper) IsMoving(ctx context.Context) (bool, error) {
	return false, nil
}

// IsHoldingSomething always reports not-holding: a simulated gripper grasps nothing.
func (g *simulatedSO101Gripper) IsHoldingSomething(
	ctx context.Context, extra map[string]interface{},
) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, nil
}

// Geometries serves the gripper meshes, with the moving part posed by the current
// opening percentage.
func (g *simulatedSO101Gripper) Geometries(
	ctx context.Context, extra map[string]interface{},
) ([]spatialmath.Geometry, error) {
	g.mu.Lock()
	pct := g.openPct / 100.0
	g.mu.Unlock()
	jawAngle := gripperJointMin + pct*(gripperJointMax-gripperJointMin)
	return buildGripperMeshes(g.gripperType, jawAngle)
}

// DoCommand supports set_position (0-100%) and get_position for posing the simulated jaw.
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
		g.openPct = clamped
		g.mu.Unlock()
		return map[string]interface{}{"position_percentage": clamped}, nil
	case "get_position":
		g.mu.Lock()
		defer g.mu.Unlock()
		return map[string]interface{}{"position_percentage": g.openPct}, nil
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
	return nil
}

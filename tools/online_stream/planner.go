//go:build nlopt

// Command online_stream is built only with -tags nlopt: armplanning's IK links the system
// nlopt library through cgo, which the module binary and CI must not start to do.
package main

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/planning"
)

// planner plans in-process from ANY start configuration; the motion service's `plan`
// DoCommand always seeds from the arm's current pose, which a mid-stream replan cannot use.
type planner struct {
	fs      *referenceframe.FrameSystem
	armName string
	cloud   planning.GoalCloudConfig
	timeout time.Duration
	logger  logging.Logger
}

// newPlanner builds the frame system from FrameSystemConfig's parts.
func newPlanner(parts []*referenceframe.FrameSystemPart, armName string, cloud planning.GoalCloudConfig,
	timeout time.Duration, logger logging.Logger,
) (*planner, error) {
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	if err != nil {
		return nil, fmt.Errorf("frame system: %w", err)
	}
	if fs.Frame(armName) == nil {
		return nil, fmt.Errorf("frame system has no frame %q: does the arm have a frame configured?", armName)
	}
	return &planner{fs: fs, armName: armName, cloud: cloud, timeout: timeout, logger: logger}, nil
}

// plan solves from `from` (radians) to pose (in <arm>_origin) around obstacles (same
// frame). Row 0 of the result is `from`. latency is planning only.
func (p *planner) plan(ctx context.Context, from []float64, pose spatialmath.Pose, obstacles []spatialmath.Geometry,
) ([][]float64, time.Duration, error) {
	started := time.Now()
	origin := p.armName + "_origin"
	// Not RobotClient.CurrentInputs: it fails on this module's gripper (ErrUnsupported).
	// Zero inputs are correct for every other frame here (the gripper model is 0-DoF).
	inputs := referenceframe.NewZeroInputs(p.fs)
	inputs[p.armName] = from

	dest, _, _, err := planning.BuildMoveDestination(origin, pose, p.cloud, nil)
	if err != nil {
		return nil, 0, err
	}
	// PlanRequest goals are keyed by frame and expressed in world; Transform keeps GoalCloud.
	tf, err := p.fs.Transform(inputs.ToLinearInputs(), dest, referenceframe.World)
	if err != nil {
		return nil, 0, fmt.Errorf("goal to world: %w", err)
	}
	worldDest, ok := tf.(*referenceframe.PoseInFrame)
	if !ok {
		return nil, 0, fmt.Errorf("goal to world: got %T", tf)
	}

	opts := armplanning.NewBasicPlannerOptions()
	opts.Timeout = p.timeout.Seconds()
	req := &armplanning.PlanRequest{
		FrameSystem:    p.fs,
		Goals:          []*armplanning.PlanState{armplanning.NewPlanState(referenceframe.FrameSystemPoses{p.armName: worldDest}, nil)},
		StartState:     armplanning.NewPlanState(nil, inputs),
		PlannerOptions: opts,
	}
	if len(obstacles) > 0 {
		// No WorldState field on PlanRequest: flatten to world as the motion service does.
		ws, err := referenceframe.NewWorldState(
			[]*referenceframe.GeometriesInFrame{referenceframe.NewGeometriesInFrame(origin, obstacles)}, nil)
		if err != nil {
			return nil, 0, fmt.Errorf("world state: %w", err)
		}
		if req.ObstaclesInWorldFrame, err = ws.ObstaclesInWorldFrame(p.fs, inputs); err != nil {
			return nil, 0, fmt.Errorf("obstacles to world: %w", err)
		}
	}

	plan, _, err := armplanning.PlanMotion(ctx, p.logger, req)
	if err != nil {
		return nil, 0, fmt.Errorf("plan: %w", err)
	}
	wps, err := plan.Trajectory().GetFrameInputs(p.armName)
	if err != nil {
		return nil, 0, err
	}
	return wps, time.Since(started), nil
}

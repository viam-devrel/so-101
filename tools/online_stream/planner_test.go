//go:build nlopt

package main

import (
	"context"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
	"so_arm/internal/planning"
)

// testPlanner builds a frame system holding one so101 arm named "arm" at world origin, the
// shape FrameSystemConfig reports for a machine whose arm has a frame.
func testPlanner(t *testing.T) (*planner, referenceframe.Model) {
	t.Helper()
	model, err := geometry.ArmModelJSON("arm")
	require.NoError(t, err)
	parts := []*referenceframe.FrameSystemPart{{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil),
		ModelFrame:  model,
	}}
	logger := logging.NewTestLogger(t)
	p, err := newPlanner(parts, "arm", planning.ResolveGoalCloudConfig(0, 0, logger), 10*time.Second, logger)
	require.NoError(t, err)
	return p, model
}

func TestNewPlannerRejectsAMissingArmFrame(t *testing.T) {
	model, err := geometry.ArmModelJSON("other")
	require.NoError(t, err)
	parts := []*referenceframe.FrameSystemPart{{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "other", nil),
		ModelFrame:  model,
	}}
	_, err = newPlanner(parts, "arm", planning.GoalCloudConfig{}, time.Second, logging.NewTestLogger(t))
	assert.ErrorContains(t, err, `no frame "arm"`)
}

func TestPlanStartsAtFromAndEndsNearTheGoal(t *testing.T) {
	p, model := testPlanner(t)
	from := []float64{0, 0, 0, 0, 0}
	// A reachable goal by construction: the FK of another configuration.
	target := []float64{0.3, -0.4, 0.5, 0.2, 0.1}
	goalPose, err := model.Transform(target)
	require.NoError(t, err)

	wps, latency, err := p.plan(context.Background(), from, goalPose, nil)
	require.NoError(t, err)
	require.GreaterOrEqual(t, len(wps), 2)
	assert.Equal(t, from, wps[0], "row 0 is the start configuration")
	assert.Greater(t, latency, time.Duration(0))
	end, err := model.Transform(wps[len(wps)-1])
	require.NoError(t, err)
	assert.InDelta(t, 0, end.Point().Sub(goalPose.Point()).Norm(), 10, "TCP within 10mm of the goal (mm)")
}

func TestPlanAcceptsObstaclesInTheArmOriginFrame(t *testing.T) {
	p, model := testPlanner(t)
	goalPose, err := model.Transform([]float64{0.3, -0.4, 0.5, 0.2, 0.1})
	require.NoError(t, err)
	// Far from the arm: exercises the WorldState flattening without changing the answer.
	far, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 2000}), r3.Vector{X: 50, Y: 50, Z: 50}, "far")
	require.NoError(t, err)

	wps, _, err := p.plan(context.Background(), []float64{0, 0, 0, 0, 0}, goalPose, []spatialmath.Geometry{far})
	require.NoError(t, err)
	assert.GreaterOrEqual(t, len(wps), 2)
}

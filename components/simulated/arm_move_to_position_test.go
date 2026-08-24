package simulated

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"

	"so_arm/internal/planning"
)

// testGoal is a tool pointing straight down, a typical SO-101 grasp pose. Mirrors
// internal/planning's own testGoal helper (moved there with goal_cloud.go); this copy
// exists because this file constructs the simulatedSO101 struct directly and so cannot
// move with it. components/arm/move_to_position_test.go carries the equivalent copy for
// the hardware so101 tests.
func testGoal() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 300, Y: 0, Z: 200},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0},
	)
}

// captureMotion returns an injected motion service that records the last MoveReq.
func captureMotion(got *motion.MoveReq) *injectmotion.MotionService {
	ms := injectmotion.NewMotionService("builtin")
	ms.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
		*got = req
		return true, nil
	}
	return ms
}

func TestSimulatedArmMoveToPositionSendsGoalCloud(t *testing.T) {
	var got motion.MoveReq
	s := &simulatedSO101{
		name:      arm.Named("simarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: planning.ResolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, s.MoveToPosition(context.Background(), testGoal(), nil))

	assert.Equal(t, "simarm_origin", got.Destination.Parent())
	require.NotNil(t, got.Destination.GoalCloud)
	assert.Equal(t, 180.0, got.Destination.GoalCloud.Theta)
}

func TestSimulatedArmMoveToPositionRequiresMotionService(t *testing.T) {
	s := &simulatedSO101{name: arm.Named("simarm"), logger: logging.NewTestLogger(t)}
	err := s.MoveToPosition(context.Background(), testGoal(), nil)
	// Pin the guard's identity, not merely "an error": without asserting the message,
	// removing the guard is still caught, but only as a SIGSEGV that panics the whole test
	// binary -- a failure that names this test while explaining nothing.
	require.Error(t, err, "the nil-motion guard must still fire")
	assert.ErrorContains(t, err, "requires a motion service")
}

// simArmWithTolerances builds a simulated arm through the REAL constructor (nil deps, as
// simulated_test.go's newTestSimArm does; simulated clock off so no goroutine starts), so
// a test can observe what newSimulatedSO101 itself resolves into goalCloud.
func simArmWithTolerances(t *testing.T, tolDeg, posTolMM float64) *simulatedSO101 {
	t.Helper()
	simulateTime := false
	conf := resource.Config{
		Name:  "simarm",
		API:   arm.API,
		Model: SO101SimulatedModel,
		ConvertedAttributes: &SO101SimulatedArmConfig{
			SimulateTime:            &simulateTime,
			OrientationToleranceDeg: tolDeg,
			PositionToleranceMM:     posTolMM,
		},
	}
	// deps is nil: resolving goalCloud does not need the motion service.
	a, err := newSimulatedSO101(context.Background(), nil, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return a.(*simulatedSO101)
}

// TestSimulatedConstructorWiresGoalCloud guards the one thing the struct-literal tests
// above cannot see: that the CONSTRUCTOR resolves the config into goalCloud. Dropping that
// line leaves goalCloud zero-valued -- X/Y/Z/OZ all 0 -- which per internal/planning is a cloud
// no IK solution realistically satisfies, so planning reverts to strict 6-DOF scoring and
// every move fails silently, with the construction-time warnings dead. The struct-literal
// tests set the field themselves and would stay green through all of that.
func TestSimulatedConstructorWiresGoalCloud(t *testing.T) {
	// Defaults: catches the line being dropped entirely.
	sim := simArmWithTolerances(t, 0, 0)
	assert.Equal(t, planning.GoalCloudConfig{OrientationToleranceDeg: 30, PositionToleranceMM: 1},
		sim.goalCloud, "the constructor must resolve defaults into goalCloud")

	// Explicit values: catches a line that resolves but ignores the config.
	sim = simArmWithTolerances(t, 12.5, 0.25)
	assert.Equal(t, planning.GoalCloudConfig{OrientationToleranceDeg: 12.5, PositionToleranceMM: 0.25},
		sim.goalCloud, "explicit config must reach goalCloud unchanged")
}

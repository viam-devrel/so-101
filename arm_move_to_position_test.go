package so_arm

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

// captureMotion returns an injected motion service that records the last MoveReq.
func captureMotion(got *motion.MoveReq) *injectmotion.MotionService {
	ms := injectmotion.NewMotionService("builtin")
	ms.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
		*got = req
		return true, nil
	}
	return ms
}

// failingMotion returns an injected motion service whose Move always fails, so tests can
// exercise wrapMoveErr -- the error message is this design's entire UX, since a failed
// plan deliberately has no fallback.
func failingMotion(err error) *injectmotion.MotionService {
	ms := injectmotion.NewMotionService("builtin")
	ms.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
		return false, err
	}
	return ms
}

func TestHardwareArmMoveToPositionSendsGoalCloud(t *testing.T) {
	var got motion.MoveReq
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, a.MoveToPosition(context.Background(), testGoal(), nil))

	assert.Equal(t, "myarm", got.ComponentName)
	assert.Equal(t, "myarm_origin", got.Destination.Parent())
	require.NotNil(t, got.Destination.GoalCloud, "the cone must ship to the planner")
	assert.Equal(t, 180.0, got.Destination.GoalCloud.Theta)
	assert.NotContains(t, got.Extra, "goal_metric_type", "the cone replaces position_only")
}

func TestHardwareArmMoveToPositionHonorsMetricTypeOverride(t *testing.T) {
	var got motion.MoveReq
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	require.NoError(t, a.MoveToPosition(context.Background(), testGoal(),
		map[string]interface{}{"goal_metric_type": "position_only"}))

	assert.Nil(t, got.Destination.GoalCloud, "no cloud when the caller picks the metric")
	assert.Equal(t, "position_only", got.Extra["goal_metric_type"])
}

// TestHardwareArmMoveToPositionWrapsConeFailure drives a REAL failing Move down the cone
// path (no extra) and pins wrapMoveErr's cone-branch message. This is the design's whole
// UX for a failed plan -- there is deliberately no fallback -- so the message must name
// BOTH tolerances (either can cause the failure), the position_only remedy, and the
// silent-ignore version hint.
func TestHardwareArmMoveToPositionWrapsConeFailure(t *testing.T) {
	sentinel := errors.New("boom")
	cfg := resolveGoalCloudConfig(0, 0, nil)
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    failingMotion(sentinel),
		goalCloud: cfg,
	}

	err := a.MoveToPosition(context.Background(), testGoal(), nil)
	require.Error(t, err)
	assert.ErrorIs(t, err, sentinel, "the underlying error must still be reachable via errors.Is")
	assert.ErrorContains(t, err, fmt.Sprintf("orientation_tolerance_deg=%g", cfg.OrientationToleranceDeg))
	assert.ErrorContains(t, err, fmt.Sprintf("position_tolerance_mm=%g", cfg.PositionToleranceMM),
		"both tolerances must be named -- either can cause the failure")
	assert.ErrorContains(t, err, "position_only", "the remedy must be named")
	assert.ErrorContains(t, err, "0.127.0", "the silent-ignore version hint must be named")
}

// TestHardwareArmMoveToPositionWrapsRawCloudFailure drives a REAL failing Move down the
// raw-cloud path (caller-supplied pose_cloud) and pins wrapMoveErr's raw-cloud branch. The
// important assertion is the NEGATIVE one: this path must NOT name
// orientation_tolerance_deg, since the config tolerance had no bearing on a caller-supplied
// cloud's failure and naming it would misdirect. This is also what catches a `case`
// swapped to emit the cone wording on the wrong path.
func TestHardwareArmMoveToPositionWrapsRawCloudFailure(t *testing.T) {
	sentinel := errors.New("boom")
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    failingMotion(sentinel),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	err := a.MoveToPosition(context.Background(), testGoal(),
		map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}})
	require.Error(t, err)
	assert.ErrorIs(t, err, sentinel)
	assert.ErrorContains(t, err, "pose_cloud")
	assert.NotContains(t, err.Error(), "orientation_tolerance_deg",
		"the config tolerance had no bearing on a caller-supplied cloud's failure")
}

// TestHardwareArmMoveToPositionMetricTypeFailureUnwrapped drives a REAL failing Move down
// the metric-type path (caller-supplied goal_metric_type; no cloud sent) and pins that
// wrapMoveErr passes the error through EXACTLY unwrapped: the caller chose the metric, so
// cone wording would just tell them to do what they already did.
func TestHardwareArmMoveToPositionMetricTypeFailureUnwrapped(t *testing.T) {
	sentinel := errors.New("boom")
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    failingMotion(sentinel),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}

	err := a.MoveToPosition(context.Background(), testGoal(),
		map[string]interface{}{"goal_metric_type": "position_only"})
	assert.Equal(t, sentinel, err, "the metric-type path must return the error exactly unwrapped")
}

func TestSimulatedArmMoveToPositionSendsGoalCloud(t *testing.T) {
	var got motion.MoveReq
	s := &simulatedSO101{
		name:      arm.Named("simarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
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
// line leaves goalCloud zero-valued -- X/Y/Z/OZ all 0 -- which per goal_cloud.go is a cloud
// no IK solution realistically satisfies, so planning reverts to strict 6-DOF scoring and
// every move fails silently, with the construction-time warnings dead. The struct-literal
// tests set the field themselves and would stay green through all of that.
func TestSimulatedConstructorWiresGoalCloud(t *testing.T) {
	// Defaults: catches the line being dropped entirely.
	sim := simArmWithTolerances(t, 0, 0)
	assert.Equal(t, goalCloudConfig{
		OrientationToleranceDeg: defaultOrientationToleranceDeg,
		PositionToleranceMM:     defaultPositionToleranceMM,
	}, sim.goalCloud, "the constructor must resolve defaults into goalCloud")

	// Explicit values: catches a line that resolves but ignores the config.
	sim = simArmWithTolerances(t, 12.5, 0.25)
	assert.Equal(t, goalCloudConfig{OrientationToleranceDeg: 12.5, PositionToleranceMM: 0.25},
		sim.goalCloud, "explicit config must reach goalCloud unchanged")
}

// TestHardwareArmMoveToPositionExitsManualMode guards PR #27's integration: a motion
// command must take the arm out of hand-guided mode. Nothing else asserts this -- the
// manual-mode tests cover exitManualLocked in isolation, not that MoveToPosition calls it.
func TestHardwareArmMoveToPositionExitsManualMode(t *testing.T) {
	var got motion.MoveReq
	a := &so101{
		name:      arm.Named("myarm"),
		logger:    logging.NewTestLogger(t),
		motion:    captureMotion(&got),
		goalCloud: resolveGoalCloudConfig(0, 0, nil),
	}
	// exitManualLocked no-ops unless the session is non-nil AND running, so the test only
	// has teeth with a started session. newFakeIO stands in for the servo bus.
	io := newFakeIO()
	a.manual = newManualSession(context.Background(), io, manualDefaults(), 0, 0, a.logger)
	a.manual.start()
	require.True(t, a.manual.running(), "precondition: the arm must be in manual mode")

	require.NoError(t, a.MoveToPosition(context.Background(), testGoal(), nil))

	assert.Nil(t, a.manual, "a motion command must tear the manual session down")
	io.mu.Lock()
	defer io.mu.Unlock()
	assert.True(t, io.complianceRestored, "exiting manual mode must restore servo stiffness")
}

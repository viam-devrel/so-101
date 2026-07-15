package so_arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
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

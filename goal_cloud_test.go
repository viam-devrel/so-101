package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"go.viam.com/rdk/logging"
)

func TestResolveGoalCloudConfigDefaults(t *testing.T) {
	// Zero and unset are indistinguishable for a float64 with omitempty; both default.
	got := resolveGoalCloudConfig(0, 0, nil)
	assert.Equal(t, defaultOrientationToleranceDeg, got.OrientationToleranceDeg)
	assert.Equal(t, defaultPositionToleranceMM, got.PositionToleranceMM)
}

func TestResolveGoalCloudConfigKeepsExplicitValues(t *testing.T) {
	got := resolveGoalCloudConfig(12.5, 0.25, nil)
	assert.Equal(t, 12.5, got.OrientationToleranceDeg)
	assert.Equal(t, 0.25, got.PositionToleranceMM)
}

// The warnings are the production path -- both tests above pass a nil logger and so
// exercise only the early return. resolveGoalCloudConfig is the SINGLE owner of these
// thresholds; nothing downstream re-checks them, so nothing downstream would catch a
// regression either.
func TestResolveGoalCloudConfigWarnsAtSaturationBoundary(t *testing.T) {
	// The boundary is acos(-0.999) = 177.4374 -- exactly the band a literal ">= 177.44"
	// check would miss, which is why poseCloudSaturationCos is a cosine.
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.438, 0, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("saturates").Len(), "just inside the boundary must warn")

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.40, 0, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("saturates").Len(), "just outside must not warn")
}

func TestResolveGoalCloudConfigWarnsOnLargePositionTolerance(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 25, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("is large").Len())

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 10, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("is large").Len(), "at the threshold must not warn")
}

func TestResolveGoalCloudConfigDefaultsDoNotWarn(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 0, logger)
	assert.Equal(t, 0, logs.Len(), "the defaults must be quiet")
}

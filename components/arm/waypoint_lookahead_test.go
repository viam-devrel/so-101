package arm

import (
	"context"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
)

// The lookahead is derived per move from the speed and acceleration actually in force,
// because a servo's deceleration distance grows with the square of its speed. A fixed value
// is necessarily wrong at one end: 2.0 deg was below the ramp of a wave replayed at
// 58.8 deg/s, so the next goal was written only after the servo had begun decelerating —
// ten velocity dips a second on a 10 Hz recording.
func TestWaypointLookaheadIsDerivedFromTheMoveProfile(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// 0.045 rad is 2.58 deg short of w0: inside the 3.0 deg derived from the default
	// 50 deg/s + 500 deg/s^2 profile, but outside the old fixed 2.0.
	short := []float64{0.20 - 0.045, 0, 0, 0, 0}

	s, calls := dwellTestArm(t, [][]float64{short})
	s.lookaheadDeg = 0 // unset: derive

	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))

	// One seeding read plus one dwell read: the derived lookahead is satisfied on the first
	// poll. Under the old fixed 2.0 this waypoint could never be satisfied and would poll
	// until its deadline instead.
	assert.Equal(t, 2, calls(),
		"a 2.58 deg residual must satisfy the lookahead derived for a 50 deg/s move")
}

func TestConfiguredWaypointLookaheadOverridesTheDerivedOne(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	short := []float64{0.20 - 0.045, 0, 0, 0, 0} // 2.58 deg short

	s, calls := dwellTestArm(t, [][]float64{short})
	s.lookaheadDeg = 1.0 // explicit: tighter than anything the profile would derive

	started := time.Now()
	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))

	// 2.58 deg never satisfies a 1.0 deg lookahead, so the dwell must poll to its deadline
	// rather than silently falling back to the derived value.
	assert.Greater(t, calls(), 2, "an explicit lookahead must not be replaced by the derived one")
	assert.Greater(t, time.Since(started), 10*time.Millisecond, "the dwell must have polled")
}

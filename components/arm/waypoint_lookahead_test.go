package arm

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/servo"
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

// A fixed poll tick quantises every waypoint to a multiple of itself: the arm needs some
// fraction of a tick to close the last of its gap, and the dwell cannot notice until the
// next one. Measured on a recorded nod, that rounding cost 5.5 ms per waypoint at a 10 ms
// tick — 2.4 s of the 5.0 s by which a 6.9 s recording overran. The wait is therefore
// predicted from the distance still to cover, not fixed.
func TestWaypointDwellWaitsOnlyAsLongAsTheArmStillNeeds(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// Just past the lookahead, so the dwell must wait — but by a distance the arm covers in
	// far less than one full dwellPollInterval at the default 50 deg/s.
	const overshootDeg = 0.2
	nearly := []float64{0.20 - (servo.DefaultLookaheadDeg+overshootDeg)*math.Pi/180, 0, 0, 0, 0}

	calls := 0
	s, _ := dwellTestArm(t, nil)
	s.readJoints = func(context.Context) ([]float64, error) {
		calls++
		if calls == 1 {
			return nearly, nil // still short: forces one wait
		}
		return w0, nil // arrived
	}

	started := time.Now()
	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))
	elapsed := time.Since(started)

	// 0.2 deg at 50 deg/s is 4ms, so the wait must be well under a full 10ms tick. A fixed
	// ticker would have taken at least one.
	assert.Less(t, elapsed, dwellPollInterval,
		"the dwell must predict a short wait, not round up to a full poll interval")
	assert.GreaterOrEqual(t, elapsed, minDwellPoll, "and must not busy-spin below the floor")
}

func TestWaypointDwellNeverWaitsBelowItsFloor(t *testing.T) {
	// The predicted wait shrinks with the remaining distance, so without a floor a nearly
	// arrived arm would be polled as fast as the bus allows. CLAUDE.md records a position
	// read failing outright under sustained max-rate polling, and in this dwell that is
	// fatal to the move.
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	// A hair past the lookahead: predicted wait is microseconds.
	nearly := []float64{0.20 - (servo.DefaultLookaheadDeg+0.001)*math.Pi/180, 0, 0, 0, 0}

	s, calls := dwellTestArm(t, [][]float64{nearly})
	started := time.Now()
	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))

	// It never arrives, so it polls to the deadline. Each poll must have taken at least the
	// floor, bounding how many happened.
	maxPolls := int(time.Since(started)/minDwellPoll) + 2
	assert.LessOrEqual(t, calls(), maxPolls,
		"polling faster than minDwellPoll risks the read failure that aborts a move")
}

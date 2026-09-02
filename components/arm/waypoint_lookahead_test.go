package arm

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"

	"so_arm/internal/servo"
)

// The lookahead is derived per move, because a servo's deceleration distance grows with the
// square of its speed, so a fixed value is necessarily wrong at one end. See docs/arm.md.
func TestWaypointLookaheadIsDerivedFromTheMoveProfile(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// The move speed has to differ from the component default, or the assertion cannot tell
	// derivation from the floor: at the shipped 50 deg/s + 500 deg/s^2 the ramp term is
	// exactly 3.0 deg, the same value DefaultLookaheadDeg carries. At 100 deg/s the ramp is
	// 12.0 deg, so a 6 deg residual satisfies the derived lookahead and nothing else.
	opts := &arm.MoveOptions{MaxVelRads: utils.DegToRad(100)}
	require.InDelta(t, 12.0, servo.LookaheadDegFor(100, servo.DefaultAccelDegsPerSecSq), 0.01)

	short := []float64{0.20 - utils.DegToRad(6), 0, 0, 0, 0}

	s, calls := dwellTestArm(t, [][]float64{short})
	s.lookaheadDeg = 0 // unset: derive

	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, opts, nil))

	// The seeding read alone: the 6 deg residual is already inside the derived 12 deg
	// lookahead, so the predictive skip takes the waypoint without reading again. Against
	// DefaultLookaheadDeg (3.0) the residual is outside, no skip is possible, and the dwell
	// polls to its deadline -- so this still fails if the derivation is dropped, and it also
	// pins that the skip uses the same lookahead the dwell would.
	assert.Equal(t, 1, calls(),
		"a 6 deg residual is inside the lookahead derived for a 100 deg/s move, so the "+
			"dwell should be skipped outright rather than polled")
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

// A fixed poll tick quantises every waypoint to a multiple of itself, so the wait is
// predicted from the distance still to cover. See docs/arm.md, "Waypoint streams".
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
	// arrived arm is polled as fast as the bus allows -- where a read can fail outright.
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

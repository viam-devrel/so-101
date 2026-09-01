package arm

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/servo"
)

// A servo parks short of its goal and reports Moving=0, past any usable lookahead at a low
// P gain, so without the stall escape the dwell's only exit is its deadline. See
// docs/arm.md, "Waypoint streams".

func TestWaypointDwellStopsWaitingOnServosThatHaveStopped(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// Parked further from the goal than any derivable lookahead, so proximity can never end
	// the dwell and only the stall escape can end it before its deadline.
	parked := shortOfGoal(0.20)
	s, _ := dwellTestArm(t, [][]float64{parked})

	moveChecks := 0
	s.servosMoving = func(context.Context) (bool, error) {
		moveChecks++
		return false, nil
	}

	started := time.Now()
	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))
	elapsed := time.Since(started)

	timeout := servo.DwellTimeoutMs(servo.MaxLookaheadDeg, 50, servo.DefaultAccelDegsPerSecSq)
	assert.Positive(t, moveChecks, "the dwell must consult the servos' moving state")
	assert.Less(t, elapsed, time.Duration(timeout)*time.Millisecond,
		"a stopped arm must end the dwell well before its deadline")
}

func TestWaypointDwellIgnoresTheMovingStateOnItsFirstPoll(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	parked := shortOfGoal(0.20)
	s, _ := dwellTestArm(t, [][]float64{parked})

	// Moving takes ~2ms to rise after a goal write, so a first-poll read can be the pre-write
	// zero. This reports "not moving" from call one; the dwell must poll twice before acting.
	positionReads := 0
	s.readJoints = func(context.Context) ([]float64, error) {
		positionReads++
		return parked, nil
	}
	s.servosMoving = func(context.Context) (bool, error) { return false, nil }

	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))

	// 1 seed read + at least 2 polls inside the single intermediate waypoint's dwell.
	assert.GreaterOrEqual(t, positionReads, 3,
		"the stall escape must not fire on the dwell's first poll")
}

func TestWaypointDwellFallsBackToItsDeadlineWhenTheMovingReadFails(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	lagging := shortOfGoal(0.20)
	s, _ := dwellTestArm(t, [][]float64{lagging})

	// The moving state is an auxiliary signal. A transient bus error on it must not fail a
	// move the deadline would have carried through -- unlike a position read, which is
	// fatal because without it there is no pacing at all.
	s.servosMoving = func(context.Context) (bool, error) {
		return false, errors.New("bus transient")
	}

	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{w0, w1}, nil, nil))
}

// Stop's zeroed velocity is exactly the condition the stall escape fires on, so a cancelled
// stream takes the escape rather than the select's ctx.Done() and reaches one more moveJoints
// write before ctx.Err() unwinds it. That write is harmless because opMgr.CancelRunning
// blocks until the stream returns. This pins termination; TestStopCancelsARunningWaypointStream
// pins the ordering.
func TestWaypointDwellTerminatesWhenCancelledWhileServosAreStopped(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	w2 := []float64{0.60, 0, 0, 0, 0}
	parked := shortOfGoal(0.20)
	s, _ := dwellTestArm(t, [][]float64{parked})

	// What Stop leaves behind: nothing moving, and the arm parked short of the goal.
	s.servosMoving = func(context.Context) (bool, error) { return false, nil }

	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	done := make(chan error, 1)
	go func() {
		done <- s.MoveThroughJointPositions(ctx, [][]referenceframe.Input{w0, w1, w2}, nil, nil)
	}()

	select {
	case err := <-done:
		require.Error(t, err, "a cancelled stream must not report success")
		assert.ErrorIs(t, err, context.Canceled)
	case <-time.After(2 * time.Second):
		t.Fatal("cancelled stream did not terminate: the stall escape returns nil, so the " +
			"waypoint loop must still reach its ctx.Err() check")
	}
}

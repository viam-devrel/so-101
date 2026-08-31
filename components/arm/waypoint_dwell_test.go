package arm

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/servo"
	"so_arm/internal/testfake"
)

// These tests pin the contract that makes a waypoint stream a trajectory rather than a jump
// to its endpoint: MoveThroughJointPositions must not command waypoint k+1 until the arm has
// actually reached waypoint k. Every SetGoals overwrites the previous goal within
// milliseconds, so a stream issued back-to-back executes ONLY its final waypoint -- which is
// how a recorded 3-rad nod replayed as a 0.17-rad twitch.
//
// Reads are the observable. The dwell polls positions once per waypoint, so the read count
// distinguishes a dwelling stream (one read per waypoint) from a flythrough (a read at the
// start and nothing per waypoint) with no reliance on wall-clock timing.

// dwellTestArm builds an so101 over a fake serial bus with a scripted position reader. The
// reader returns script[i] on call i, repeating the last entry once exhausted, and reports
// how many times it was called.
func dwellTestArm(t *testing.T, script [][]float64) (*so101, func() int) {
	t.Helper()
	calls := 0
	s := &so101{
		logger:       logging.NewTestLogger(t),
		opMgr:        operation.NewSingleOperationManager(),
		controller:   testArmHandle(t, testfake.NewFakeTransport()),
		armServoIDs:  []int{1, 2, 3, 4, 5},
		defaultSpeed: 50,
		defaultAcc:   servo.DefaultAccelDegsPerSecSq,
		lookaheadDeg: servo.DefaultLookaheadDeg,
	}
	s.readJoints = func(context.Context) ([]float64, error) {
		p := script[min(calls, len(script)-1)]
		calls++
		return p, nil
	}
	return s, func() int { return calls }
}

func TestMoveThroughJointPositionsDwellsOnEveryIntermediateWaypoint(t *testing.T) {
	start := []float64{0, 0, 0, 0, 0}
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}
	w2 := []float64{0.60, 0, 0, 0, 0}
	w3 := []float64{0.80, 0, 0, 0, 0}

	// The arm arrives exactly on each waypoint, so every dwell is satisfied by its first
	// poll: one read to seed the travel reference, then one per intermediate waypoint.
	s, calls := dwellTestArm(t, [][]float64{start, w0, w1, w2})

	require.NoError(t, s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{w0, w1, w2, w3}, nil, nil))

	// 1 seed + 3 intermediate waypoints. The flythrough this replaced read exactly twice
	// (seed + a re-read before the final waypoint) no matter how long the stream was.
	assert.Equal(t, 4, calls(), "one dwell read per intermediate waypoint, plus the seeding read")
}

func TestMoveThroughJointPositionsHoldsUntilTheArmIsWithinTheLookahead(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// 0.05 rad is 2.9 deg, outside the 2 deg default lookahead, so the first waypoint's
	// dwell can never be satisfied and must run to its timeout instead of hanging.
	lagging := []float64{0.15, 0, 0, 0, 0}
	s, calls := dwellTestArm(t, [][]float64{lagging})

	started := time.Now()
	require.NoError(t, s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{w0, w1}, nil, nil))
	elapsed := time.Since(started)

	// Best-effort: a lagging waypoint is given up on, not turned into an error.
	assert.Greater(t, calls(), 2, "an unsatisfiable dwell polls repeatedly before giving up")
	assert.GreaterOrEqual(t, elapsed, 40*time.Millisecond,
		"the stream is paced by the dwell, not issued back-to-back")
}

func TestMoveThroughJointPositionsDwellHonorsContextCancellation(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// Far enough from w0 that the dwell keeps polling until something stops it.
	s, _ := dwellTestArm(t, [][]float64{{-1.0, 0, 0, 0, 0}})

	ctx, cancel := context.WithCancel(context.Background())
	go func() {
		time.Sleep(20 * time.Millisecond)
		cancel()
	}()

	err := s.MoveThroughJointPositions(ctx, [][]referenceframe.Input{w0, w1}, nil, nil)
	assert.True(t, errors.Is(err, context.Canceled), "got %v", err)
}

func TestMoveThroughJointPositionsFailsWhenTheDwellLosesPositionFeedback(t *testing.T) {
	w0 := []float64{0.20, 0, 0, 0, 0}
	w1 := []float64{0.40, 0, 0, 0, 0}

	// The seeding read succeeds and the dwell's read fails: continuing would silently
	// revert to the flythrough, so the move fails instead.
	calls := 0
	s, _ := dwellTestArm(t, [][]float64{{0, 0, 0, 0, 0}})
	s.readJoints = func(context.Context) ([]float64, error) {
		calls++
		if calls == 1 {
			return []float64{0, 0, 0, 0, 0}, nil
		}
		return nil, errors.New("bus went away")
	}

	err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{w0, w1}, nil, nil)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "while holding a waypoint")
}

// A single-waypoint stream -- what GoToInputs emits most of the time, and what the recorder's
// safe-entry move uses -- has no intermediate waypoint, so it must not pay for a dwell.
func TestMoveThroughJointPositionsSingleWaypointDoesNotDwell(t *testing.T) {
	s, calls := dwellTestArm(t, [][]float64{{0, 0, 0, 0, 0}})

	require.NoError(t, s.MoveThroughJointPositions(
		context.Background(), [][]referenceframe.Input{{0.20, 0, 0, 0, 0}}, nil, nil))

	assert.Equal(t, 1, calls(), "just the seeding read")
}

// The dwell timeout floor is what keeps a densified path from stalling: MoveTimeoutMs' 1s
// floor across hundreds of waypoints would be minutes of dead time.
func TestDwellTimeoutFloorIsFarBelowTheMoveTimeoutFloor(t *testing.T) {
	// A 0.9 deg segment -- one step of a 10 Hz recording densified 10x.
	const tinyTravelDeg = 0.9

	dwell := servo.DwellTimeoutMs(tinyTravelDeg, 50, servo.DefaultAccelDegsPerSecSq)
	move := servo.MoveTimeoutMs(tinyTravelDeg, 50, servo.DefaultAccelDegsPerSecSq)

	assert.Equal(t, 1000, move, "the whole-move floor is unchanged")
	assert.Less(t, dwell, move, "a per-waypoint dwell must not inherit the whole-move floor")

	// Both still model the same ramp once the travel is large enough to clear the floors.
	assert.Equal(t,
		servo.MoveTimeoutMs(90, 50, servo.DefaultAccelDegsPerSecSq),
		servo.DwellTimeoutMs(90, 50, servo.DefaultAccelDegsPerSecSq))
}

// A paced stream runs for seconds, so Stop has to cancel it. Zeroing velocity alone is
// overwritten by the loop's next goal write.
func TestStopCancelsARunningWaypointStream(t *testing.T) {
	// Far from every waypoint, so each dwell polls until something stops it.
	s, calls := dwellTestArm(t, [][]float64{{-1.0, 0, 0, 0, 0}})

	errCh := make(chan error, 1)
	go func() {
		errCh <- s.MoveThroughJointPositions(context.Background(),
			[][]referenceframe.Input{{0.2, 0, 0, 0, 0}, {0.4, 0, 0, 0, 0}, {0.6, 0, 0, 0, 0}}, nil, nil)
	}()

	// Let the stream get into its first dwell before stopping it.
	time.Sleep(30 * time.Millisecond)
	require.NoError(t, s.Stop(context.Background(), nil))

	select {
	case err := <-errCh:
		assert.True(t, errors.Is(err, context.Canceled), "got %v", err)
	case <-time.After(time.Second):
		t.Fatal("Stop did not end the waypoint stream")
	}

	// Stop blocks until the move returns, so the stream cannot still be issuing waypoints.
	after := calls()
	time.Sleep(30 * time.Millisecond)
	assert.Equal(t, after, calls(), "no reads after Stop returned")
}

package controller

import (
	"context"
	"errors"
	"fmt"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// scriptedMotion returns a probe that reports each value in turn, repeating the last one forever,
// and counts how many times it was asked.
func scriptedMotion(values ...bool) (func(context.Context) (bool, error), *int) {
	calls := 0
	return func(context.Context) (bool, error) {
		v := values[len(values)-1]
		if calls < len(values) {
			v = values[calls]
		}
		calls++
		return v, nil
	}, &calls
}

// TestWaitForMotionWaitsForTheMoveToBegin is the regression this function exists for. A servo does
// not set its Moving register the instant a goal is written, so polling immediately reads
// "stopped" and reports the move complete before the jaw has budged. Measured on hardware: a
// 1.5-second sweep returned in 9ms, and Grab's confirming position read -- and the grasp latch
// built on it -- were both deciding on a pre-move position.
func TestWaitForMotionWaitsForTheMoveToBegin(t *testing.T) {
	// Not moving yet, not moving yet, then moving, then done.
	moving, calls := scriptedMotion(false, false, true, true, false)

	started, stopped, err := waitForMotion(context.Background(), moving, time.Second, time.Second)
	require.NoError(t, err)
	assert.True(t, started, "must notice the move begin rather than returning on the first poll")
	assert.True(t, stopped, "and must then wait for it to finish")
	assert.GreaterOrEqual(t, *calls, 5, "should have polled through start and stop, not returned early")
}

// TestWaitForMotionNoOpGoalDoesNotHang: a goal the servo is already at never registers as motion.
// That must return promptly rather than blocking for the full timeout -- Open and Grab both issue
// goals that are frequently already satisfied.
func TestWaitForMotionNoOpGoalDoesNotHang(t *testing.T) {
	moving, _ := scriptedMotion(false)

	start := time.Now()
	started, stopped, err := waitForMotion(context.Background(), moving,
		80*time.Millisecond, 10*time.Second)
	elapsed := time.Since(start)

	require.NoError(t, err)
	assert.False(t, started, "no motion ever appeared")
	assert.True(t, stopped, "and nothing is moving, so the wait is satisfied")
	assert.Less(t, elapsed, 2*time.Second,
		"a no-op goal must give up after the start window, not wait out the full timeout (took %v)", elapsed)
}

// TestWaitForMotionTinyMoveDoesNotHang: a move that completes inside a single poll interval is
// indistinguishable from a no-op -- the probe never observes motion. It must be treated the same
// way, since in both cases there is nothing left to wait for.
func TestWaitForMotionTinyMoveDoesNotHang(t *testing.T) {
	moving, _ := scriptedMotion(false)

	start := time.Now()
	started, stopped, err := waitForMotion(context.Background(), moving,
		50*time.Millisecond, 10*time.Second)

	require.NoError(t, err)
	assert.False(t, started)
	assert.True(t, stopped)
	assert.Less(t, time.Since(start), 2*time.Second)
}

// TestWaitForMotionTimesOutWhileStillMoving: a servo that never settles must not block forever.
// It reports stopped=false so the caller can decide whether that matters.
func TestWaitForMotionTimesOutWhileStillMoving(t *testing.T) {
	moving, _ := scriptedMotion(true) // moving forever

	start := time.Now()
	started, stopped, err := waitForMotion(context.Background(), moving,
		time.Second, 120*time.Millisecond)
	elapsed := time.Since(start)

	require.NoError(t, err)
	assert.True(t, started)
	assert.False(t, stopped, "still moving at the deadline")
	assert.Less(t, elapsed, 3*time.Second, "must honour the timeout (took %v)", elapsed)
}

// TestWaitForMotionAlreadyMoving: motion already underway on entry counts as started.
func TestWaitForMotionAlreadyMoving(t *testing.T) {
	moving, _ := scriptedMotion(true, true, false)

	started, stopped, err := waitForMotion(context.Background(), moving, time.Second, time.Second)
	require.NoError(t, err)
	assert.True(t, started)
	assert.True(t, stopped)
}

func TestWaitForMotionPropagatesProbeErrors(t *testing.T) {
	boom := errors.New("bus timeout")
	fail := func(context.Context) (bool, error) { return false, boom }

	_, _, err := waitForMotion(context.Background(), fail, time.Second, time.Second)
	assert.ErrorIs(t, err, boom, "a failed read must not be mistaken for 'not moving'")
}

func TestWaitForMotionHonoursContextCancellation(t *testing.T) {
	moving, _ := scriptedMotion(true) // would otherwise spin until the timeout
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Millisecond)
	defer cancel()

	start := time.Now()
	_, _, err := waitForMotion(ctx, moving, time.Second, 30*time.Second)
	assert.ErrorIs(t, err, context.DeadlineExceeded)
	assert.Less(t, time.Since(start), 5*time.Second, "cancellation must cut the wait short")
}

// TestIsConditionErrorDistinguishesConditionsFromFailures: a servo reporting overload answered the
// question -- the library just refuses to hand back the payload. Treating that as a failed read
// aborts moves that are physically fine. Measured on hardware: an Open that opened the jaw
// correctly returned "failed to read moving state ... [overload]", which left the gripper's grasp
// latch stuck set with an empty jaw.
func TestIsConditionErrorDistinguishesConditionsFromFailures(t *testing.T) {
	for _, tc := range []struct {
		name string
		err  error
		want bool
	}{
		{"nil", nil, false},
		{"typed overload", &feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrOverload}, true},
		{"typed overheat", &feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrOverheat}, true},
		{"bare StatusError overload", feetech.ErrOverload, true},
		{"remote string form", errors.New("failed to read moving state for servo 6: servo status error: [overload]"), true},
		{"wrapped typed", fmt.Errorf("wrapped: %w", &feetech.ServoError{ID: 6, Status: feetech.ErrVoltage}), true},
		{"genuine comms failure", errors.New("read tcp: connection reset by peer"), false},
		{"timeout", context.DeadlineExceeded, false},
		{"checksum is a request-rejection, not a condition",
			&feetech.ServoError{ID: 6, Op: "read", Status: feetech.ErrChecksum}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			assert.Equal(t, tc.want, isConditionError(tc.err))
		})
	}
}

// TestWaitForMotionKeepsWaitingThroughOverload: while a servo is overloaded every register read
// fails, so motion is unobservable. The wait must keep going rather than abort -- the move is
// underway and the caller's timeout bounds it.
func TestWaitForMotionKeepsWaitingThroughOverload(t *testing.T) {
	overload := &feetech.ServoError{ID: 6, Op: "read moving", Status: feetech.ErrOverload}
	calls := 0
	moving := func(context.Context) (bool, error) {
		calls++
		if calls <= 3 {
			return false, overload // unobservable while clamping
		}
		return false, nil // overload cleared, settled
	}
	// The real wrapper WaitForServosToStop installs -- not a copy of it, so removing the
	// tolerance from production code fails this test.
	started, stopped, err := waitForMotion(context.Background(),
		tolerateConditions(moving), time.Second, time.Second)
	require.NoError(t, err, "an overload must not abort the wait")
	assert.True(t, started, "unobservable motion still counts as started")
	assert.True(t, stopped, "and the wait completes once it becomes observable again")
}

package controller

import (
	"context"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

const (
	// motionStartWindow is how long to wait for commanded motion to actually begin before
	// concluding it never will. The servo does not set its Moving register the instant a goal is
	// written -- it takes a few milliseconds to begin -- so polling immediately reads "stopped"
	// and reports a move complete before the servo has budged. Measured on an STS3215 gripper: a
	// 1.5-second jaw sweep returned in 9ms, and every caller that then read position got a
	// pre-move value.
	motionStartWindow = 250 * time.Millisecond

	// motionStartPoll is deliberately much shorter than motionStopPoll. A move small enough to
	// finish inside one poll interval never registers as moving at all, and the shorter the poll
	// the smaller that blind spot gets.
	motionStartPoll = 10 * time.Millisecond
	motionStopPoll  = 50 * time.Millisecond
)

// waitForMotion waits for commanded motion to begin and then finish.
//
// moving reports whether anything is still in motion. It returns started=false when motion never
// appeared within startWindow, which is the normal outcome for a no-op goal (already at the
// target) or a move too small to span a poll -- neither is an error, and neither may hang. Once
// motion is observed, it waits for it to cease, giving up at timeout and reporting stopped=false
// so the caller can decide whether a still-moving servo matters.
//
// Split out from WaitForServosToStop so this decision can be tested without a serial bus.
func waitForMotion(
	ctx context.Context,
	moving func(context.Context) (bool, error),
	startWindow, timeout time.Duration,
) (started, stopped bool, err error) {
	// Phase 1: give motion a bounded window to appear.
	startDeadline := time.Now().Add(startWindow)
	for {
		isMoving, err := moving(ctx)
		if err != nil {
			return false, false, err
		}
		if isMoving {
			started = true
			break
		}
		if !time.Now().Before(startDeadline) {
			// Never moved. Either the goal was already satisfied or the travel was too short to
			// observe; both mean there is nothing left to wait for.
			return false, true, nil
		}
		if err := sleepCtx(ctx, motionStartPoll); err != nil {
			return false, false, err
		}
	}

	// Phase 2: wait for it to cease. The full timeout applies here, not to the search above.
	stopDeadline := time.Now().Add(timeout)
	for {
		isMoving, err := moving(ctx)
		if err != nil {
			return started, false, err
		}
		if !isMoving {
			return started, true, nil
		}
		if !time.Now().Before(stopDeadline) {
			return started, false, nil
		}
		if err := sleepCtx(ctx, motionStopPoll); err != nil {
			return started, false, err
		}
	}
}

func sleepCtx(ctx context.Context, d time.Duration) error {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}

// isConditionError reports whether err is a servo describing its own physical condition --
// overload, overheat, voltage -- rather than a communication failure or a rejected request.
//
// ConditionStatus is the authority: it returns ok only when the data alongside err is safe to
// use, which is exactly the question being asked here.
func isConditionError(err error) bool {
	_, ok := feetech.ConditionStatus(err)
	return ok
}

// tolerateConditions wraps a motion probe so that a servo reporting a physical condition --
// overload while clamping, overheat -- counts as "still moving" instead of aborting the wait.
//
// The servo answered; we simply cannot observe the Moving register's meaning while it is unhappy,
// and the move is underway. Measured on hardware: without this an Open that opened the jaw
// correctly returned "failed to read moving state ... [overload]", which left the gripper's grasp
// latch stuck set with an empty jaw until a second command cleared it.
//
// Genuine transport failures still propagate -- those are not the servo talking.
func tolerateConditions(probe func(context.Context) (bool, error)) func(context.Context) (bool, error) {
	return func(ctx context.Context) (bool, error) {
		moving, err := probe(ctx)
		if isConditionError(err) {
			return true, nil
		}
		return moving, err
	}
}

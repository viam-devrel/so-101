package controller

import (
	"context"
	"time"
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

package arm

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/arm"

	"so_arm/internal/servo"
)

// streamStartGapDeg is how close the arm must already be to a stream's first point before
// it is written on the unbounded (Speed 0 / Acc 0) path. Same value as teleop's
// maxStreamStartGapDeg; not imported because components must not depend on services.
const streamStartGapDeg = 5.0

// MoveThroughJointPositionsStreamed writes each point as an unbounded goal at start + Time.
// The producer owns the timing: a late point is written immediately, never dropped, and
// Constraints are ignored.
//
// Only the FIRST point is gated. A large jump between consecutive points, or a schedule
// slip that lets the goal race ahead of the arm, reaches the servos unbounded; no policy
// for either has been measured yet -- see docs/arm.md, "Streaming trajectories". extra is
// accepted and ignored.
func (s *so101) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()

	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	s.mu.RLock()
	speed := float64(s.defaultSpeed)
	accel := float64(s.defaultAcc)
	s.mu.RUnlock()

	var start time.Time
	var prev time.Duration
	idx := 0
	for {
		// Not `range batches`: Stop cancels ctx but cannot close the channel, and
		// CancelRunning blocks until this returns.
		var batch []arm.TrajectoryPoint
		var ok bool
		select {
		case <-ctx.Done():
			return ctx.Err()
		case batch, ok = <-batches:
		}
		if !ok {
			break
		}
		for _, p := range batch {
			if err := servo.CheckTrajectoryTime(idx, prev, p.Time); err != nil {
				return err
			}
			clamped, err := s.clampPositions(p.Positions)
			if err != nil {
				return err
			}
			if idx == 0 {
				if err := s.closeStreamStartGap(ctx, clamped, speed, accel); err != nil {
					return err
				}
				start = s.clock.Time()
			}
			if err := s.clock.WaitUntil(ctx, start.Add(p.Time)); err != nil {
				return err
			}
			if err := s.moveJointsUniform(ctx, clamped, speed, accel, true, false); err != nil {
				return err
			}
			prev, idx = p.Time, idx+1
		}
		if len(batch) == 0 {
			continue
		}
		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	if idx == 0 {
		return nil
	}
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, servo.MaxMoveTimeoutMs)
}

// closeStreamStartGap runs one blocking profiled move to `target` when any joint is more than
// streamStartGapDeg away. The streamed path removes every bound, so the FIRST point must not
// hand it a large position error.
func (s *so101) closeStreamStartGap(ctx context.Context, target []float64, speed, accel float64) error {
	current, err := s.currentJoints(ctx)
	if err != nil {
		return fmt.Errorf("failed to read positions before a streamed trajectory: %w", err)
	}
	_, gapDeg := servo.JointTravelsDeg(current, target)
	if gapDeg <= streamStartGapDeg {
		return nil
	}
	s.logger.Debugf("streamed trajectory starts %.1f deg away; profiled move to its first point", gapDeg)
	_, err = s.moveJoints(ctx, current, target, speed, accel, nil, true)
	return err
}

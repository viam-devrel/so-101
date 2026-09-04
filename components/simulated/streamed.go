package simulated

import (
	"context"
	"errors"
	"time"

	"go.viam.com/rdk/components/arm"

	"so_arm/internal/servo"
)

// MoveThroughJointPositionsStreamed re-targets the interpolator at each point's scheduled
// time and returns once the arm has converged on the last one. Constraints are ignored.
// Batches are received with a select on ctx, not range: a cancelled ctx does not close the
// channel.
func (s *simulatedSO101) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	_ map[string]interface{},
) error {
	var start time.Time
	var prev time.Duration
	idx := 0
	for {
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
			if idx == 0 {
				start = s.clock.Time()
			}
			if err := s.clock.WaitUntil(ctx, start.Add(p.Time)); err != nil {
				return err
			}
			if idx > 0 {
				s.mu.Lock()
				stopped := s.operation.stopped
				s.mu.Unlock()
				if stopped {
					return errors.New("stopped before reaching target")
				}
			}
			if err := s.startMove(ctx, p.Positions); err != nil {
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
	return s.awaitOperation(ctx)
}

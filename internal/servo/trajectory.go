package servo

import (
	"context"
	"fmt"
	"time"
)

// CheckTrajectoryTime validates a point's Time against the previous point's: the first
// (prev < 0, i.e. none) must be at 0 and every later one strictly after its predecessor.
func CheckTrajectoryTime(prev, t time.Duration) error {
	switch {
	case prev < 0 && t != 0:
		return fmt.Errorf("first trajectory point must have Time 0, got %v", t)
	case prev >= 0 && t <= prev:
		return fmt.Errorf("trajectory point Time %v must exceed the previous point's %v", t, prev)
	}
	return nil
}

// Clock is the wall clock a streamed trajectory is scheduled against. The zero value is the
// real clock; tests substitute both funcs so no test sleeps trajectory time.
type Clock struct {
	Now        func() time.Time
	SleepUntil func(context.Context, time.Time) error
}

// Time returns the current instant.
func (c Clock) Time() time.Time {
	if c.Now != nil {
		return c.Now()
	}
	return time.Now()
}

// WaitUntil blocks until t or ctx ends. A deadline already past returns immediately -- with
// ctx.Err(), so a cancelled stream still stops on a late point.
func (c Clock) WaitUntil(ctx context.Context, t time.Time) error {
	if c.SleepUntil != nil {
		return c.SleepUntil(ctx, t)
	}
	d := t.Sub(c.Time())
	if d <= 0 {
		return ctx.Err()
	}
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

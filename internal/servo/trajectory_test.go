package servo

import (
	"context"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestCheckTrajectoryTime(t *testing.T) {
	cases := []struct {
		name    string
		i       int
		prev, t time.Duration
		wantErr bool
	}{
		{"first at zero", 0, 0, 0, false},
		{"first non-zero", 0, 0, 10 * time.Millisecond, true},
		{"first negative", 0, 0, -time.Millisecond, true},
		{"increasing", 1, 10 * time.Millisecond, 20 * time.Millisecond, false},
		{"equal", 2, 20 * time.Millisecond, 20 * time.Millisecond, true},
		{"decreasing", 2, 20 * time.Millisecond, 5 * time.Millisecond, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			err := CheckTrajectoryTime(tc.i, tc.prev, tc.t)
			if tc.wantErr {
				assert.Error(t, err)
			} else {
				assert.NoError(t, err)
			}
		})
	}
}

func TestClockZeroValueIsTheRealClock(t *testing.T) {
	var c Clock
	before := time.Now()
	assert.False(t, c.Time().Before(before))
	// A deadline already past returns at once, and reports nil on a live ctx.
	require.NoError(t, c.WaitUntil(context.Background(), before))
	// The timer path, which production takes on every point.
	require.NoError(t, c.WaitUntil(context.Background(), time.Now().Add(time.Millisecond)))
}

func TestClockWaitUntilHonoursCancel(t *testing.T) {
	var c Clock
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	err := c.WaitUntil(ctx, time.Now().Add(time.Hour))
	assert.ErrorIs(t, err, context.Canceled)
	// Also on an already-past deadline: a cancelled stream must not slip a late point through.
	assert.ErrorIs(t, c.WaitUntil(ctx, time.Now().Add(-time.Second)), context.Canceled)
}

func TestClockSeamsOverrideBothHalves(t *testing.T) {
	epoch := time.Date(2026, 9, 4, 12, 0, 0, 0, time.UTC)
	var got time.Time
	c := Clock{
		Now:        func() time.Time { return epoch },
		SleepUntil: func(_ context.Context, d time.Time) error { got = d; return nil },
	}
	assert.Equal(t, epoch, c.Time())
	require.NoError(t, c.WaitUntil(context.Background(), epoch.Add(time.Second)))
	assert.Equal(t, epoch.Add(time.Second), got)
}

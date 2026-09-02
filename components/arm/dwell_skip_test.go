package arm

import (
	"context"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"

	"so_arm/internal/servo"
)

// A densified stream has sub-degree segments against a multi-degree lookahead, so the goal
// may legitimately lead by many waypoints. Reading the arm once per waypoint to discover the
// dwell would return immediately is pure bus traffic — one SyncWrite plus one SyncRead per
// waypoint, and each transaction pays the bus's ~1ms command gap.
func TestDenseStreamSkipsDwellReadsWithinTheLookahead(t *testing.T) {
	// Ten waypoints of 0.2 deg: 2.0 deg of total travel, inside the 3.0 deg lookahead the
	// default profile derives. Every goal in the stream is therefore provably within the
	// lookahead of the seeding read, so not one of them needs a read of its own.
	const n = 10
	stream := make([][]referenceframe.Input, n)
	for i := range stream {
		stream[i] = []referenceframe.Input{utils.DegToRad(0.2 * float64(i+1)), 0, 0, 0, 0}
	}

	s, calls := dwellTestArm(t, [][]float64{{0, 0, 0, 0, 0}})

	require.NoError(t, s.MoveThroughJointPositions(context.Background(), stream, nil, nil))

	// Without the skip this is 1 seeding read plus 9 dwell reads, one per intermediate
	// waypoint, every one of which would have returned on its first poll.
	assert.Equal(t, 1, calls(),
		"a stream whose whole travel fits inside the lookahead needs only the seeding read")

	// The skip hands the COMMANDED waypoint forward as the next travel reference instead of
	// a live read, which is only safe while a low-k joint is still commanded a speed the
	// servo executes. Without that floor this optimisation silently starves such a joint.
	profiles := servo.CoordinatedProfiles([]float64{20, 0.2}, 50, servo.DefaultAccelDegsPerSecSq, nil)
	assert.GreaterOrEqual(t, profiles[1].SpeedSteps,
		servo.DegPerSecToStepsPerSec(servo.MinExecutableSpeedDegsPerSec),
		"the predictive skip depends on MinExecutableSpeedDegsPerSec")
}

// The skip must never be taken on a bound that could understate the remaining distance: a
// joint may have REVERSED since the last read, so the estimate carries a speed*elapsed
// allowance for motion in any direction. Without it, a stale position would authorise skips
// indefinitely and the stream would degenerate into the unpaced flythrough the dwell exists
// to prevent.
func TestDwellSkipIsBoundedBySpeedAndElapsedTime(t *testing.T) {
	// One waypoint far outside any lookahead the default profile derives, so no skip is
	// admissible and the dwell must actually run.
	far := []referenceframe.Input{utils.DegToRad(90), 0, 0, 0, 0}
	next := []referenceframe.Input{utils.DegToRad(180), 0, 0, 0, 0}

	s, calls := dwellTestArm(t, [][]float64{{0, 0, 0, 0, 0}})
	require.NoError(t, s.MoveThroughJointPositions(context.Background(),
		[][]referenceframe.Input{far, next}, nil, nil))

	// Seeding read plus at least one dwell poll: 90 deg cannot be skipped.
	assert.GreaterOrEqual(t, calls(), 2, "a goal outside the lookahead must be dwelled on")
}

// The skip and the dwell must agree on the lookahead, including one supplied by MoveOptions.
func TestDwellSkipHonoursAConfiguredLookahead(t *testing.T) {
	w0 := []referenceframe.Input{utils.DegToRad(4), 0, 0, 0, 0}
	w1 := []referenceframe.Input{utils.DegToRad(8), 0, 0, 0, 0}

	// 4 deg of travel. Derived at the default profile the lookahead is 3.0, so no skip; with
	// an explicit 6.0 override the first waypoint is inside it and is skipped.
	for _, tc := range []struct {
		name      string
		lookahead float64
		wantSkip  bool
	}{
		{"tight lookahead dwells", 0, false},
		{"wide lookahead skips", 6.0, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, calls := dwellTestArm(t, [][]float64{{0, 0, 0, 0, 0}})
			s.lookaheadDeg = tc.lookahead
			require.NoError(t, s.MoveThroughJointPositions(context.Background(),
				[][]referenceframe.Input{w0, w1}, nil, nil))
			if tc.wantSkip {
				assert.Equal(t, 1, calls(), "inside the configured lookahead: no dwell read")
			} else {
				assert.Greater(t, calls(), 1, "outside the lookahead: the dwell must read")
			}
		})
	}
}

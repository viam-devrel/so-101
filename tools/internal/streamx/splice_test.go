package streamx

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
)

// pts builds a 2-joint trajectory: point i at i*100ms with positions (i, 2i).
func pts(n int) []arm.TrajectoryPoint {
	out := make([]arm.TrajectoryPoint, n)
	for i := range out {
		f := float64(i)
		out[i] = arm.TrajectoryPoint{Time: time.Duration(i) * 100 * time.Millisecond, Positions: []float64{f, 2 * f}}
	}
	return out
}

func TestAtClampsAndInterpolates(t *testing.T) {
	p := pts(4) // 0, 100, 200, 300 ms
	assert.Equal(t, []float64{0, 0}, At(p, -time.Second), "before the first sample")
	assert.Equal(t, []float64{3, 6}, At(p, time.Minute), "after the last sample")
	assert.Equal(t, []float64{2, 4}, At(p, 200*time.Millisecond), "exactly on a sample")
	assert.InDeltaSlice(t, []float64{1.5, 3}, At(p, 150*time.Millisecond), 1e-12, "midpoint")
	assert.InDeltaSlice(t, []float64{0.25, 0.5}, At(p, 25*time.Millisecond), 1e-12)
	assert.Nil(t, At(nil, 0))
}

func TestAtReturnsACopy(t *testing.T) {
	p := pts(2)
	q := At(p, 0)
	q[0] = 99
	assert.Equal(t, []float64{0, 0}, []float64(p[0].Positions))
}

func TestWindow(t *testing.T) {
	p := pts(5) // 0..400 ms
	assert.Equal(t, p[0:3], Window(p, 0, 250*time.Millisecond), "some due")
	assert.Equal(t, p[3:5], Window(p, 3, time.Second), "the rest")
	assert.Empty(t, Window(p, 5, time.Second), "sent == len")
	assert.Empty(t, Window(p, 2, 200*time.Millisecond), "a point exactly at upTo is NOT included")
	assert.Equal(t, p[0:1], Window(p, 0, time.Nanosecond), "any positive runway sends point 0")
	assert.Empty(t, Window(p, 0, 0), "none due")
	assert.Empty(t, Window(p, 3, 100*time.Millisecond), "k < sent clamps to nothing, never a negative slice")
}

func TestSpliceMidTrajectory(t *testing.T) {
	cur := pts(6)    // 0..500 ms
	newPts := pts(3) // 0, 100, 200 ms
	tStitch := 250 * time.Millisecond
	got, err := Splice(cur, newPts, tStitch)
	require.NoError(t, err)
	require.Len(t, got, 3+3, "prefix Time < 250ms (0,100,200) + 3 new")
	for i := 0; i < 3; i++ {
		assert.Equal(t, cur[i], got[i])
	}
	assert.Equal(t, 250*time.Millisecond, got[3].Time)
	assert.Equal(t, 350*time.Millisecond, got[4].Time)
	assert.Equal(t, 450*time.Millisecond, got[5].Time)
	assert.Equal(t, []float64{2, 4}, []float64(got[5].Positions), "new positions untouched")
	for i := 1; i < len(got); i++ {
		assert.Less(t, got[i-1].Time, got[i].Time, "strictly increasing")
	}
	assert.Len(t, cur, 6, "cur is not mutated")
	assert.Equal(t, 300*time.Millisecond, cur[3].Time)
}

func TestSpliceAStitchOnASampleDropsThatSample(t *testing.T) {
	// The prefix predicate is strict: a cur point AT tStitch would collide with newPts[0].
	got, err := Splice(pts(4), pts(2), 200*time.Millisecond)
	require.NoError(t, err)
	require.Len(t, got, 2+2)
	assert.Equal(t, 100*time.Millisecond, got[1].Time)
	assert.Equal(t, 200*time.Millisecond, got[2].Time)
}

func TestSplicePastTheEndKeepsTheWholePrefix(t *testing.T) {
	cur := pts(3) // ends at 200 ms
	got, err := Splice(cur, pts(2), time.Second)
	require.NoError(t, err)
	require.Len(t, got, 3+2)
	assert.Equal(t, time.Second, got[3].Time)
	assert.Equal(t, time.Second+100*time.Millisecond, got[4].Time)
}

func TestSpliceRejectsBadInput(t *testing.T) {
	_, err := Splice(pts(3), nil, time.Second)
	assert.ErrorContains(t, err, "no points")

	late := pts(2)
	late[0].Time = time.Millisecond
	_, err = Splice(pts(3), late, time.Second)
	assert.ErrorContains(t, err, "Time 0")
}

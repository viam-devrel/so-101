package main

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestDensifyInterpolatesLinearlyAndKeepsEndpoints(t *testing.T) {
	frames := [][]float64{{0, 0}, {1, 2}, {2, 4}} // 10 Hz: 0.2 s total
	out, rate := densify(frames, 10, 40)          // 4x -> 9 samples over 0.2 s
	assert.Equal(t, 40.0, rate)
	require.Len(t, out, 9)
	assert.Equal(t, []float64{0, 0}, out[0])
	assert.InDeltaSlice(t, []float64{0.25, 0.5}, out[1], 1e-9)
	assert.InDeltaSlice(t, []float64{1, 2}, out[4], 1e-9)
	assert.Equal(t, []float64{2, 4}, out[8])

	// Non-integer ratio: the last sample must still be the last frame, not a near-miss.
	out, rate = densify([][]float64{{0}, {1}, {2}, {3}}, 10, 25)
	assert.Equal(t, 25.0, rate)
	assert.Equal(t, []float64{3}, out[len(out)-1])
}

func TestDensifyIsANoOpAtOrBelowTheSourceRate(t *testing.T) {
	frames := [][]float64{{0}, {1}}
	out, rate := densify(frames, 10, 10)
	assert.Equal(t, 10.0, rate)
	assert.Equal(t, frames, out)
}

func TestToPointsTimesStartAtZeroAndIncrease(t *testing.T) {
	pts := toPoints([][]float64{{0}, {1}, {2}}, 100)
	require.Len(t, pts, 3)
	assert.Equal(t, time.Duration(0), pts[0].Time)
	assert.Equal(t, 10*time.Millisecond, pts[1].Time)
	assert.Equal(t, 20*time.Millisecond, pts[2].Time)
}

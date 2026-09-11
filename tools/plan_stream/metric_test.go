package main

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// rad builds a radian joint vector from degrees, so the tests read in the metric's output unit.
func rad(deg ...float64) []float64 {
	out := make([]float64, len(deg))
	for i, d := range deg {
		out[i] = deg2rad(d)
	}
	return out
}

// Two-segment polyline in degrees: (0,0) -> (10,0) -> (10,10).
var polyline = [][]float64{rad(0, 0), rad(10, 0), rad(10, 10)}

func TestPathDeviationIsZeroOnThePath(t *testing.T) {
	trace := []sample{
		{t: 0, q: rad(0, 0)},
		{t: time.Second, q: rad(5, 0)},
		{t: 2 * time.Second, q: rad(10, 5)},
		{t: 3 * time.Second, q: rad(10, 10)},
	}
	d := pathDeviation(trace, polyline)
	assert.InDelta(t, 0, d.mean, 1e-9)
	assert.InDelta(t, 0, d.p95, 1e-9)
	assert.InDelta(t, 0, d.max, 1e-9)
	assert.InDeltaSlice(t, []float64{0, 0}, d.finalErr, 1e-9)
}

func TestPathDeviationMeasuresPerpendicularOffsetToASegmentInterior(t *testing.T) {
	d := pathDeviation([]sample{{q: rad(5, 1)}}, polyline)
	assert.InDelta(t, 1, d.mean, 1e-9)
	assert.InDelta(t, 1, d.max, 1e-9)
}

func TestPathDeviationPastTheEndIsDistanceToTheEndpoint(t *testing.T) {
	// (10,13) is 3 deg beyond the last waypoint; a clamped projection must not extrapolate.
	d := pathDeviation([]sample{{q: rad(10, 13)}}, polyline)
	assert.InDelta(t, 3, d.max, 1e-9)
	assert.InDeltaSlice(t, []float64{0, 3}, d.finalErr, 1e-9)
}

func TestPathDeviationP95IsTheNearestRankOver20Samples(t *testing.T) {
	// Samples before the start clamp to the first waypoint, k deg away (an offset beside the
	// first segment would fold onto the second one past 5 deg).
	trace := make([]sample, 20)
	for k := range trace {
		trace[k] = sample{q: rad(-float64(k+1), 0)} // distances 1..20 deg
	}
	d := pathDeviation(trace, polyline)
	assert.InDelta(t, 19, d.p95, 1e-9)
	assert.InDelta(t, 20, d.max, 1e-9)
	assert.InDelta(t, 10.5, d.mean, 1e-9)
}

func TestPathDeviationEmptyTraceIsZeroValue(t *testing.T) {
	var d deviation
	require.NotPanics(t, func() { d = pathDeviation(nil, polyline) })
	assert.Equal(t, deviation{}, d)
}

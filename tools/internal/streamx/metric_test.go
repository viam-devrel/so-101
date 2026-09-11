package streamx

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/utils"
)

// rad builds a radian joint vector from degrees, so the tests read in the metric's output unit.
func rad(deg ...float64) []float64 {
	out := make([]float64, len(deg))
	for i, d := range deg {
		out[i] = utils.DegToRad(d)
	}
	return out
}

// Two-segment polyline in degrees: (0,0) -> (10,0) -> (10,10).
var polyline = [][]float64{rad(0, 0), rad(10, 0), rad(10, 10)}

func TestPathDeviationIsZeroOnThePath(t *testing.T) {
	trace := []Sample{
		{T: 0, Q: rad(0, 0)},
		{T: time.Second, Q: rad(5, 0)},
		{T: 2 * time.Second, Q: rad(10, 5)},
		{T: 3 * time.Second, Q: rad(10, 10)},
	}
	d := PathDeviation(trace, polyline)
	assert.InDelta(t, 0, d.Mean, 1e-9)
	assert.InDelta(t, 0, d.P95, 1e-9)
	assert.InDelta(t, 0, d.Max, 1e-9)
	assert.InDeltaSlice(t, []float64{0, 0}, d.FinalErr, 1e-9)
}

func TestPathDeviationMeasuresPerpendicularOffsetToASegmentInterior(t *testing.T) {
	d := PathDeviation([]Sample{{Q: rad(5, 1)}}, polyline)
	assert.InDelta(t, 1, d.Mean, 1e-9)
	assert.InDelta(t, 1, d.Max, 1e-9)
}

func TestPathDeviationPastTheEndIsDistanceToTheEndpoint(t *testing.T) {
	// (10,13) is 3 deg beyond the last waypoint; a clamped projection must not extrapolate.
	d := PathDeviation([]Sample{{Q: rad(10, 13)}}, polyline)
	assert.InDelta(t, 3, d.Max, 1e-9)
	assert.InDeltaSlice(t, []float64{0, 3}, d.FinalErr, 1e-9)
}

func TestPathDeviationP95IsTheNearestRankOver20Samples(t *testing.T) {
	// Samples before the start clamp to the first waypoint, k deg away (an offset beside the
	// first segment would fold onto the second one past 5 deg).
	trace := make([]Sample, 20)
	for k := range trace {
		trace[k] = Sample{Q: rad(-float64(k+1), 0)} // distances 1..20 deg
	}
	d := PathDeviation(trace, polyline)
	assert.InDelta(t, 19, d.P95, 1e-9)
	assert.InDelta(t, 20, d.Max, 1e-9)
	assert.InDelta(t, 10.5, d.Mean, 1e-9)
}

func TestPathDeviationEmptyTraceIsZeroValue(t *testing.T) {
	var d Deviation
	require.NotPanics(t, func() { d = PathDeviation(nil, polyline) })
	assert.Equal(t, Deviation{}, d)
}

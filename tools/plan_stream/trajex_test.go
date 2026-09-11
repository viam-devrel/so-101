package main

import (
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/ml"
	"gorgonia.org/tensor"
)

func TestTrajexInputsShapesDtypeAndRadians(t *testing.T) {
	wp := [][]float64{{0, 0, 0, 0, 0}, {0.1, 0.2, 0.3, 0.4, 0.5}, {0.2, 0.4, 0.6, 0.8, 1.0}}
	in := trajexInputs(wp, 90, 180, 0.5, 100)

	require.Len(t, in, 7)
	for _, k := range []string{inWaypoints, inVel, inAcc, inPathTol, inColinear, inDedup} {
		require.Contains(t, in, k)
		assert.Equal(t, tensor.Float64, in[k].Dtype(), k)
	}
	assert.Equal(t, tensor.Int64, in[inHz].Dtype(), "the C++ service wants an int64 sampling rate")
	assert.Equal(t, tensor.Shape{3, 5}, in[inWaypoints].Shape())
	assert.Equal(t, tensor.Shape{5}, in[inVel].Shape())
	assert.Equal(t, tensor.Shape{5}, in[inAcc].Shape())
	assert.Equal(t, tensor.Shape{1}, in[inPathTol].Shape())
	assert.Equal(t, tensor.Shape{1}, in[inColinear].Shape())
	assert.Equal(t, tensor.Shape{1}, in[inDedup].Shape())
	assert.Equal(t, tensor.Shape{1}, in[inHz].Shape())

	// Waypoints pass through untouched (already radians); limits/tolerance are deg -> rad.
	assert.Equal(t, []float64{0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.2, 0.4, 0.6, 0.8, 1.0},
		in[inWaypoints].Data().([]float64))
	assert.InDeltaSlice(t, []float64{math.Pi / 2, math.Pi / 2, math.Pi / 2, math.Pi / 2, math.Pi / 2},
		in[inVel].Data().([]float64), 1e-12)
	assert.InDeltaSlice(t, []float64{math.Pi, math.Pi, math.Pi, math.Pi, math.Pi},
		in[inAcc].Data().([]float64), 1e-12)
	assert.InDelta(t, 0.5*math.Pi/180, in[inPathTol].Data().([]float64)[0], 1e-12)
	assert.Equal(t, []float64{0}, in[inColinear].Data().([]float64), "0 = colinearization off")
	assert.Equal(t, []float64{dedupToleranceRads}, in[inDedup].Data().([]float64))
	assert.Equal(t, []int64{100}, in[inHz].Data().([]int64))
}

func TestTrajexPointsPairsTimesWithConfigurationRows(t *testing.T) {
	out := ml.Tensors{
		outTimes:   tensor.New(tensor.WithShape(3), tensor.WithBacking([]float64{0, 0.01, 0.02})),
		outConfigs: tensor.New(tensor.WithShape(3, 2), tensor.WithBacking([]float64{0, 0, 1, 2, 3, 4})),
		// Extra outputs are ignored.
		"velocities_rads_per_sec": tensor.New(tensor.WithShape(3, 2), tensor.WithBacking(make([]float64, 6))),
	}
	pts, err := trajexPoints(out)
	require.NoError(t, err)
	require.Len(t, pts, 3)
	assert.Equal(t, time.Duration(0), pts[0].Time)
	assert.Equal(t, 10*time.Millisecond, pts[1].Time)
	assert.Equal(t, 20*time.Millisecond, pts[2].Time)
	assert.Equal(t, []float64{0, 0}, []float64(pts[0].Positions))
	assert.Equal(t, []float64{1, 2}, []float64(pts[1].Positions))
	assert.Equal(t, []float64{3, 4}, []float64(pts[2].Positions))
}

func TestTrajexPointsRebasesAFirstSampleThatIsNotAtZero(t *testing.T) {
	out := ml.Tensors{
		outTimes:   tensor.New(tensor.WithShape(3), tensor.WithBacking([]float64{0.00996, 0.01996, 0.02996})),
		outConfigs: tensor.New(tensor.WithShape(3, 1), tensor.WithBacking([]float64{0, 1, 2})),
	}
	pts, err := trajexPoints(out)
	require.NoError(t, err)
	assert.Equal(t, time.Duration(0), pts[0].Time, "the arm requires the stream to start at 0")
	assert.Equal(t, 10*time.Millisecond, pts[1].Time)
	assert.Equal(t, 20*time.Millisecond, pts[2].Time)
}

func TestTrajexPointsErrors(t *testing.T) {
	_, err := trajexPoints(ml.Tensors{
		outTimes: tensor.New(tensor.WithShape(2), tensor.WithBacking([]float64{0, 0.01})),
	})
	assert.ErrorContains(t, err, outConfigs)

	_, err = trajexPoints(ml.Tensors{
		outTimes:   tensor.New(tensor.WithShape(2), tensor.WithBacking([]float64{0, 0.01})),
		outConfigs: tensor.New(tensor.WithShape(3, 2), tensor.WithBacking(make([]float64, 6))),
	})
	assert.ErrorContains(t, err, "does not match")

	_, err = trajexPoints(ml.Tensors{
		outTimes:   tensor.New(tensor.WithShape(2), tensor.WithBacking([]float32{0, 0.01})),
		outConfigs: tensor.New(tensor.WithShape(2, 2), tensor.WithBacking(make([]float64, 4))),
	})
	assert.ErrorContains(t, err, "float64")
}

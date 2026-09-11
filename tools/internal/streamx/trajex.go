// Package streamx holds the pure helpers shared by tools/plan_stream and
// tools/online_stream: trajex tensors, the path metric, goal/obstacle parsing, and the
// splice math for a live stream. Importable only from under tools/.
package streamx

import (
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/ml"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
	"gorgonia.org/tensor"
)

// trajex ABI tensor names (its capi.cpp). Literals on purpose: the Go bindings are cgo.
const (
	InWaypoints = "waypoints_rads"
	InVel       = "velocity_limits_rads_per_sec"
	InAcc       = "acceleration_limits_rads_per_sec2"
	InPathTol   = "path_tolerance_delta_rads"
	InColinear  = "path_colinearization_ratio"
	InDedup     = "waypoint_deduplication_tolerance_rads"
	InHz        = "trajectory_sampling_freq_hz"
	OutTimes    = "sample_times_sec"
	OutConfigs  = "configurations_rads"

	// DedupToleranceRads matches rdk's sim arm; colinearization 0 means "off" to the service.
	DedupToleranceRads = 1e-5
)

// Inputs builds every tensor the registry trajex service requires (the C++ service, unlike
// the Go adapter, rejects a missing optional and wants the sampling frequency as int64).
// waypoints are radians; velDeg/accDeg/pathTolDeg are degrees.
func Inputs(waypoints [][]float64, velDeg, accDeg, pathTolDeg, hz float64) ml.Tensors {
	n, dof := len(waypoints), len(waypoints[0])
	flat := make([]float64, 0, n*dof)
	for _, w := range waypoints {
		flat = append(flat, w...)
	}
	vel := make([]float64, dof)
	acc := make([]float64, dof)
	for i := range vel {
		vel[i] = utils.DegToRad(velDeg)
		acc[i] = utils.DegToRad(accDeg)
	}
	return ml.Tensors{
		InWaypoints: tensor.New(tensor.WithShape(n, dof), tensor.WithBacking(flat)),
		InVel:       tensor.New(tensor.WithShape(dof), tensor.WithBacking(vel)),
		InAcc:       tensor.New(tensor.WithShape(dof), tensor.WithBacking(acc)),
		InPathTol:   tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{utils.DegToRad(pathTolDeg)})),
		InColinear:  tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{0})),
		InDedup:     tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{DedupToleranceRads})),
		InHz:        tensor.New(tensor.WithShape(1), tensor.WithBacking([]int64{int64(math.Round(hz))})),
	}
}

// Points turns sample_times_sec [n] + configurations_rads [n,dof] into arm points. The
// times are rebased so the first is 0: on multi-waypoint paths trajex has returned a first
// sample near one period in, and the arm requires the stream to start at Time 0.
func Points(out ml.Tensors) ([]arm.TrajectoryPoint, error) {
	times, err := float64s(out, OutTimes)
	if err != nil {
		return nil, err
	}
	flat, err := float64s(out, OutConfigs)
	if err != nil {
		return nil, err
	}
	shape := out[OutConfigs].Shape()
	if len(shape) != 2 || shape[0] != len(times) {
		return nil, fmt.Errorf("trajex: %s shape %v does not match %d sample times", OutConfigs, shape, len(times))
	}
	dof := shape[1]
	points := make([]arm.TrajectoryPoint, len(times))
	for i, t := range times {
		row := make([]referenceframe.Input, dof)
		copy(row, flat[i*dof:(i+1)*dof])
		points[i] = arm.TrajectoryPoint{Time: time.Duration(math.Round((t - times[0]) * float64(time.Second))), Positions: row}
	}
	return points, nil
}

// float64s returns key's backing slice; Data() is []float64 for any shape with >= 1 dim.
func float64s(ts ml.Tensors, key string) ([]float64, error) {
	t, ok := ts[key]
	if !ok {
		return nil, fmt.Errorf("trajex output missing %q", key)
	}
	data, ok := t.Data().([]float64)
	if !ok {
		return nil, fmt.Errorf("trajex output %q: want float64, got %v", key, t.Dtype())
	}
	return data, nil
}

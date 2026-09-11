package main

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
	inWaypoints = "waypoints_rads"
	inVel       = "velocity_limits_rads_per_sec"
	inAcc       = "acceleration_limits_rads_per_sec2"
	inPathTol   = "path_tolerance_delta_rads"
	inColinear  = "path_colinearization_ratio"
	inDedup     = "waypoint_deduplication_tolerance_rads"
	inHz        = "trajectory_sampling_freq_hz"
	outTimes    = "sample_times_sec"
	outConfigs  = "configurations_rads"

	// dedupToleranceRads matches rdk's sim arm; colinearization 0 means "off" to the service.
	dedupToleranceRads = 1e-5
)

// trajexInputs builds every tensor the registry trajex service requires (the C++ service, unlike
// the Go adapter, rejects a missing optional and wants the sampling frequency as int64).
// waypoints are radians (from armWaypoints); velDeg/accDeg/pathTolDeg are degrees.
func trajexInputs(waypoints [][]float64, velDeg, accDeg, pathTolDeg, hz float64) ml.Tensors {
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
		inWaypoints: tensor.New(tensor.WithShape(n, dof), tensor.WithBacking(flat)),
		inVel:       tensor.New(tensor.WithShape(dof), tensor.WithBacking(vel)),
		inAcc:       tensor.New(tensor.WithShape(dof), tensor.WithBacking(acc)),
		inPathTol:   tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{utils.DegToRad(pathTolDeg)})),
		inColinear:  tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{0})),
		inDedup:     tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{dedupToleranceRads})),
		inHz:        tensor.New(tensor.WithShape(1), tensor.WithBacking([]int64{int64(math.Round(hz))})),
	}
}

// trajexPoints turns sample_times_sec [n] + configurations_rads [n,dof] into arm points.
func trajexPoints(out ml.Tensors) ([]arm.TrajectoryPoint, error) {
	times, err := float64s(out, outTimes)
	if err != nil {
		return nil, err
	}
	flat, err := float64s(out, outConfigs)
	if err != nil {
		return nil, err
	}
	shape := out[outConfigs].Shape()
	if len(shape) != 2 || shape[0] != len(times) {
		return nil, fmt.Errorf("trajex: %s shape %v does not match %d sample times", outConfigs, shape, len(times))
	}
	dof := shape[1]
	points := make([]arm.TrajectoryPoint, len(times))
	for i, t := range times {
		row := make([]referenceframe.Input, dof)
		copy(row, flat[i*dof:(i+1)*dof])
		points[i] = arm.TrajectoryPoint{Time: time.Duration(math.Round(t * float64(time.Second))), Positions: row}
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

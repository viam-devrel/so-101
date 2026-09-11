package main

import (
	"math"
	"sort"
	"time"

	"go.viam.com/rdk/utils"
)

// sample is one JointPositions read: t since the run started, q in radians.
type sample struct {
	t time.Duration
	q []float64
}

// deviation is a trace's joint-space distance from the planned polyline, in degrees.
type deviation struct {
	mean, p95, max float64
	finalErr       []float64 // per joint, |last sample - last waypoint|
}

// pathDeviation scores trace against pathRad (both radians; output degrees): each sample's
// L2 distance over all joints to the nearest point on any polyline segment. Both runs are
// scored against the same polyline, so the numbers are comparable.
func pathDeviation(trace []sample, pathRad [][]float64) deviation {
	var d deviation
	if len(trace) == 0 || len(pathRad) == 0 {
		return d
	}
	path := make([][]float64, len(pathRad))
	for i, w := range pathRad {
		path[i] = toDeg(w)
	}
	dists := make([]float64, len(trace))
	for i, s := range trace {
		dists[i] = distToPolyline(toDeg(s.q), path)
		d.mean += dists[i]
		d.max = math.Max(d.max, dists[i])
	}
	d.mean /= float64(len(dists))
	sort.Float64s(dists)
	d.p95 = dists[int(math.Ceil(0.95*float64(len(dists))))-1] // nearest rank
	last, goal := toDeg(trace[len(trace)-1].q), path[len(path)-1]
	d.finalErr = make([]float64, len(goal))
	for j := range goal {
		d.finalErr[j] = math.Abs(last[j] - goal[j])
	}
	return d
}

func distToPolyline(p []float64, path [][]float64) float64 {
	if len(path) == 1 {
		return distToSegment(p, path[0], path[0])
	}
	best := math.Inf(1)
	for i := 0; i+1 < len(path); i++ {
		best = math.Min(best, distToSegment(p, path[i], path[i+1]))
	}
	return best
}

// distToSegment is the distance from p to segment ab, projection clamped to [a, b].
func distToSegment(p, a, b []float64) float64 {
	var ab2, apab float64
	for j := range a {
		ab := b[j] - a[j]
		ab2 += ab * ab
		apab += (p[j] - a[j]) * ab
	}
	t := 0.0
	if ab2 > 0 {
		t = math.Max(0, math.Min(1, apab/ab2))
	}
	var sum float64
	for j := range a {
		d := p[j] - (a[j] + t*(b[j]-a[j]))
		sum += d * d
	}
	return math.Sqrt(sum)
}

func toDeg(rad []float64) []float64 {
	out := make([]float64, len(rad))
	for i, r := range rad {
		out[i] = utils.RadToDeg(r)
	}
	return out
}

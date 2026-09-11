package streamx

import (
	"math"
	"sort"
	"time"

	"go.viam.com/rdk/utils"
)

// Sample is one JointPositions read: T since the run started, Q in radians.
type Sample struct {
	T time.Duration
	Q []float64
}

// Deviation is a trace's joint-space distance from the planned polyline, in degrees.
type Deviation struct {
	Mean, P95, Max float64
	FinalErr       []float64 // per joint, |last sample - last waypoint|
}

// PathDeviation scores trace against pathRad (both radians; output degrees): each sample's
// L2 distance over all joints to the nearest point on any polyline segment. Runs scored
// against the same polyline are comparable.
func PathDeviation(trace []Sample, pathRad [][]float64) Deviation {
	var d Deviation
	if len(trace) == 0 || len(pathRad) == 0 {
		return d
	}
	path := make([][]float64, len(pathRad))
	for i, w := range pathRad {
		path[i] = toDeg(w)
	}
	dists := make([]float64, len(trace))
	for i, s := range trace {
		dists[i] = distToPolyline(toDeg(s.Q), path)
		d.Mean += dists[i]
		d.Max = math.Max(d.Max, dists[i])
	}
	d.Mean /= float64(len(dists))
	sort.Float64s(dists)
	d.P95 = dists[int(math.Ceil(0.95*float64(len(dists))))-1] // nearest rank
	last, goal := toDeg(trace[len(trace)-1].Q), path[len(path)-1]
	d.FinalErr = make([]float64, len(goal))
	for j := range goal {
		d.FinalErr[j] = math.Abs(last[j] - goal[j])
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

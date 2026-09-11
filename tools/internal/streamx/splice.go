package streamx

import (
	"fmt"
	"time"

	"go.viam.com/rdk/components/arm"
)

// At is the trajectory's position at t: clamped to the first/last sample outside its span,
// linear between neighbouring samples inside it. Returns a fresh slice.
func At(points []arm.TrajectoryPoint, t time.Duration) []float64 {
	if len(points) == 0 {
		return nil
	}
	if t <= points[0].Time {
		return append([]float64(nil), points[0].Positions...)
	}
	last := points[len(points)-1]
	if t >= last.Time {
		return append([]float64(nil), last.Positions...)
	}
	i := 1
	for points[i].Time < t {
		i++
	}
	a, b := points[i-1], points[i]
	f := float64(t-a.Time) / float64(b.Time-a.Time)
	out := make([]float64, len(a.Positions))
	for j := range out {
		out[j] = a.Positions[j] + f*(b.Positions[j]-a.Positions[j])
	}
	return out
}

// Window is points[sent:k], k the first index with Time >= upTo: the not-yet-sent points
// strictly before upTo. Strict, to match Splice's prefix predicate, so a point sent under a
// clamped window is never one Splice drops. k is clamped to sent so a shrinking upTo cannot
// produce a negative slice.
func Window(points []arm.TrajectoryPoint, sent int, upTo time.Duration) []arm.TrajectoryPoint {
	k := sent
	for k < len(points) && points[k].Time < upTo {
		k++
	}
	return points[sent:k]
}

// Splice keeps cur's points with Time < tStitch and appends newPts shifted by tStitch.
// newPts must start at Time 0. The result is a new slice; cur is not modified, since the
// producer has already handed slices of it to the RPC.
func Splice(cur, newPts []arm.TrajectoryPoint, tStitch time.Duration) ([]arm.TrajectoryPoint, error) {
	if len(newPts) == 0 {
		return nil, fmt.Errorf("splice: no points to splice in")
	}
	if newPts[0].Time != 0 {
		return nil, fmt.Errorf("splice: new points must start at Time 0, got %v", newPts[0].Time)
	}
	k := 0
	for k < len(cur) && cur[k].Time < tStitch {
		k++
	}
	out := make([]arm.TrajectoryPoint, 0, k+len(newPts))
	out = append(out, cur[:k]...)
	for _, p := range newPts {
		p.Time += tStitch
		out = append(out, p)
	}
	return out, nil
}

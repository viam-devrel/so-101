package streamx

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// Goal is a -goal flag: a position in the <arm>_origin frame, mm, with an optional
// orientation vector. A nil Orient keeps the arm's current orientation.
type Goal struct {
	Pos    r3.Vector
	Orient *spatialmath.OrientationVector
}

// ParseGoal reads "x,y,z" or "x,y,z,ox,oy,oz" (mm; orientation vector with theta 0).
func ParseGoal(s string) (Goal, error) {
	fields := strings.Split(s, ",")
	if len(fields) != 3 && len(fields) != 6 {
		return Goal{}, fmt.Errorf("goal %q: want x,y,z or x,y,z,ox,oy,oz", s)
	}
	v, err := floats(fields)
	if err != nil {
		return Goal{}, fmt.Errorf("goal %q: %w", s, err)
	}
	g := Goal{Pos: r3.Vector{X: v[0], Y: v[1], Z: v[2]}}
	if len(v) == 6 {
		g.Orient = &spatialmath.OrientationVector{OX: v[3], OY: v[4], OZ: v[5]}
		if err := g.Orient.IsValid(); err != nil {
			return Goal{}, fmt.Errorf("goal %q: %w", s, err)
		}
	}
	return g, nil
}

// Pose is the goal as a pose, taking fallback's orientation when the goal has none.
func (g Goal) Pose(fallback spatialmath.Orientation) spatialmath.Pose {
	if g.Orient != nil {
		return spatialmath.NewPose(g.Pos, g.Orient)
	}
	return spatialmath.NewPose(g.Pos, fallback)
}

// ParseObstacle reads "x,y,z,dx,dy,dz": an axis-aligned box centred at x,y,z with those
// side lengths, all mm, labelled obstacle<i+1>.
func ParseObstacle(s string, i int) (spatialmath.Geometry, error) {
	fields := strings.Split(s, ",")
	if len(fields) != 6 {
		return nil, fmt.Errorf("obstacle %q: want x,y,z,dx,dy,dz", s)
	}
	v, err := floats(fields)
	if err != nil {
		return nil, fmt.Errorf("obstacle %q: %w", s, err)
	}
	return box(v, i)
}

// box builds obstacle<i+1> from [x,y,z,dx,dy,dz].
func box(v []float64, i int) (spatialmath.Geometry, error) {
	if v[3] <= 0 || v[4] <= 0 || v[5] <= 0 {
		return nil, fmt.Errorf("obstacle: side lengths must be positive, got %v", v[3:6])
	}
	return spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: v[0], Y: v[1], Z: v[2]}),
		r3.Vector{X: v[3], Y: v[4], Z: v[5]}, fmt.Sprintf("obstacle%d", i+1))
}

func floats(fields []string) ([]float64, error) {
	v := make([]float64, len(fields))
	for i, f := range fields {
		x, err := strconv.ParseFloat(strings.TrimSpace(f), 64)
		if err != nil {
			return nil, err
		}
		v[i] = x
	}
	return v, nil
}

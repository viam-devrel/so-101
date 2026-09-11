package streamx

import (
	"fmt"
	"strings"
	"time"

	"go.viam.com/rdk/spatialmath"
)

// Event is new input arriving At seconds into the stream: the target becomes goal B, an
// obstacle appears, or both (two events due on one tick are applied as one replan).
type Event struct {
	At         time.Duration
	SwitchGoal bool
	Obstacle   spatialmath.Geometry // nil for a pure goal switch
}

// ParseObstacleAfter reads "seconds,x,y,z,dx,dy,dz": when the box appears, then the box
// (as ParseObstacle). i numbers the label after the obstacles already present.
func ParseObstacleAfter(s string, i int) (Event, error) {
	fields := strings.Split(s, ",")
	if len(fields) != 7 {
		return Event{}, fmt.Errorf("obstacle-after %q: want seconds,x,y,z,dx,dy,dz", s)
	}
	v, err := floats(fields)
	if err != nil {
		return Event{}, fmt.Errorf("obstacle-after %q: %w", s, err)
	}
	if v[0] < 0 {
		return Event{}, fmt.Errorf("obstacle-after %q: seconds must be >= 0", s)
	}
	b, err := box(v[1:], i)
	if err != nil {
		return Event{}, fmt.Errorf("obstacle-after %q: %w", s, err)
	}
	return Event{At: time.Duration(v[0] * float64(time.Second)), Obstacle: b}, nil
}

// Due returns the indices of the not-yet-fired events with At <= elapsed, in index order,
// and marks them fired. fired is the caller's, len(events).
func Due(events []Event, fired []bool, elapsed time.Duration) []int {
	var due []int
	for i, e := range events {
		if !fired[i] && e.At <= elapsed {
			fired[i] = true
			due = append(due, i)
		}
	}
	return due
}

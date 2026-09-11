package streamx

import (
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestParseObstacleAfter(t *testing.T) {
	e, err := ParseObstacleAfter("0.5,300,0,205,60,60,120", 1)
	require.NoError(t, err)
	assert.Equal(t, 500*time.Millisecond, e.At)
	assert.False(t, e.SwitchGoal)
	require.NotNil(t, e.Obstacle)
	assert.Equal(t, "obstacle2", e.Obstacle.Label(), "numbering continues after the initial obstacles")
	assert.Equal(t, r3.Vector{X: 300, Y: 0, Z: 205}, e.Obstacle.Pose().Point())

	_, err = ParseObstacleAfter("0.5,300,0,205", 0)
	assert.ErrorContains(t, err, "want seconds,x,y,z,dx,dy,dz")
	_, err = ParseObstacleAfter("abc,300,0,205,60,60,120", 0)
	assert.Error(t, err)
	_, err = ParseObstacleAfter("0.5,300,0,205,0,60,120", 0)
	assert.ErrorContains(t, err, "positive")
	_, err = ParseObstacleAfter("-1,300,0,205,60,60,120", 0)
	assert.ErrorContains(t, err, "seconds")
}

func TestDueCollectsEveryUnfiredEventUpToElapsedAndMarksThem(t *testing.T) {
	events := []Event{{At: 600 * time.Millisecond, SwitchGoal: true}, {At: 500 * time.Millisecond}, {At: 2 * time.Second}}
	fired := make([]bool, len(events))

	assert.Empty(t, Due(events, fired, 400*time.Millisecond), "none due")
	assert.Equal(t, []bool{false, false, false}, fired)

	assert.Equal(t, []int{0, 1}, Due(events, fired, 600*time.Millisecond), "two events on one tick, in index order")
	assert.Equal(t, []bool{true, true, false}, fired)

	assert.Empty(t, Due(events, fired, time.Second), "already fired are not returned again")

	assert.Equal(t, []int{2}, Due(events, fired, 2*time.Second), "At == elapsed is due")
	assert.Equal(t, []bool{true, true, true}, fired)
}

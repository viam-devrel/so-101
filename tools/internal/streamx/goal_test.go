package streamx

import (
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/spatialmath"
)

func TestParseGoal(t *testing.T) {
	g, err := ParseGoal("200, 0,150")
	require.NoError(t, err)
	assert.Equal(t, r3.Vector{X: 200, Y: 0, Z: 150}, g.Pos)
	assert.Nil(t, g.Orient, "3 fields keep the current orientation")

	g, err = ParseGoal("200,0,150,0,0,-1")
	require.NoError(t, err)
	assert.Equal(t, &spatialmath.OrientationVector{OX: 0, OY: 0, OZ: -1}, g.Orient)

	_, err = ParseGoal("200,0")
	assert.ErrorContains(t, err, "want x,y,z")
	_, err = ParseGoal("200,0,abc")
	assert.Error(t, err)
	_, err = ParseGoal("200,0,150,0,0,0")
	assert.ErrorContains(t, err, "normal of 0")
}

func TestGoalPoseFallsBackToTheGivenOrientation(t *testing.T) {
	fallback := &spatialmath.OrientationVector{OX: 1, OY: 0, OZ: 0}
	p := Goal{Pos: r3.Vector{X: 1, Y: 2, Z: 3}}.Pose(fallback)
	assert.Equal(t, r3.Vector{X: 1, Y: 2, Z: 3}, p.Point())
	assert.True(t, spatialmath.OrientationAlmostEqual(fallback, p.Orientation()))

	own := &spatialmath.OrientationVector{OX: 0, OY: 0, OZ: -1}
	p = Goal{Pos: r3.Vector{X: 1, Y: 2, Z: 3}, Orient: own}.Pose(fallback)
	assert.True(t, spatialmath.OrientationAlmostEqual(own, p.Orientation()))
}

func TestParseObstacle(t *testing.T) {
	box, err := ParseObstacle("250,0,100,50,50,200", 0)
	require.NoError(t, err)
	assert.Equal(t, "obstacle1", box.Label())
	assert.Equal(t, r3.Vector{X: 250, Y: 0, Z: 100}, box.Pose().Point())

	_, err = ParseObstacle("250,0,100", 0)
	assert.ErrorContains(t, err, "want x,y,z,dx,dy,dz")
	_, err = ParseObstacle("250,0,100,0,50,200", 0)
	assert.Error(t, err, "a zero side length is not a box")
}

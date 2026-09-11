package main

import (
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/spatialmath"
)

func TestParseGoal(t *testing.T) {
	g, err := parseGoal("200, 0,150")
	require.NoError(t, err)
	assert.Equal(t, r3.Vector{X: 200, Y: 0, Z: 150}, g.pos)
	assert.Nil(t, g.orient, "3 fields keep the current orientation")

	g, err = parseGoal("200,0,150,0,0,-1")
	require.NoError(t, err)
	assert.Equal(t, &spatialmath.OrientationVector{OX: 0, OY: 0, OZ: -1}, g.orient)

	_, err = parseGoal("200,0")
	assert.ErrorContains(t, err, "want x,y,z")
	_, err = parseGoal("200,0,abc")
	assert.Error(t, err)
	_, err = parseGoal("200,0,150,0,0,0")
	assert.ErrorContains(t, err, "normal of 0")
}

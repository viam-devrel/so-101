package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestArmWaypointsDecodesTheArmFramesInOrder(t *testing.T) {
	// Wire shape of motion's `plan` DoCommand at a Go client; other frames ride along empty.
	resp := []any{
		map[string]any{"arm": []any{0.1, 0.2, 0.3, 0.4, 0.5}, "gripper": []any{}, "world": []any{}},
		map[string]any{"arm": []any{0.6, 0.7, 0.8, 0.9, 1.0}, "gripper": []any{}, "world": []any{}},
	}
	got, err := armWaypoints(resp, "arm")
	require.NoError(t, err)
	assert.Equal(t, [][]float64{{0.1, 0.2, 0.3, 0.4, 0.5}, {0.6, 0.7, 0.8, 0.9, 1.0}}, got)
}

func TestArmWaypointsErrors(t *testing.T) {
	_, err := armWaypoints([]any{}, "arm")
	assert.ErrorContains(t, err, "no waypoints")

	_, err = armWaypoints([]any{map[string]any{"gripper": []any{}}}, "arm")
	assert.ErrorContains(t, err, `no frame "arm"`)

	_, err = armWaypoints([]any{map[string]any{"arm": []any{0.1, "x"}}}, "arm")
	assert.ErrorContains(t, err, "float64")

	_, err = armWaypoints("nope", "arm")
	assert.ErrorContains(t, err, "[]any")
}

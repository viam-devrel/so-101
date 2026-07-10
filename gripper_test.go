package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/spatialmath"
)

func TestSO101GripperConfigValidate(t *testing.T) {
	// An unset gripper_type is accepted (defaults to follower).
	_, _, err := (&SO101GripperConfig{Port: "/dev/ttyUSB0"}).Validate("")
	require.NoError(t, err)

	for _, gt := range []string{leaderGripper, followerGripper} {
		_, _, err := (&SO101GripperConfig{Port: "/dev/ttyUSB0", GripperType: gt}).Validate("")
		require.NoError(t, err, "gripper_type %q should be valid", gt)
	}

	_, _, err = (&SO101GripperConfig{Port: "/dev/ttyUSB0", GripperType: "bogus"}).Validate("")
	require.Error(t, err)
}

func TestGripperSetPositionPercent(t *testing.T) {
	// "percentage" is the documented key (README + set_position error message);
	// "position_percentage" is the key get_position returns, kept for round-trip
	// compatibility (e.g. arm-recorder mirrors a single get/set key).
	for _, tc := range []struct {
		name string
		cmd  map[string]interface{}
		want float64
		ok   bool
	}{
		{"documented percentage key", map[string]interface{}{"percentage": 42.0}, 42, true},
		{"position_percentage round-trip key", map[string]interface{}{"position_percentage": 73.0}, 73, true},
		{"percentage wins when both present", map[string]interface{}{"percentage": 10.0, "position_percentage": 90.0}, 10, true},
		{"neither present", map[string]interface{}{"servo_position": 2048.0}, 0, false},
		{"non-numeric ignored", map[string]interface{}{"percentage": "nope"}, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, ok := gripperSetPositionPercent(tc.cmd)
			assert.Equal(t, tc.ok, ok)
			if tc.ok {
				assert.Equal(t, tc.want, got)
			}
		})
	}
}

func TestBuildGripperMeshes(t *testing.T) {
	// Exercises the pose math and confirms each embedded PLY parses into a mesh.
	for _, tc := range []struct {
		gripperType string
		meshes      int
	}{
		{followerGripper, 2}, // body + jaw
		{leaderGripper, 3},   // wrist-roll + handle + trigger
	} {
		geoms, err := buildGripperMeshes(tc.gripperType, 0.5)
		require.NoError(t, err, "gripper_type %q", tc.gripperType)
		require.Len(t, geoms, tc.meshes)
		for _, geom := range geoms {
			mesh, ok := geom.(*spatialmath.Mesh)
			require.True(t, ok, "gripper geometry should be a mesh")
			assert.NotEmpty(t, mesh.Triangles(), "mesh should have triangles")
		}
	}
}

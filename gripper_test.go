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

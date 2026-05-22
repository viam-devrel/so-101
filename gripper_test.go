package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/spatialmath"
)

func TestSO101GripperConfigValidate(t *testing.T) {
	// An unset gripper_type / mesh_detail is accepted (defaults applied).
	_, _, err := (&SO101GripperConfig{Port: "/dev/ttyUSB0"}).Validate("")
	require.NoError(t, err)

	for _, gt := range []string{leaderGripper, followerGripper} {
		_, _, err := (&SO101GripperConfig{Port: "/dev/ttyUSB0", GripperType: gt}).Validate("")
		require.NoError(t, err, "gripper_type %q should be valid", gt)
	}
	for _, md := range []string{highDetail, lowDetail} {
		_, _, err := (&SO101GripperConfig{Port: "/dev/ttyUSB0", MeshDetail: md}).Validate("")
		require.NoError(t, err, "mesh_detail %q should be valid", md)
	}

	_, _, err = (&SO101GripperConfig{Port: "/dev/ttyUSB0", GripperType: "bogus"}).Validate("")
	require.Error(t, err)
	_, _, err = (&SO101GripperConfig{Port: "/dev/ttyUSB0", MeshDetail: "bogus"}).Validate("")
	require.Error(t, err)
}

func TestBuildGripperMeshes(t *testing.T) {
	// Exercises the pose math, confirms each embedded PLY parses into a mesh at
	// both detail levels, and that "high" really is denser than "low".
	for _, tc := range []struct {
		gripperType string
		meshes      int
	}{
		{followerGripper, 2}, // body + jaw
		{leaderGripper, 3},   // wrist-roll + handle + trigger
	} {
		triCount := map[string]int{}
		for _, detail := range []string{highDetail, lowDetail} {
			geoms, err := buildGripperMeshes(tc.gripperType, detail, 0.5)
			require.NoError(t, err, "gripper_type %q detail %q", tc.gripperType, detail)
			require.Len(t, geoms, tc.meshes)
			for _, geom := range geoms {
				mesh, ok := geom.(*spatialmath.Mesh)
				require.True(t, ok, "gripper geometry should be a mesh")
				assert.NotEmpty(t, mesh.Triangles(), "mesh should have triangles")
				triCount[detail] += len(mesh.Triangles())
			}
		}
		assert.Greater(t, triCount[highDetail], triCount[lowDetail],
			"high detail should be denser than low for %q", tc.gripperType)
	}
}

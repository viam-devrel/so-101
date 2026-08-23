package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// TestSO101ArmModels covers the shared helper backing both arms' Get3DModels, so the
// hardware arm (whose Get3DModels can't be unit-tested without a serial controller) has
// direct coverage of the meshes it now serves. The keys are the arm's frame names, which
// are identical in JSON and URDF modes (assets/urdf/so101.urdf uses so101.json's frame names), so
// Get3DModels no longer varies by use_urdf.
func TestSO101ArmModels(t *testing.T) {
	linkKeys := []string{"base", "shoulder", "upper_arm", "lower_arm", "wrist"}

	// visualize_ee_frame off: only the 5 geometry-bearing link meshes, no "tool" marker.
	off := so101ArmModels(false)
	require.Len(t, off, 5)
	assert.NotContains(t, off, so101EEFrameMeshKey)
	for _, k := range linkKeys {
		mesh, ok := off[k]
		require.True(t, ok, "missing mesh for %q", k)
		assert.Equal(t, gltfBinary, mesh.ContentType)
		assert.NotEmpty(t, mesh.Mesh, "mesh bytes for %q should not be empty", k)
	}

	// visualize_ee_frame on: the 5 link meshes plus the EE-frame marker at "tool".
	on := so101ArmModels(true)
	require.Len(t, on, 6)
	tool, ok := on[so101EEFrameMeshKey]
	require.True(t, ok, "expected an EE-frame mesh keyed %q", so101EEFrameMeshKey)
	assert.Equal(t, gltfBinary, tool.ContentType)
	assert.NotEmpty(t, tool.Mesh)
}

package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// TestSO101ArmModels covers the shared helper backing both arms' Get3DModels, so the
// hardware arm (whose Get3DModels can't be unit-tested without a serial controller) has
// direct coverage of the meshes it now serves.
func TestSO101ArmModels(t *testing.T) {
	linkKeys := []string{"base", "shoulder", "upper_arm", "lower_arm", "wrist"}

	// visualize_ee_frame off: only the 5 geometry-bearing link meshes, no "tool" marker.
	off := so101ArmModels(false, false)
	require.Len(t, off, 5)
	assert.NotContains(t, off, so101EEFrameMeshKey)
	for _, k := range linkKeys {
		mesh, ok := off[k]
		require.True(t, ok, "missing mesh for %q", k)
		assert.Equal(t, gltfBinary, mesh.ContentType)
		assert.NotEmpty(t, mesh.Mesh, "mesh bytes for %q should not be empty", k)
	}

	// visualize_ee_frame on: the 5 link meshes plus the EE-frame marker at "tool".
	on := so101ArmModels(true, false)
	require.Len(t, on, 6)
	tool, ok := on[so101EEFrameMeshKey]
	require.True(t, ok, "expected an EE-frame mesh keyed %q", so101EEFrameMeshKey)
	assert.Equal(t, gltfBinary, tool.ContentType)
	assert.NotEmpty(t, tool.Mesh)
}

// TestSO101ArmModelsURDFKeys covers use_urdf mode, where the 3D viewer's model frame
// names come from arm/so101.urdf (base_link, shoulder_link, ...) instead of so101.json
// (base, shoulder, ...). The same embedded GLBs serve both; only the map keys differ.
func TestSO101ArmModelsURDFKeys(t *testing.T) {
	urdf := so101ArmModels(false, true) // visualizeEE=false, useURDF=true
	require.Len(t, urdf, 5)
	for _, k := range []string{"base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link"} {
		mesh, ok := urdf[k]
		require.True(t, ok, "missing URDF-keyed mesh %q", k)
		require.Equal(t, gltfBinary, mesh.ContentType)
		require.NotEmpty(t, mesh.Mesh)
	}
	// JSON-mode keys are NOT present under URDF mode.
	require.NotContains(t, urdf, "base")

	// JSON mode still keys by JSON names.
	jsonModels := so101ArmModels(false, false)
	require.Contains(t, jsonModels, "base")
	require.NotContains(t, jsonModels, "base_link")
}

package so_arm

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestMakeModelFrameURDF(t *testing.T) {
	// URDF path: VIAM_MODULE_ROOT resolves arm/so101.urdf. During `go test .` the
	// working dir is the package dir (repo root), so "." locates the arm/ assets.
	t.Setenv("VIAM_MODULE_ROOT", ".")
	m, err := makeModelFrame(&SO101ArmConfig{UseURDF: true}, "so101")
	require.NoError(t, err)
	require.Equal(t, 5, len(m.DoF()))

	// Default (JSON) path still works and is 5-DOF.
	m2, err := makeModelFrame(&SO101ArmConfig{}, "so101")
	require.NoError(t, err)
	require.Equal(t, 5, len(m2.DoF()))

	// VisualizeEEFrame on the JSON path still builds (uses the EE-marker variant).
	m3, err := makeModelFrame(&SO101ArmConfig{VisualizeEEFrame: true}, "so101")
	require.NoError(t, err)
	require.Equal(t, 5, len(m3.DoF()))
}

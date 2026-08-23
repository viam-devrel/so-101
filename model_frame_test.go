package so_arm

import (
	"testing"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"

	"so_arm/internal/geometry"
	"so_arm/internal/testfake"
)

func TestMakeModelFrameURDF(t *testing.T) {
	// URDF path: VIAM_MODULE_ROOT resolves assets/urdf/so101.urdf. testfake.RepoRoot()
	// locates the repo root regardless of the test package's working directory.
	t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())
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

// TestMakeSO101ModelURDFVisualizeEE covers the URDF-mode visualize_ee_frame branch: the
// grafted "tool" leaf gains the placeholder box geometry so the viewer draws the EE marker,
// exactly as on the JSON path.
func TestMakeSO101ModelURDFVisualizeEE(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())
	inputsGeoms := func(m referenceframe.Model) int {
		g, err := m.Geometries(make([]referenceframe.Input, len(m.DoF())))
		require.NoError(t, err)
		return len(g.Geometries())
	}

	// URDF without the marker: 5 per-link collision meshes; the grafted tool has no geometry.
	off, err := geometry.ArmModel(true, nil, false, "so101")
	require.NoError(t, err)
	require.Equal(t, 5, inputsGeoms(off))

	// URDF with the marker: the grafted tool gains the placeholder box => 6 geometries.
	on, err := geometry.ArmModel(true, nil, true, "so101")
	require.NoError(t, err)
	require.Equal(t, 6, inputsGeoms(on))
}

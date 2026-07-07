package so_arm

import (
	"testing"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
)

func TestParseSO101URDF(t *testing.T) {
	// The arm-only URDF has 6 mesh collision geometries consumed by rdk: one
	// Collision[0] per link for base/shoulder/upper_arm/lower_arm/wrist/gripper_link.
	// (gripper_frame_link, the end-effector / "tool" analog, has no collision mesh.)
	numMeshes := 6
	ratios := make([]float64, numMeshes) // 0 = full resolution
	m, err := referenceframe.ParseModelXMLFile("arm/so101.urdf", "so101", ratios)
	require.NoError(t, err)
	require.Equal(t, 5, len(m.DoF())) // servos 1-5: shoulder_pan/lift, elbow_flex, wrist_flex/roll
	inputs := make([]referenceframe.Input, len(m.DoF()))
	gif, err := m.Geometries(inputs)
	require.NoError(t, err)
	require.Equal(t, 6, len(gif.Geometries())) // MESH collision geometry per link
	t.Logf("DoF=%d, geometries=%d", len(m.DoF()), len(gif.Geometries()))
}

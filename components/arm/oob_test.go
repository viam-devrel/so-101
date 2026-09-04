package arm

import (
	"testing"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
	"so_arm/internal/testfake"
)

// TestComputeOOBPositionFullChain pins that ComputeOOBPosition clamps an out-of-range joint
// to its limit and composes the full chain. It began as a guard against rdk v0.123's
// Frame.Transform truncating the pose at the first out-of-bounds joint; rdk >= v1.6.0
// composes the full chain itself, so the clamp is now the only behavior this adds.
func TestComputeOOBPositionFullChain(t *testing.T) {
	for _, useURDF := range []bool{false, true} {
		name := "json"
		if useURDF {
			name = "urdf"
		}
		t.Run(name, func(t *testing.T) {
			if useURDF {
				t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())
			}
			model, err := geometry.ArmModel(useURDF, nil, false, "oob-test")
			require.NoError(t, err)

			limits := model.DoF()
			require.GreaterOrEqual(t, len(limits), 5)

			// Joints 1-4 nonzero so a full-chain FK is clearly distinguishable from a
			// pose truncated at joint 0.
			base := []referenceframe.Input{0, 0.5, 0.3, 0.1, 0.1}

			// Drive joint 0 just past its max limit.
			overMax := make([]referenceframe.Input, 5)
			copy(overMax, base)
			overMax[0] = limits[0].Max + 0.02

			pose, err := geometry.ComputeOOBPosition(model, overMax)
			require.NoError(t, err) // best-effort pose, no surfaced error
			require.NotNil(t, pose)

			// The OOB pose must equal the full-chain pose at the clamped (limit) angle.
			atMax, err := model.Transform([]referenceframe.Input{limits[0].Max, 0.5, 0.3, 0.1, 0.1})
			require.NoError(t, err)
			require.InDelta(t, 0, spatialmath.PoseDelta(pose, atMax).Point().Norm(), 1e-6,
				"OOB pose should match the full-chain pose clamped to the joint limit")

			// It must NOT collapse to a pose truncated at joint 0. If joint 0 were dropped,
			// the TCP would be independent of joint 0's value/direction.
			overMin := make([]referenceframe.Input, 5)
			copy(overMin, base)
			overMin[0] = limits[0].Min - 0.02
			poseMin, err := geometry.ComputeOOBPosition(model, overMin)
			require.NoError(t, err)
			require.Greater(t, spatialmath.PoseDelta(pose, poseMin).Point().Norm(), 1.0,
				"OOB pose must depend on joint 0's value (full chain), not be truncated to the base pose")
		})
	}
}

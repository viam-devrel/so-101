package arm

import (
	"testing"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
	"so_arm/internal/testfake"
)

// EndPosition calls model.Transform directly, so it depends on rdk >= v1.6.0 composing the
// full chain for a joint past its limit (a servo drooped under load does this routinely)
// and reporting where the arm actually is. rdk v0.123 truncated the chain there instead,
// which is why a clamping wrapper used to exist; if rdk ever bounds-checks again, this is
// the test that says so.
func TestTransformReportsAnOutOfLimitJointWhereItIs(t *testing.T) {
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

			atMax := []referenceframe.Input{limits[0].Max, 0.5, 0.3, 0.1, 0.1}
			overMax := []referenceframe.Input{limits[0].Max + 0.02, 0.5, 0.3, 0.1, 0.1}

			poseAtMax, err := model.Transform(atMax)
			require.NoError(t, err)
			poseOver, err := model.Transform(overMax)
			require.NoError(t, err, "an out-of-limit joint must not be an error on the read path")

			// Not clamped: 0.02 rad on the base joint moves the TCP by millimetres.
			require.Greater(t, spatialmath.PoseDelta(poseOver, poseAtMax).Point().Norm(), 1.0,
				"the pose must reflect the joint's actual value, not its limit")

			// Not truncated: the TCP must still depend on the downstream joints.
			poseOverBent, err := model.Transform([]referenceframe.Input{limits[0].Max + 0.02, 0.9, 0.3, 0.1, 0.1})
			require.NoError(t, err)
			require.Greater(t, spatialmath.PoseDelta(poseOver, poseOverBent).Point().Norm(), 1.0,
				"the full chain must be composed past the out-of-limit joint")
		})
	}
}

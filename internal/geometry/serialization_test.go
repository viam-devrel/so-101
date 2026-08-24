package geometry

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/testfake"
)

// TestURDFModelSurvivesSerialization is the guard for the module-boundary bug: a component ships
// its kinematics to viam-server as ModelConfig().OriginalFile.Bytes (KinematicModelToProtobuf) and
// the server rebuilds the model from those bytes (KinematicModelFromProtobuf). The URDF model's
// tool-frame graft and frame rename are in-memory edits to the parsed LinkConfigs; if OriginalFile
// still points at the raw URDF, the transmitted model is the UN-grafted arm (native end effector
// ~98mm past so101.json's TCP) even though the in-process model is correct. That mismatch put the
// motion service's EE -- and the gripper parented to the arm -- in the wrong place while every
// in-process unit test stayed green. This round-trips the model through the exact gRPC
// (de)serialization and asserts the result matches the JSON TCP and keeps its meshes. Do NOT
// weaken it to an in-process Transform check -- the boundary is the whole point.
func TestURDFModelSurvivesSerialization(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())

	roundTrip := func(t *testing.T, m referenceframe.Model) referenceframe.Model {
		t.Helper()
		resp := referenceframe.KinematicModelToProtobuf(m)
		out, err := referenceframe.KinematicModelFromProtobuf("follower-sim", resp)
		require.NoError(t, err, "server-side reconstruction from transmitted kinematics")
		return out
	}

	jsonModel, err := ArmModelJSON("follower-sim")
	require.NoError(t, err)

	urdfModel, err := ArmModel(true, nil, false, "follower-sim")
	require.NoError(t, err)
	transmitted := roundTrip(t, urdfModel)

	// The transmitted URDF EE must match the JSON TCP (position AND orientation) at several
	// configs -- this is the frame the gripper follows.
	configs := [][]float64{
		{0, 0, 0, 0, 0},
		{0.3, -0.4, 0.5, -0.2, 0.6},
		{-0.5, 0.6, -0.3, 0.4, -0.6},
	}
	for _, cfg := range configs {
		ins := toInputs(cfg)
		jp, err := jsonModel.Transform(ins)
		require.NoError(t, err)
		tp, err := transmitted.Transform(ins)
		require.NoError(t, err)
		posD := jp.Point().Sub(tp.Point()).Norm()
		oriD := spatialmath.QuatToR3AA(
			spatialmath.OrientationBetween(jp.Orientation(), tp.Orientation()).Quaternion(),
		).Norm()
		require.Lessf(t, posD, 1e-3,
			"config %v: transmitted URDF EE %v != JSON TCP %v (%.3fmm) -- graft lost over gRPC boundary",
			cfg, tp.Point(), jp.Point(), posD)
		require.Lessf(t, oriD, 1e-4,
			"config %v: transmitted URDF EE orientation drifted %.6frad from JSON TCP", cfg, oriD)
	}

	// Collision meshes must survive the boundary too (SVA JSON embeds GeometryConfig.MeshData).
	g, err := transmitted.Geometries(make([]referenceframe.Input, len(transmitted.DoF())))
	require.NoError(t, err)
	require.Equal(t, 5, len(g.Geometries()), "transmitted model lost its per-link collision meshes")

	// visualize_ee_frame's placeholder box must also survive (it was lost the same way): 6 geoms.
	urdfEE, err := ArmModel(true, nil, true, "follower-sim")
	require.NoError(t, err)
	tEE := roundTrip(t, urdfEE)
	gEE, err := tEE.Geometries(make([]referenceframe.Input, len(tEE.DoF())))
	require.NoError(t, err)
	require.Equal(t, 6, len(gEE.Geometries()), "transmitted model lost the visualize_ee_frame marker geometry")
}

// TestArticulatedModelSurvivesSerialization extends the module-boundary guard to the jaw joint.
// A component ships its kinematics as ModelConfig().OriginalFile.Bytes, so a joint that exists
// only in the in-memory model transmits as a 0-DoF gripper: viam-server would then never ask for
// jaw inputs, the planner would never see the DoF, and every in-process test here would still
// pass. Do NOT weaken this to an in-process DoF check.
func TestArticulatedModelSurvivesSerialization(t *testing.T) {
	m, err := BuildGripperModel(FollowerGripper, LowDetail, "sim-gripper", true)
	require.NoError(t, err)

	resp := referenceframe.KinematicModelToProtobuf(m)
	out, err := referenceframe.KinematicModelFromProtobuf("sim-gripper", resp)
	require.NoError(t, err, "server-side reconstruction from transmitted kinematics")

	dof := out.DoF()
	require.Len(t, dof, 1, "the jaw joint was lost across the gRPC boundary")
	assert.InDelta(t, GripperJointMin, dof[0].Min, 1e-9, "jaw lower limit did not survive")
	assert.InDelta(t, GripperJointMax, dof[0].Max, 1e-9, "jaw upper limit did not survive")

	// The TCP must still be the output frame, and still invariant, on the server's copy.
	for _, theta := range []float64{GripperJointMin, GripperJointMax} {
		p, err := out.Transform([]referenceframe.Input{theta})
		require.NoError(t, err)
		assert.Lessf(t, p.Point().Sub(GripperTCPPose.Point()).Norm(), 1e-6,
			"transmitted TCP at %v (jaw=%.3f), want %v", p.Point(), theta, GripperTCPPose.Point())
	}

	gif, err := out.Geometries([]referenceframe.Input{GripperJointMin})
	require.NoError(t, err)
	assert.Len(t, gif.Geometries(), 2, "transmitted model lost its gripper meshes")
}

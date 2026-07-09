package so_arm

import (
	"testing"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// worldPosesForInputs walks a ModelConfigJSON's links and joints (as produced by both
// UnmarshalModelJSON/ParseConfig for so101.json and UnmarshalModelXML/ParseModelXMLFile for
// arm/so101.urdf -- both kinematic sources funnel through the same SVA ModelConfigJSON
// representation) and returns the world-frame pose of every named link/joint frame, given a
// map of joint-frame-ID -> radian value. Joint frames not present in jointRadians (e.g. fixed
// joints folded into links) transform with a zero input. This mirrors the technique in
// ../viam-dobot/arm/cr10a_test.go's TestPerLinkFrameAlignment, generalized from a fixed
// zero-position query to arbitrary joint configurations.
func worldPosesForInputs(t *testing.T, cfg *referenceframe.ModelConfigJSON, jointRadians map[string]float64) map[string]spatialmath.Pose {
	t.Helper()
	transforms := map[string]referenceframe.Frame{}
	parent := map[string]string{}
	for i := range cfg.Links {
		l := &cfg.Links[i]
		lif, err := l.ParseConfig()
		require.NoError(t, err)
		f, err := lif.ToStaticFrame(l.ID)
		require.NoError(t, err)
		transforms[l.ID] = f
		parent[l.ID] = l.Parent
	}
	for i := range cfg.Joints {
		j := &cfg.Joints[i]
		f, err := j.ToFrame()
		require.NoError(t, err)
		transforms[j.ID] = f
		parent[j.ID] = j.Parent
	}

	memo := map[string]spatialmath.Pose{}
	var world func(name string) spatialmath.Pose
	world = func(name string) spatialmath.Pose {
		if name == "" || name == referenceframe.World {
			return spatialmath.NewZeroPose()
		}
		if p, ok := memo[name]; ok {
			return p
		}
		fr, ok := transforms[name]
		if !ok {
			t.Fatalf("frame %q not found", name)
		}
		var in []referenceframe.Input
		if d := len(fr.DoF()); d > 0 {
			in = make([]referenceframe.Input, d)
			in[0] = referenceframe.Input(jointRadians[name]) // 0 if not one of the joints we're driving
		}
		local, err := fr.Transform(in)
		require.NoErrorf(t, err, "frame %s Transform", name)
		res := spatialmath.Compose(world(parent[name]), local)
		memo[name] = res
		return res
	}
	for name := range transforms {
		world(name)
	}
	return memo
}

// jointRadiansByChainOrder resolves a flat, base-to-tip joint-input vector (the same order as
// model.DoF()) into a map keyed by the model's own joint-frame IDs, using SimpleModel's
// exported MoveableFrameNames() (schema order == DoF order). This is required because the
// underlying ModelConfigJSON.Joints slice order differs between the two sources: so101.json
// lists joints "1".."5" already base-to-tip, but arm/so101.urdf's XML document order is
// tip-to-base (joints "5","4","3","2","1") -- relying on slice order would silently pair the
// wrong joints across models.
func jointRadiansByChainOrder(t *testing.T, m referenceframe.Model, values []float64) map[string]float64 {
	t.Helper()
	sm, ok := m.(*referenceframe.SimpleModel)
	require.True(t, ok, "expected *referenceframe.SimpleModel")
	names := sm.MoveableFrameNames()
	require.Len(t, names, len(values))
	out := make(map[string]float64, len(names))
	for i, n := range names {
		out[n] = values[i]
	}
	return out
}

func toInputs(values []float64) []referenceframe.Input {
	ins := make([]referenceframe.Input, len(values))
	for i, v := range values {
		ins[i] = referenceframe.Input(v)
	}
	return ins
}

// TestURDFExposesJSONFrameNames is the drop-in guard: use_urdf must expose the exact same
// frame-system frame names as the default JSON arm, so any config that references an arm frame
// by name (a gripper, camera, or obstacle parented to an arm link) keeps resolving when
// use_urdf is toggled. arm/so101.urdf therefore uses so101.json's names directly (base/1/...,
// not base_link/shoulder_pan/...); the upstream names would silently detach such children toward
// the world origin. Covers moveable (joint) frames, link geometry labels, and the tool leaf.
func TestURDFExposesJSONFrameNames(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")

	jsonModel, err := makeSO101ModelFrame("so101")
	require.NoError(t, err)
	urdfModel, err := makeSO101Model(true, nil, false, "so101")
	require.NoError(t, err)

	jSM := jsonModel.(*referenceframe.SimpleModel)
	uSM := urdfModel.(*referenceframe.SimpleModel)
	require.Equal(t, jSM.MoveableFrameNames(), uSM.MoveableFrameNames(),
		"URDF moveable (joint) frame names must match JSON's exactly")

	labels := func(m referenceframe.Model) []string {
		gif, err := m.Geometries(make([]referenceframe.Input, len(m.DoF())))
		require.NoError(t, err)
		out := []string{}
		for _, g := range gif.Geometries() {
			out = append(out, g.Label())
		}
		return out
	}
	require.ElementsMatch(t, labels(jsonModel), labels(urdfModel),
		"URDF geometry frame labels must match JSON's exactly")
}

// TestURDFvsJSONFrameAlignment proves that so101.json's SVA kinematics and arm/so101.urdf agree,
// per-frame, across a range of joint configurations -- for every arm link AND the "tool" TCP
// frame. This is the guard referenced by so101ArmMeshParts' doc comment (meshes.go): it makes it
// safe to key the same bundled link GLBs by the shared frame names, and it proves the URDF's
// baked-in "tool" leaf (joint-5 output + 180deg flip) lands on so101.json's TCP, preserving the
// convention gripper.go's toolFromGripperLink transform depends on. If this test ever fails, the
// two kinematic sources have drifted apart -- do NOT loosen the tolerances here to force a pass;
// stop and report the deltas instead.
func TestURDFvsJSONFrameAlignment(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")

	jsonModel, err := makeSO101ModelFrame("so101")
	require.NoError(t, err)
	urdfModel, err := makeSO101Model(true, nil, false, "so101")
	require.NoError(t, err)

	// Both models are 5-DOF (servos 1-5). Retains the URDF geometry-count sanity check: one mesh
	// collision geometry per link (base/shoulder/upper_arm/lower_arm/wrist) -- the "tool" leaf has
	// no geometry here (visualizeEE=false), and the removed gripper chain contributes none.
	require.Equal(t, 5, len(jsonModel.DoF()))
	require.Equal(t, 5, len(urdfModel.DoF()))
	gif, err := urdfModel.Geometries(make([]referenceframe.Input, len(urdfModel.DoF())))
	require.NoError(t, err)
	require.Equal(t, 5, len(gif.Geometries()))

	jsonSM, ok := jsonModel.(*referenceframe.SimpleModel)
	require.True(t, ok, "expected *referenceframe.SimpleModel for JSON model")
	urdfSM, ok := urdfModel.(*referenceframe.SimpleModel)
	require.True(t, ok, "expected *referenceframe.SimpleModel for URDF model")
	jsonNames := jsonSM.MoveableFrameNames()
	urdfNames := urdfSM.MoveableFrameNames()
	require.Len(t, jsonNames, 5)
	require.Len(t, urdfNames, 5)
	t.Logf("JSON moveable frames (chain order): %v", jsonNames)
	t.Logf("URDF moveable frames (chain order): %v", urdfNames)

	// Joint limits (converted to a common unit, degrees) must line up per chain position --
	// otherwise the flat input vectors below wouldn't represent "the same servo angle" for
	// both models even though the DoF counts match.
	const limitTolDeg = 0.01
	for i, lim := range jsonModel.DoF() {
		jMinDeg := lim.Min * 180 / 3.141592653589793
		jMaxDeg := lim.Max * 180 / 3.141592653589793
		uLim := urdfModel.DoF()[i]
		uMinDeg := uLim.Min * 180 / 3.141592653589793
		uMaxDeg := uLim.Max * 180 / 3.141592653589793
		t.Logf("joint[%d] json=%s(%.4f..%.4f deg) urdf=%s(%.4f..%.4f deg)",
			i, jsonNames[i], jMinDeg, jMaxDeg, urdfNames[i], uMinDeg, uMaxDeg)
		require.InDeltaf(t, jMinDeg, uMinDeg, limitTolDeg, "joint[%d] min limit mismatch", i)
		require.InDeltaf(t, jMaxDeg, uMaxDeg, limitTolDeg, "joint[%d] max limit mismatch", i)
	}

	// Frame-name correspondence: the URDF model renames its frames to so101.json's names
	// (arm/so101.urdf uses so101.json's names), so the JSON and URDF frames pair up by identical
	// name. "tool" is the load-bearing end-effector / TCP pair -- both models use the exact
	// same LinkConfig for it (see makeSO101ModelURDF), so this aligns almost exactly.
	pairs := [][2]string{
		{"base", "base"},
		{"shoulder", "shoulder"},
		{"upper_arm", "upper_arm"},
		{"lower_arm", "lower_arm"},
		{"wrist", "wrist"},
		{"tool", "tool"},
	}

	configs := [][]float64{
		{0, 0, 0, 0, 0},
		{0.3, -0.4, 0.5, -0.2, 0.6},
		{-0.5, 0.6, -0.3, 0.4, -0.6},
		// Near (but inside) each joint's limits, alternating sign.
		{1.85, -1.70, 1.65, -1.60, 2.70},
	}

	const posTolMM = 1e-3
	const oriTolRad = 1e-4

	maxPosD := map[string]float64{}
	maxOriD := map[string]float64{}

	for ci, values := range configs {
		jsonRadians := jointRadiansByChainOrder(t, jsonModel, values)
		urdfRadians := jointRadiansByChainOrder(t, urdfModel, values)

		jsonPoses := worldPosesForInputs(t, jsonModel.ModelConfig(), jsonRadians)
		urdfPoses := worldPosesForInputs(t, urdfModel.ModelConfig(), urdfRadians)

		// Self-consistency: the manual walk's "tool" pose (for each model) should match rdk's
		// own Model.Transform() for the same model and inputs (both resolve the primary output
		// / sole leaf frame). This validates the walk harness itself, independent of the
		// JSON-vs-URDF comparison below.
		jsonTransformPose, err := jsonModel.Transform(toInputs(values))
		require.NoError(t, err)
		require.Less(t, jsonPoses["tool"].Point().Sub(jsonTransformPose.Point()).Norm(), 1e-6,
			"config %d: manual walk vs Model.Transform mismatch on JSON tool frame", ci)
		urdfTransformPose, err := urdfModel.Transform(toInputs(values))
		require.NoError(t, err)
		require.Less(t, urdfPoses["tool"].Point().Sub(urdfTransformPose.Point()).Norm(), 1e-6,
			"config %d: manual walk vs Model.Transform mismatch on URDF tool frame", ci)

		for _, pr := range pairs {
			jp, ok := jsonPoses[pr[0]]
			require.Truef(t, ok, "config %d: missing JSON pose for %q", ci, pr[0])
			up, ok := urdfPoses[pr[1]]
			require.Truef(t, ok, "config %d: missing URDF pose for %q", ci, pr[1])

			posD := jp.Point().Sub(up.Point()).Norm()
			oriD := spatialmath.QuatToR3AA(
				spatialmath.OrientationBetween(jp.Orientation(), up.Orientation()).Quaternion(),
			).Norm()

			key := pr[0] + "<->" + pr[1]
			if posD > maxPosD[key] {
				maxPosD[key] = posD
			}
			if oriD > maxOriD[key] {
				maxOriD[key] = oriD
			}

			if posD > posTolMM {
				t.Errorf("config %d (%v): %s position diff %.6fmm exceeds %.g mm tolerance", ci, values, key, posD, posTolMM)
			}
			if oriD > oriTolRad {
				t.Errorf("config %d (%v): %s orientation diff %.6frad exceeds %.g rad tolerance", ci, values, key, oriD, oriTolRad)
			}
		}
	}

	for _, pr := range pairs {
		key := pr[0] + "<->" + pr[1]
		t.Logf("%s: max posΔ=%.6fmm max oriΔ=%.6frad (tol posΔ<=%.gmm oriΔ<=%.grad)",
			key, maxPosD[key], maxOriD[key], posTolMM, oriTolRad)
	}
}

package so_arm

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

func TestResolveGoalCloudConfigDefaults(t *testing.T) {
	// Zero and unset are indistinguishable for a float64 with omitempty; both default.
	got := resolveGoalCloudConfig(0, 0, nil)
	assert.Equal(t, defaultOrientationToleranceDeg, got.OrientationToleranceDeg)
	assert.Equal(t, defaultPositionToleranceMM, got.PositionToleranceMM)
}

func TestResolveGoalCloudConfigKeepsExplicitValues(t *testing.T) {
	got := resolveGoalCloudConfig(12.5, 0.25, nil)
	assert.Equal(t, 12.5, got.OrientationToleranceDeg)
	assert.Equal(t, 0.25, got.PositionToleranceMM)
}

// The warnings are the production path -- both tests above pass a nil logger and so
// exercise only the early return. resolveGoalCloudConfig is the SINGLE owner of these
// thresholds; nothing downstream re-checks them, so nothing downstream would catch a
// regression either.
func TestResolveGoalCloudConfigWarnsAtSaturationBoundary(t *testing.T) {
	// The boundary is acos(-0.999) = 177.4374 -- exactly the band a literal ">= 177.44"
	// check would miss, which is why poseCloudSaturationCos is a cosine.
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.438, 0, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("saturates").Len(), "just inside the boundary must warn")

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(177.40, 0, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("saturates").Len(), "just outside must not warn")
}

func TestResolveGoalCloudConfigWarnsOnLargePositionTolerance(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 25, logger)
	assert.Equal(t, 1, logs.FilterMessageSnippet("is large").Len())

	logger, logs = logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 10, logger)
	assert.Equal(t, 0, logs.FilterMessageSnippet("is large").Len(), "at the threshold must not warn")
}

func TestResolveGoalCloudConfigDefaultsDoNotWarn(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	resolveGoalCloudConfig(0, 0, logger)
	assert.Equal(t, 0, logs.Len(), "the defaults must be quiet")
}

// testGoal is a tool pointing straight down, a typical SO-101 grasp pose.
func testGoal() spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: 300, Y: 0, Z: 200},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0},
	)
}

// tiltedBy returns testGoal() tilted by deg about the goal's local (ax, ay, 0) axis. The
// rotation composes onto the goal, so it applies in the goal's own frame -- and since the
// goal points straight down, that local frame is not the world frame.
func tiltedBy(ax, ay, deg float64) spatialmath.Pose {
	return spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
		&spatialmath.R4AA{RX: ax, RY: ay, RZ: 0, Theta: utils.DegToRad(deg)}))
}

// coneCloud builds the cloud under test for a cone half-angle. Its position tolerance is
// deliberately 2.5 rather than 1: that keeps it distinct from the unconstrained OX/OY
// sentinel of 1, so the position -> X/Y/Z mapping stays legible and a transposition of the
// two would be caught.
func coneCloud(tolDeg float64) *referenceframe.PoseCloud {
	return coneToPoseCloud(goalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: 2.5})
}

// TestConeToPoseCloudFields pins the complete field set: X, Y, Z, OX, OY, OZ, Theta is the
// WHOLE struct in rdk v1.0.0. Do not look for a ReferenceFrame field -- v0.123.0 had one,
// v1.0.0 removed it, and the struct's free-floating doc comment about reference frames
// misleadingly survives.
func TestConeToPoseCloudFields(t *testing.T) {
	c := coneCloud(30)
	// The position tolerance maps to all three positional axes...
	assert.Equal(t, 2.5, c.X)
	assert.Equal(t, 2.5, c.Y)
	assert.Equal(t, 2.5, c.Z)
	// ...while OX/OY are deliberately unconstrained: OZ alone defines the cone.
	assert.Equal(t, 1.0, c.OX)
	assert.Equal(t, 1.0, c.OY)
	assert.InDelta(t, 1-math.Cos(utils.DegToRad(30)), c.OZ, 1e-12)
	// MUST be 180: Theta encodes tilt AZIMUTH, not roll. See the spec.
	assert.Equal(t, 180.0, c.Theta)
}

func TestConeBoundsTiltIsotropically(t *testing.T) {
	c := coneCloud(30)
	for _, azDeg := range []float64{0, 45, 90, 135, 180, 270} {
		ax, ay := math.Cos(utils.DegToRad(azDeg)), math.Sin(utils.DegToRad(azDeg))
		assert.True(t, c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 29)),
			"29deg tilt at azimuth %.0f should be accepted", azDeg)
		assert.False(t, c.PoseInCloud(testGoal(), tiltedBy(ax, ay, 31)),
			"31deg tilt at azimuth %.0f should be rejected", azDeg)
	}
}

func TestConeBoundsAbove90Degrees(t *testing.T) {
	// The cone does NOT go degenerate above 90, despite OZ = 1-cos(a) exceeding 1.
	assert.True(t, coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 89)))
	assert.False(t, coneCloud(90).PoseInCloud(testGoal(), tiltedBy(1, 0, 91)))
	assert.True(t, coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 119)))
	assert.False(t, coneCloud(120).PoseInCloud(testGoal(), tiltedBy(1, 0, 121)))
}

// TestConeSaturationBoundary is the empirical anchor for saturation: it brackets the
// boundary with literal MEASURED degrees, owing nothing to poseCloudSaturationCos. Its
// sibling TestSaturationConstantMatchesConeMapping is the drift guard, deriving the same
// boundary FROM that constant. Both are needed: this one would still pass if the constant
// were wrong, and the sibling would still pass if both the constant and the mapping drifted
// together.
func TestConeSaturationBoundary(t *testing.T) {
	// Saturation begins at acos(-0.999) = 177.4374, NOT 179.
	assert.False(t, coneCloud(177.40).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"177.40 must still reject a 180deg tilt")
	assert.True(t, coneCloud(177.44).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"177.44 must be saturated")
}

// TestSaturationConstantMatchesConeMapping ties Task 2's poseCloudSaturationCos to this
// task's cone mapping. The constant is only correct while coneToPoseCloud maps the cone to
// OZ = 1-cos(a); if that formula ever changes, resolveGoalCloudConfig would keep warning at
// the wrong angle with nothing to catch it. The tests above use literal degrees and would
// not notice. This one derives the angle FROM the constant, so the two cannot drift apart.
func TestSaturationConstantMatchesConeMapping(t *testing.T) {
	satDeg := math.Acos(poseCloudSaturationCos) * 180 / math.Pi // 177.4374...
	assert.True(t, coneCloud(satDeg+0.001).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"at the angle where resolveGoalCloudConfig warns, the cone must actually be saturated")
	assert.False(t, coneCloud(satDeg-0.01).PoseInCloud(testGoal(), tiltedBy(1, 0, 180)),
		"just below that angle the cone must still bound tilt")
}

func TestConeRollIsFree(t *testing.T) {
	c := coneCloud(30)
	// 180 is the actual boundary case: |Theta| = 180 against a leeway of 180 + 0.001.
	for _, roll := range []float64{0, 45, 90, 179, 180} {
		p := spatialmath.Compose(testGoal(), spatialmath.NewPoseFromOrientation(
			&spatialmath.R4AA{RX: 0, RY: 0, RZ: 1, Theta: utils.DegToRad(roll)}))
		assert.True(t, c.PoseInCloud(testGoal(), p), "roll %.0f should be accepted", roll)
	}
}

func TestConeTiltPlusRollStaysBounded(t *testing.T) {
	c := coneCloud(30)
	for _, roll := range []float64{0, 70, 170} {
		for _, tc := range []struct {
			tilt float64
			want bool
		}{{20, true}, {29, true}, {31, false}} {
			p := spatialmath.Compose(testGoal(), spatialmath.Compose(
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RX: 1, Theta: utils.DegToRad(tc.tilt)}),
				spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{RZ: 1, Theta: utils.DegToRad(roll)})))
			assert.Equal(t, tc.want, c.PoseInCloud(testGoal(), p),
				"tilt %.0f roll %.0f", tc.tilt, roll)
		}
	}
}

// effectiveConeDeg measures the cone rdk's PoseInCloud actually enforces, by binary
// search on acceptance.
func effectiveConeDeg(c *referenceframe.PoseCloud) float64 {
	lo, hi := 0.0, 180.0
	for i := 0; i < 60; i++ {
		mid := (lo + hi) / 2
		if c.PoseInCloud(testGoal(), tiltedBy(1, 0, mid)) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

func TestConeEffectiveWideningFormula(t *testing.T) {
	// rdk ADDS its 0.001 epsilon to the leeway, so the cone actually enforced is always
	// slightly wider than requested. Comparing measurement (LHS, from rdk's real
	// PoseInCloud) against our model (RHS) is not tautological: it pins that our
	// understanding of rdk's behavior is right, for any angle rather than six points.
	// The sample spans all three regimes: 0 is epsilon-dominated, 2.6 is the crossover
	// where the requested leeway equals the epsilon, 90+ is epsilon-negligible.
	for _, requested := range []float64{0, 1, 2.6, 10, 30, 90, 150} {
		want := math.Acos(math.Cos(utils.DegToRad(requested))-0.001) * 180 / math.Pi
		assert.InDelta(t, want, effectiveConeDeg(coneCloud(requested)), 0.01,
			"effective cone for requested %.1f", requested)
	}
	// One literal anchor, independent of the formula: at tolerance 0 the epsilon alone
	// still admits ~2.5626deg. That is the floor a caller cannot get below.
	assert.InDelta(t, 2.5626, effectiveConeDeg(coneCloud(0)), 0.01)
}

func TestConePositionalBoxNotRadius(t *testing.T) {
	// Deliberately NOT coneCloud (which uses 2.5): the bounds below are measured values
	// tied to a 1.0mm tolerance, so this test pins its own.
	c := coneToPoseCloud(goalCloudConfig{OrientationToleranceDeg: 30, PositionToleranceMM: 1.0})
	offset := func(dx, dy, dz float64) spatialmath.Pose {
		g := testGoal()
		return spatialmath.NewPose(
			r3.Vector{X: g.Point().X + dx, Y: g.Point().Y + dy, Z: g.Point().Z + dz},
			g.Orientation())
	}
	// The epsilon is additive: at X=1.0 the true bound is 1.001mm.
	assert.True(t, c.PoseInCloud(testGoal(), offset(1.0009, 0, 0)))
	assert.False(t, c.PoseInCloud(testGoal(), offset(1.0015, 0, 0)))
	// Per-axis box, not a radius: all three axes at the bound is accepted at norm ~1.73mm.
	assert.True(t, c.PoseInCloud(testGoal(), offset(1.0005, 1.0005, 1.0005)))
}

// TestRDKContractThetaReportsTiltAzimuth documents why coneToPoseCloud sets Theta: 180, by
// pinning the mechanism directly: the relative Theta reports a tilt's AZIMUTH, not its roll.
//
// This is a characterization test for rdk's Theta semantics, not a guard on coneToPoseCloud
// (it overwrites Theta, so a mutation there is a no-op here -- TestConeBoundsTiltIsotropically
// is what guards the mapping end-to-end). It earns its place by pinning the underlying rdk
// contract that nothing else asserts: ovX.Theta == 90 and ovY.Theta == 0. Were an rdk upgrade
// to change those semantics, the acceptance tests would fail with vague mismatches while this
// one names the broken contract outright.
//
// Note the tempting shorthand -- "a smaller Theta rejects tilts" -- is FALSE: Theta=0 below
// rejects a tilt about X but would ACCEPT one about Y (azimuth 90 reports Theta=0). What a
// smaller Theta actually does is carve out a wedge of tilt DIRECTIONS. Since reported Theta
// sweeps the full [-180, 180] as azimuth goes around, only a leeway of 180 admits every
// azimuth.
func TestRDKContractThetaReportsTiltAzimuth(t *testing.T) {
	broken := coneCloud(30)
	broken.Theta = 0 // the "obvious simplification"
	assert.False(t, broken.PoseInCloud(testGoal(), tiltedBy(1, 0, 5)),
		"with Theta=0 even a 5deg tilt is rejected, defeating the cone entirely")

	// Proof of the cause: a pure tilt about X reports Theta=90, about Y reports Theta=0.
	ovX := spatialmath.PoseBetween(testGoal(), tiltedBy(1, 0, 5)).Orientation().OrientationVectorDegrees()
	assert.InDelta(t, 90.0, ovX.Theta, 1e-6, "tilt about X reports Theta=90, not roll")
	ovY := spatialmath.PoseBetween(testGoal(), tiltedBy(0, 1, 5)).Orientation().OrientationVectorDegrees()
	assert.InDelta(t, 0.0, ovY.Theta, 1e-6, "tilt about Y reports Theta=0")
}

// TestRDKContractZeroLeewayDemandsMicronMatch documents why X/Y/Z must be > 0: the 0.001
// epsilon means a zero leeway demands a match within 1 micron -- which no IK solution will
// realistically achieve, making the cloud useless in practice even though it does technically
// accept an exact match (measured: it still accepts 0.0009mm, and rejects 0.0011mm).
//
// This is a characterization test for rdk's epsilon rule, not a guard on coneToPoseCloud
// (it overwrites X/Y/Z, so a mutation there is a no-op here -- TestConePositionalBoxNotRadius
// is what guards the mapping). It earns its place by demonstrating the failure mode that
// motivates defaultPositionToleranceMM: the cloud a caller gets with zero leeway.
func TestRDKContractZeroLeewayDemandsMicronMatch(t *testing.T) {
	broken := coneCloud(30)
	broken.X, broken.Y, broken.Z = 0, 0, 0
	g := testGoal()
	nudged := spatialmath.NewPose(
		r3.Vector{X: g.Point().X + 0.01, Y: g.Point().Y, Z: g.Point().Z}, g.Orientation())
	assert.False(t, broken.PoseInCloud(g, nudged),
		"with zero positional leeway even a 0.01mm offset is rejected")
}

func TestParsePoseCloudValid(t *testing.T) {
	// extra arrives via structpb, so every JSON number is a float64.
	//
	// Every value is DISTINCT and all seven are asserted: identical values (e.g. x==y)
	// would let a setter transposition survive undetected, and coneToPoseCloud's OX:1/OY:1
	// means no other test would catch it either.
	got, err := parsePoseCloud(map[string]interface{}{
		"x": 2.0, "y": 3.0, "z": 0.5, "ox": 0.25, "oy": 0.5, "oz": 0.1, "theta": 180.0,
	})
	require.NoError(t, err)
	assert.Equal(t, 2.0, got.X)
	assert.Equal(t, 3.0, got.Y)
	assert.Equal(t, 0.5, got.Z)
	assert.Equal(t, 0.25, got.OX)
	assert.Equal(t, 0.5, got.OY)
	assert.Equal(t, 0.1, got.OZ)
	assert.Equal(t, 180.0, got.Theta)
}

func TestParsePoseCloudOmittedFieldsStayZero(t *testing.T) {
	got, err := parsePoseCloud(map[string]interface{}{"oz": 0.1})
	require.NoError(t, err)
	assert.Equal(t, 0.0, got.X, "omitted fields stay zero -- sharp, but faithful passthrough")
}

func TestParsePoseCloudRejects(t *testing.T) {
	for name, input := range map[string]interface{}{
		"not an object":       "nope",
		"docs-style casing":   map[string]interface{}{"OX": 1.0},
		"protobuf casing":     map[string]interface{}{"o_x": 1.0},
		"not a field in v1.0": map[string]interface{}{"reference_frame": 1.0},
		"unknown key":         map[string]interface{}{"wobble": 1.0},
		"non-numeric":         map[string]interface{}{"oz": "0.1"},
		"negative":            map[string]interface{}{"oz": -0.1},
		"NaN":                 map[string]interface{}{"oz": math.NaN()},
		"Inf":                 map[string]interface{}{"oz": math.Inf(1)},
		"nil value":           nil,
	} {
		t.Run(name, func(t *testing.T) {
			_, err := parsePoseCloud(input)
			assert.Error(t, err)
		})
	}
}

func TestBuildMoveDestinationConePath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	require.NoError(t, err)
	assert.Equal(t, pathCone, path)
	assert.Equal(t, "myarm_origin", dest.Parent(), "the frame name is passed through unmodified")
	require.NotNil(t, dest.GoalCloud)
	assert.Equal(t, coneToPoseCloud(cfg), dest.GoalCloud, "the destination carries the config's cone")
	assert.NotContains(t, planExtra, "goal_metric_type", "the cone replaces position_only")
}

func TestBuildMoveDestinationForwardsCallerExtras(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	_, planExtra, _, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"timeout": 5.0})
	require.NoError(t, err)
	assert.Equal(t, 5.0, planExtra["timeout"], "unrelated caller keys still pass through")
}

func TestBuildMoveDestinationRawCloudPath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}, "timeout": 5.0})
	require.NoError(t, err)
	assert.Equal(t, pathRawCloud, path)
	require.NotNil(t, dest.GoalCloud)
	assert.Equal(t, 0.5, dest.GoalCloud.OZ, "the raw cloud replaces the cone entirely")
	assert.NotContains(t, planExtra, "pose_cloud", "pose_cloud is not a planner key")
	assert.Equal(t, 5.0, planExtra["timeout"])
}

func TestBuildMoveDestinationMetricTypePath(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, path, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"goal_metric_type": "position_only"})
	require.NoError(t, err)
	assert.Equal(t, pathMetricType, path)
	assert.Nil(t, dest.GoalCloud, "no cloud: position_only sets orientScale=0, making a cloud meaningless")
	assert.Equal(t, "position_only", planExtra["goal_metric_type"], "the caller's metric is forwarded")
}

func TestBuildMoveDestinationRejectsBothKeys(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	_, _, _, err := buildMoveDestination("a_origin", testGoal(), cfg, map[string]interface{}{
		"pose_cloud":       map[string]interface{}{"oz": 0.5},
		"goal_metric_type": "position_only",
	})
	require.Error(t, err, "incoherent: a raw cloud plus a metric that ignores clouds")
	// The message is the entire UX here -- there is no fallback -- so pin that it names
	// both offending keys rather than leaving the caller to guess which one to drop.
	assert.ErrorContains(t, err, "pose_cloud")
	assert.ErrorContains(t, err, "goal_metric_type")
}

func TestBuildMoveDestinationDoesNotMutateCallerExtra(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	extra := map[string]interface{}{"pose_cloud": map[string]interface{}{"oz": 0.5}}
	_, _, _, err := buildMoveDestination("a_origin", testGoal(), cfg, extra)
	require.NoError(t, err)
	assert.Contains(t, extra, "pose_cloud", "the caller's map must not be mutated")
}

func TestBuildMoveDestinationPropagatesParseError(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	dest, planExtra, _, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"OX": 1.0}})
	require.Error(t, err)
	assert.Nil(t, dest)
	assert.Nil(t, planExtra)
}

func TestBuildMoveDestinationReturnsPathInvalidOnError(t *testing.T) {
	cfg := resolveGoalCloudConfig(0, 0, nil)
	_, _, path, err := buildMoveDestination("a_origin", testGoal(), cfg,
		map[string]interface{}{"pose_cloud": map[string]interface{}{"OX": 1.0}})
	require.Error(t, err)
	assert.Equal(t, pathInvalid, path,
		"an error must not return pathCone, or wrapMoveErr would blame the cone for a parse error")
	// The guarantee that makes this matter: wrapMoveErr must not dress it up as a cone failure.
	assert.Equal(t, err, wrapMoveErr(err, path, cfg), "pathInvalid must pass the error through unwrapped")
}

// TestGoalCloudSurvivesSerialization guards the module-boundary gotcha: the cloud is only
// useful if it crosses gRPC. An in-process check would pass even if it never shipped.
func TestGoalCloudSurvivesSerialization(t *testing.T) {
	cfg := resolveGoalCloudConfig(30, 1.0, nil)
	dest, _, _, err := buildMoveDestination("myarm_origin", testGoal(), cfg, nil)
	require.NoError(t, err)

	proto := referenceframe.PoseInFrameToProtobuf(dest)
	require.NotNil(t, proto.GoalCloud, "the cloud must be serialized onto the wire")

	got := referenceframe.ProtobufToPoseInFrame(proto)
	require.NotNil(t, got.GoalCloud, "the cloud must survive the round trip")

	want := dest.GoalCloud
	assert.InDelta(t, want.X, got.GoalCloud.X, 1e-9)
	assert.InDelta(t, want.Y, got.GoalCloud.Y, 1e-9)
	assert.InDelta(t, want.Z, got.GoalCloud.Z, 1e-9)
	assert.InDelta(t, want.OX, got.GoalCloud.OX, 1e-9)
	assert.InDelta(t, want.OY, got.GoalCloud.OY, 1e-9)
	assert.InDelta(t, want.OZ, got.GoalCloud.OZ, 1e-9)
	assert.InDelta(t, want.Theta, got.GoalCloud.Theta, 1e-9)
}

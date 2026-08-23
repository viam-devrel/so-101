package geometry

import (
	"maps"
	"math"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// meshWorldPoints returns every triangle vertex of a mesh, transformed by the mesh's pose.
func meshWorldPoints(m *spatialmath.Mesh) []r3.Vector {
	pose := m.Pose()
	var out []r3.Vector
	for _, tri := range m.Triangles() {
		for _, p := range tri.Points() {
			out = append(out, spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(p)).Point())
		}
	}
	return out
}

// TestGripperTCPLiesBetweenTheJawTips anchors GripperTCPPose to the meshes rather than to a
// magic constant: the TCP must sit in the gap between the two contact pads when the jaws are
// closed, on the pads' span along the approach axis, and centered across the jaw width. If the
// gripper meshes are ever regenerated with different geometry, this fails instead of silently
// leaving the TCP floating inside a finger.
func TestGripperTCPLiesBetweenTheJawTips(t *testing.T) {
	geoms, err := BuildGripperMeshes(FollowerGripper, HighDetail, GripperJointMin)
	require.NoError(t, err)
	require.Len(t, geoms, 2, "follower gripper is a static body plus a moving jaw")

	static := meshWorldPoints(geoms[0].(*spatialmath.Mesh))
	moving := meshWorldPoints(geoms[1].(*spatialmath.Mesh))

	// The contact pads are the innermost faces of the two fingertips. Isolate them by taking
	// the points in the tip region (top 10mm along the +Z approach axis) of each mesh.
	tipRegion := func(pts []r3.Vector) []r3.Vector {
		maxZ := math.Inf(-1)
		for _, p := range pts {
			maxZ = math.Max(maxZ, p.Z)
		}
		var tip []r3.Vector
		for _, p := range pts {
			if p.Z > maxZ-10 {
				tip = append(tip, p)
			}
		}
		return tip
	}
	staticTip, movingTip := tipRegion(static), tipRegion(moving)

	// Static finger closes from +X, moving jaw from -X: the gap is between the static tip's
	// lowest X and the moving tip's highest X.
	staticFaceX, movingFaceX := math.Inf(1), math.Inf(-1)
	padMinZ, padMaxZ := math.Inf(1), math.Inf(-1)
	for _, p := range staticTip {
		staticFaceX = math.Min(staticFaceX, p.X)
		padMinZ, padMaxZ = math.Min(padMinZ, p.Z), math.Max(padMaxZ, p.Z)
	}
	for _, p := range movingTip {
		movingFaceX = math.Max(movingFaceX, p.X)
	}
	require.Greater(t, staticFaceX, movingFaceX, "closed jaws should not interpenetrate")

	tcp := GripperTCPPose.Point()
	assert.Greaterf(t, tcp.X, movingFaceX, "TCP X %.3f is inside the moving jaw (face at %.3f)", tcp.X, movingFaceX)
	assert.Lessf(t, tcp.X, staticFaceX, "TCP X %.3f is inside the static finger (face at %.3f)", tcp.X, staticFaceX)
	assert.InDeltaf(t, (staticFaceX+movingFaceX)/2, tcp.X, 0.5,
		"TCP X should be centered in the closed-jaw gap [%.3f, %.3f]", movingFaceX, staticFaceX)
	assert.Greaterf(t, tcp.Z, padMinZ, "TCP Z %.3f is behind the contact pads (start %.3f)", tcp.Z, padMinZ)
	assert.Lessf(t, tcp.Z, padMaxZ, "TCP Z %.3f is past the fingertips (end %.3f)", tcp.Z, padMaxZ)
	assert.InDeltaf(t, 0, tcp.Y, 1.0, "TCP should be centered across the jaw width, got Y=%.3f", tcp.Y)

	// The TCP is a pure translation: it keeps the arm tool frame's axes, so +Z stays the
	// approach axis that the goal-cloud orientation planning assumes.
	assert.True(t, spatialmath.OrientationAlmostEqual(GripperTCPPose.Orientation(), spatialmath.NewZeroOrientation()),
		"TCP must not rotate the tool frame")
}

// TestBuildGripperModelPlacesFrameAtTCP is the core of the reference-frame change: the gripper's
// kinematic model must be zero-DoF and end at the TCP, so the frame system reports the gripper's
// pose at the jaw tips rather than at the arm's wrist.
func TestBuildGripperModelPlacesFrameAtTCP(t *testing.T) {
	for _, gt := range []string{FollowerGripper, LeaderGripper} {
		t.Run(gt, func(t *testing.T) {
			model, err := BuildGripperModel(gt, LowDetail, "gripper")
			require.NoError(t, err)
			require.NotNil(t, model)

			assert.Empty(t, model.DoF(), "the gripper model must be static -- a DoF would become a planning variable")

			pose, err := model.Transform(nil)
			require.NoError(t, err)
			want := GripperTCPPose
			if gt == LeaderGripper {
				// The leader gripper is a hand-held trigger, not a pair of jaws: it has no TCP,
				// so its frame stays at the mount.
				want = spatialmath.NewZeroPose()
			}
			assert.Lessf(t, pose.Point().Sub(want.Point()).Norm(), 1e-6,
				"model frame at %v, want TCP %v", pose.Point(), want.Point())
		})
	}
}

// TestBuildGripperModelCarriesMeshes checks that the model carries the gripper meshes, since the
// frame system takes an arm/gantry/gripper part's collision geometry from its kinematic model and
// never calls Geometries() for it. Without this the gripper is an invisible obstacle to planning.
func TestBuildGripperModelCarriesMeshes(t *testing.T) {
	for _, tc := range []struct {
		gripperType string
		meshes      int
	}{
		{FollowerGripper, 2},
		{LeaderGripper, 3},
	} {
		t.Run(tc.gripperType, func(t *testing.T) {
			model, err := BuildGripperModel(tc.gripperType, LowDetail, "gripper")
			require.NoError(t, err)

			gif, err := model.Geometries(nil)
			require.NoError(t, err)
			require.Len(t, gif.Geometries(), tc.meshes)

			// Model geometry must land exactly where Geometries() draws the closed gripper,
			// so the collision volume and the rendered meshes agree.
			want, err := BuildGripperMeshes(tc.gripperType, LowDetail, GripperJointMin)
			require.NoError(t, err)
			for i, got := range gif.Geometries() {
				assert.Lessf(t, got.Pose().Point().Sub(want[i].Pose().Point()).Norm(), 1e-6,
					"model geometry %d at %v, want %v", i, got.Pose().Point(), want[i].Pose().Point())
			}
		})
	}
}

// TestGripperModelSurvivesSerialization is the module-boundary guard, the same one the arm needs
// (see TestURDFModelSurvivesSerialization): a component ships its kinematics as
// ModelConfig().OriginalFile.Bytes and viam-server rebuilds the model from those bytes. A model
// assembled in memory without OriginalFile transmits as UNSPECIFIED and the server sees nothing --
// which every in-process check would miss.
func TestGripperModelSurvivesSerialization(t *testing.T) {
	model, err := BuildGripperModel(FollowerGripper, LowDetail, "gripper")
	require.NoError(t, err)

	resp := referenceframe.KinematicModelToProtobuf(model)
	transmitted, err := referenceframe.KinematicModelFromProtobuf("gripper", resp)
	require.NoError(t, err, "server-side reconstruction from transmitted kinematics")

	pose, err := transmitted.Transform(nil)
	require.NoError(t, err)
	assert.Lessf(t, pose.Point().Sub(GripperTCPPose.Point()).Norm(), 1e-6,
		"transmitted model frame at %v, want TCP %v", pose.Point(), GripperTCPPose.Point())

	gif, err := transmitted.Geometries(nil)
	require.NoError(t, err)
	assert.Len(t, gif.Geometries(), 2, "transmitted model lost its gripper meshes")
}

// TestGripperFrameResolvesToTCPInFrameSystem is the end-to-end contract: assembled the way
// viam-server assembles it -- the gripper part parented to the arm's "tool" frame with a zero
// offset, exactly what a machine config with `"frame": {"parent": "<arm>"}` produces -- the
// frame named after the gripper component must resolve to the TCP, and the gripper meshes must
// appear as frame-system geometry.
func TestGripperFrameResolvesToTCPInFrameSystem(t *testing.T) {
	armModel, err := ArmModelJSON("arm")
	require.NoError(t, err)
	gripperModel, err := BuildGripperModel(FollowerGripper, LowDetail, "gripper")
	require.NoError(t, err)

	armLink, err := (&referenceframe.LinkConfig{ID: "arm", Parent: referenceframe.World}).ParseConfig()
	require.NoError(t, err)
	gripperLink, err := (&referenceframe.LinkConfig{ID: "gripper", Parent: "arm"}).ParseConfig()
	require.NoError(t, err)

	fs, err := referenceframe.NewFrameSystem("test", []*referenceframe.FrameSystemPart{
		{FrameConfig: armLink, ModelFrame: armModel},
		{FrameConfig: gripperLink, ModelFrame: gripperModel},
	}, nil)
	require.NoError(t, err)

	inputs := referenceframe.NewZeroInputs(fs)
	toolPose, err := fs.Transform(inputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame("arm", spatialmath.NewZeroPose()), referenceframe.World)
	require.NoError(t, err)
	gripperPose, err := fs.Transform(inputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame("gripper", spatialmath.NewZeroPose()), referenceframe.World)
	require.NoError(t, err)

	// The gripper frame must sit GripperTCPPose ahead of the arm's tool frame.
	offset := spatialmath.PoseBetween(
		toolPose.(*referenceframe.PoseInFrame).Pose(), gripperPose.(*referenceframe.PoseInFrame).Pose())
	assert.Lessf(t, offset.Point().Sub(GripperTCPPose.Point()).Norm(), 1e-6,
		"gripper frame is %v from the arm tool frame, want the TCP at %v", offset.Point(), GripperTCPPose.Point())

	geoms, err := referenceframe.FrameSystemGeometries(fs, inputs)
	require.NoError(t, err)
	gif, ok := geoms["gripper"]
	require.True(t, ok, "the gripper contributed no geometry to the frame system: %v", slices.Collect(maps.Keys(geoms)))
	assert.Len(t, gif.Geometries(), 2, "gripper meshes missing from frame-system geometry")
}

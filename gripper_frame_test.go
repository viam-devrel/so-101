package so_arm

import (
	"context"
	"maps"
	"slices"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
)

// TestSimulatedGripperKinematics covers the constructor wiring: on viam-server >= 1.0.0 a gripper
// whose Kinematics() errors is dropped from the frame system entirely, so returning a model is
// what puts the gripper back on the robot at all.
func TestSimulatedGripperKinematics(t *testing.T) {
	ctx := context.Background()
	g, err := newSimulatedSO101Gripper(ctx, nil,
		resource.Config{Name: "sim-gripper", ConvertedAttributes: &SO101SimulatedGripperConfig{}},
		logging.NewTestLogger(t))
	require.NoError(t, err)
	defer func() { require.NoError(t, g.Close(ctx)) }()

	model, err := g.Kinematics(ctx)
	require.NoError(t, err)
	require.NotNil(t, model)

	pose, err := model.Transform(nil)
	require.NoError(t, err)
	assert.Lessf(t, pose.Point().Sub(geometry.GripperTCPPose.Point()).Norm(), 1e-6,
		"simulated gripper frame at %v, want TCP %v", pose.Point(), geometry.GripperTCPPose.Point())
}

// TestGripperFrameResolvesToTCPInFrameSystem is the end-to-end contract: assembled the way
// viam-server assembles it -- the gripper part parented to the arm's "tool" frame with a zero
// offset, exactly what a machine config with `"frame": {"parent": "<arm>"}` produces -- the
// frame named after the gripper component must resolve to the TCP, and the gripper meshes must
// appear as frame-system geometry.
func TestGripperFrameResolvesToTCPInFrameSystem(t *testing.T) {
	armModel, err := geometry.ArmModelJSON("arm")
	require.NoError(t, err)
	gripperModel, err := geometry.BuildGripperModel(geometry.FollowerGripper, geometry.LowDetail, "gripper")
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

	// The gripper frame must sit geometry.GripperTCPPose ahead of the arm's tool frame.
	offset := spatialmath.PoseBetween(
		toolPose.(*referenceframe.PoseInFrame).Pose(), gripperPose.(*referenceframe.PoseInFrame).Pose())
	assert.Lessf(t, offset.Point().Sub(geometry.GripperTCPPose.Point()).Norm(), 1e-6,
		"gripper frame is %v from the arm tool frame, want the TCP at %v", offset.Point(), geometry.GripperTCPPose.Point())

	geoms, err := referenceframe.FrameSystemGeometries(fs, inputs)
	require.NoError(t, err)
	gif, ok := geoms["gripper"]
	require.True(t, ok, "the gripper contributed no geometry to the frame system: %v", slices.Collect(maps.Keys(geoms)))
	assert.Len(t, gif.Geometries(), 2, "gripper meshes missing from frame-system geometry")
}

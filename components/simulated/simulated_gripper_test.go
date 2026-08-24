package simulated

import (
	"context"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
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

func newTestSimGripper(t *testing.T, gripperType, meshDetail string) gripper.Gripper {
	t.Helper()
	conf := resource.Config{
		Name:                "testGripper",
		API:                 gripper.API,
		Model:               SO101SimulatedGripperModel,
		ConvertedAttributes: &SO101SimulatedGripperConfig{GripperType: gripperType, MeshDetail: meshDetail},
	}
	g, err := newSimulatedSO101Gripper(context.Background(), nil, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return g
}

func TestSimulatedGripperConfigValidate(t *testing.T) {
	// An unset gripper_type / mesh_detail is accepted (defaults applied).
	_, _, err := (&SO101SimulatedGripperConfig{}).Validate("")
	require.NoError(t, err)

	for _, gt := range []string{geometry.LeaderGripper, geometry.FollowerGripper} {
		_, _, err := (&SO101SimulatedGripperConfig{GripperType: gt}).Validate("")
		require.NoError(t, err, "gripper_type %q should be valid", gt)
	}
	for _, md := range []string{geometry.HighDetail, geometry.LowDetail} {
		_, _, err := (&SO101SimulatedGripperConfig{MeshDetail: md}).Validate("")
		require.NoError(t, err, "mesh_detail %q should be valid", md)
	}

	_, _, err = (&SO101SimulatedGripperConfig{GripperType: "bogus"}).Validate("")
	require.Error(t, err)
	_, _, err = (&SO101SimulatedGripperConfig{MeshDetail: "bogus"}).Validate("")
	require.Error(t, err)
}

func TestSimulatedGripperGeometries(t *testing.T) {
	ctx := context.Background()
	for _, tc := range []struct {
		gripperType string
		meshes      int
	}{
		{geometry.FollowerGripper, 2}, // body + jaw
		{geometry.LeaderGripper, 3},   // wrist-roll + handle + trigger
	} {
		g := newTestSimGripper(t, tc.gripperType, "")
		geoms, err := g.Geometries(ctx, nil)
		require.NoError(t, err, "gripper_type %q", tc.gripperType)
		require.Len(t, geoms, tc.meshes)
		for _, geom := range geoms {
			_, ok := geom.(*spatialmath.Mesh)
			require.True(t, ok, "gripper geometry should be a mesh")
		}
		require.NoError(t, g.Close(ctx))
	}
}

// TestSimulatedGripperMeshDetail confirms the mesh_detail attribute selects denser
// meshes for "high" than for "low".
func TestSimulatedGripperMeshDetail(t *testing.T) {
	ctx := context.Background()
	triangles := func(detail string) int {
		g := newTestSimGripper(t, geometry.FollowerGripper, detail)
		defer func() { require.NoError(t, g.Close(ctx)) }()
		geoms, err := g.Geometries(ctx, nil)
		require.NoError(t, err)
		n := 0
		for _, geom := range geoms {
			n += len(geom.(*spatialmath.Mesh).Triangles())
		}
		return n
	}
	assert.Greater(t, triangles(geometry.HighDetail), triangles(geometry.LowDetail),
		"high mesh_detail should serve denser meshes than low")
}

// TestSimulatedGripperArticulates checks that Open interpolates over time -- the gripper
// reports moving mid-travel -- and that the moving mesh ends up posed differently.
func TestSimulatedGripperArticulates(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, geometry.FollowerGripper, "")
	defer func() { require.NoError(t, g.Close(ctx)) }()

	closedGeoms, err := g.Geometries(ctx, nil)
	require.NoError(t, err)

	// Open in the background so motion can be observed partway through.
	openErr := make(chan error, 1)
	go func() { openErr <- g.Open(ctx, nil) }()

	sawMoving := false
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		moving, err := g.IsMoving(ctx)
		require.NoError(t, err)
		if moving {
			sawMoving = true
			break
		}
		time.Sleep(time.Millisecond)
	}
	assert.True(t, sawMoving, "gripper should report moving while opening")

	select {
	case err := <-openErr:
		require.NoError(t, err)
	case <-time.After(3 * time.Second):
		t.Fatal("Open did not return")
	}

	moving, err := g.IsMoving(ctx)
	require.NoError(t, err)
	assert.False(t, moving, "gripper should be stopped once Open returns")

	openGeoms, err := g.Geometries(ctx, nil)
	require.NoError(t, err)
	assert.False(t,
		spatialmath.PoseAlmostEqual(closedGeoms[1].Pose(), openGeoms[1].Pose()),
		"the moving part should be posed differently when open vs closed")
}

// TestSimulatedGripperOpenReachesMaxAngle pins the magnitude of a fully-open pose, not just its
// direction. A wrong percent<->radian mapping (e.g. leaving the percent as a 0..1 fraction) still
// produces a pose distinct from closed, so TestSimulatedGripperArticulates's "not equal" check
// alone would not have caught it.
func TestSimulatedGripperOpenReachesMaxAngle(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, geometry.FollowerGripper, "")
	defer func() { require.NoError(t, g.Close(ctx)) }()

	require.NoError(t, g.Open(ctx, nil))

	openGeoms, err := g.Geometries(ctx, nil)
	require.NoError(t, err)

	wantGeoms, err := geometry.BuildGripperMeshes(geometry.FollowerGripper, geometry.LowDetail, geometry.GripperJointMax)
	require.NoError(t, err)

	assert.True(t, spatialmath.PoseAlmostEqual(openGeoms[1].Pose(), wantGeoms[1].Pose()),
		"open moving-part pose %v, want %v (fully open)", openGeoms[1].Pose(), wantGeoms[1].Pose())
}

package so_arm

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
)

func newTestSimGripper(t *testing.T, gripperType string) gripper.Gripper {
	t.Helper()
	conf := resource.Config{
		Name:                "testGripper",
		API:                 gripper.API,
		Model:               SO101SimulatedGripperModel,
		ConvertedAttributes: &SO101SimulatedGripperConfig{GripperType: gripperType},
	}
	g, err := newSimulatedSO101Gripper(context.Background(), nil, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return g
}

func TestSimulatedGripperConfigValidate(t *testing.T) {
	// An unset gripper_type is accepted (defaults to follower).
	_, _, err := (&SO101SimulatedGripperConfig{}).Validate("")
	require.NoError(t, err)

	for _, gt := range []string{leaderGripper, followerGripper} {
		_, _, err := (&SO101SimulatedGripperConfig{GripperType: gt}).Validate("")
		require.NoError(t, err, "gripper_type %q should be valid", gt)
	}

	_, _, err = (&SO101SimulatedGripperConfig{GripperType: "bogus"}).Validate("")
	require.Error(t, err)
}

func TestSimulatedGripperGeometries(t *testing.T) {
	ctx := context.Background()
	for _, tc := range []struct {
		gripperType string
		meshes      int
	}{
		{followerGripper, 2}, // body + jaw
		{leaderGripper, 3},   // wrist-roll + handle + trigger
	} {
		g := newTestSimGripper(t, tc.gripperType)
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

// TestSimulatedGripperArticulates checks that Open interpolates over time -- the gripper
// reports moving mid-travel -- and that the moving mesh ends up posed differently.
func TestSimulatedGripperArticulates(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, followerGripper)
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

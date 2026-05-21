package so_arm

import (
	"context"
	"testing"

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
	for _, gt := range []string{followerGripper, leaderGripper} {
		g := newTestSimGripper(t, gt)
		defer func() { require.NoError(t, g.Close(ctx)) }()

		geoms, err := g.Geometries(ctx, nil)
		require.NoError(t, err, "gripper_type %q", gt)
		require.Len(t, geoms, 2)
		for _, geom := range geoms {
			_, ok := geom.(*spatialmath.Mesh)
			require.True(t, ok, "gripper geometry should be a mesh")
		}
	}
}

func TestSimulatedGripperArticulates(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, followerGripper)
	defer func() { require.NoError(t, g.Close(ctx)) }()

	require.NoError(t, g.Open(ctx, nil))
	openGeoms, err := g.Geometries(ctx, nil)
	require.NoError(t, err)

	_, err = g.Grab(ctx, nil)
	require.NoError(t, err)
	closedGeoms, err := g.Geometries(ctx, nil)
	require.NoError(t, err)

	// geoms[1] is the moving part; opening vs closing rotates it about the joint.
	assert.False(t,
		spatialmath.PoseAlmostEqual(openGeoms[1].Pose(), closedGeoms[1].Pose()),
		"the moving part should be posed differently when open vs closed")

	moving, err := g.IsMoving(ctx)
	require.NoError(t, err)
	assert.False(t, moving)
}

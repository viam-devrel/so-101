package simulated

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/gripper"
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
	t.Cleanup(func() { require.NoError(t, g.Close(context.Background())) })
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

// TestArticulatedJawConfig covers the flag end to end. The ErrUnsupported half matters as much
// as the enabled half: robot/framesystem's CurrentInputs walks every frame with DoF and hard-
// errors if the component is not InputEnabled, so a 1-DoF model with unwired inputs would break
// the WHOLE machine's frame system, arm moves included.
func TestArticulatedJawConfig(t *testing.T) {
	ctx := context.Background()

	newGripper := func(t *testing.T, articulated bool) gripper.Gripper {
		t.Helper()
		g, err := newSimulatedSO101Gripper(ctx, nil, resource.Config{
			Name:                "sim-gripper",
			ConvertedAttributes: &SO101SimulatedGripperConfig{ArticulatedJaw: articulated},
		}, logging.NewTestLogger(t))
		require.NoError(t, err)
		t.Cleanup(func() { require.NoError(t, g.Close(ctx)) })
		return g
	}

	t.Run("default is static", func(t *testing.T) {
		g := newGripper(t, false)
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		assert.Empty(t, m.DoF(), "articulated_jaw must default to false")

		_, err = g.CurrentInputs(ctx)
		assert.ErrorIs(t, err, errors.ErrUnsupported)
		assert.ErrorIs(t, g.GoToInputs(ctx, []referenceframe.Input{0}), errors.ErrUnsupported)
	})

	t.Run("articulated", func(t *testing.T) {
		g := newGripper(t, true)
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		require.Len(t, m.DoF(), 1)

		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		require.Len(t, in, 1)
	})
}

// TestSetPositionStaysNonBlocking is a regression guard for the teleop loop: teleop.go issues a
// set_position DoCommand every tick and depends on it returning before the jaw arrives. Open and
// Grab block; set_position must not.
func TestSetPositionStaysNonBlocking(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, geometry.FollowerGripper, geometry.LowDetail)

	_, err := g.Grab(ctx, nil) // start closed
	require.NoError(t, err)
	start := time.Now()
	_, err = g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position", "percentage": 100.0,
	})
	require.NoError(t, err)
	elapsed := time.Since(start)

	// A full 0->100 sweep takes ~500ms at gripperTravelPctPerSec. Returning in under 50ms proves
	// we did not wait for arrival.
	assert.Lessf(t, elapsed, 50*time.Millisecond,
		"set_position blocked for %v; the teleop loop ticks faster than that", elapsed)

	moving, err := g.IsMoving(ctx)
	require.NoError(t, err)
	assert.True(t, moving, "set_position should have started the jaw moving")
}

func newArticulatedTestGripper(t *testing.T) *simulatedSO101Gripper {
	t.Helper()
	ctx := context.Background()
	g, err := newSimulatedSO101Gripper(ctx, nil, resource.Config{
		Name:                "sim-gripper",
		ConvertedAttributes: &SO101SimulatedGripperConfig{ArticulatedJaw: true},
	}, logging.NewTestLogger(t))
	require.NoError(t, err)
	t.Cleanup(func() { require.NoError(t, g.Close(ctx)) })
	return g.(*simulatedSO101Gripper)
}

// TestGripperCurrentInputsTracksJaw checks the input reported to the frame system follows the
// simulated jaw through the same bijection the meshes use.
func TestGripperCurrentInputsTracksJaw(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	require.NoError(t, g.Open(ctx, nil))
	in, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	require.Len(t, in, 1)
	assert.InDelta(t, geometry.GripperJointMax, in[0], 1e-9, "open jaw should report the upper limit")

	_, err = g.Grab(ctx, nil)
	require.NoError(t, err)
	in, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.InDelta(t, geometry.GripperJointMin, in[0], 1e-9, "closed jaw should report the lower limit")
}

// TestGoToInputsValidation covers what the motion service will actually send. execute() batches
// consecutive trajectory steps into one variadic GoToInputs call, so multi-step batches are the
// normal case, not the exception.
func TestGoToInputsValidation(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)

	t.Run("rejects wrong arity", func(t *testing.T) {
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{0, 0}))
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{}))
	})

	t.Run("rejects out-of-limit angles", func(t *testing.T) {
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax + 0.5}))
		assert.Error(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMin - 0.5}))
	})

	// Pins the chosen ULP-overshoot handling (see jawLimitEpsilon in simulated_gripper.go): a
	// value 1 ULP above the limit -- exactly what Task 4's left-associative sweep produced --
	// must not be rejected as out of bounds.
	t.Run("accepts a 1-ULP overshoot at the limit", func(t *testing.T) {
		overshoot := math.Nextafter(geometry.GripperJointMax, math.Inf(1))
		assert.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{overshoot}))
	})

	t.Run("applies a multi-step batch in order, ending at the last", func(t *testing.T) {
		mid := (geometry.GripperJointMin + geometry.GripperJointMax) / 2
		require.NoError(t, g.GoToInputs(ctx,
			[]referenceframe.Input{mid},
			[]referenceframe.Input{geometry.GripperJointMax},
			[]referenceframe.Input{geometry.GripperJointMin},
		))
		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		assert.InDelta(t, geometry.GripperJointMin, in[0], 1e-9)
	})
}

// TestGoToInputsAbortsOnStop is a regression guard: Stop() sets targetPct = currentPct, which
// without an explicit stopped flag makes an in-flight awaitArrival indistinguishable from
// "arrived". GoToInputs must report the stop as an error and must not silently carry on to
// (or land on) the batch's final target.
func TestGoToInputsAbortsOnStop(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)
	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMin})) // start closed

	stopErr := make(chan error, 1)
	go func() {
		time.Sleep(60 * time.Millisecond)
		stopErr <- g.Stop(ctx, nil)
	}()

	mid := (geometry.GripperJointMin + geometry.GripperJointMax) / 2
	err := g.GoToInputs(ctx,
		[]referenceframe.Input{mid},
		[]referenceframe.Input{geometry.GripperJointMax},
	)
	require.NoError(t, <-stopErr)
	require.Error(t, err, "GoToInputs should report the stop rather than silently succeeding")

	in, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Greaterf(t, math.Abs(in[0]-geometry.GripperJointMax), 1e-6,
		"batch should not have reached its final target (%v) after Stop, got %v",
		geometry.GripperJointMax, in[0])
}

// TestGoToInputsAwaitsOnlyFinalStep pins the per-batch latency fix: intermediate steps must not
// each pay a full gripperUpdateInterval tick just to set an in-between waypoint. 50 microsteps
// covering a negligible total angle should complete far faster than 50*gripperUpdateInterval,
// which is what per-step blocking would cost.
func TestGoToInputsAwaitsOnlyFinalStep(t *testing.T) {
	ctx := context.Background()
	g := newArticulatedTestGripper(t)
	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMin}))

	const steps = 50
	const totalRad = 0.0005 // negligible total motion
	batch := make([][]referenceframe.Input, steps)
	for i := range steps {
		batch[i] = []referenceframe.Input{geometry.GripperJointMin + totalRad*float64(i+1)/float64(steps)}
	}

	start := time.Now()
	require.NoError(t, g.GoToInputs(ctx, batch...))
	elapsed := time.Since(start)
	t.Logf("50-step batch of negligible motion took %v (per-step blocking would cost ~%v)",
		elapsed, steps*gripperUpdateInterval)

	assert.Lessf(t, elapsed, steps*gripperUpdateInterval/2,
		"50-step batch of negligible motion took %v; per-step blocking would cost ~%v",
		elapsed, steps*gripperUpdateInterval)
}

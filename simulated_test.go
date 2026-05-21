package so_arm

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

// newTestSimArm constructs a simulated SO-101 with the simulated clock disabled, so tests
// drive time deterministically via updateForTime. speedRadPerSec sets the joint speed in
// radians per second (1.0 makes interpolation arithmetic exact).
func newTestSimArm(t *testing.T, speedRadPerSec float64) *simulatedSO101 {
	t.Helper()
	simulateTime := false
	conf := resource.Config{
		Name:  "testSimArm",
		API:   arm.API,
		Model: SO101SimulatedModel,
		ConvertedAttributes: &SO101SimulatedArmConfig{
			SpeedDegsPerSec: speedRadPerSec * 180.0 / math.Pi,
			SimulateTime:    &simulateTime,
		},
	}
	// deps is nil: the joint-level simulation needs no motion service.
	a, err := newSimulatedSO101(context.Background(), nil, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return a.(*simulatedSO101)
}

func waitForMoving(t *testing.T, a arm.Arm) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		moving, err := a.IsMoving(context.Background())
		require.NoError(t, err)
		if moving {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("arm never started moving")
}

func TestSimulatedConfigValidate(t *testing.T) {
	t.Run("default config declares the builtin motion dependency", func(t *testing.T) {
		deps, optional, err := (&SO101SimulatedArmConfig{}).Validate("")
		require.NoError(t, err)
		assert.Nil(t, optional)
		assert.Equal(t, []string{"rdk:service:motion/builtin"}, deps)
	})

	t.Run("custom motion service name is declared as a dependency", func(t *testing.T) {
		deps, _, err := (&SO101SimulatedArmConfig{Motion: "myMotion"}).Validate("")
		require.NoError(t, err)
		assert.Equal(t, []string{"rdk:service:motion/myMotion"}, deps)
	})

	t.Run("negative speed is rejected", func(t *testing.T) {
		_, _, err := (&SO101SimulatedArmConfig{SpeedDegsPerSec: -1}).Validate("")
		require.Error(t, err)
	})
}

func TestSimulatedKinematics(t *testing.T) {
	sim := newTestSimArm(t, 1.0)
	defer func() { require.NoError(t, sim.Close(context.Background())) }()

	model, err := sim.Kinematics(context.Background())
	require.NoError(t, err)
	// The SO-101 arm has 5 revolute joints.
	assert.Len(t, model.DoF(), 5)

	inputs, err := sim.CurrentInputs(context.Background())
	require.NoError(t, err)
	assert.Equal(t, []referenceframe.Input{0, 0, 0, 0, 0}, inputs)
}

func TestSimulatedMoveToJointPositions(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0) // 1 radian/second
	defer func() { require.NoError(t, sim.Close(ctx)) }()

	// Joint 1 must travel 1.0 rad (the farthest), so the move takes 1 second. Joint 0
	// travels half as far, so it moves at half speed.
	target := []referenceframe.Input{0.5, -1.0, 0, 0, 0}

	moveErr := make(chan error, 1)
	go func() { moveErr <- sim.MoveToJointPositions(ctx, target, nil) }()
	waitForMoving(t, sim)

	// Time has not advanced yet; the arm has not moved.
	inputs, err := sim.JointPositions(ctx, nil)
	require.NoError(t, err)
	assert.Equal(t, []referenceframe.Input{0, 0, 0, 0, 0}, inputs)

	// Advance the simulated clock half a second: the move should be half complete.
	base := time.Time{}
	sim.updateForTime(base.Add(500 * time.Millisecond))
	inputs, err = sim.JointPositions(ctx, nil)
	require.NoError(t, err)
	assert.InDeltaSlice(t, []float64{0.25, -0.5, 0, 0, 0}, inputs, 1e-9)

	moving, err := sim.IsMoving(ctx)
	require.NoError(t, err)
	assert.True(t, moving)
	select {
	case <-moveErr:
		t.Fatal("MoveToJointPositions returned before the move completed")
	default:
	}

	// Advance to one second: the move should be complete and the call should return.
	sim.updateForTime(base.Add(time.Second))
	inputs, err = sim.JointPositions(ctx, nil)
	require.NoError(t, err)
	assert.InDeltaSlice(t, []float64{0.5, -1.0, 0, 0, 0}, inputs, 1e-9)

	select {
	case err := <-moveErr:
		require.NoError(t, err)
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after the move completed")
	}

	moving, err = sim.IsMoving(ctx)
	require.NoError(t, err)
	assert.False(t, moving)
}

func TestSimulatedMoveRejectsWrongJointCount(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { require.NoError(t, sim.Close(ctx)) }()

	err := sim.MoveToJointPositions(ctx, []referenceframe.Input{0, 0, 0}, nil)
	require.Error(t, err)
}

func TestSimulatedStop(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { require.NoError(t, sim.Close(ctx)) }()

	moveErr := make(chan error, 1)
	go func() {
		moveErr <- sim.MoveToJointPositions(ctx, []referenceframe.Input{0.5, -1.0, 0, 0, 0}, nil)
	}()
	waitForMoving(t, sim)

	require.NoError(t, sim.Stop(ctx, nil))

	select {
	case err := <-moveErr:
		require.Error(t, err)
		assert.Contains(t, err.Error(), "stopped before reaching target")
	case <-time.After(time.Second):
		t.Fatal("MoveToJointPositions did not return after Stop")
	}
}

func TestSimulatedEndPosition(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { require.NoError(t, sim.Close(ctx)) }()

	pose, err := sim.EndPosition(ctx, nil)
	require.NoError(t, err)
	require.NotNil(t, pose)

	// Moving a joint should change the end-effector pose.
	sim.updateForTime(time.Time{})
	sim.mu.Lock()
	sim.currInputs = []float64{1.0, 0, 0, 0, 0}
	sim.mu.Unlock()

	moved, err := sim.EndPosition(ctx, nil)
	require.NoError(t, err)
	assert.False(t, pose.Point().X == moved.Point().X &&
		pose.Point().Y == moved.Point().Y &&
		pose.Point().Z == moved.Point().Z,
		"end position should change after a joint moves")
}

func TestSimulatedGet3DModels(t *testing.T) {
	ctx := context.Background()
	sim := newTestSimArm(t, 1.0)
	defer func() { require.NoError(t, sim.Close(ctx)) }()

	models, err := sim.Get3DModels(ctx, nil)
	require.NoError(t, err)
	// One mesh per geometry-bearing link in so101.json.
	require.Len(t, models, 5)
	for _, part := range []string{"base", "shoulder", "upper_arm", "lower_arm", "wrist"} {
		mesh, ok := models[part]
		require.True(t, ok, "missing mesh for %q", part)
		assert.Equal(t, "model/gltf-binary", mesh.ContentType)
		assert.NotEmpty(t, mesh.Mesh, "mesh bytes for %q should not be empty", part)
	}
}

func TestSimulatedTimeSimulation(t *testing.T) {
	ctx := context.Background()
	// With simulate_time left at its default (true), the background goroutine advances
	// the arm on its own, so MoveToJointPositions completes without manual updateForTime.
	conf := resource.Config{
		Name:  "testSimArm",
		API:   arm.API,
		Model: SO101SimulatedModel,
		ConvertedAttributes: &SO101SimulatedArmConfig{
			SpeedDegsPerSec: 2000, // fast, so the move finishes quickly
		},
	}
	a, err := newSimulatedSO101(ctx, nil, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	defer func() { require.NoError(t, a.Close(ctx)) }()

	target := []referenceframe.Input{0.3, -0.3, 0.2, 0, 0}
	require.NoError(t, a.MoveToJointPositions(ctx, target, nil))

	inputs, err := a.JointPositions(ctx, nil)
	require.NoError(t, err)
	assert.InDeltaSlice(t, []float64{0.3, -0.3, 0.2, 0, 0}, inputs, 1e-6)

	moving, err := a.IsMoving(ctx)
	require.NoError(t, err)
	assert.False(t, moving)
}

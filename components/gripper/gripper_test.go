package gripper

import (
	"context"
	"errors"
	"sync"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
	"so_arm/internal/servocmd"
	"so_arm/internal/testfake"
)

func TestSO101GripperConfigValidate(t *testing.T) {
	// An unset gripper_type / mesh_detail is accepted (defaults applied).
	_, _, err := (&SO101GripperConfig{Arm: "my-arm"}).Validate("")
	require.NoError(t, err)

	for _, gt := range []string{geometry.LeaderGripper, geometry.FollowerGripper} {
		_, _, err := (&SO101GripperConfig{Arm: "my-arm", GripperType: gt}).Validate("")
		require.NoError(t, err, "gripper_type %q should be valid", gt)
	}
	for _, md := range []string{geometry.HighDetail, geometry.LowDetail} {
		_, _, err := (&SO101GripperConfig{Arm: "my-arm", MeshDetail: md}).Validate("")
		require.NoError(t, err, "mesh_detail %q should be valid", md)
	}

	_, _, err = (&SO101GripperConfig{Arm: "my-arm", GripperType: "bogus"}).Validate("")
	require.Error(t, err)
	_, _, err = (&SO101GripperConfig{Arm: "my-arm", MeshDetail: "bogus"}).Validate("")
	require.Error(t, err)
}

// The gripper reaches the serial bus only through the arm, so the arm is a required
// dependency: Validate must reject a config without one and must report it so viam-server
// builds the arm first.
func TestGripperConfigRequiresArmDependency(t *testing.T) {
	t.Run("missing arm is rejected", func(t *testing.T) {
		_, _, err := (&SO101GripperConfig{}).Validate("")
		require.Error(t, err)
		assert.Contains(t, err.Error(), "arm")
	})

	t.Run("arm is returned as a required dependency", func(t *testing.T) {
		deps, optional, err := (&SO101GripperConfig{Arm: "my-arm"}).Validate("")
		require.NoError(t, err)
		assert.Equal(t, []string{"my-arm"}, deps)
		assert.Empty(t, optional)
	})
}

func TestGripperConfigDefaultsServoID(t *testing.T) {
	cfg := &SO101GripperConfig{Arm: "my-arm"}
	_, _, err := cfg.Validate("")
	require.NoError(t, err)
	assert.Equal(t, 6, cfg.ServoID)
}

func TestBuildGripperMeshes(t *testing.T) {
	for _, gt := range []string{geometry.FollowerGripper, geometry.LeaderGripper} {
		geoms, err := geometry.BuildGripperMeshes(gt, geometry.LowDetail, 0)
		require.NoError(t, err, "gripper_type %q", gt)
		require.NotEmpty(t, geoms)
		for _, g := range geoms {
			_, ok := g.(*spatialmath.Mesh)
			assert.True(t, ok, "expected mesh geometry for %q", gt)
		}
	}
}

// fakeServoArm is an arm.Arm that records the servo_* DoCommands the gripper sends and
// replies with canned data. It is the whole point of the dependency refactor: with the arm
// as the seam, the gripper is unit-testable without a serial port.
type fakeServoArm struct {
	arm.Arm
	name resource.Name

	mu       sync.Mutex
	commands []map[string]any

	capabilities []int   // servo_ids reported by servo_capabilities
	percent      float64 // returned by servo_position
	raw          int
	doErr        error // returned for every non-capabilities command

	moving         bool           // returned by servo_moving, unless movingResponse is set
	movingResponse map[string]any // overrides the servo_moving response entirely, for shape tests
	onServoMoving  func()         // called on a servo_moving command, before it is answered
}

func newFakeServoArm() *fakeServoArm {
	return &fakeServoArm{
		name:         arm.Named("test-arm"),
		capabilities: []int{6},
	}
}

func (f *fakeServoArm) Name() resource.Name { return f.name }

func (f *fakeServoArm) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.commands = append(f.commands, cmd)

	if cmd["command"] == servocmd.CmdServoCapabilities {
		return map[string]any{"servo_commands": true, "servo_ids": f.capabilities}, nil
	}
	if cmd["command"] == servocmd.CmdServoMoving && f.onServoMoving != nil {
		f.onServoMoving()
	}
	if f.doErr != nil {
		return nil, f.doErr
	}
	switch cmd["command"] {
	case servocmd.CmdServoPosition:
		return map[string]any{"percent": f.percent, "raw": f.raw}, nil
	case servocmd.CmdServoMoving:
		if f.movingResponse != nil {
			return f.movingResponse, nil
		}
		return map[string]any{"moving": f.moving}, nil
	default:
		return map[string]any{}, nil
	}
}

// issued returns the command names sent so far, in order, ignoring the constructor probe.
func (f *fakeServoArm) issued() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	var names []string
	for _, c := range f.commands {
		name, _ := c["command"].(string)
		if name == servocmd.CmdServoCapabilities {
			continue
		}
		names = append(names, name)
	}
	return names
}

// lastCommand returns the most recent command with the given name.
func (f *fakeServoArm) lastCommand(name string) map[string]any {
	f.mu.Lock()
	defer f.mu.Unlock()
	for i := len(f.commands) - 1; i >= 0; i-- {
		if n, _ := f.commands[i]["command"].(string); n == name {
			return f.commands[i]
		}
	}
	return nil
}

func newTestGripper(t *testing.T, fa *fakeServoArm) *so101Gripper {
	t.Helper()
	deps := resource.Dependencies{fa.name: fa}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: fa.name.Name},
	}
	g, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return g.(*so101Gripper)
}

func TestGripperConstructorResolvesArmDependency(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	assert.Equal(t, 6, g.servoID)
	assert.Same(t, fa, g.arm, "gripper must hold the arm dependency itself")
}

// A misconfigured arm reference should fail at construction, not at the first move.
func TestGripperConstructorProbesCapabilities(t *testing.T) {
	fa := newFakeServoArm()
	_ = newTestGripper(t, fa)

	require.NotEmpty(t, fa.commands)
	assert.Equal(t, servocmd.CmdServoCapabilities, fa.commands[0]["command"],
		"constructor must probe before being used")
}

func TestGripperConstructorRejectsUnofferedServo(t *testing.T) {
	fa := newFakeServoArm()
	fa.capabilities = []int{5} // arm does not offer servo 6

	deps := resource.Dependencies{fa.name: fa}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: fa.name.Name},
	}
	_, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.Error(t, err)
	assert.Contains(t, err.Error(), "6")
}

func TestGripperConstructorRejectsMissingDependency(t *testing.T) {
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: "nonexistent"},
	}
	_, err := newSO101Gripper(context.Background(), resource.Dependencies{}, conf,
		logging.NewTestLogger(t))
	require.Error(t, err)
}

func TestGripperOpenCommandsServoAndWaits(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	require.NoError(t, g.Open(context.Background(), nil))

	assert.Equal(t, []string{servocmd.CmdServoMove, servocmd.CmdServoWaitStop}, fa.issued(),
		"Open should command the servo then wait for it to settle")

	move := fa.lastCommand(servocmd.CmdServoMove)
	assert.Equal(t, 6, move["servo_id"])
	assert.Equal(t, 95.0, move["percent"], "open position, as a percentage")
}

// Stopping the gripper must not stop the arm. The old implementation called a controller
// Stop that zeroed velocity on every servo on the bus.
func TestGripperStopIsScopedToItsOwnServo(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	require.NoError(t, g.Stop(context.Background(), nil))

	assert.Equal(t, []string{servocmd.CmdServoStop}, fa.issued())
	assert.Equal(t, 6, fa.lastCommand(servocmd.CmdServoStop)["servo_id"])
}

// The atomic intent flag is a fast path: when set, IsMoving must return true without
// sending any servo_moving DoCommand at all.
func TestGripperIsMovingFastPathShortCircuits(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)
	g.isMoving.Store(true)

	moving, err := g.IsMoving(context.Background())
	require.NoError(t, err)
	assert.True(t, moving)
	assert.Empty(t, fa.issued(), "fast path must not query the hardware")
}

func TestGripperIsMovingQueriesHardwareWhenFlagFalse(t *testing.T) {
	for _, want := range []bool{true, false} {
		fa := newFakeServoArm()
		fa.moving = want
		g := newTestGripper(t, fa)

		moving, err := g.IsMoving(context.Background())
		require.NoError(t, err)
		assert.Equal(t, want, moving)
		assert.Equal(t, []string{servocmd.CmdServoMoving}, fa.issued())
	}
}

// A response missing "moving" entirely (e.g. an older module build that never wrote the
// key) must not be silently read as "stopped".
func TestGripperIsMovingFallsBackOnMissingKey(t *testing.T) {
	fa := newFakeServoArm()
	fa.movingResponse = map[string]any{}
	g := newTestGripper(t, fa)

	moving, err := g.IsMoving(context.Background())
	require.NoError(t, err)
	assert.False(t, moving, "flag is false, so fallback reports not-moving")
}

// A remote arm on an older module build returns "unknown servo command" for servo_moving;
// IsMoving must fall back to the intent flag rather than fail outright.
func TestGripperIsMovingFallsBackOnUnknownCommandError(t *testing.T) {
	fa := newFakeServoArm()
	fa.doErr = errors.New("unknown servo command: servo_moving")
	g := newTestGripper(t, fa)

	moving, err := g.IsMoving(context.Background())
	require.NoError(t, err)
	assert.False(t, moving)
}

// The fallback's two Load()s are separated by a full DoCommand round trip, so a concurrent
// Open() can legitimately flip the flag between them. This pins that re-read: the hook flips
// the flag mid-round-trip, so a value cached before the call would wrongly report false.
func TestGripperIsMovingFallbackReReadsFlagAfterRoundTrip(t *testing.T) {
	fa := newFakeServoArm()
	fa.doErr = errors.New("unknown servo command: servo_moving")
	g := newTestGripper(t, fa)
	fa.onServoMoving = func() { g.isMoving.Store(true) }

	moving, err := g.IsMoving(context.Background())
	require.NoError(t, err)
	assert.True(t, moving, "fallback must re-read the flag, not a value cached before the round trip")
}

func TestGripperGetPositionReturnsPercent(t *testing.T) {
	fa := newFakeServoArm()
	fa.percent = 42.5
	g := newTestGripper(t, fa)

	res, err := g.DoCommand(context.Background(), map[string]any{"command": "get_position"})
	require.NoError(t, err)
	assert.Equal(t, 42.5, res["position_percentage"])
}

func TestGripperSetPositionClampsAndCommands(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	_, err := g.DoCommand(context.Background(), map[string]any{
		"command": "set_position", "percentage": 150.0,
	})
	require.NoError(t, err)
	assert.Equal(t, 100.0, fa.lastCommand(servocmd.CmdServoMove)["percent"])
}

// Geometries must degrade rather than fail when the arm cannot be reached, so a leased or
// erroring bus does not break the frame system.
func TestGripperGeometriesFallsBackWhenArmErrors(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)
	fa.doErr = errors.New("port is leased")

	geoms, err := g.Geometries(context.Background(), nil)
	require.NoError(t, err)
	assert.NotEmpty(t, geoms)
}

// The full loop: a real servocmd.HandleServoCommand backed by fake single-servo ops, driven by the
// real gripper over DoCommand. This is what proves the protocol actually connects.
type dispatchingArm struct {
	arm.Arm
	name resource.Name
	ops  *testfake.FakeServoOps
}

func (d *dispatchingArm) Name() resource.Name { return d.name }

func (d *dispatchingArm) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return servocmd.HandleServoCommand(ctx, cmd, d.ops, []int{1, 2, 3, 4, 5})
}

func TestGripperDrivesRealDispatcherEndToEnd(t *testing.T) {
	ops := &testfake.FakeServoOps{Percent: 12.5, Raw: 900}
	da := &dispatchingArm{name: arm.Named("real-dispatch"), ops: ops}

	deps := resource.Dependencies{da.name: da}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: da.name.Name},
	}
	res, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err, "capabilities probe must succeed against the real dispatcher")
	g := res.(*so101Gripper)

	require.NoError(t, g.Open(context.Background(), nil))
	assert.Equal(t, 6, ops.MovedID)
	assert.Equal(t, 95.0, ops.MovedPercent)
	assert.Equal(t, []int{6}, ops.WaitedIDs)

	got, err := g.DoCommand(context.Background(), map[string]any{"command": "get_position"})
	require.NoError(t, err)
	assert.Equal(t, 12.5, got["position_percentage"])
}

// Grab infers a grasp from position error. Under the old 500/3500 default an *empty* jaw
// resting at its closed stop normalized to 51.6%, clearing the 15% threshold, so Grab
// returned true whether or not it held anything.
func TestGrabReportsNothingHeldWhenTheJawReachesItsClosedStop(t *testing.T) {
	restingPercent, err := controller.DefaultSO101FullCalibration.Gripper.Normalize(controller.GripperClosedStopTick)
	require.NoError(t, err)

	fake := newFakeServoArm()
	fake.percent = restingPercent
	g := newTestGripper(t, fake)

	grabbed, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)
	assert.False(t, grabbed, "an empty jaw at its closed stop is not holding anything")
}

func TestGrabReportsHeldWhenTheJawStopsShortOfClosed(t *testing.T) {
	fake := newFakeServoArm()
	fake.percent = 50
	g := newTestGripper(t, fake)

	grabbed, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)
	assert.True(t, grabbed, "a jaw held open by an object is holding something")
}

// Grab reads the position back to decide its boolean return, so without the settle wait it
// samples a jaw still in flight. Assert on the emitted command: the grab-classification
// tests cannot catch a missing wait, because fakeServoArm returns a static f.percent that
// CmdServoMove never changes. Open's wait is already covered by the ordered-slice
// assertion in TestGripperOpenCommandsServoAndWaits.
func TestGrabWaitsForTheJawToSettle(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	_, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)

	assert.NotNil(t, fa.lastCommand(servocmd.CmdServoWaitStop),
		"Grab reads the position back, so its wait is load-bearing")
}

// A caller streaming setpoints faster than the jaw travels passes "wait": false, and gets
// the move commanded with no settle wait behind it. The flag rides in the command map
// rather than an extra argument because Gripper.DoCommand has no extra parameter.
func TestGripperWriteCommandsHonorTheWaitFlag(t *testing.T) {
	cases := []struct {
		name     string
		cmd      map[string]any
		wantWait bool
	}{
		{
			name:     "set shorthand with wait false skips the settle wait",
			cmd:      map[string]any{"set": 42.0, "wait": false},
			wantWait: false,
		},
		{
			name:     "set shorthand without the flag still waits",
			cmd:      map[string]any{"set": 42.0},
			wantWait: true,
		},
		{
			name:     "set_position with wait false skips the settle wait",
			cmd:      map[string]any{"command": "set_position", "percentage": 42.0, "wait": false},
			wantWait: false,
		},
		{
			name:     "set_position without the flag still waits",
			cmd:      map[string]any{"command": "set_position", "percentage": 42.0},
			wantWait: true,
		},
		{
			// A JSON string sneaking through must not silently disable the wait on the
			// paths that need it, so a non-bool falls back to waiting.
			name:     "a non-bool wait falls back to waiting",
			cmd:      map[string]any{"set": 42.0, "wait": "false"},
			wantWait: true,
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			g := newTestGripper(t, fa)

			_, err := g.DoCommand(context.Background(), tc.cmd)
			require.NoError(t, err)

			move := fa.lastCommand(servocmd.CmdServoMove)
			require.NotNil(t, move, "the move must be commanded either way")
			assert.Equal(t, 42.0, move["percent"])

			waited := fa.lastCommand(servocmd.CmdServoWaitStop) != nil
			assert.Equal(t, tc.wantWait, waited,
				"issued commands: %v", fa.issued())
		})
	}
}

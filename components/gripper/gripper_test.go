package gripper

import (
	"context"
	"errors"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
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
	onRead func() // invoked (outside f.mu) after a servo_position command, if set
	onMove func() // invoked (outside f.mu) after a servo_move command, if set
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
	f.commands = append(f.commands, cmd)

	if cmd["command"] == servocmd.CmdServoCapabilities {
		resp := map[string]any{"servo_commands": true, "servo_ids": f.capabilities}
		f.mu.Unlock()
		return resp, nil
	}
	if cmd["command"] == servocmd.CmdServoMoving && f.onServoMoving != nil {
		f.onServoMoving()
	}
	if f.doErr != nil {
		err := f.doErr
		f.mu.Unlock()
		return nil, err
	}

	var hook func()
	var resp map[string]any
	switch cmd["command"] {
	case servocmd.CmdServoPosition:
		hook, resp = f.onRead, map[string]any{"percent": f.percent, "raw": f.raw}
	case servocmd.CmdServoMoving:
		if f.movingResponse != nil {
			resp = f.movingResponse
		} else {
			resp = map[string]any{"moving": f.moving}
		}
	case servocmd.CmdServoMove:
		hook, resp = f.onMove, map[string]any{}
	default:
		resp = map[string]any{}
	}
	f.mu.Unlock()

	if hook != nil {
		hook() // outside the lock: hooks may issue commands or call back into the gripper
	}
	return resp, nil
}

// setErr sets the error returned for every subsequent non-capabilities command.
func (f *fakeServoArm) setErr(err error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.doErr = err
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
	return newTestGripperWithConfig(t, fa, SO101GripperConfig{})
}

func newTestGripperWithConfig(t *testing.T, fa *fakeServoArm, cfg SO101GripperConfig) *so101Gripper {
	t.Helper()
	cfg.Arm = fa.name.Name
	deps := resource.Dependencies{fa.name: fa}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &cfg,
	}
	g, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	return g.(*so101Gripper)
}

// newJawGripper builds an articulated-jaw gripper backed by a fake servo arm reporting pct, for
// tests that don't need the fake arm afterward.
func newJawGripper(t *testing.T, pct float64) (*fakeServoArm, *so101Gripper) {
	t.Helper()
	fa := newFakeServoArm()
	fa.percent = pct
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
	return fa, g
}

// countCommands returns how many times name appears among the commands issued so far.
func countCommands(fa *fakeServoArm, name string) int {
	n := 0
	for _, c := range fa.issued() {
		if c == name {
			n++
		}
	}
	return n
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
// CmdServoMove never changes.
func TestGrabWaitsForTheJawToSettle(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	_, err := g.Grab(context.Background(), nil)
	require.NoError(t, err)

	assert.NotNil(t, fa.lastCommand(servocmd.CmdServoWaitStop),
		"Grab reads the position back, so its wait is load-bearing")
}

// A caller streaming setpoints faster than the jaw travels passes "wait": false, and gets
// the move commanded with no settle wait behind it. The raw servo_position form is absent
// here on purpose: it never waited, so there is nothing for the flag to gate.
func TestGripperPercentWritesHonorTheWaitFlag(t *testing.T) {
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
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			g := newTestGripper(t, fa)

			_, err := g.DoCommand(context.Background(), tc.cmd)
			require.NoError(t, err)

			require.NotNil(t, fa.lastCommand(servocmd.CmdServoMove),
				"the move must be commanded either way")

			waited := fa.lastCommand(servocmd.CmdServoWaitStop) != nil
			assert.Equal(t, tc.wantWait, waited,
				"issued commands: %v", fa.issued())
		})
	}
}

// TestIsHoldingSomethingReportsGrasp: Grab already infers a grasp from the jaw stopping short of
// closed. IsHoldingSomething was a stub that always said "not holding"; GoToInputs's safety guard
// depends on it telling the truth.
func TestIsHoldingSomethingReportsGrasp(t *testing.T) {
	ctx := context.Background()
	for _, tc := range []struct {
		name    string
		percent float64
		holding bool
	}{
		{"jaw fully closed", 0, false},
		{"just under the threshold", graspThresholdPct - 1, false},
		{"just over the threshold", graspThresholdPct + 1, true},
		{"jaw wide open on a big part", 60, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			fa.percent = tc.percent
			g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
			st, err := g.IsHoldingSomething(ctx, nil)
			require.NoError(t, err)
			assert.Equal(t, tc.holding, st.IsHoldingSomething)
		})
	}
}

// TestIsHoldingSomethingRespectsCalibratedClosedPosition guards against the threshold expression
// silently degrading to "pct > graspThresholdPct": every other fixture leaves closedPosition at
// its 0.0 zero value, so that mutation would leave the rest of the suite green. With a calibrated
// closed_position of 20%, a jaw reading 30% is only 10 percentage points past closed -- under the
// 15pp threshold -- so it must read as NOT holding.
func TestIsHoldingSomethingRespectsCalibratedClosedPosition(t *testing.T) {
	ctx := context.Background()
	_, g := newJawGripper(t, 30)

	_, err := g.DoCommand(ctx, map[string]interface{}{
		"command": "calibrate_positions", "closed_position": 20.0,
	})
	require.NoError(t, err)

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething,
		"30%% is only 10pp past a calibrated closed_position of 20%%, under the 15pp threshold")
}

// TestConcurrentCurrentInputsPollingWithOpenGrab exercises the interleaving the jaw cache design
// assumes: a viewer polling CurrentInputs continuously while Open/Grab command the same jaw from
// another goroutine. Must pass under `go test -race`.
func TestConcurrentCurrentInputsPollingWithOpenGrab(t *testing.T) {
	ctx := context.Background()
	_, g := newJawGripper(t, 0)

	stop := make(chan struct{})
	done := make(chan struct{})
	go func() {
		defer close(done)
		for {
			select {
			case <-stop:
				return
			default:
				_, _ = g.CurrentInputs(ctx)
			}
		}
	}()

	for i := 0; i < 20; i++ {
		require.NoError(t, g.Open(ctx, nil))
		_, err := g.Grab(ctx, nil)
		require.NoError(t, err)
	}

	close(stop)
	<-done
}

// TestArticulatedJawConfigHardware covers the flag end to end. The ErrUnsupported half matters as
// much as the enabled half: robot/framesystem's CurrentInputs walks every frame with DoF and hard-
// errors if the component is not InputEnabled, so a 1-DoF model with unwired inputs would break
// the WHOLE machine's frame system, arm moves included.
func TestArticulatedJawConfigHardware(t *testing.T) {
	ctx := context.Background()

	t.Run("default is static", func(t *testing.T) {
		fa := newFakeServoArm()
		g := newTestGripper(t, fa)
		assert.Empty(t, fa.issued(),
			"flag off is the shipping default: the constructor must issue only the capabilities "+
				"probe, no jaw-cache prime")

		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		assert.Empty(t, m.DoF(), "articulated_jaw must default to false")

		_, err = g.CurrentInputs(ctx)
		assert.ErrorIs(t, err, errors.ErrUnsupported)
		assert.ErrorIs(t, g.GoToInputs(ctx, []referenceframe.Input{0}), errors.ErrUnsupported)
	})

	t.Run("articulated", func(t *testing.T) {
		g := newTestGripperWithConfig(t, newFakeServoArm(), SO101GripperConfig{ArticulatedJaw: true})
		m, err := g.Kinematics(ctx)
		require.NoError(t, err)
		require.Len(t, m.DoF(), 1)
	})
}

// clearJawCacheForTest drops jawHaveRead, simulating a never-successful start. Test-only seam;
// belongs with the tests, not the production file.
func (g *so101Gripper) clearJawCacheForTest() {
	g.jawMu.Lock()
	defer g.jawMu.Unlock()
	g.jawHaveRead = false
	g.jawReadAt = time.Time{}
}

// TestJawCacheBoundsBusReads pins the reason the cache exists: every read is a DoCommand round
// trip plus a serial read on a bus shared with five arm servos, and the 3D viewer polls
// CurrentInputs continuously.
func TestJawCacheBoundsBusReads(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	// Pin the clock (as TestJawCacheExpires does) so this doesn't depend on 20 calls completing
	// inside a real 50ms wall-clock window.
	base := time.Now()
	g.now = func() time.Time { return base }

	before := countCommands(fa, servocmd.CmdServoPosition)
	for i := 0; i < 20; i++ {
		_, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
	}
	assert.Equalf(t, before, countCommands(fa, servocmd.CmdServoPosition),
		"20 calls inside the TTL should have hit the bus zero extra times")
}

// TestJawCacheExpires uses the injectable clock rather than sleeping.
func TestJawCacheExpires(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	base := time.Now()
	g.now = func() time.Time { return base }
	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	n := countCommands(fa, servocmd.CmdServoPosition)

	g.now = func() time.Time { return base.Add(jawCacheTTL + time.Millisecond) }
	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition), "expired cache must re-read")
}

// TestRawSetPositionInvalidatesCache proves servoDo is the right choke point: this path bypasses
// moveToPercent entirely.
func TestRawSetPositionInvalidatesCache(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	n := countCommands(fa, servocmd.CmdServoPosition)

	_, err = g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position", "servo_position": 2500.0,
	})
	require.NoError(t, err)

	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition),
		"a raw set_position must invalidate the cache")
}

// TestCurrentInputsSurvivesReadFailure: framesystem.CurrentInputs hard-errors the entire machine's
// frame system when a DoF-bearing component errors, so one dropped serial frame must not abort
// every arm move.
func TestCurrentInputsSurvivesReadFailure(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 40
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	good, err := g.CurrentInputs(ctx)
	require.NoError(t, err)

	fa.setErr(errors.New("bus timeout"))
	g.invalidateJawCache()
	after, err := g.CurrentInputs(ctx)
	require.NoError(t, err, "a dropped read must not break the frame system")
	assert.InDelta(t, good[0], after[0], 1e-12, "should serve the last known good reading")
}

// TestCurrentInputsErrorsBeforeFirstRead is the one case where erroring is correct.
func TestCurrentInputsErrorsBeforeFirstRead(t *testing.T) {
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})
	fa.setErr(errors.New("bus down"))
	g.clearJawCacheForTest() // drops jawHaveRead, simulating a never-successful start
	_, err := g.CurrentInputs(context.Background())
	assert.Error(t, err, "with no reading ever taken there is nothing honest to return")
}

func TestHardwareCurrentInputsTracksJaw(t *testing.T) {
	ctx := context.Background()
	for _, pct := range []float64{0, 50, 95} {
		_, g := newJawGripper(t, pct)
		in, err := g.CurrentInputs(ctx)
		require.NoError(t, err)
		require.Len(t, in, 1)
		assert.InDelta(t, geometry.JawRadiansFromPct(pct), in[0], 1e-9)
	}
}

// TestStopDuringReadDoesNotStampStaleValue closes the window opened by releasing jawMu across the
// bus read: an invalidation landing mid-read must not be overwritten by the in-flight pre-write
// value and stamped fresh. The hook drives this through the real Stop path (Stop -> servoDo ->
// invalidateJawCache), not a direct invalidateJawCache call, so it also proves Stop itself
// invalidates. Hooks are invoked outside f.mu specifically so re-entering the gripper here is
// safe.
func TestStopDuringReadDoesNotStampStaleValue(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 10
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	// The constructor primed the cache, so drop it first or the next call is a hit and the hook
	// never runs.
	g.invalidateJawCache()

	fa.onRead = func() { _ = g.Stop(ctx, nil) } // lands inside the unlocked window
	_, err := g.CurrentInputs(ctx)
	require.NoError(t, err)
	fa.onRead = nil

	n := countCommands(fa, servocmd.CmdServoPosition)
	_, err = g.CurrentInputs(ctx)
	require.NoError(t, err)
	assert.Equal(t, n+1, countCommands(fa, servocmd.CmdServoPosition),
		"a reading invalidated mid-flight must not have been stamped fresh")
}

// TestGoToInputsNoOpBatchIssuesNoCommand is the pick-and-place regression guard, and the most
// important test in this file. builtin.go skips a component in a trajectory step only when
// len(inputs) == 0 -- never when unchanged -- so a 1-DoF gripper appears in EVERY step of EVERY
// plan carrying its current jaw value. Rejecting those would fail every arm move made while
// holding a part.
func TestGoToInputsNoOpBatchIssuesNoCommand(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 60 // wide enough that IsHoldingSomething reports true
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	current := geometry.JawRadiansFromPct(60)
	require.NoError(t, g.GoToInputs(ctx,
		[]referenceframe.Input{current},
		[]referenceframe.Input{current + jawHoldToleranceRad/2},
	), "a no-op batch must succeed even while holding")

	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"a no-op batch must not touch the bus; issued %v", fa.issued())
}

// TestGoToInputsRejectsWhileHolding: the planner moves the jaw 4-20%% of its range under
// obstruction, which on hardware means loosening a grip mid-trajectory.
func TestGoToInputsRejectsWhileHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 60
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	err := g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "holding")
	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"a rejected batch must not have moved the jaw")
}

// TestGoToInputsMovesWhenNotHolding is the complement -- the guard must not block ordinary use.
func TestGoToInputsMovesWhenNotHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 0 // closed, nothing held
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax}))
	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoMove))
}

// TestGoToInputsIssuesOnlyFinalTarget: a position-controlled STS3215 tracks the most recent
// command, so intermediate targets are never traversed. Sending them spends bus writes -- on the
// bus the arm is actively using -- for no change in behaviour.
func TestGoToInputsIssuesOnlyFinalTarget(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	steps := [][]referenceframe.Input{}
	for _, pct := range []float64{20, 40, 60, 80} {
		steps = append(steps, []referenceframe.Input{geometry.JawRadiansFromPct(pct)})
	}
	require.NoError(t, g.GoToInputs(ctx, steps...))

	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoMove))
	assert.Equal(t, 1, countCommands(fa, servocmd.CmdServoWaitStop))
	cmd := fa.lastCommand(servocmd.CmdServoMove)
	assert.InDelta(t, 80.0, cmd["percent"], 0.01, "must command the FINAL step's target")
}

// TestGoToInputsValidatesBeforeMoving: a batch that fails halfway leaves the jaw somewhere the
// planner did not intend.
func TestGoToInputsValidatesBeforeMoving(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	err := g.GoToInputs(ctx,
		[]referenceframe.Input{geometry.JawRadiansFromPct(50)},
		[]referenceframe.Input{geometry.GripperJointMax + 0.5}, // invalid
	)
	require.Error(t, err)
	assert.Zerof(t, countCommands(fa, servocmd.CmdServoMove),
		"nothing may move when any step is invalid")
}

// TestGoToInputsToleratesULPOvershoot: rdk rejects an input over a limit by even one ULP, and a
// planner trajectory riding a limit can produce exactly that.
func TestGoToInputsToleratesULPOvershoot(t *testing.T) {
	ctx := context.Background()
	_, g := newJawGripper(t, 0)
	over := math.Nextafter(geometry.GripperJointMax, math.Inf(1))
	assert.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{over}))
}

// TestGoToInputsAbortsOnStop. Note GoToInputs must set isMoving, or Stop never latches and this
// passes vacuously.
func TestGoToInputsAbortsOnStop(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	fa.onMove = func() { _ = g.Stop(context.Background(), nil) } // fires mid-batch
	err := g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	assert.Error(t, err, "a stopped batch must report that it did not complete")
}

// TestOpenDoesNotDeadlock guards the reentrancy hazard: Open/Grab hold g.mu across moveToPercent,
// which invalidates the cache. Invalidating under g.mu would deadlock on the first call.
func TestOpenDoesNotDeadlock(t *testing.T) {
	g := newTestGripperWithConfig(t, newFakeServoArm(), SO101GripperConfig{ArticulatedJaw: true})
	done := make(chan error, 1)
	go func() { done <- g.Open(context.Background(), nil) }()
	select {
	case err := <-done:
		require.NoError(t, err)
	case <-time.After(5 * time.Second):
		t.Fatal("Open deadlocked: cache invalidation must not take g.mu")
	}
}

package gripper

import (
	"context"
	"errors"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
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
	onRead         func()         // invoked (outside f.mu) after a servo_position command, if set
	onMove         func()         // invoked (outside f.mu) after a servo_move command, if set
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

	// The trailing servo_position is the latch's confirming read: moveToPercent must learn whether
	// the jaw reached its target to decide whether something is between the fingers. One extra bus
	// read per move is the accepted cost -- moves are user-initiated, unlike the continuous polling
	// the position cache exists to bound.
	assert.Equal(t,
		[]string{servocmd.CmdServoMove, servocmd.CmdServoWaitStop, servocmd.CmdServoPosition},
		fa.issued(), "Open should command the servo, wait for it to settle, then confirm position")

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

// TestGripperGetLoadReturnsRawSignedLoad drives the real servocmd dispatcher (not the
// fakeServoArm) end to end, same as TestGripperDrivesRealDispatcherEndToEnd, to prove get_load
// actually connects through servo_load rather than just being wired against a mock.
func TestGripperGetLoadReturnsRawSignedLoad(t *testing.T) {
	ops := &testfake.FakeServoOps{Load: -412}
	da := &dispatchingArm{name: arm.Named("real-dispatch"), ops: ops}
	deps := resource.Dependencies{da.name: da}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: da.name.Name},
	}
	res, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	g := res.(*so101Gripper)

	got, err := g.DoCommand(context.Background(), map[string]any{"command": "get_load"})
	require.NoError(t, err)
	assert.Equal(t, -412, got["load"])
}

// TestGripperGetLoadPropagatesError proves a servo_load failure surfaces to the caller rather
// than being swallowed.
func TestGripperGetLoadPropagatesError(t *testing.T) {
	boom := errors.New("bus is on fire")
	ops := &testfake.FakeServoOps{LoadErr: boom}
	da := &dispatchingArm{name: arm.Named("real-dispatch"), ops: ops}
	deps := resource.Dependencies{da.name: da}
	conf := resource.Config{
		Name:                "gripper",
		API:                 gripper.API,
		Model:               SO101GripperModel,
		ConvertedAttributes: &SO101GripperConfig{Arm: da.name.Name},
	}
	res, err := newSO101Gripper(context.Background(), deps, conf, logging.NewTestLogger(t))
	require.NoError(t, err)
	g := res.(*so101Gripper)

	_, err = g.DoCommand(context.Background(), map[string]any{"command": "get_load"})
	require.ErrorIs(t, err, boom)
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
//
// IsHoldingSomething now compares the reading against the jaw's last COMMANDED target, not a
// bare closed-stop constant, so each case establishes that target honestly with a real Grab (to
// closedPosition, the 0.0 default) before setting the fake's reading and asking.
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
			g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

			fa.percent = tc.percent // the fake reports the jaw stopped here despite being told to close
			_, err := g.Grab(ctx, nil)
			require.NoError(t, err)

			st, err := g.IsHoldingSomething(ctx, nil)
			require.NoError(t, err)
			assert.Equal(t, tc.holding, st.IsHoldingSomething)
		})
	}
}

// TestIsHoldingSomethingRespectsCalibratedClosedPosition guards against the threshold expression
// silently degrading to "pct > graspThresholdPct": every other fixture leaves closedPosition at
// its 0.0 zero value, so that mutation would leave the rest of the suite green. Calibrate
// closed_position to 20%% first, so Grab's subsequent close commands the jaw to 20%% (not 0%%) --
// that becomes the commanded target IsHoldingSomething compares against. A jaw reading 30%% is
// then only 10 percentage points past its commanded target -- under the 15pp threshold -- so it
// must read as NOT holding.
func TestIsHoldingSomethingRespectsCalibratedClosedPosition(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	_, err := g.DoCommand(ctx, map[string]interface{}{
		"command": "calibrate_positions", "closed_position": 20.0,
	})
	require.NoError(t, err)

	fa.percent = 30 // the fake reports the jaw stopped here despite being told to close to 20%%
	_, err = g.Grab(ctx, nil)
	require.NoError(t, err)

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething,
		"30%% is only 10pp past a calibrated closed_position of 20%%, under the 15pp threshold")
}

// TestIsHoldingSomethingWithNoCommandedTarget: before anything has ever been commanded there is
// no evidence of obstruction, so IsHoldingSomething must default to false even if the jaw happens
// to read open -- guessing "holding" is the more damaging error (see isHoldingAt).
func TestIsHoldingSomethingWithNoCommandedTarget(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 60            // an arbitrary open reading; irrelevant since nothing has been commanded yet
	g := newTestGripper(t, fa) // ArticulatedJaw false: the constructor never primes a commanded target

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething, "no evidence of obstruction without a prior command")
}

// TestGoToInputsDoesNotLatchOpenAsHolding reproduces the live failure found on hardware: opening
// the jaw to an ordinary angle made IsHoldingSomething claim it was holding something (comparing
// against the closed stop, any open jaw looks held), and GoToInputs's guard then refused every
// subsequent jaw command, including one to close the gripper back up. Fails against the pre-fix
// isHoldingAt(pct, closedPct) formula.
func TestGoToInputsDoesNotLatchOpenAsHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	// Open the jaw to ~50%% via GoToInputs, with the fake faithfully reporting that the servo
	// reached it -- nothing obstructed the move.
	fa.percent = 50
	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.JawRadiansFromPct(50)}))

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething, "an open, empty jaw must not read as holding")

	// A further command to a different angle -- including closing -- must succeed. The bug
	// latched the gripper open: once IsHoldingSomething wrongly said "holding", every subsequent
	// GoToInputs was refused.
	require.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.JawRadiansFromPct(20)}),
		"a further jaw command must succeed once the jaw is merely open, not holding")
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

// TestRawSetPositionClearsCommandedTarget: a raw set_position sends opaque ticks, not a percent,
// so it cannot update lastCommandedPct -- it must instead clear haveCommanded, so a stale
// percent-valued target left over from before is never compared against a reading taken after.
func TestRawSetPositionClearsCommandedTarget(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	// Establish a real commanded target with a reading far enough away from it to count as
	// holding.
	fa.percent = 60
	_, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	require.True(t, st.IsHoldingSomething, "sanity: this setup must look like holding beforehand")

	_, err = g.DoCommand(ctx, map[string]interface{}{
		"command": "set_position", "servo_position": 2500.0,
	})
	require.NoError(t, err)

	// The latch deliberately SURVIVES a raw set_position. A raw tick target is opaque -- it may be
	// an open or a close -- so there is no evidence either way, and clearing the latch on no
	// evidence would silently disable the grasp-retention guard while a part is still held.
	// Erring toward "still holding" costs a refused jaw command; erring the other way drops the
	// part. Open is the documented way to release.
	st, err = g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.True(t, st.IsHoldingSomething,
		"an opaque raw command is not evidence of release; the latch must persist")

	// ...and Open clears it.
	fa.percent = 95
	require.NoError(t, g.Open(ctx, nil))
	st, err = g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething, "Open releases, so the latch must clear")
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
	// The constructor's cache prime adopts 60 as both the reading AND the commanded target, so
	// this alone does not make IsHoldingSomething report holding -- it doesn't matter either way,
	// since the no-op path below returns before the holding guard is ever consulted.
	fa.percent = 60
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
//
// The holding guard now compares against the jaw's last COMMANDED target, not the closed stop, so
// the setup must actually command the jaw closed (via Grab) before the fake reports it stuck open
// -- setting fa.percent alone, with no preceding command, would no longer read as holding.
func TestGoToInputsRejectsWhileHolding(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	// Command the jaw closed, then have the fake report it stuck at 60%% instead of reaching
	// closed -- the jaw failed to reach its commanded target, which is real evidence of an
	// obstruction.
	fa.percent = 60
	_, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	movesSoFar := countCommands(fa, servocmd.CmdServoMove)

	err = g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "holding")
	assert.Equalf(t, movesSoFar, countCommands(fa, servocmd.CmdServoMove),
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

// overloadErr builds the error a Feetech servo returns while straining past its torque limit.
// Two shapes, because both reach the gripper in practice: the typed feetech.ServoError when the
// arm is a local dependency, and a flattened string when the arm is remote and the error has
// crossed the servocmd DoCommand boundary as gRPC status text.
func overloadErrTyped() error {
	return &feetech.ServoError{ID: 6, Op: "read position", Status: feetech.ErrOverload}
}

func overloadErrRemote() error {
	return errors.New("failed to read position for servo 6: servo status error: [overload]")
}

// TestGraspLatchOverloadOnThinObject is the case the whole latch exists for, and the one position
// alone can never see. Measured on hardware: with a part genuinely held the jaw read 0.83% open --
// indistinguishable from an empty closed jaw -- while the servo reported overload. Inferring from
// position said "not holding" and silently disabled the grasp-retention guard.
func TestGraspLatchOverloadOnThinObject(t *testing.T) {
	ctx := context.Background()
	for _, tc := range []struct {
		name string
		err  error
	}{
		{"local arm, typed error", overloadErrTyped()},
		{"remote arm, flattened string", overloadErrRemote()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			fa := newFakeServoArm()
			g := newTestGripper(t, fa)

			// The jaw closes onto a thin part: it reaches ~0.8%, looking fully closed, and the
			// servo overloads clamping it. Fail the confirming read the way the servo does.
			fa.percent = 0.83
			fa.onMove = func() { fa.setErr(tc.err) }

			grabbed, err := g.Grab(ctx, nil)
			require.NoError(t, err)
			assert.True(t, grabbed, "an overloading close is a grasp, even at a closed-looking reading")

			fa.setErr(nil)
			st, err := g.IsHoldingSomething(ctx, nil)
			require.NoError(t, err)
			assert.True(t, st.IsHoldingSomething, "the latch must survive the overload clearing")
			assert.Equal(t, grabbed, st.IsHoldingSomething, "Grab and IsHoldingSomething must agree")
		})
	}
}

// TestGraspLatchClearedByOpen is the live false positive: an open, empty jaw must not read as
// holding. Before the latch, any jaw more than graspThresholdPct from closed reported held, which
// refused every subsequent command including one to close again.
func TestGraspLatchClearedByOpen(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	// Establish a real grasp first.
	fa.percent = 40
	_, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	st, _ := g.IsHoldingSomething(ctx, nil)
	require.True(t, st.IsHoldingSomething, "sanity: a close that stops short is a grasp")

	// Now open. The jaw reaches its target, so nothing is held.
	fa.percent = 95
	require.NoError(t, g.Open(ctx, nil))
	st, err = g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething, "an open that reaches its target releases whatever was held")
}

// TestGraspLatchNotSetByCloseThatReaches: an empty jaw closing all the way holds nothing.
func TestGraspLatchNotSetByCloseThatReaches(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripper(t, fa)

	fa.percent = 0
	grabbed, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	assert.False(t, grabbed, "closing onto nothing is not a grasp")
	st, _ := g.IsHoldingSomething(ctx, nil)
	assert.False(t, st.IsHoldingSomething)
	assert.Equal(t, grabbed, st.IsHoldingSomething, "Grab and IsHoldingSomething must agree")
}

// TestGraspLatchRefusesPlannerJawCommand closes the loop: a latched grasp must make GoToInputs
// refuse, which is the safety property the whole feature exists to provide.
func TestGraspLatchRefusesPlannerJawCommand(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	g := newTestGripperWithConfig(t, fa, SO101GripperConfig{ArticulatedJaw: true})

	// Thin part: closes to a closed-looking reading, overloads while clamping.
	fa.percent = 0.83
	fa.onMove = func() { fa.setErr(overloadErrTyped()) }
	grabbed, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	require.True(t, grabbed)
	fa.setErr(nil)
	fa.onMove = nil

	moves := countCommands(fa, servocmd.CmdServoMove)
	err = g.GoToInputs(ctx, []referenceframe.Input{geometry.GripperJointMax})
	require.Error(t, err, "a latched grasp must refuse a planner jaw command")
	assert.Contains(t, err.Error(), "holding")
	assert.Equal(t, moves, countCommands(fa, servocmd.CmdServoMove),
		"a refused batch must not have moved the jaw")

	// Releasing clears the latch and unblocks the planner again.
	fa.percent = 95
	require.NoError(t, g.Open(ctx, nil))
	assert.NoError(t, g.GoToInputs(ctx, []referenceframe.Input{geometry.JawRadiansFromPct(50)}),
		"once released, planner jaw commands must work again")
}

// TestGraspLatchUpgradesOnLateOverload is the hardware case that motivated refreshHoldingLatch.
// An STS3215 raises overload only after protection_time -- 2s of sustained strain -- so a close
// that lands on an object looks normal at the instant motion stops: the position read succeeds and
// shows the jaw at its target. The evidence arrives seconds later, after the latch has already
// been decided. Measured on a real gripper: Grab returned false, and the servo was overloaded one
// second afterwards and stayed that way.
func TestGraspLatchUpgradesOnLateOverload(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	// The jaw reaches its commanded target, so nothing looks wrong when the move completes.
	fa.percent = 0
	g := newTestGripper(t, fa)

	grabbed, err := g.Grab(ctx, nil)
	require.NoError(t, err)
	require.False(t, grabbed, "at decision time the jaw is at target -- nothing to see yet")

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	require.False(t, st.IsHoldingSomething)

	// Now the servo starts reporting overload, as it does once protection_time elapses.
	fa.setErr(overloadErrTyped())

	st, err = g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err, "a condition must not surface as an error here")
	assert.True(t, st.IsHoldingSomething,
		"an overload appearing after the close must latch the grasp retroactively")
}

// TestGraspLatchIgnoresLateOverloadAfterOpening: straining while OPENING is obstruction, not a
// grasp, and must not latch -- otherwise a jaw jammed on the way open reports holding forever.
func TestGraspLatchIgnoresLateOverloadAfterOpening(t *testing.T) {
	ctx := context.Background()
	fa := newFakeServoArm()
	fa.percent = 95
	g := newTestGripper(t, fa)

	require.NoError(t, g.Open(ctx, nil))
	fa.setErr(overloadErrTyped())

	st, err := g.IsHoldingSomething(ctx, nil)
	require.NoError(t, err)
	assert.False(t, st.IsHoldingSomething,
		"the last command opened the jaw, so straining is obstruction rather than a grasp")
}

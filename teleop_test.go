package so_arm

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/test"
)

type fakeArm struct {
	arm.Arm
	name resource.Name

	mu        sync.Mutex
	jp        []referenceframe.Input
	jpErr     error
	moved     []referenceframe.Input
	moveWait  interface{}
	moveErr   error
	torqueLog []bool
}

func (f *fakeArm) Name() resource.Name { return f.name }

func (f *fakeArm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.jpErr != nil {
		return nil, f.jpErr
	}
	return f.jp, nil
}

func (f *fakeArm) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.moveErr != nil {
		return f.moveErr
	}
	f.moved = positions
	f.moveWait = extra["wait"]
	return nil
}

func (f *fakeArm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if cmd["command"] == "set_torque" {
		if enable, ok := cmd["enable"].(bool); ok {
			f.torqueLog = append(f.torqueLog, enable)
		}
	}
	return map[string]interface{}{}, nil
}

type fakeGripper struct {
	gripper.Gripper
	name resource.Name

	mu     sync.Mutex
	pct    float64
	getErr error
	setPct []float64
	setErr error
}

func (f *fakeGripper) Name() resource.Name { return f.name }

func (f *fakeGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	switch cmd["command"] {
	case "get_position":
		if f.getErr != nil {
			return nil, f.getErr
		}
		return map[string]interface{}{"position_percentage": f.pct}, nil
	case "set_position":
		if f.setErr != nil {
			return nil, f.setErr
		}
		if pct, ok := cmd["percentage"].(float64); ok {
			f.setPct = append(f.setPct, pct)
		}
		return map[string]interface{}{"success": true}, nil
	}
	return nil, fmt.Errorf("unknown command: %v", cmd["command"])
}

func newTestTeleop(t testing.TB, la, fa *fakeArm, lg, fg *fakeGripper) *so101Teleop {
	tp := &so101Teleop{
		logger:            logging.NewTestLogger(t),
		leaderArm:         la,
		followerArm:       fa,
		rateHz:            20,
		maxConsecutiveErr: 10,
	}
	if lg != nil {
		tp.leaderGripper = lg
	}
	if fg != nil {
		tp.followerGripper = fg
	}
	return tp
}

func TestTeleopSyncOnce(t *testing.T) {
	t.Run("forwards joints non-blocking and gripper percentage", func(t *testing.T) {
		la := &fakeArm{name: arm.Named("l"), jp: []referenceframe.Input{0.1, 0.2, 0.3, 0.4, 0.5}}
		fa := &fakeArm{name: arm.Named("f")}
		lg := &fakeGripper{name: gripper.Named("lg"), pct: 42}
		fg := &fakeGripper{name: gripper.Named("fg")}
		tp := newTestTeleop(t, la, fa, lg, fg)

		err := tp.syncOnce(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fa.moved, test.ShouldResemble, la.jp)
		test.That(t, fa.moveWait, test.ShouldEqual, false)
		test.That(t, fg.setPct, test.ShouldResemble, []float64{42})
	})

	t.Run("arm-only when grippers absent", func(t *testing.T) {
		la := &fakeArm{name: arm.Named("l"), jp: []referenceframe.Input{1}}
		fa := &fakeArm{name: arm.Named("f")}
		tp := newTestTeleop(t, la, fa, nil, nil)
		err := tp.syncOnce(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fa.moved, test.ShouldResemble, la.jp)
	})

	t.Run("leader read error propagates and skips follower write", func(t *testing.T) {
		la := &fakeArm{name: arm.Named("l"), jpErr: errors.New("serial timeout")}
		fa := &fakeArm{name: arm.Named("f")}
		tp := newTestTeleop(t, la, fa, nil, nil)
		err := tp.syncOnce(context.Background())
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, fa.moved, test.ShouldBeNil)
	})
}

func TestTeleopConfigValidate(t *testing.T) {
	t.Run("requires leader_arm and follower_arm", func(t *testing.T) {
		_, _, err := (&SO101TeleopConfig{FollowerArm: "f"}).Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
		_, _, err = (&SO101TeleopConfig{LeaderArm: "l"}).Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("rejects negative rate", func(t *testing.T) {
		cfg := &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f", RateHz: -1}
		_, _, err := cfg.Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("rejects rate above max", func(t *testing.T) {
		cfg := &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f", RateHz: 1e12}
		_, _, err := cfg.Validate("p")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("deps are arms plus configured grippers only", func(t *testing.T) {
		cfg := &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f"}
		req, _, err := cfg.Validate("p")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req, test.ShouldResemble, []string{"l", "f"})

		cfg2 := &SO101TeleopConfig{
			LeaderArm: "l", FollowerArm: "f",
			LeaderGripper: "lg", FollowerGripper: "fg",
		}
		req2, _, err := cfg2.Validate("p")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req2, test.ShouldResemble, []string{"l", "f", "lg", "fg"})
	})
}

func TestTeleopStartStopStatus(t *testing.T) {
	la := &fakeArm{name: arm.Named("l"), jp: []referenceframe.Input{0.1}}
	fa := &fakeArm{name: arm.Named("f")}
	tp := newTestTeleop(t, la, fa, nil, nil)
	tp.manageLeaderTorque = true

	resp, err := tp.DoCommand(context.Background(), map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["running"], test.ShouldBeTrue)

	// torque disabled on start
	la.mu.Lock()
	test.That(t, la.torqueLog, test.ShouldResemble, []bool{false})
	la.mu.Unlock()

	st, err := tp.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeTrue)

	resp, err = tp.DoCommand(context.Background(), map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["running"], test.ShouldBeFalse)

	// torque restored on stop (restored in runLoop's defer before wg.Wait returns)
	la.mu.Lock()
	test.That(t, la.torqueLog, test.ShouldResemble, []bool{false, true})
	la.mu.Unlock()
}

func TestTeleopTickErrorThreshold(t *testing.T) {
	la := &fakeArm{name: arm.Named("l"), jpErr: errors.New("boom")}
	fa := &fakeArm{name: arm.Named("f")}
	tp := newTestTeleop(t, la, fa, nil, nil)
	tp.maxConsecutiveErr = 3
	tp.running = true

	// first two failures: keep going, counter climbs
	test.That(t, tp.tick(context.Background()), test.ShouldBeFalse)
	test.That(t, tp.tick(context.Background()), test.ShouldBeFalse)
	test.That(t, tp.consecutiveErrors, test.ShouldEqual, 2)

	// third failure hits threshold -> signal stop
	test.That(t, tp.tick(context.Background()), test.ShouldBeTrue)
	test.That(t, tp.lastError, test.ShouldNotBeBlank)

	// a success resets the counter
	la.mu.Lock()
	la.jpErr = nil
	la.jp = []referenceframe.Input{0.5}
	la.mu.Unlock()
	tp.consecutiveErrors = 1
	test.That(t, tp.tick(context.Background()), test.ShouldBeFalse)
	test.That(t, tp.consecutiveErrors, test.ShouldEqual, 0)
	test.That(t, tp.cycles, test.ShouldEqual, uint64(1))
}

func TestTeleopCloseStopsLoop(t *testing.T) {
	la := &fakeArm{name: arm.Named("l"), jp: []referenceframe.Input{0.1}}
	fa := &fakeArm{name: arm.Named("f")}
	tp := newTestTeleop(t, la, fa, nil, nil)
	tp.manageLeaderTorque = true

	_, err := tp.DoCommand(context.Background(), map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)

	test.That(t, tp.Close(context.Background()), test.ShouldBeNil)

	tp.mu.Lock()
	test.That(t, tp.running, test.ShouldBeFalse)
	tp.mu.Unlock()
	// torque restored exactly once (false on start, true on close)
	la.mu.Lock()
	test.That(t, la.torqueLog, test.ShouldResemble, []bool{false, true})
	la.mu.Unlock()
}

func TestTeleopConstructorWiring(t *testing.T) {
	logger := logging.NewTestLogger(t)
	la := &fakeArm{name: arm.Named("l")}
	fa := &fakeArm{name: arm.Named("f")}
	deps := resource.Dependencies{
		la.name: la,
		fa.name: fa,
	}
	conf := resource.Config{
		Name:                "teleop",
		API:                 generic.API,
		Model:               SO101TeleopModel,
		ConvertedAttributes: &SO101TeleopConfig{LeaderArm: "l", FollowerArm: "f"},
	}

	svc, err := newSO101Teleop(context.Background(), deps, conf, logger)
	test.That(t, err, test.ShouldBeNil)
	tp := svc.(*so101Teleop)
	test.That(t, tp.rateHz, test.ShouldEqual, defaultTeleopRateHz)
	test.That(t, tp.manageLeaderTorque, test.ShouldBeTrue)
	test.That(t, tp.maxConsecutiveErr, test.ShouldEqual, defaultMaxConsecutiveErrors)
	test.That(t, tp.leaderGripper, test.ShouldBeNil)
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
}

package so_arm

import (
	"context"
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
	if extra != nil {
		f.moveWait = extra["wait"]
	}
	return nil
}

func (f *fakeArm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if cmd["command"] == "set_torque" {
		f.torqueLog = append(f.torqueLog, cmd["enable"].(bool))
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
		f.setPct = append(f.setPct, cmd["percentage"].(float64))
		return map[string]interface{}{"success": true}, nil
	}
	return nil, nil
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

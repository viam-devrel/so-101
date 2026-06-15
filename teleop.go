// teleop.go
package so_arm

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

var SO101TeleopModel = resource.NewModel("devrel", "so101", "teleop")

const (
	defaultTeleopRateHz         = 20.0
	defaultMaxConsecutiveErrors = 10
)

func init() {
	resource.RegisterService(
		generic.API,
		SO101TeleopModel,
		resource.Registration[resource.Resource, *SO101TeleopConfig]{
			Constructor: newSO101Teleop,
		})
}

type SO101TeleopConfig struct {
	LeaderArm            string  `json:"leader_arm"`
	FollowerArm          string  `json:"follower_arm"`
	LeaderGripper        string  `json:"leader_gripper,omitempty"`
	FollowerGripper      string  `json:"follower_gripper,omitempty"`
	RateHz               float64 `json:"rate_hz,omitempty"`
	ManageLeaderTorque   *bool   `json:"manage_leader_torque,omitempty"`
	AutoStart            bool    `json:"auto_start,omitempty"`
	MaxConsecutiveErrors int     `json:"max_consecutive_errors,omitempty"`
}

func (cfg *SO101TeleopConfig) Validate(path string) ([]string, []string, error) {
	if cfg.LeaderArm == "" {
		return nil, nil, fmt.Errorf("leader_arm is required")
	}
	if cfg.FollowerArm == "" {
		return nil, nil, fmt.Errorf("follower_arm is required")
	}
	if cfg.RateHz < 0 {
		return nil, nil, fmt.Errorf("rate_hz must not be negative, got %v", cfg.RateHz)
	}
	deps := []string{cfg.LeaderArm, cfg.FollowerArm}
	if cfg.LeaderGripper != "" {
		deps = append(deps, cfg.LeaderGripper)
	}
	if cfg.FollowerGripper != "" {
		deps = append(deps, cfg.FollowerGripper)
	}
	return deps, nil, nil
}

type so101Teleop struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger

	leaderArm       arm.Arm
	followerArm     arm.Arm
	leaderGripper   gripper.Gripper // may be nil
	followerGripper gripper.Gripper // may be nil

	rateHz             float64
	manageLeaderTorque bool
	maxConsecutiveErr  int

	mu                sync.Mutex
	cancel            context.CancelFunc
	wg                sync.WaitGroup
	running           bool
	cycles            uint64
	consecutiveErrors int
	lastError         string
}

func newSO101Teleop(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*SO101TeleopConfig](conf)
	if err != nil {
		return nil, err
	}

	leaderArm, err := arm.FromDependencies(deps, cfg.LeaderArm)
	if err != nil {
		return nil, fmt.Errorf("failed to get leader_arm %q: %w", cfg.LeaderArm, err)
	}
	followerArm, err := arm.FromDependencies(deps, cfg.FollowerArm)
	if err != nil {
		return nil, fmt.Errorf("failed to get follower_arm %q: %w", cfg.FollowerArm, err)
	}

	var leaderGripper, followerGripper gripper.Gripper
	if cfg.LeaderGripper != "" {
		leaderGripper, err = gripper.FromDependencies(deps, cfg.LeaderGripper)
		if err != nil {
			return nil, fmt.Errorf("failed to get leader_gripper %q: %w", cfg.LeaderGripper, err)
		}
	}
	if cfg.FollowerGripper != "" {
		followerGripper, err = gripper.FromDependencies(deps, cfg.FollowerGripper)
		if err != nil {
			return nil, fmt.Errorf("failed to get follower_gripper %q: %w", cfg.FollowerGripper, err)
		}
	}

	rateHz := cfg.RateHz
	if rateHz == 0 {
		rateHz = defaultTeleopRateHz
	}
	maxErr := cfg.MaxConsecutiveErrors
	if maxErr == 0 {
		maxErr = defaultMaxConsecutiveErrors
	}
	manageTorque := true
	if cfg.ManageLeaderTorque != nil {
		manageTorque = *cfg.ManageLeaderTorque
	}

	tp := &so101Teleop{
		Named:              conf.ResourceName().AsNamed(),
		logger:             logger,
		leaderArm:          leaderArm,
		followerArm:        followerArm,
		leaderGripper:      leaderGripper,
		followerGripper:    followerGripper,
		rateHz:             rateHz,
		manageLeaderTorque: manageTorque,
		maxConsecutiveErr:  maxErr,
	}

	if cfg.AutoStart {
		if err := tp.start(ctx); err != nil {
			return nil, fmt.Errorf("auto_start failed: %w", err)
		}
	}

	return tp, nil
}

func (tp *so101Teleop) start(ctx context.Context) error { return nil }
func (tp *so101Teleop) Close(ctx context.Context) error { return nil }

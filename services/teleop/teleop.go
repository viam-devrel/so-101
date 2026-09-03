// Package teleop implements devrel:so101:teleop, which mirrors a leader arm's joint
// positions and gripper onto a follower arm for hand-guided teleoperation.
package teleop

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/utils"
)

var SO101TeleopModel = resource.NewModel("devrel", "so101", "teleop")

const (
	// 100 Hz, from the 2026-09-02 servo characterization: command rate dominates teleop
	// smoothness far more than gain tuning (30->100 Hz cut chatter ~3.5x and pinned lag at
	// 40 ms). Measured at unlimited acceleration, which is what the follower now gets.
	// The loop's ticker drops ticks, so a bus that cannot sustain this degrades to the rate
	// it can rather than backing up.
	defaultTeleopRateHz         = 100.0
	defaultMaxConsecutiveErrors = 10
	maxTeleopRateHz             = 1000.0

	// maxStreamStartGapDeg is how close the follower must get before the mirror switches to
	// streamed setpoints, which carry no speed cap and no ramp. Above the arm's steady-state
	// droop (~0.22 deg tuned) so it is reachable, small enough that the switch happens at a
	// speed a hand is already tracking.
	maxStreamStartGapDeg = 5.0
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
	if cfg.RateHz > maxTeleopRateHz {
		return nil, nil, fmt.Errorf("rate_hz must be <= %v, got %v", maxTeleopRateHz, cfg.RateHz)
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
	// streaming latches true once the follower has caught up with the leader. Until then the
	// mirror runs profiled -- see maxStreamStartGapDeg.
	streaming bool
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

// syncOnce performs a single leader->follower mirror cycle. Any error means the
// cycle did not fully complete; the caller decides how to count it.
func (tp *so101Teleop) syncOnce(ctx context.Context) error {
	positions, err := tp.leaderArm.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("read leader joints: %w", err)
	}

	var gripperPct float64
	haveGripper := tp.leaderGripper != nil && tp.followerGripper != nil
	if haveGripper {
		resp, err := tp.leaderGripper.DoCommand(ctx, map[string]interface{}{"command": "get_position"})
		if err != nil {
			return fmt.Errorf("read leader gripper: %w", err)
		}
		pct, ok := resp["position_percentage"].(float64)
		if !ok {
			return fmt.Errorf("leader gripper get_position missing position_percentage")
		}
		gripperPct = pct
	}

	// No speed cap, no ramp, no coordination read once the follower has caught up.
	streamed, err := tp.readyToStream(ctx, positions)
	if err != nil {
		return err
	}
	if err := tp.followerArm.MoveToJointPositions(ctx, positions,
		map[string]interface{}{"wait": false, "streamed": streamed}); err != nil {
		return fmt.Errorf("write follower joints: %w", err)
	}

	if haveGripper {
		if _, err := tp.followerGripper.DoCommand(ctx, map[string]interface{}{
			"command":    "set_position",
			"percentage": gripperPct,
			"wait":       false,
		}); err != nil {
			return fmt.Errorf("write follower gripper: %w", err)
		}
	}
	return nil
}

// readyToStream reports whether this cycle may use a streamed setpoint, latching true the
// first time the follower is within maxStreamStartGapDeg of the leader on every joint.
//
// It LATCHES because a servo goal write supersedes the previous one: profiling only the first
// cycle would be overwritten 10 ms later while the arm was still far away. Costs one extra
// follower read per cycle until it trips. See CLAUDE.md, "A streamed setpoint".
func (tp *so101Teleop) readyToStream(ctx context.Context, leader []referenceframe.Input) (bool, error) {
	if tp.latched() {
		return true, nil
	}

	follower, err := tp.followerArm.JointPositions(ctx, nil)
	if err != nil {
		return false, fmt.Errorf("read follower joints: %w", err)
	}
	if len(follower) != len(leader) {
		return false, fmt.Errorf("follower reports %d joints, leader %d", len(follower), len(leader))
	}
	for i := range leader {
		if math.Abs(utils.RadToDeg(float64(leader[i])-float64(follower[i]))) > maxStreamStartGapDeg {
			return false, nil
		}
	}

	tp.mu.Lock()
	tp.streaming = true
	tp.mu.Unlock()
	tp.logger.Infof("Follower caught up (within %.1f deg); streaming setpoints", maxStreamStartGapDeg)
	return true, nil
}

func (tp *so101Teleop) latched() bool {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	return tp.streaming
}

func (tp *so101Teleop) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "start":
		if err := tp.start(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"running": true}, nil
	case "stop":
		tp.stop()
		return map[string]interface{}{"running": false}, nil
	case "status":
		tp.mu.Lock()
		defer tp.mu.Unlock()
		return map[string]interface{}{
			"running":            tp.running,
			"cycles":             tp.cycles,
			"consecutive_errors": tp.consecutiveErrors,
			"last_error":         tp.lastError,
			"rate_hz":            tp.rateHz,
			"streaming":          tp.streaming,
		}, nil
	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (tp *so101Teleop) start(ctx context.Context) error {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	if tp.running {
		return nil
	}
	if tp.manageLeaderTorque {
		tp.setLeaderTorque(ctx, false)
	}
	loopCtx, cancel := context.WithCancel(context.Background())
	tp.cancel = cancel
	tp.running = true
	tp.cycles = 0
	tp.consecutiveErrors = 0
	tp.lastError = ""
	tp.streaming = false
	tp.wg.Add(1)
	go tp.runLoop(loopCtx)
	tp.logger.Infof("Teleop started at %.1f Hz", tp.rateHz)
	return nil
}

func (tp *so101Teleop) stop() {
	tp.mu.Lock()
	if !tp.running {
		tp.mu.Unlock()
		return
	}
	cancel := tp.cancel
	tp.mu.Unlock()

	if cancel != nil {
		cancel()
	}
	tp.wg.Wait()

	tp.mu.Lock()
	tp.running = false
	tp.mu.Unlock()
	tp.logger.Info("Teleop stopped")
}

// setLeaderTorque best-effort enables/disables torque on the leader arm.
// SetTorqueEnable acts on the whole 6-servo group and the leader arm + gripper share
// one controller, so this single call also covers the leader gripper servo. We do not
// call set_torque on the gripper (it has no such command and it would be redundant).
func (tp *so101Teleop) setLeaderTorque(ctx context.Context, enable bool) {
	if _, err := tp.leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "set_torque", "enable": enable,
	}); err != nil {
		tp.logger.Warnf("Failed to set leader arm torque=%v: %v", enable, err)
	}
}

func (tp *so101Teleop) runLoop(ctx context.Context) {
	defer tp.wg.Done()
	// Restore leader torque on ANY loop exit (cancel or future self-stop), exactly once.
	// Deferred AFTER wg.Done is registered, so by LIFO it runs BEFORE wg.Done —
	// the restore therefore completes before stop()'s wg.Wait() returns.
	if tp.manageLeaderTorque {
		defer tp.setLeaderTorque(context.Background(), true)
	}

	interval := time.Duration(float64(time.Second) / tp.rateHz)
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if tp.tick(ctx) {
				return // threshold exceeded (implemented in a later task)
			}
		}
	}
}

// tick runs one syncOnce, updates counters, and returns true when the loop
// should stop (consecutive-error threshold exceeded). All shared-state writes
// happen under tp.mu so a concurrent status DoCommand reads a consistent view.
func (tp *so101Teleop) tick(ctx context.Context) bool {
	err := tp.syncOnce(ctx)

	tp.mu.Lock()
	defer tp.mu.Unlock()
	if err != nil {
		tp.consecutiveErrors++
		tp.lastError = err.Error()
		tp.logger.Warnf("Teleop cycle error (%d/%d): %v",
			tp.consecutiveErrors, tp.maxConsecutiveErr, err)
		if tp.consecutiveErrors >= tp.maxConsecutiveErr {
			tp.logger.Errorf("Teleop stopping after %d consecutive errors", tp.consecutiveErrors)
			tp.running = false
			return true
		}
		return false
	}
	tp.consecutiveErrors = 0
	tp.lastError = ""
	tp.cycles++
	return false
}

func (tp *so101Teleop) Close(ctx context.Context) error {
	tp.stop()
	return nil
}

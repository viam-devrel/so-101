// Package gripper implements devrel:so101:gripper, the hardware gripper (servo 6).
// It owns no serial connection: it drives its servo by sending servo_* DoCommands to
// the arm component it depends on, which may be remote.
package gripper

import (
	"context"
	"errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/geometry"
	"so_arm/internal/servocmd"
)

var (
	SO101GripperModel = resource.NewModel("devrel", "so101", "gripper")
)

type SO101GripperConfig struct {
	// Arm names the devrel:so101:arm component that owns this gripper's serial bus.
	// The gripper issues no serial traffic of its own -- it drives its servo through
	// the arm's servo_* DoCommand family -- so the arm is a required dependency.
	Arm string `json:"arm"`

	// Default to 6
	ServoID int `json:"servo_id,omitempty"`

	// GripperType selects which meshes Geometries() serves: "follower" (moving
	// jaw, the default) or "leader" (thumb-loop handle + trigger).
	GripperType string `json:"gripper_type,omitempty"`

	// MeshDetail selects the gripper mesh resolution Geometries() serves:
	// "low" (decimated, the default -- keeps motion planning fast) or "high"
	// (full resolution).
	MeshDetail string `json:"mesh_detail,omitempty"`

	// ArticulatedJaw gives the gripper a 1-DoF revolute jaw joint in its kinematic model and makes
	// it InputEnabled, so the motion planner sees the jaw as a variable it may drive. Off by
	// default: the jaw does not affect the TCP, so the planner gains a degree of freedom that
	// changes only collision geometry. See docs/gripper.md.
	ArticulatedJaw bool `json:"articulated_jaw,omitempty"`
}

// Validate ensures all parts of the config are valid, and reports the arm as a required
// dependency so viam-server constructs it first and hands this module the real arm object.
func (cfg *SO101GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf(
			"must specify arm: the SO-101 gripper shares the arm's serial connection. " +
				"Set \"arm\" to your devrel:so101:arm component name (and remove \"port\")")
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	if cfg.ServoID < 1 || cfg.ServoID > 6 {
		return nil, nil, fmt.Errorf("servo_id must be between 1 and 6, got %d", cfg.ServoID)
	}

	if cfg.GripperType != "" && cfg.GripperType != geometry.LeaderGripper && cfg.GripperType != geometry.FollowerGripper {
		return nil, nil, fmt.Errorf("gripper_type must be %q or %q, got %q",
			geometry.LeaderGripper, geometry.FollowerGripper, cfg.GripperType)
	}

	if cfg.MeshDetail != "" && cfg.MeshDetail != geometry.HighDetail && cfg.MeshDetail != geometry.LowDetail {
		return nil, nil, fmt.Errorf("mesh_detail must be %q or %q, got %q",
			geometry.HighDetail, geometry.LowDetail, cfg.MeshDetail)
	}

	return []string{cfg.Arm}, nil, nil
}

type so101Gripper struct {
	resource.AlwaysRebuild

	name resource.Name
	// arm owns the serial bus. The gripper drives its servo exclusively through this
	// dependency's servo_* DoCommand family and holds no controller, port, or
	// calibration state of its own.
	arm            arm.Arm
	logger         logging.Logger
	gripperType    string
	meshDetail     string
	servoID        int
	model          referenceframe.Model
	articulatedJaw bool

	mu       sync.Mutex
	isMoving atomic.Bool
	// stopped latches when Stop interrupts an in-flight GoToInputs. Must be an atomic, not
	// g.mu-guarded: Stop runs from another goroutine while GoToInputs holds g.mu, and sync.Mutex
	// is not reentrant.
	stopped atomic.Bool

	// Gripper positions in percentage, 0-100%
	openPosition   float64
	closedPosition float64

	// lastCommandedPct is the percent value the servo was last told to move to, via
	// moveToPercent -- the choke point every percent-valued command passes through. It is the
	// evidence isHoldingAt compares an actual reading against: a jaw that failed to reach where
	// it was told to go is obstructed. haveCommanded distinguishes "never commanded" from a
	// legitimate zero-valued command (0% is closed, a real target). Both are guarded by g.mu.
	lastCommandedPct float64
	haveCommanded    bool
	// holdingLatched is whether the gripper is holding something. Latched at command time by
	// updateHoldingLatch rather than inferred live -- see that function for why neither position
	// nor overload can answer the question on demand.
	holdingLatched bool

	speed        float32
	acceleration float32

	// jawMu guards the position cache ONLY. It is deliberately not g.mu: Open/Grab/DoCommand hold
	// g.mu across moveToPercent, and sync.Mutex is not reentrant, so invalidating under g.mu would
	// deadlock. jawMu is never held across a DoCommand -- a viewer poll must not block behind an
	// Open sitting in servo_wait_stop.
	jawMu sync.Mutex
	// jawPct is the last known-good reading. It is meaningful only when jawHaveRead is true --
	// 0 is itself a legal reading, so jawHaveRead (not a zero check here) is what distinguishes
	// "never read" from "read zero".
	jawPct float64
	// jawReadAt is when jawPct was last refreshed from the bus; zero means "no fresh reading".
	// invalidateJawCache zeroes this to force a re-read but deliberately leaves jawHaveRead set,
	// so a dropped re-read can still fall back to the last known good jawPct.
	jawReadAt time.Time
	// jawHaveRead is true once any read has ever succeeded, and survives invalidation (see
	// jawReadAt) so last-known-good keeps serving across it.
	jawHaveRead bool
	jawGen      uint64
	// jawConditionSeen is true when the most recent bus read came back with a servo condition
	// flag (overload while clamping). It is how a grasp that only becomes visible AFTER the move
	// completes still reaches the latch -- see refreshHoldingLatch.
	jawConditionSeen bool

	now func() time.Time // injectable for tests; time.Now in production
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		SO101GripperModel,
		resource.Registration[gripper.Gripper, *SO101GripperConfig]{
			Constructor: newSO101Gripper,
		},
	)
}

func newSO101Gripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*SO101GripperConfig](conf)
	if err != nil {
		return nil, err
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	armDep, err := arm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm %q for gripper: %w", cfg.Arm, err)
	}

	if err := probeServoSupport(ctx, armDep, cfg.Arm, cfg.ServoID); err != nil {
		return nil, err
	}

	gripperType := cfg.GripperType
	if gripperType == "" {
		gripperType = geometry.FollowerGripper
	}

	meshDetail := cfg.MeshDetail
	if meshDetail == "" {
		meshDetail = geometry.LowDetail
	}

	model, err := geometry.BuildGripperModel(gripperType, meshDetail, conf.ResourceName().ShortName(), cfg.ArticulatedJaw)
	if err != nil {
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	g := &so101Gripper{
		name:           conf.ResourceName(),
		logger:         logger,
		arm:            armDep,
		gripperType:    gripperType,
		meshDetail:     meshDetail,
		servoID:        cfg.ServoID,
		model:          model,
		articulatedJaw: cfg.ArticulatedJaw,
		speed:          30,
		acceleration:   50,
		openPosition:   95.0,
		closedPosition: 0.0,
		now:            time.Now,
	}

	if cfg.ArticulatedJaw {
		if pct, err := g.positionPercentCached(ctx); err != nil {
			logger.Warnf("could not prime the jaw position cache: %v", err)
		} else {
			// The gripper is sitting wherever it is at boot with no command in flight, so
			// treat that resting position as its own commanded target: actual == commanded,
			// so IsHoldingSomething/GoToInputs report NOT holding until a real command moves
			// it. A failed prime just leaves haveCommanded false (see moveToPercent).
			g.lastCommandedPct, g.haveCommanded = pct, true
		}
	}

	logger.Debugf("SO-101 gripper initialized with servo ID %d, open=%.1f%%, closed=%.1f%%",
		cfg.ServoID, g.openPosition, g.closedPosition)

	return g, nil
}

func (g *so101Gripper) Name() resource.Name {
	return g.name
}

// probeServoSupport verifies at construction time that the configured arm actually speaks
// the servo_* protocol and offers this gripper's servo. Without it, a gripper pointed at a
// non-SO-101 arm (or a stale arm name) would build cleanly and fail only on the first move.
func probeServoSupport(ctx context.Context, a arm.Arm, armName string, servoID int) error {
	res, err := a.DoCommand(ctx, map[string]interface{}{"command": servocmd.CmdServoCapabilities})
	if err != nil {
		return fmt.Errorf(
			"arm %q does not support the servo_* command family (is it a devrel:so101:arm?): %w",
			armName, err)
	}
	for _, id := range servoIDsFromCapabilities(res) {
		if id == servoID {
			return nil
		}
	}
	return fmt.Errorf("arm %q does not offer servo %d (it reports %v)",
		armName, servoID, servoIDsFromCapabilities(res))
}

// servoIDsFromCapabilities reads the servo_ids list out of a capabilities response,
// tolerating both the local ([]int) and remote ([]interface{} of float64) encodings.
func servoIDsFromCapabilities(res map[string]interface{}) []int {
	switch ids := res["servo_ids"].(type) {
	case []int:
		return ids
	case []interface{}:
		out := make([]int, 0, len(ids))
		for i := range ids {
			if n, ok := servocmd.NumArg(map[string]any{"v": ids[i]}, "v"); ok {
				out = append(out, n)
			}
		}
		return out
	default:
		return nil
	}
}

// servoDo sends one servo_* command for this gripper's servo. Every command except a position
// read, a load read, or the capabilities probe invalidates the jaw cache -- excluding
// CmdServoPosition is not just an optimization: if a read's own servoDo bumped the generation
// counter, the cache would discard the very reading it just took and never populate at all.
// CmdServoLoad is excluded for the same reason it isn't a write: reading load moves nothing.
func (g *so101Gripper) servoDo(
	ctx context.Context, command string, args map[string]interface{},
) (map[string]interface{}, error) {
	if command != servocmd.CmdServoPosition && command != servocmd.CmdServoCapabilities &&
		command != servocmd.CmdServoLoad {
		g.invalidateJawCache()
	}

	cmd := map[string]interface{}{"command": command, "servo_id": g.servoID}
	for k, v := range args {
		cmd[k] = v
	}
	return g.arm.DoCommand(ctx, cmd)
}

// moveToPercent commands the servo; when wait is true it blocks until the servo settles. It is
// the single choke point for every percent-valued command, so it is also where lastCommandedPct
// is recorded -- callers must already hold g.mu (Open, Grab, set_position, the "set" DoCommand
// branch, and GoToInputs all do). The target is recorded once the move command itself succeeds,
// whether or not the caller waits and even if the wait-stop times out: the servo was told to go
// there regardless of whether it settled, or of whether anyone watched it settle.
//
// A non-blocking move leaves the grasp latch alone: arrival is the only evidence updateHoldingLatch
// has, and it has not happened yet. refreshHoldingLatch still picks up evidence from a later read.
func (g *so101Gripper) moveToPercent(ctx context.Context, percent float64, wait bool) error {
	target := servocmd.ClampPercent(percent)
	if _, err := g.servoDo(ctx, servocmd.CmdServoMove,
		map[string]interface{}{"percent": target}); err != nil {
		return err
	}
	g.lastCommandedPct, g.haveCommanded = target, true
	if !wait {
		return nil
	}
	_, err := g.servoDo(ctx, servocmd.CmdServoWaitStop,
		map[string]interface{}{"timeout_ms": gripperSettleTimeoutMs})
	g.updateHoldingLatch(ctx, target, err)

	// An overload while CLOSING is not a failure: the jaw clamped onto something, which is the
	// successful outcome of a close, and updateHoldingLatch has already recorded the grasp. While
	// the flag is up every register read fails, so reporting it as an error would make grabbing a
	// part indistinguishable from a broken bus. This is the uncommon path -- a stock STS3215 does
	// not trip overload under a normal grip -- but it is the one where failing loudly would be
	// most wrong. Opening is different: an overload there means the jaw is obstructed and the move
	// genuinely did not do what was asked, so it stays an error.
	if isOverload(err) && target-g.closedPosition <= graspThresholdPct {
		return nil
	}
	return err
}

// positionPercent reads the servo's current opening as a percentage, always hitting the bus.
// Use this only where the caller must distinguish a failed read from a stale one (Grab,
// get_position); everything else should use positionPercentCached.
func (g *so101Gripper) positionPercent(ctx context.Context) (float64, error) {
	pct, _, err := g.positionPercentCondition(ctx)
	return pct, err
}

// positionPercentCondition reads the opening and reports any servo condition the arm attached to
// the reading. A condition does NOT invalidate the value -- an overloaded servo answers correctly
// while reporting that it is straining, which for a gripper is the evidence that it is holding
// something. The flags travel as a response field rather than as an error precisely so they
// survive the DoCommand boundary to a remote arm, where a typed error would not.
func (g *so101Gripper) positionPercentCondition(ctx context.Context) (float64, string, error) {
	res, err := g.servoDo(ctx, servocmd.CmdServoPosition, nil)
	if err != nil {
		return 0, "", err
	}
	pct, ok := servocmd.FloatArg(res, "percent")
	if !ok {
		return 0, "", fmt.Errorf("no position data available")
	}
	condition, _ := servocmd.ConditionArg(res)
	return pct, condition, nil
}

// gripperSettleTimeoutMs bounds the wait for the gripper servo to stop moving. It replaces
// the fixed 500ms sleep the previous implementation used after every command.
const gripperSettleTimeoutMs = 2000

// jawCacheTTL bounds how stale a jaw reading may be. It expires on wall clock regardless of
// writes, so invalidation only ever shortens staleness -- including a jaw back-driven by hand
// with torque off.
const jawCacheTTL = 50 * time.Millisecond

func (g *so101Gripper) invalidateJawCache() {
	g.jawMu.Lock()
	defer g.jawMu.Unlock()
	g.jawReadAt = time.Time{}
	g.jawGen++
}

// positionPercentCached returns the jaw opening, reading the bus at most once per jawCacheTTL.
func (g *so101Gripper) positionPercentCached(ctx context.Context) (float64, error) {
	g.jawMu.Lock()
	fresh := !g.jawReadAt.IsZero() && g.now().Sub(g.jawReadAt) < jawCacheTTL
	gen, havePrev, prev := g.jawGen, g.jawHaveRead, g.jawPct
	g.jawMu.Unlock()

	if fresh {
		return prev, nil
	}

	pct, condition, err := g.positionPercentCondition(ctx) // bus read, jawMu NOT held
	if err != nil {
		// A read that fails outright can still be a condition on an older arm that has not
		// adopted the condition field; keep believing it.
		if isOverload(err) {
			g.jawMu.Lock()
			g.jawConditionSeen = true
			g.jawMu.Unlock()
		}
		if havePrev {
			g.logger.Warnf("jaw position read failed, serving last known good %.1f%%: %v", prev, err)
			return prev, nil
		}
		return 0, err
	}

	g.jawMu.Lock()
	defer g.jawMu.Unlock()
	// Discard if the jaw was commanded while this read was in flight, or if it is still moving.
	// The moving guard's real purpose is narrower than it looks: it is not the servo_wait_stop
	// timeout case (moveToPercent returning on timeout doesn't stamp anything -- the caller's own
	// defer isMoving.Store(false) fires microseconds later, and a read landing after that is not
	// covered here at all; it can still stamp a mid-flight value, bounded by the TTL to 50ms). What
	// this guard actually covers is the narrow window between a move's last servoDo and the
	// caller's isMoving.Store(false) defer.
	g.jawConditionSeen = condition != ""
	if g.jawGen == gen && !g.isMoving.Load() {
		g.jawPct, g.jawReadAt, g.jawHaveRead = pct, g.now(), true
	}
	return pct, nil
}

func (g *so101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Opening gripper")

	if err := g.moveToPercent(ctx, g.openPosition, true); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	g.logger.Debug("Gripper opened")
	return nil
}

// graspThresholdPct is how far short of closed the jaw must stop for Grab to conclude something
// is between the fingers.
const graspThresholdPct = 15.0

// isHoldingAt is the shared grasp policy behind Grab, IsHoldingSomething, and GoToInputs's
// holding guard: the jaw failed to reach its commanded target by more than graspThresholdPct, so
// something must be between the fingers blocking it.
//
// commandedPct must be the target the jaw was actually just told to move to -- Grab satisfies
// this by construction (it commands closedPosition immediately before reading), while
// IsHoldingSomething and GoToInputs use lastCommandedPct, the last percent moveToPercent
// recorded. Comparing actualPct against the closed stop instead (the pre-fix behavior) is wrong:
// any merely-open jaw then looks "held", because an open jaw is always far from closed regardless
// of whether anything is between the fingers. That bug was found on real hardware: once the
// planner opened the jaw past the threshold, IsHoldingSomething reported holding forever after,
// and GoToInputs refused every subsequent jaw command, including one to close it back up.
func isHoldingAt(actualPct, commandedPct float64) bool {
	return math.Abs(actualPct-commandedPct) > graspThresholdPct
}

// isOverload reports whether err is a servo condition surfacing as an error rather than as data.
//
// This is now a COMPATIBILITY path, not the primary one. Since feetech v0.7.0 a condition no
// longer costs the reading: internal/controller keeps the value and reports the flags, and
// servocmd carries them in the response's condition field (servocmd.ConditionKey), which is what
// positionPercentCondition reads. That path works identically whether the arm is local or remote,
// because a field survives the DoCommand boundary where a typed error does not.
//
// An error can still arrive carrying a condition when the arm this gripper depends on is a REMOTE
// machine running an older build of this module -- one that predates the condition field and still
// turns a status flag into a failure. Matching the rendered text is the only option there, since
// the typed error was flattened to gRPC status text on the way over.
//
// Delete this once no supported arm build can still error on a condition.
func isOverload(err error) bool {
	if err == nil {
		return false
	}
	var se *feetech.ServoError
	if errors.As(err, &se) && se.Status&feetech.ErrOverload != 0 {
		return true
	}
	var st feetech.StatusError
	if errors.As(err, &st) && st&feetech.ErrOverload != 0 {
		return true
	}
	return strings.Contains(err.Error(), "overload")
}

// updateHoldingLatch records whether the gripper is holding something, given the target a move was
// just commanded to and any error that move produced. Callers must hold g.mu.
//
// The latch exists because neither available signal answers "am I holding?" on demand, as measured
// on real hardware:
//
//   - Position is blind to thin objects. With a part genuinely held the jaw read 0.83% open --
//     indistinguishable from an empty closed jaw, so no position threshold can see it.
//   - Overload is unreliable as a grasp signal. Measured against an STS3215's stock protection
//     registers, a hard clamp on a part draws only ~40% torque while overload_torque trips at 80%,
//     so a normal grip never raises the flag at all. When it does fire it is a level, not an edge:
//     it tracks the standing condition and clears when the goal is released, not when the servo
//     settles. So it is corroborating evidence at best, never the primary signal.
//
// So holding is established at command time and held until something releases it.
// refreshHoldingLatch upgrades the latch when evidence of a grasp arrives after the move that
// caused it has already completed.
//
// The STS3215 raises overload only after protection_time -- 2 seconds of sustained strain -- so a
// close that lands on an object looks entirely normal at the instant motion stops: the position
// read succeeds and shows the jaw at its target. The evidence appears seconds later, long after
// updateHoldingLatch has decided. Measured on hardware: Grab returned false, and the servo was
// overloaded one second afterwards and stayed that way.
//
// So a condition observed on any later read, while the last thing commanded was a close, latches
// the grasp retroactively. Callers must hold g.mu.
// latchNeedsFreshRead reports whether a bus read could still change the latch, in either
// direction: a close whose grasp has not shown up yet, or an open whose release could not be
// confirmed because the servo was still straining when the move finished. Takes g.mu itself, so
// callers must not hold it.
func (g *so101Gripper) latchNeedsFreshRead() bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	if !g.haveCommanded {
		return false
	}
	closing := g.lastCommandedPct-g.closedPosition <= graspThresholdPct
	if closing {
		return !g.holdingLatched // a grasp may still appear
	}
	return g.holdingLatched // an open may still turn out to have released
}

func (g *so101Gripper) refreshHoldingLatch() {
	if !g.haveCommanded {
		return
	}
	g.jawMu.Lock()
	seen, pct, havePct := g.jawConditionSeen, g.jawPct, g.jawHaveRead
	g.jawMu.Unlock()

	closing := g.lastCommandedPct-g.closedPosition <= graspThresholdPct

	// A close whose grasp only became visible later.
	if closing && !g.holdingLatched && seen {
		g.logger.Infof("jaw reported a servo condition after closing to %.1f%%: latching a grasp",
			g.lastCommandedPct)
		g.holdingLatched = true
		return
	}

	// An open whose release could not be confirmed at the time. Opening away from a clamped part
	// keeps the servo straining until the grip actually lets go, so updateHoldingLatch's read
	// fails and it cannot clear the latch. Once the strain is gone and the jaw is demonstrably
	// open, the part is released -- otherwise the gripper stays latched with an empty open jaw and
	// refuses every planner command, which is what happened on hardware.
	if !closing && g.holdingLatched && !seen && havePct && pct-g.closedPosition > graspThresholdPct {
		g.logger.Infof("jaw open at %.1f%% with no strain: clearing the grasp latch", pct)
		g.holdingLatched = false
	}
}

func (g *so101Gripper) updateHoldingLatch(ctx context.Context, commandedPct float64, moveErr error) {
	closing := commandedPct-g.closedPosition <= graspThresholdPct

	// Overload during a close is a grasp, and for a thin object it is the ONLY evidence there is.
	// During an open it means something is obstructing the jaw, which is not a grasp.
	if isOverload(moveErr) {
		if closing {
			g.logger.Warnf("jaw overloaded closing to %.1f%%: treating as a grasp, not re-commanding", commandedPct)
			g.holdingLatched = true
		} else {
			g.logger.Warnf("jaw overloaded opening to %.1f%%: obstructed, not re-commanding", commandedPct)
		}
		return
	}
	if moveErr != nil {
		return // some other failure: no new evidence either way, leave the latch alone
	}

	actual, err := g.positionPercent(ctx)
	if err != nil {
		if isOverload(err) && closing {
			g.logger.Warnf("jaw overloaded after closing to %.1f%%: treating as a grasp", commandedPct)
			g.holdingLatched = true
		}
		return
	}

	switch {
	case closing && isHoldingAt(actual, commandedPct):
		// Told to close, stopped short: something is between the fingers.
		g.holdingLatched = true
	case !closing && !isHoldingAt(actual, commandedPct) && actual-g.closedPosition > graspThresholdPct:
		// Opened past the grasp threshold and reached the target: whatever was held is released.
		g.holdingLatched = false
	}
}

func (g *so101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Attempting to grab with gripper")

	if err := g.moveToPercent(ctx, g.closedPosition, true); err != nil {
		return false, fmt.Errorf("failed to close gripper: %w", err)
	}

	// moveToPercent already settled the latch via updateHoldingLatch, using both signals: the jaw
	// stopping short of closed, and an overload while clamping. Reading it back here rather than
	// re-deriving it is what keeps Grab and IsHoldingSomething from disagreeing -- on hardware they
	// once returned true and false for the same physical grasp, one instant apart, because Grab was
	// optimistic about a failed read while IsHoldingSomething served a stale position.
	grabbed := g.holdingLatched
	if grabbed {
		g.logger.Debug("Gripper grabbed an object")
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}
	return grabbed, nil
}

// Stop halts only this gripper's servo. The previous implementation called a controller
// Stop that zeroed velocity on every servo on the shared bus, so stopping the gripper also
// killed any in-flight arm motion.
func (g *so101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	if g.isMoving.Load() {
		g.stopped.Store(true) // only while moving, or "reached the goal" and "was stopped" blur
	}
	g.isMoving.Store(false)
	_, err := g.servoDo(ctx, servocmd.CmdServoStop, nil)
	return err
}

func (g *so101Gripper) IsMoving(ctx context.Context) (bool, error) {
	// Fast path: mid-command is never wrong, and avoids bus traffic during a move.
	if g.isMoving.Load() {
		return true, nil
	}

	res, err := g.servoDo(ctx, servocmd.CmdServoMoving, nil)
	if err != nil {
		// A remote arm may run an older module build with no servo_moving; fall back to
		// the intent flag rather than fail a call teleop may poll repeatedly.
		g.logger.Debugf("servo_moving failed, falling back to intent flag: %v", err)
		return g.isMoving.Load(), nil
	}
	moving, ok := res["moving"].(bool)
	if !ok {
		g.logger.Debug("servo_moving response missing 'moving', falling back to intent flag")
		return g.isMoving.Load(), nil
	}
	return moving, nil
}

// jawAngle maps the gripper's current open percentage onto the URDF gripper-joint
// range. Reads through the cache -- Geometries() (its only caller) is polled continuously
// by the 3D viewer, so a dropped read serves last-known-good rather than snapping the
// rendered jaw shut.
func (g *so101Gripper) jawAngle(ctx context.Context) float64 {
	pct, err := g.positionPercentCached(ctx)
	if err != nil {
		return geometry.GripperJointMin
	}
	return geometry.JawRadiansFromPct(pct)
}

// Geometries serves the gripper as meshes: a static body and a moving part posed by
// the live gripper opening, so the jaw/trigger articulates in the 3D viewer.
func (g *so101Gripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return geometry.BuildGripperMeshes(g.gripperType, g.meshDetail, g.jawAngle(ctx))
}

// Status returns the current status of the resource as a map of key-value pairs.
func (g *so101Gripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

// gripperSetPositionPercent resolves a set_position target percentage from either
// "percentage" (the key documented in the README and used by the simulated gripper
// and the teleop service) or "position_percentage" (the key get_position returns,
// kept so a single-key get/set round-trip — e.g. arm-recorder — still works).
// "percentage" takes precedence when both are supplied.
func gripperSetPositionPercent(cmd map[string]interface{}) (float64, bool) {
	if pct, ok := cmd["percentage"].(float64); ok {
		return pct, true
	}
	if pct, ok := cmd["position_percentage"].(float64); ok {
		return pct, true
	}
	return 0, false
}

func (g *so101Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["get"] == true {
		percentPos, err := g.positionPercent(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": percentPos}, nil
	}

	if percentPos, ok := cmd["set"].(float64); ok {
		percentPos = servocmd.ClampPercent(percentPos)

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		if err := g.moveToPercent(ctx, percentPos, servocmd.WaitArg(cmd)); err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": percentPos}, nil
	}

	switch cmd["command"] {
	case "get_position":
		percentPos, err := g.positionPercent(ctx)
		if err != nil {
			return nil, err
		}

		// calibrate_positions writes these under g.mu; read them the same way.
		g.mu.Lock()
		openPos, closedPos := g.openPosition, g.closedPosition
		g.mu.Unlock()

		return map[string]interface{}{
			// position_radians is now derived from the percentage rather than read as a
			// radian value, so single-key get/set round-trips (arm-recorder) keep working.
			"position_radians":    (percentPos/100.0*2.0 - 1.0) * math.Pi,
			"position_percentage": percentPos,
			"open_position":       openPos,
			"closed_position":     closedPos,
		}, nil

	case "set_position":
		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		// The raw servo_position form goes to the arm as raw ticks; the arm owns the
		// calibration needed to interpret them, so the gripper no longer converts. It bypasses
		// moveToPercent, so it cannot record a percent-valued commanded target -- clear
		// haveCommanded instead of leaving a stale one, so the next percent-valued command
		// re-establishes it rather than isHoldingAt comparing against an opaque target.
		if servoPos, ok := servocmd.FloatArg(cmd, "servo_position"); ok {
			_, err := g.servoDo(ctx, servocmd.CmdServoMove, map[string]interface{}{"raw": int(servoPos)})
			g.haveCommanded = false
			return map[string]interface{}{"success": err == nil}, err
		}

		targetPercent, ok := gripperSetPositionPercent(cmd)
		if !ok {
			return nil, fmt.Errorf("set_position command requires 'percentage', 'position_percentage', or 'servo_position' parameter")
		}

		err := g.moveToPercent(ctx, targetPercent, servocmd.WaitArg(cmd))
		return map[string]interface{}{"success": err == nil}, err

	case "get_load":
		// Exists purely to make the raw signed load register sample-able (e.g. from the CLI).
		// Not used to infer whether the gripper is holding something.
		res, err := g.servoDo(ctx, servocmd.CmdServoLoad, nil)
		if err != nil {
			return nil, err
		}
		load, _ := servocmd.NumArg(res, "load")
		return map[string]interface{}{"load": load}, nil

	case "controller_status":
		// The arm owns the controller now, so forward and re-shape to this component's
		// documented keys.
		res, err := g.arm.DoCommand(ctx, map[string]interface{}{"command": "controller_status"})
		if err != nil {
			return nil, err
		}
		out := map[string]interface{}{"servo_id": g.servoID}
		for _, k := range []string{"ref_count", "has_controller", "config"} {
			if v, ok := res[k]; ok {
				out[k] = v
			}
		}
		return out, nil

	case "calibrate_positions":
		g.mu.Lock()
		if openPos, ok := cmd["open_position"].(float64); ok {
			if openPos >= 0 && openPos <= 100 {
				g.openPosition = openPos
			}
		}
		if closedPos, ok := cmd["closed_position"].(float64); ok {
			if closedPos >= 0 && closedPos <= 100 {
				g.closedPosition = closedPos
			}
		}
		openPos, closedPos := g.openPosition, g.closedPosition
		g.mu.Unlock()

		g.logger.Debugf("Gripper positions calibrated: open=%.1f%%, closed=%.1f%%", openPos, closedPos)

		return map[string]interface{}{
			"success":         true,
			"open_position":   openPos,
			"closed_position": closedPos,
		}, nil

	case "set_motion_params":
		if speed, ok := cmd["speed"].(float64); ok {
			if speed > 0 && speed <= 180 {
				g.speed = float32(speed)
			}
		}
		if acc, ok := cmd["acceleration"].(float64); ok {
			if acc > 0 && acc <= 500 {
				g.acceleration = float32(acc)
			}
		}

		return map[string]interface{}{
			"success":      true,
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	case "get_motion_params":
		return map[string]interface{}{
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

// Close releases nothing: the gripper owns no serial connection. The arm it depends on
// owns the bus and is torn down on its own schedule.
func (g *so101Gripper) Close(ctx context.Context) error {
	return nil
}

// CurrentInputs reports the jaw joint angle for the articulated (1-DoF) model. Unsupported for
// the default static model -- see BuildGripperModel's jawDoF branch. Reads through the position
// cache (positionPercentCached), so a dropped bus read serves the last known good value rather
// than erroring the whole frame system.
func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	if !g.articulatedJaw {
		return nil, errors.ErrUnsupported
	}
	pct, err := g.positionPercentCached(ctx)
	if err != nil {
		return nil, err
	}
	return []referenceframe.Input{geometry.JawRadiansFromPct(pct)}, nil
}

// jawHoldToleranceRad is how far a commanded jaw angle must differ from the current one to count
// as actually moving the jaw. 0.01 rad is ~0.5% of the 1.92 rad range and about 6 servo ticks --
// well above quantization, well below the 0.083-0.383 rad the planner was measured moving under
// obstruction.
//
// Numerically identical to rdk's defaultExecuteEpsilon and COMPLETELY unrelated to it: that one
// bounds how far a plan's first step may sit from CurrentInputs, on a path Move() does not take.
const jawHoldToleranceRad = 0.01

func (g *so101Gripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	if !g.articulatedJaw {
		return errors.ErrUnsupported
	}
	// Validate EVERY step before moving any of them.
	if err := geometry.ValidateJawSteps(g.model, inputSteps); err != nil {
		return err
	}

	// g.mu is held from here through the move: the no-op classification and the holding guard
	// below must be decided against a fresh read taken at execute time, not one taken before the
	// lock -- otherwise a concurrent Open (which can hold g.mu for up to 2000ms inside
	// servo_wait_stop) can change the jaw between the decision and the move that acts on it.
	//
	// Accepted limitation: a Stop arriving during this read/decide phase is not latched, because
	// isMoving is still false at that point, so the jaw may move after that Stop lands. Setting
	// isMoving earlier would suppress legitimate cache stores during the read (see
	// positionPercentCached's moving guard), so this is accepted rather than fixed.
	g.mu.Lock()
	defer g.mu.Unlock()

	current, err := g.positionPercentCached(ctx)
	if err != nil {
		return err
	}
	currentRad := geometry.JawRadiansFromPct(current)

	// A no-op batch is the common case: the gripper appears in every trajectory step of every
	// plan carrying its unchanged value. It must neither reject nor touch the bus.
	moves := false
	for _, step := range inputSteps {
		if math.Abs(step[0]-currentRad) > jawHoldToleranceRad {
			moves = true
			break
		}
	}
	if !moves {
		return nil
	}

	// The latch, not a live read (see updateHoldingLatch). Not g.IsHoldingSomething either: that
	// takes g.mu itself and g.mu is already held here, so calling it would self-deadlock.
	final := inputSteps[len(inputSteps)-1][0]
	g.refreshHoldingLatch()
	if g.holdingLatched {
		g.logger.Infof("refusing planner jaw command while holding: current %.4f rad, requested %.4f rad",
			currentRad, final)
		return fmt.Errorf(
			"gripper is holding something; refusing the planner's request to move the jaw from "+
				"%.4f to %.4f rad -- add the held object to the motion request's WorldState so the "+
				"planner stops trying to move the jaw", currentRad, final)
	}
	g.logger.Infof("jaw GoToInputs: %d step(s), %.4f -> %.4f rad", len(inputSteps), currentRad, final)

	g.stopped.Store(false) // clear BEFORE isMoving, or a Stop landing between the two is wiped
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	// Only the final target: a position servo never traverses superseded waypoints.
	if err := g.moveToPercent(ctx, geometry.JawPctFromRadians(final), true); err != nil {
		return err
	}
	if g.stopped.Load() {
		return errors.New("jaw motion was stopped before completing the trajectory")
	}
	return nil
}

// Kinematics returns the gripper's static model, whose leaf frame is the TCP between the jaw
// tips. See geometry.BuildGripperModel for why the gripper needs one at all.
func (g *so101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

// IsHoldingSomething infers a grasp by comparing the jaw's actual position against the last
// target it was commanded to (see isHoldingAt): it failed to reach where it was told to go, so
// something is between the fingers. Reads through the cache. This is the standalone public-API
// path only -- GoToInputs no longer calls it (see the comment there), so taking g.mu here to read
// lastCommandedPct safely cannot self-deadlock against GoToInputs's own g.mu hold.
//
// If nothing has ever been commanded, there is no evidence to compare against -- report NOT
// holding rather than guessing. A false "holding" is the more damaging error: it was observed on
// hardware latching the gripper open, refusing every subsequent jaw command including closing.
func (g *so101Gripper) IsHoldingSomething(
	ctx context.Context, extra map[string]interface{},
) (gripper.HoldingStatus, error) {
	// Reads the latch rather than the servo: holding is established when a command completes and
	// stays true until something opens the jaw. Querying position here produced both live failures
	// -- an open empty jaw reporting held, and a clamped part reporting free while the servo was
	// overloaded and every read was failing.
	// When a grasp could still be pending -- not yet latched, and the last thing commanded was a
	// close -- force a fresh read rather than trusting the cache. The whole point is to notice a
	// condition that appeared AFTER the move completed, and a cached value from before it would
	// hide exactly that. Otherwise the cached path is fine and this costs nothing.
	if g.latchNeedsFreshRead() {
		g.invalidateJawCache()
	}
	_, _ = g.positionPercentCached(ctx)

	g.mu.Lock()
	defer g.mu.Unlock()
	g.refreshHoldingLatch()
	return gripper.HoldingStatus{IsHoldingSomething: g.holdingLatched}, nil
}

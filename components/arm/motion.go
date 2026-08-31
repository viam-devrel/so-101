package arm

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
	"so_arm/internal/planning"
	"so_arm/internal/servo"
	"so_arm/internal/servocmd"
)

// currentJoints reads live joint positions in radians. It exists as a seam: the waypoint
// dwell's whole contract is about WHEN it reads relative to the goals it writes, and a
// concrete *controller.ControllerHandle cannot express that without serial hardware. Tests
// substitute readJoints; production leaves it nil and goes to the bus.
func (s *so101) currentJoints(ctx context.Context) ([]float64, error) {
	if s.readJoints != nil {
		return s.readJoints(ctx)
	}
	return s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
}

// calculateJointLimits dynamically calculates joint limits from calibration data
func (s *so101) calculateJointLimits() [][2]float64 {
	limits := make([][2]float64, len(s.armServoIDs))

	calibration := s.controller.GetCalibration()

	// Map servo IDs to calibration data
	jointCals := []*servo.MotorCalibration{
		calibration.ShoulderPan,
		calibration.ShoulderLift,
		calibration.ElbowFlex,
		calibration.WristFlex,
		calibration.WristRoll,
	}

	for i, cal := range jointCals {
		if cal == nil {
			// Use default limits if calibration is missing
			limits[i] = [2]float64{-math.Pi, math.Pi}
			continue
		}

		// Convert calibration range to radians using the same logic as before
		center := float64(cal.RangeMin+cal.RangeMax) / 2
		halfRange := float64(cal.RangeMax-cal.RangeMin) / 2

		// Calculate min limit (RangeMin -> radians)
		minNormalized := (float64(cal.RangeMin) - center) / halfRange
		minRadians := minNormalized * math.Pi

		// Calculate max limit (RangeMax -> radians)
		maxNormalized := (float64(cal.RangeMax) - center) / halfRange
		maxRadians := maxNormalized * math.Pi

		limits[i] = [2]float64{minRadians, maxRadians}
	}

	return limits
}

func (s *so101) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}

	pose, err := geometry.ComputeOOBPosition(s.model, inputs)
	if err != nil {
		return nil, fmt.Errorf("failed to compute end position: %w", err)
	}

	return pose, nil
}

// MoveToPosition moves the arm's end-effector to the target pose.
//
// The SO-101 is a 5-DOF arm, so most six-DOF pose targets are unreachable. Rather than discard
// orientation wholesale, the planner gets an approach-axis cone (a referenceframe.PoseCloud)
// around the goal: the tool's pointing direction is held within orientation_tolerance_deg while
// roll about that axis stays free.
//
// Callers may bypass the cone via extra -- "goal_metric_type" restores the old
// orientation-agnostic behavior, "pose_cloud" supplies a raw cloud. See internal/planning.
//
// Requires viam-server >= 0.127.0; older servers silently ignore goal clouds, which makes
// planning revert to strict six-DOF scoring and fail.
func (s *so101) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	// A motion command takes the arm out of hand-guided ("manual") mode.
	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	dest, planExtra, path, err := planning.BuildMoveDestination(
		fmt.Sprintf("%v_origin", s.Name().Name), pose, s.goalCloud, extra)
	if err != nil {
		// Return the build error UNWRAPPED: path is the zero GoalPath, and wrapping a pose_cloud
		// parse error as a cone-planning failure would tell the caller to widen tolerances
		// they never set.
		return err
	}
	s.logger.Debugf("MoveToPosition goal cloud: %+v", dest.GoalCloud)

	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().Name,
		Destination:   dest,
		Extra:         planExtra,
	})
	return planning.WrapMoveErr(err, path, s.goalCloud)
}

// clampPositions clamps joint inputs (already in radians) to each joint's calibrated
// limits, warning on out-of-range values.
func (s *so101) clampPositions(positions []referenceframe.Input) ([]float64, error) {
	if len(positions) != len(s.armServoIDs) {
		return nil, fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(positions))
	}

	values := make([]float64, len(positions))
	copy(values, positions) // referenceframe.Input is an alias for float64

	jointLimits := s.calculateJointLimits()

	clamped := make([]float64, len(values))
	for i, pos := range values {
		min, max := jointLimits[i][0], jointLimits[i][1]
		if pos < min || pos > max {
			s.logger.Warnf("Joint %d position %.3f rad (%.1f°) out of range [%.3f, %.3f] rad ([%.1f°, %.1f°]), clamping",
				s.armServoIDs[i], pos, pos*180/math.Pi, min, max, min*180/math.Pi, max*180/math.Pi)
		}
		clamped[i] = math.Max(min, math.Min(max, pos))
	}
	return clamped, nil
}

// moveJoints commands a coordinated move from `from` to `to` (both radians, per arm servo),
// scaling every joint's speed and acceleration by its share of the longest travel so all
// joints arrive together. When wait is true it blocks until the arm stops.
//
// It returns this segment's dwell timeout: the same ramp-model duration the completion wait
// times against, floored low enough that a long waypoint stream cannot stall for a second
// per waypoint. A caller streaming waypoints hands it to dwellUntilNear; a caller issuing a
// single move ignores it.
//
// The caller must hold s.moveLock and manage s.isMoving.
func (s *so101) moveJoints(
	ctx context.Context,
	from, to []float64,
	speedDegsPerSec, accelDegsPerSecSq float64,
	caps []servo.JointLimits,
	wait bool,
) (int, error) {
	if len(from) != len(to) {
		return 0, fmt.Errorf("coordinated move needs matching position counts, got %d current and %d target",
			len(from), len(to))
	}
	travelsDeg, maxTravelDeg := servo.JointTravelsDeg(from, to)

	profiles := servo.CoordinatedProfiles(travelsDeg, speedDegsPerSec, accelDegsPerSecSq, caps)
	if profiles == nil {
		// Every joint is already at its target. Note this is a behavior change: the old
		// path still issued a write, so a caller re-asserting its current pose used to
		// reach the bus and now does not.
		//
		// Nothing was commanded, so a dwell on this waypoint is satisfied by the position
		// the arm is already in; the floor is the right timeout to hand back.
		s.logger.Debug("moveJoints: all joints already at target, nothing to command")
		return servo.DwellTimeoutMs(0, speedDegsPerSec, accelDegsPerSecSq), nil
	}

	servoProfiles := make([]controller.ServoProfile, len(profiles))
	for i, p := range profiles {
		servoProfiles[i] = controller.ServoProfile{SpeedSteps: p.SpeedSteps, AccUnits: p.AccUnits}
	}

	// Count moving joints pinned at the acceleration floor. Acc 1 delivers ~43 deg/s^2, so
	// a short-travel joint can need less than the register can express and will arrive
	// early. This is the diagnostic for that documented limitation.
	floored := 0
	for i, d := range travelsDeg {
		if d > 0 && profiles[i].AccUnits == servo.MinAccUnits {
			floored++
		}
	}

	// Time against the speed ACTUALLY COMMANDED, not the reference the caller requested.
	// A MoveOptions cap can reduce the shared reference well below the request -- the cap
	// tests in motion_profile_test.go reduce 50 deg/s to 20, a 2.5x slowdown against
	// moveTimeoutFactor = 2.0 -- so timing against the request would abort a move that is
	// proceeding perfectly correctly.
	effectiveSpeed := speedDegsPerSec
	maxSteps := 0
	for _, p := range profiles {
		if p.SpeedSteps > maxSteps {
			maxSteps = p.SpeedSteps
		}
	}
	if maxSteps > 0 {
		effectiveSpeed = float64(maxSteps) / servo.StepsPerDegree
	}

	// Use the acceleration actually achievable: the Acc register floors at ~43 deg/s^2, so
	// a cap-reduced reference below that is silently raised by the hardware and timing
	// against the lower value would over-estimate the duration.
	effectiveAccel := math.Max(accelDegsPerSecSq, servo.AccFloorDegsPerSecSq)
	dwellTimeout := servo.DwellTimeoutMs(maxTravelDeg, effectiveSpeed, effectiveAccel)

	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, servoProfiles); err != nil {
		return 0, fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	if !wait {
		return dwellTimeout, nil
	}

	timeout := servo.MoveTimeoutMs(maxTravelDeg, effectiveSpeed, effectiveAccel)
	s.logger.Debugf("moveJoints: ref %.1f deg/s (effective %.1f), %.0f deg/s^2, max travel "+
		"%.1f deg, %d joint(s) at the acceleration floor, wait timeout %d ms",
		speedDegsPerSec, effectiveSpeed, accelDegsPerSecSq, maxTravelDeg, floored, timeout)
	return dwellTimeout, s.controller.WaitForServosToStop(ctx, s.armServoIDs, timeout)
}

// dwellPollInterval is how often the waypoint dwell re-reads joint positions. One read is a
// single SyncRead (~1ms on feetech-servo v0.6.1), and the interval only costs anything when
// the arm has NOT yet reached the lookahead -- the first read of each dwell is immediate.
// WaitForServosToStop's 50ms would put a hard 50ms floor on every waypoint it has to wait
// for, which on a path densified 10x from a 10 Hz recording is minutes of pure polling.
const dwellPollInterval = 10 * time.Millisecond

// dwellUntilNear holds an intermediate waypoint until every joint is within lookaheadDeg of it,
// then returns the last position read so the caller can use it as the next segment's travel
// reference.
//
// Deliberately NOT WaitForServosToStop: a servo keeps its velocity across a goal rewrite, so
// returning while the arm is still moving -- merely close enough -- is what makes a waypoint
// stream one continuous motion rather than a full stop at every waypoint. The lookahead is
// the leash: the commanded goal never runs more than lookaheadDeg ahead of the arm, which
// both bounds how much corner the stream can cut and sets the pace (see
// servo.DefaultLookaheadDeg).
//
// Best-effort on the timeout, like WaitForServosToStop: it logs and returns the last read
// rather than failing a move that is merely lagging. A read FAILURE is fatal, though --
// without position feedback the dwell degenerates into the flythrough it exists to replace,
// and every remaining segment would be scaled against a stale travel reference.
func (s *so101) dwellUntilNear(ctx context.Context, goal []float64, lookaheadDeg float64, timeoutMs int) ([]float64, error) {
	deadline := time.Now().Add(time.Duration(timeoutMs) * time.Millisecond)
	ticker := time.NewTicker(dwellPollInterval)
	defer ticker.Stop()

	for {
		current, err := s.currentJoints(ctx)
		if err != nil {
			return nil, fmt.Errorf("failed to read positions while holding a waypoint: %w", err)
		}
		if len(current) != len(goal) {
			return nil, fmt.Errorf("waypoint dwell read %d joint positions, want %d", len(current), len(goal))
		}

		_, remainingDeg := servo.JointTravelsDeg(current, goal)
		if remainingDeg <= lookaheadDeg {
			return current, nil
		}
		if time.Now().After(deadline) {
			// Debug, not Warn: on a dense path a lagging waypoint is routine, and one log
			// line per waypoint would bury everything else.
			s.logger.Debugf("waypoint dwell: still %.2f deg away after %dms (lookahead "+
				"%.2f deg), moving to the next waypoint", remainingDeg, timeoutMs, lookaheadDeg)
			return current, nil
		}

		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-ticker.C:
		}
	}
}

// moveJointsUniform is the pre-coordination path, retained only as the fallback for when
// the travel-reference read fails. It commands every joint at one shared speed, which is
// why joints arrive at different times.
func (s *so101) moveJointsUniform(ctx context.Context, to []float64, speedDegsPerSec, accelDegsPerSecSq float64, wait bool) error {
	// Writes an explicit UNIFORM profile rather than a bare position write. A coordinated
	// move leaves each servo's Speed and Acc registers at its own scaled value -- as low as
	// 1 step/s and Acc 1 -- and a position-only write does not touch either. So without
	// this the "uniform" fallback would inherit whatever the previous coordinated move left
	// behind, and would be uniform in neither speed nor acceleration.
	profile := controller.ServoProfile{
		SpeedSteps: servo.DegPerSecToStepsPerSec(speedDegsPerSec),
		AccUnits:   servo.DegPerSecSqToAccUnits(accelDegsPerSecSq),
	}
	profiles := make([]controller.ServoProfile, len(s.armServoIDs))
	for i := range profiles {
		profiles[i] = profile
	}
	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, profiles); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}
	if !wait {
		return nil
	}
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, servo.MaxMoveTimeoutMs)
}

func (s *so101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()

	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	clamped, err := s.clampPositions(positions)
	if err != nil {
		return err
	}

	s.mu.RLock()
	speed := float64(s.defaultSpeed)
	accel := float64(s.defaultAcc)
	s.mu.RUnlock()

	// The position read is now unconditional, where it used to happen only when waiting.
	// Coordination needs a travel reference, and on v0.6.1 this read costs about 1ms
	// against roughly 12ms on v0.6.0 -- which is why the version bump is a correctness
	// dependency, not just a performance one.
	//
	// A read failure degrades to the old uniform-speed path rather than failing the move.
	// Before this change the read happened only when waiting and a failure was a warning;
	// making it fatal would mean a transient bus error aborts a teleop setpoint that used
	// to go through, at 30-50 Hz on the wait:false path.
	current, err := s.currentJoints(ctx)
	if err != nil {
		s.logger.Warnf("failed to read positions for coordinated move, falling back to "+
			"uniform speed: %v", err)
		// No arm.MoveOptions here, so caps is always nil -- kept for symmetry with
		// MoveThroughJointPositions' fallback.
		return s.moveJointsUniform(ctx, clamped, servo.UniformSpeedUnderCaps(speed, nil), accel, servocmd.WaitArg(extra))
	}

	// Single move: there is no next waypoint, so the dwell timeout is not wanted.
	_, err = s.moveJoints(ctx, current, clamped, speed, accel, nil, servocmd.WaitArg(extra))
	return err
}

func (s *so101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	// Registers the stream so Stop can cancel it. Not done in MoveToPosition: it calls back
	// into here through the motion service, and the op marker cannot cross gRPC.
	ctx, done := s.opMgr.New(ctx)
	defer done()

	s.mu.Lock()
	s.exitManualLocked("motion command received")
	s.mu.Unlock()

	s.moveLock.Lock()
	defer s.moveLock.Unlock()

	s.isMoving.Store(true)
	defer s.isMoving.Store(false)

	s.mu.RLock()
	defaultSpeed := float64(s.defaultSpeed)
	defaultAccel := float64(s.defaultAcc)
	s.mu.RUnlock()
	accel := defaultAccel

	caps, err := servo.JointLimitsFromMoveOptions(options, len(s.armServoIDs))
	if err != nil {
		return err
	}

	// MaxVelRads also becomes the reference speed, not just a per-joint cap. That is
	// deliberate double application: as the reference it sets the pace, and as a cap it
	// bypasses servo.ResolveSpeedDegsPerSec's 3 deg/s floor for very small values. Per arm.proto
	// the scalar is ignored entirely when the per-joint slice is set.
	speed := defaultSpeed
	if options != nil && len(options.MaxVelRadsJoints) == 0 && options.MaxVelRads > 0 {
		speed = servo.ResolveSpeedDegsPerSec(options.MaxVelRads, defaultSpeed)
	}

	// One read before the stream starts; from here on each waypoint's dwell supplies the
	// next segment's travel reference, so this is the only read a single-waypoint call
	// makes. GoToInputs routinely emits single-waypoint streams, which would otherwise have
	// no travel reference at all.
	// Fatal, unlike the single-move path: with no feedback there is no dwell, and an unpaced
	// stream executes only its final waypoint.
	from, err := s.currentJoints(ctx)
	if err != nil {
		return fmt.Errorf("failed to read positions to pace a waypoint stream: %w", err)
	}

	for idx, jointPositions := range positions {
		clamped, err := s.clampPositions(jointPositions)
		if err != nil {
			return err
		}
		isLast := idx == len(positions)-1

		dwellTimeout, err := s.moveJoints(ctx, from, clamped, speed, accel, caps, isLast)
		if err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
		if isLast {
			// moveJoints already waited for the arm to stop.
			break
		}

		// Hold this waypoint until the arm has actually reached it (within the lookahead)
		// before writing the next. Without this the whole stream lands on the bus in a few
		// tens of milliseconds, each SetGoals superseding the last, and the arm executes
		// only the final waypoint -- a recorded trajectory replays as a straight line from
		// its first pose to its last, and a planned path skips the obstacle avoidance it
		// was planned for.
		//
		// The dwell's own read is the next segment's travel reference: it is where the arm
		// ACTUALLY is, which matters because a joint whose travel is concentrated earlier
		// in the path would otherwise show a near-zero delta and be floored to 1 step/s
		// while its goal is still far away.
		from, err = s.dwellUntilNear(ctx, clamped, s.lookaheadDeg, dwellTimeout)
		if err != nil {
			return err
		}
	}
	return nil
}

func (s *so101) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	radians, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("Failed to read joint positions: %v", err)
		return nil, fmt.Errorf("failed to read joint positions: %w. Try running 'diagnose' command for more details", err)
	}

	if len(radians) != len(s.armServoIDs) {
		return nil, fmt.Errorf("expected %d joint positions for SO-101 arm, got %d", len(s.armServoIDs), len(radians))
	}

	positions := make([]referenceframe.Input, len(radians))
	copy(positions, radians)

	return positions, nil
}

func (s *so101) Stop(ctx context.Context, extra map[string]interface{}) error {
	// Cancel before zeroing velocity, and block until the move returns: a running waypoint
	// stream would otherwise write its next goal after the zero and the arm would resume.
	s.opMgr.CancelRunning(ctx)

	s.mu.Lock()
	s.exitManualLocked("stop command received")
	s.mu.Unlock()

	s.isMoving.Store(false)
	return s.controller.Stop(ctx)
}

func (s *so101) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *so101) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *so101) IsMoving(ctx context.Context) (bool, error) {
	// Fast path: mid-command is never wrong, and avoids bus traffic during a move.
	if s.isMoving.Load() {
		return true, nil
	}
	// The arm owns the bus, so a read error is the answer and propagates. The gripper
	// deliberately does the opposite (it may be talking to a remote arm) -- don't
	// "fix" one to match the other; see so101Gripper.IsMoving.
	return s.controller.AnyServoMoving(ctx, s.armServoIDs)
}

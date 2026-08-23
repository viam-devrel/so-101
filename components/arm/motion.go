package arm

import (
	"context"
	"fmt"
	"math"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
	"so_arm/internal/planning"
	"so_arm/internal/servo"
)

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
// The SO-101 is a 5-DOF arm, so most six-DOF pose targets are unreachable. Rather than
// discard orientation wholesale, the planner is given an approach-axis cone (a
// referenceframe.PoseCloud) around the goal orientation: the tool's pointing direction is
// constrained to within orientation_tolerance_deg, while roll about that axis is free.
//
// Callers may bypass the cone via extra: "goal_metric_type" restores the old
// orientation-agnostic behavior, and "pose_cloud" supplies a raw cloud. See internal/planning.
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
// The caller must hold s.moveLock and manage s.isMoving.
func (s *so101) moveJoints(
	ctx context.Context,
	from, to []float64,
	speedDegsPerSec, accelDegsPerSecSq float64,
	caps []servo.JointLimits,
	wait bool,
) error {
	if len(from) != len(to) {
		return fmt.Errorf("coordinated move needs matching position counts, got %d current and %d target",
			len(from), len(to))
	}
	travelsDeg, maxTravelDeg := servo.JointTravelsDeg(from, to)

	profiles := servo.CoordinatedProfiles(travelsDeg, speedDegsPerSec, accelDegsPerSecSq, caps)
	if profiles == nil {
		// Every joint is already at its target. Note this is a behavior change: the old
		// path still issued a write, so a caller re-asserting its current pose used to
		// reach the bus and now does not.
		s.logger.Debug("moveJoints: all joints already at target, nothing to command")
		return nil
	}

	servoProfiles := make([]controller.ServoProfile, len(profiles))
	for i, p := range profiles {
		servoProfiles[i] = controller.ServoProfile{SpeedSteps: p.SpeedSteps, AccUnits: p.AccUnits}
	}

	if err := s.controller.MoveServosWithProfiles(ctx, s.armServoIDs, to, servoProfiles); err != nil {
		return fmt.Errorf("failed to move SO-101 arm: %w", err)
	}

	if !wait {
		return nil
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
	timeout := servo.MoveTimeoutMs(maxTravelDeg, effectiveSpeed, effectiveAccel)
	s.logger.Debugf("moveJoints: ref %.1f deg/s (effective %.1f), %.0f deg/s^2, max travel "+
		"%.1f deg, %d joint(s) at the acceleration floor, wait timeout %d ms",
		speedDegsPerSec, effectiveSpeed, accelDegsPerSecSq, maxTravelDeg, floored, timeout)
	return s.controller.WaitForServosToStop(ctx, s.armServoIDs, timeout)
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

// parseWaitExtra reads the optional "wait" bool from a DoCommand/extra map.
// Absent or non-bool values default to true so the motion planner and existing
// callers keep the blocking behavior; teleop passes wait=false to stream setpoints.
func parseWaitExtra(extra map[string]interface{}) bool {
	if extra != nil {
		if w, ok := extra["wait"].(bool); ok {
			return w
		}
	}
	return true
}

func (s *so101) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
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
	current, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("failed to read positions for coordinated move, falling back to "+
			"uniform speed: %v", err)
		// No arm.MoveOptions here, so caps is always nil -- kept for symmetry with
		// MoveThroughJointPositions' fallback.
		return s.moveJointsUniform(ctx, clamped, servo.UniformSpeedUnderCaps(speed, nil), accel, parseWaitExtra(extra))
	}

	return s.moveJoints(ctx, current, clamped, speed, accel, nil, parseWaitExtra(extra))
}

func (s *so101) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
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

	// One read per CALL, not per waypoint. The first waypoint has no predecessor to
	// difference against, and GoToInputs routinely emits single-waypoint streams which
	// would otherwise have no travel reference at all.
	from, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
	if err != nil {
		s.logger.Warnf("failed to read positions for coordinated move, falling back to "+
			"uniform speed: %v", err)
		from = nil
	}

	for idx, jointPositions := range positions {
		clamped, err := s.clampPositions(jointPositions)
		if err != nil {
			return err
		}
		isLast := idx == len(positions)-1

		// The hardware executes ONLY the final write. Each SetGoals overwrites the previous
		// goal, and the whole stream is issued in a few tens of milliseconds, so every
		// intermediate waypoint is superseded before the arm can act on it. The last
		// waypoint's profile therefore has to be computed against where the arm ACTUALLY
		// is, not against the previous commanded waypoint: a joint whose travel is
		// concentrated earlier in the path has a near-zero delta in the final segment,
		// which floors it to 1 step/s while its goal is still far away, and collapses the
		// completion timeout to the 1s floor so the call returns mid-motion.
		if isLast && from != nil {
			if cur, rerr := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs); rerr == nil {
				from = cur
			} else {
				s.logger.Warnf("failed to re-read positions before the final waypoint; "+
					"coordination and timeout will use the commanded predecessor: %v", rerr)
			}
		}

		if from == nil {
			if err := s.moveJointsUniform(ctx, clamped, servo.UniformSpeedUnderCaps(speed, caps), accel, isLast); err != nil {
				return err
			}
		} else if err := s.moveJoints(ctx, from, clamped, speed, accel, caps, isLast); err != nil {
			return err
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
		// Later segments difference against the previous COMMANDED waypoint.
		if from != nil {
			from = clamped
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
	return s.isMoving.Load(), nil
}

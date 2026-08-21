package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
)

// isGripperServo checks if a servo ID is the gripper (servo 6)
func isGripperServo(servoID int) bool {
	return servoID == 6
}

var globalRegistry = NewControllerRegistry()

// ControllerHandle is one resource's view of a port's shared bus. It holds no bus itself;
// every operation goes through the entry's session, so all holders share one state.
type ControllerHandle struct {
	entry  *ControllerEntry
	logger logging.Logger // per-holder, so log lines name the component that acted

	// owner is for diagnostics only, never an identity key: resource.Name survives an
	// AlwaysRebuild cycle, so a rebuilt resource would look like its own predecessor.
	owner       resource.Name
	releaseOnce sync.Once
}

// writePositions writes goal positions to the servo group. When speed > 0 it commands
// every servo at the same goal velocity (independent-joint speed mode, steps/sec);
// speed <= 0 uses the legacy max-speed path, which the gripper component relies on (it
// always calls with speed 0).
func writePositions(ctx context.Context, sess *busSession, rawPositions feetech.PositionMap, speed int) error {
	if speed > 0 {
		speeds := make(feetech.PositionMap, len(rawPositions))
		for id := range rawPositions {
			speeds[id] = speed
		}
		return sess.group.SetPositionsWithSpeed(ctx, rawPositions, speeds)
	}
	return sess.group.SetPositions(ctx, rawPositions)
}

func (h *ControllerHandle) MoveToJointPositions(ctx context.Context, jointAngles []float64, speed, acc int) error {
	armServoIDs := []int{1, 2, 3, 4, 5}
	if len(jointAngles) != len(armServoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(armServoIDs), len(jointAngles))
	}

	return h.withSession(func(sess *busSession) error {
		// Convert radians to appropriate normalized values based on servo type
		rawPositions := make(map[int]int, len(jointAngles))
		for i, servoID := range armServoIDs {
			// Arm servos: convert radians to degrees
			normalizedValue := utils.RadToDeg(jointAngles[i])

			cal := sess.servos[servoID].Calibration()
			raw, err := cal.Denormalize(normalizedValue)
			if err != nil {
				return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
			}
			rawPositions[servoID] = raw
		}

		return writePositions(ctx, sess, rawPositions, speed)
	})
}

func (h *ControllerHandle) MoveServosToPositions(ctx context.Context, servoIDs []int, jointAngles []float64, speed, acc int) error {
	if len(servoIDs) != len(jointAngles) {
		return fmt.Errorf("servo IDs and joint angles length mismatch")
	}

	return h.withSession(func(sess *busSession) error {
		// Convert radians to appropriate normalized values based on servo type
		rawPositions := make(map[int]int, len(jointAngles))
		for i, servoID := range servoIDs {
			var normalizedValue float64

			if isGripperServo(servoID) {
				// Gripper: input is in radians representation but encodes percentage
				// Convert from radians representation back to percentage (0-100)
				normalizedValue = (jointAngles[i]/math.Pi + 1.0) / 2.0 * 100.0
			} else {
				// Arm servos: convert radians to degrees
				normalizedValue = utils.RadToDeg(jointAngles[i])
			}

			cs, ok := sess.servos[servoID]
			if !ok {
				return fmt.Errorf("servo %d not available", servoID)
			}
			raw, err := cs.Calibration().Denormalize(normalizedValue)
			if err != nil {
				return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
			}
			rawPositions[servoID] = raw
		}

		return writePositions(ctx, sess, rawPositions, speed)
	})
}

// ServoProfile is one servo's speed and acceleration for a coordinated move.
type ServoProfile struct {
	SpeedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	AccUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// MoveServosWithProfiles commands each servo to its angle under its OWN speed and
// acceleration, in a single SetGoals sync write.
//
// This exists alongside writePositions rather than replacing it. writePositions can only
// issue one shared speed and cannot set acceleration at all, which is why joints arrive at
// different times today; but two of its callers are single-servo gripper paths where
// coordination is meaningless, so it stays as it is.
//
// Time is always written as 0. RegGoalTime (register 44) is inert on STS servos: it is an
// SCS-series feature, and FEETECH's own SDK reflects that -- SMS_STS.h defines
// SMS_STS_GOAL_TIME_L but SMS_STS.cpp never references it, and WritePosEx takes no time
// parameter and writes a hardcoded 0 to those bytes. Confirmed on hardware: a 10 degree move
// takes ~335ms whether commanded at 200ms or 3000ms. Writing anything else is ignored.
func (h *ControllerHandle) MoveServosWithProfiles(
	ctx context.Context,
	servoIDs []int,
	jointAngles []float64,
	profiles []ServoProfile,
) error {
	if len(servoIDs) != len(jointAngles) || len(servoIDs) != len(profiles) {
		return fmt.Errorf("servo IDs (%d), angles (%d) and profiles (%d) must be the same length",
			len(servoIDs), len(jointAngles), len(profiles))
	}

	return h.withSession(func(sess *busSession) error {
		goals := make(map[int]feetech.GoalRequest, len(servoIDs))
		for i, servoID := range servoIDs {
			normalized := utils.RadToDeg(jointAngles[i])
			if isGripperServo(servoID) {
				normalized = (jointAngles[i]/math.Pi + 1.0) / 2.0 * 100.0
			}
			cs, ok := sess.servos[servoID]
			if !ok {
				return fmt.Errorf("servo %d not available", servoID)
			}
			raw, err := cs.Calibration().Denormalize(normalized)
			if err != nil {
				return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
			}
			goals[servoID] = feetech.GoalRequest{
				Position: raw,
				Speed:    profiles[i].SpeedSteps,
				Acc:      profiles[i].AccUnits,
				Time:     0,
			}
		}
		return sess.group.SetGoals(ctx, goals)
	})
}

// --- Single-servo operations backing the servo_* DoCommand family ---
//
// These exist so a gripper can drive one servo on this bus without holding a controller of
// its own. Each holds the shared bus lock only for the duration of its bus transaction, and
// each denormalizes through the servo's own calibration rather than assuming a NormMode --
// servo 6 is Range100, but a 4-DOF arm may host a gripper on servo 5, which is Degrees.

// MoveServoPercent moves one servo to a position given as a percentage of its calibrated
// range. speed <= 0 uses the servo's configured speed.
func (h *ControllerHandle) MoveServoPercent(ctx context.Context, id int, percent float64, speed int) error {
	return h.withSession(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		cal := cs.Calibration()
		if cal == nil {
			return fmt.Errorf("no calibration for servo %d", id)
		}

		raw, err := cal.Denormalize(percentToNormalized(cal, percent))
		if err != nil {
			return fmt.Errorf("failed to denormalize percent for servo %d: %w", id, err)
		}
		return writePositions(ctx, sess, feetech.PositionMap{id: raw}, speed)
	})
}

// MoveServoRaw moves one servo to a raw encoder tick value, clamped to its calibrated range.
func (h *ControllerHandle) MoveServoRaw(ctx context.Context, id, raw int) error {
	return h.withSession(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		if cal := cs.Calibration(); cal != nil {
			if raw < cal.RangeMin {
				raw = cal.RangeMin
			}
			if raw > cal.RangeMax {
				raw = cal.RangeMax
			}
		}
		return writePositions(ctx, sess, feetech.PositionMap{id: raw}, 0)
	})
}

// ServoPositionPercent reads one servo's position as both a percentage of its calibrated
// range and the underlying raw tick value.
func (h *ControllerHandle) ServoPositionPercent(ctx context.Context, id int) (percent float64, rawOut int, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		servo := sess.group.ServoByID(id)
		if servo == nil {
			return fmt.Errorf("servo %d not available", id)
		}

		raw, err := servo.Position(ctx)
		if err != nil {
			return fmt.Errorf("failed to read position for servo %d: %w", id, err)
		}
		rawOut = raw

		cal := cs.Calibration()
		if cal == nil {
			return fmt.Errorf("no calibration for servo %d", id)
		}
		normalized, err := cal.Normalize(raw)
		if err != nil {
			return fmt.Errorf("failed to normalize servo %d: %w", id, err)
		}
		percent = normalizedToPercent(cal, normalized)
		return nil
	})
	return percent, rawOut, err
}

// StopServo halts a single servo. Unlike Stop, it leaves every other servo on the bus
// untouched, so stopping a gripper cannot kill an in-flight arm move.
func (h *ControllerHandle) StopServo(ctx context.Context, id int) error {
	return h.withSession(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		return cs.SetVelocity(ctx, 0)
	})
}

// percentToNormalized converts a 0-100 percentage into whatever normalized unit the servo's
// calibration expects: Range100 servos take the percentage directly, while degree-normalized
// servos take a proportional position within their normalized range.
func percentToNormalized(cal *MotorCalibration, percent float64) float64 {
	if cal.NormMode == NormModeRange100 {
		return percent
	}
	return degreeRangeMin + (percent/100.0)*(degreeRangeMax-degreeRangeMin)
}

// normalizedToPercent is the inverse of percentToNormalized.
func normalizedToPercent(cal *MotorCalibration, normalized float64) float64 {
	if cal.NormMode == NormModeRange100 {
		return normalized
	}
	return (normalized - degreeRangeMin) / (degreeRangeMax - degreeRangeMin) * 100.0
}

// The normalized degree span a degree-mode servo reports across its calibrated range.
const (
	degreeRangeMin = -100.0
	degreeRangeMax = 100.0
)

func (h *ControllerHandle) GetJointPositions(ctx context.Context) (out []float64, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		servoIDs := []int{1, 2, 3, 4, 5, 6}
		positions := make([]float64, len(servoIDs))

		// Read arm positions using ServoGroup
		servoPositions, err := sess.group.Positions(ctx)
		if err != nil {
			return fmt.Errorf("failed to read servo positions: %w", err)
		}

		// Normalize arm positions (servos 1-5)
		for i := range 5 {
			servoId := servoIDs[i]
			cal := sess.servos[servoId].Calibration()
			normalized, err := cal.Normalize(servoPositions[servoId])
			if err != nil {
				return fmt.Errorf("failed to normalize servo %d: %w", servoId, err)
			}
			// Convert degrees to radians
			positions[i] = utils.DegToRad(normalized)
		}

		// Normalize gripper position (servo 6)
		cal := sess.servos[6].Calibration()
		normalized, err := cal.Normalize(servoPositions[6])
		if err != nil {
			return fmt.Errorf("failed to normalize gripper: %w", err)
		}
		// Gripper uses 0-100 range, convert to radians representation for API consistency
		// normalized is already 0-100, convert to [-π, +π] range
		positions[5] = (normalized/100.0*2.0 - 1.0) * math.Pi

		out = positions
		return nil
	})
	return out, err
}

func (h *ControllerHandle) GetJointPositionsForServos(ctx context.Context, servoIDs []int) (out []float64, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		positions := make([]float64, len(servoIDs))

		rawPositions, err := sess.group.Positions(ctx)
		if err != nil {
			return fmt.Errorf("failed to get raw positions for servos: %w", err)
		}

		for i, servoID := range servoIDs {
			rawPos := rawPositions[servoID]
			cs, ok := sess.servos[servoID]
			if !ok {
				return fmt.Errorf("servo %d not available", servoID)
			}
			// Calibration(), not the field: another resource on this bus can be updating
			// it concurrently.
			normalized, err := cs.Calibration().Normalize(rawPos)
			if err != nil {
				return fmt.Errorf("failed to normalize raw servo value for id %d: %w", servoID, err)
			}
			if isGripperServo(servoID) {
				positions[i] = (normalized/100.0*2.0 - 1.0) * math.Pi
			} else {
				positions[i] = utils.DegToRad(normalized)
			}
		}

		out = positions
		return nil
	})
	return out, err
}

func (h *ControllerHandle) SetTorqueEnable(ctx context.Context, enable bool) error {
	return h.withSession(func(sess *busSession) error {
		if enable {
			if err := sess.group.EnableAll(ctx); err != nil {
				return fmt.Errorf("failed to set torque enable: %w", err)
			}
		} else {
			if err := sess.group.DisableAll(ctx); err != nil {
				return fmt.Errorf("failed to set torque enable: %w", err)
			}
		}
		return nil
	})
}

func (h *ControllerHandle) Stop(ctx context.Context) error {
	return h.withSession(func(sess *busSession) error {
		for id, servo := range sess.servos {
			if err := servo.SetVelocity(ctx, 0); err != nil {
				h.logger.Warnf("Failed to stop servo %d: %v", id, err)
			}
		}
		return nil
	})
}

// WaitForServosToStop polls only the given servos' Moving register until all report
// stopped, ctx is cancelled, or timeoutMs elapses (best-effort: on timeout it logs a
// warning and returns nil, so a still-settling servo doesn't fail an otherwise-complete
// move).
//
// Scoped to the requested servos so an in-flight gripper move on the shared bus cannot
// block an arm move's completion wait. Polls lock-free after capturing servo references
// under a brief read lock, so a concurrent Stop is not blocked.
//
// Each cycle issues one Moving read per servo; those reads serialize at the feetech bus
// mutex alongside in-flight motion writes. At the 50ms cadence this is negligible, but a
// future optimization could batch them into a single SyncRead of the Moving register.
func (h *ControllerHandle) WaitForServosToStop(ctx context.Context, servoIDs []int, timeoutMs int) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}

	h.entry.busMu.RLock()
	ids := make([]int, 0, len(servoIDs))
	servos := make([]*CalibratedServo, 0, len(servoIDs))
	for _, id := range servoIDs {
		if cs, ok := sess.servos[id]; ok {
			ids = append(ids, id)
			servos = append(servos, cs)
		}
	}
	h.entry.busMu.RUnlock()

	deadline := time.Now().Add(time.Duration(timeoutMs) * time.Millisecond)
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	for {
		allStopped := true
		for i, cs := range servos {
			moving, err := cs.Moving(ctx)
			if err != nil {
				return fmt.Errorf("failed to read moving state for servo %d: %w", ids[i], err)
			}
			if moving {
				allStopped = false
				break
			}
		}
		if allStopped {
			return nil
		}
		if time.Now().After(deadline) {
			h.logger.Warnf("WaitForServosToStop: servos %v still moving after %dms timeout", ids, timeoutMs)
			return nil
		}

		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func (h *ControllerHandle) Ping(ctx context.Context) error {
	return h.withSessionRead(func(sess *busSession) error {
		for id, servo := range sess.servos {
			if _, err := servo.Ping(ctx); err != nil {
				return fmt.Errorf("ping failed for servo %d: %w", id, err)
			}
		}
		return nil
	})
}

// LoadForServos reads present load (signed, sign-magnitude decoded) for the given
// servos. One Load read per servo; at 50 Hz over 5 servos this is well within the
// 1 Mbaud bus budget. Future optimization: a single SyncRead of RegPresentLoad.
func (h *ControllerHandle) LoadForServos(ctx context.Context, servoIDs []int) (out map[int]int, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		loads := make(map[int]int, len(servoIDs))
		for _, id := range servoIDs {
			servo := sess.group.ServoByID(id)
			if servo == nil {
				return fmt.Errorf("servo %d not available", id)
			}
			load, err := servo.Load(ctx)
			if err != nil {
				return fmt.Errorf("failed to read load for servo %d: %w", id, err)
			}
			loads[id] = load
		}
		out = loads
		return nil
	})
	return out, err
}

// WriteServoRegister writes to a specific servo register by name
func (h *ControllerHandle) WriteServoRegister(ctx context.Context, servoID int, registerName string, data []byte) error {
	return h.withSession(func(sess *busSession) error {
		servo := sess.group.ServoByID(servoID)
		if servo == nil {
			return fmt.Errorf("servo %d not available", servoID)
		}
		return servo.WriteRegister(ctx, registerName, data)
	})
}

// ReadServoRegister reads a named register from a specific servo.
func (h *ControllerHandle) ReadServoRegister(ctx context.Context, servoID int, registerName string) (out []byte, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		servo := sess.group.ServoByID(servoID)
		if servo == nil {
			return fmt.Errorf("servo %d not available", servoID)
		}
		data, err := servo.ReadRegister(ctx, registerName)
		if err != nil {
			return err
		}
		out = data
		return nil
	})
	return out, err
}

// SetCalibration replaces the calibration every servo on this bus normalizes through.
// Writes both the per-servo copies, which every read and write path uses, and
// entry.calibration, which seeds any later session -- skip the seed and a rebuilt session
// would restore the old values.
func (h *ControllerHandle) SetCalibration(calibration SO101FullCalibration) error {
	h.entry.busMu.Lock()
	defer h.entry.busMu.Unlock()

	if sess := h.entry.session.Load(); sess != nil {
		for id := 1; id <= 6; id++ {
			sess.servos[id].UpdateCalibration(copyMotorCalibration(calibration.GetMotorCalibrationByID(id)))
		}
	}

	h.entry.mu.Lock()
	h.entry.calibration = calibration
	h.entry.mu.Unlock()
	return nil
}

// GetCalibration reassembles the full calibration from the per-servo copies, which are the
// source of truth. It falls back to the entry's seed only when there is no session to read.
func (h *ControllerHandle) GetCalibration() SO101FullCalibration {
	sess := h.entry.session.Load()
	if sess == nil {
		return h.entry.currentCalibration()
	}
	return SO101FullCalibration{
		ShoulderPan:  sess.servos[1].Calibration(),
		ShoulderLift: sess.servos[2].Calibration(),
		ElbowFlex:    sess.servos[3].Calibration(),
		WristFlex:    sess.servos[4].Calibration(),
		WristRoll:    sess.servos[5].Calibration(),
		Gripper:      sess.servos[6].Calibration(),
	}
}

func configsEqual(a, b *SoArm101Config) bool {
	if a == nil && b == nil {
		return true
	}
	if a == nil || b == nil {
		return false
	}
	return a.Port == b.Port &&
		a.Baudrate == b.Baudrate &&
		a.Timeout == b.Timeout
}

func fullCalibrationsEqual(a, b SO101FullCalibration) bool {
	return a.Equal(b)
}

// AcquireController delegates to the process-wide registry. Production uses this one;
// tests use the registry method so they can inject a bus factory.
func AcquireController(
	owner resource.Name, config *SoArm101Config,
	calibration SO101FullCalibration, fromFile bool,
) (*ControllerHandle, error) {
	return globalRegistry.AcquireController(owner, config, calibration, fromFile)
}

func GetControllerStatus() (int64, bool, string) {
	globalRegistry.mu.RLock()
	defer globalRegistry.mu.RUnlock()

	var totalRefCount int64
	hasController := len(globalRegistry.entries) > 0
	configSummaries := make([]string, 0, len(globalRegistry.entries))

	for _, entry := range globalRegistry.entries {
		entry.mu.RLock()
		refCount := entry.refCount
		totalRefCount += int64(refCount)

		if entry.config != nil {
			calibrationInfo := "default"
			if entry.calibration.ShoulderPan != nil &&
				entry.calibration.ShoulderPan.HomingOffset != DefaultSO101FullCalibration.ShoulderPan.HomingOffset {
				calibrationInfo = "custom"
			}
			summary := fmt.Sprintf("%s@%d(refs:%d,cal:%s)",
				entry.config.Port, entry.config.Baudrate, refCount, calibrationInfo)
			configSummaries = append(configSummaries, summary)
		}
		entry.mu.RUnlock()
	}

	configSummary := ""
	if len(configSummaries) > 0 {
		configSummary = "Controllers: " + fmt.Sprintf("%v", configSummaries)
	}

	return totalRefCount, hasController, configSummary
}

// With multiple controllers, this returns the default calibration
// Use GetCurrentCalibrationForPort for port-specific calibration
func GetCurrentCalibration() SO101FullCalibration {
	return DefaultSO101FullCalibration
}

func GetCurrentCalibrationForPort(portPath string) SO101FullCalibration {
	return globalRegistry.GetCurrentCalibration(portPath)
}

// --- Operations the calibration sensor needs ---
//
// The sensor used to reach past the handle straight into the bus and the servo map, taking
// no controller lock at all. These give it the same reach under the shared lock.

// SyncReadPositions reads present position for the given servos and returns decoded ticks.
// Decoding stays inside the handle: exposing Protocol() would hand the raw bus back out.
//
// dataLen is 2, a position register's width. The old call sites passed len(ids), which
// happened to be harmless at exactly six servos.
func (h *ControllerHandle) SyncReadPositions(ctx context.Context, ids []int) (out map[int]int, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		data, err := sess.bus.SyncRead(ctx, feetech.RegPresentPosition.Address, 2, ids)
		if err != nil {
			return err
		}
		proto := sess.bus.Protocol()
		positions := make(map[int]int, len(ids))
		for _, id := range ids {
			if d, ok := data[id]; ok {
				positions[id] = int(proto.DecodeWord(d))
			}
		}
		out = positions
		return nil
	})
	return out, err
}

// PingServo pings one servo and returns its model number.
func (h *ControllerHandle) PingServo(ctx context.Context, id int) (model int, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		m, err := cs.Ping(ctx)
		if err != nil {
			return err
		}
		model = m
		return nil
	})
	return model, err
}

// DetectServoModel runs the servo's model auto-detection and returns the detected name.
func (h *ControllerHandle) DetectServoModel(ctx context.Context, id int) (name string, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		if err := cs.DetectModel(ctx); err != nil {
			return err
		}
		if m := cs.Model(); m != nil {
			name = m.Name
		}
		return nil
	})
	return name, err
}

// ServoPresent reports whether the session has a servo with this ID. No bus traffic.
// motorSetupVerify needs "absent" and "did not answer" to be different answers.
func (h *ControllerHandle) ServoPresent(id int) bool {
	sess := h.entry.session.Load()
	if sess == nil {
		return false
	}
	_, ok := sess.servos[id]
	return ok
}

// Discover sweeps every bus ID. Holds the shared lock for the whole sweep, which is
// seconds on a sparse bus, so arm motion waits during motor setup.
func (h *ControllerHandle) Discover(ctx context.Context) (out []feetech.FoundServo, err error) {
	err = h.withSessionRead(func(sess *busSession) error {
		found, err := sess.bus.Discover(ctx)
		if err != nil {
			return err
		}
		out = found
		return nil
	})
	return out, err
}

// SetServoID changes a servo's ID. The session's servo map stays keyed by the old ID until
// the next acquire, as it did before this refactor; motor setup reconfigures afterwards.
func (h *ControllerHandle) SetServoID(ctx context.Context, currentID, targetID int) error {
	return h.withSession(func(sess *busSession) error {
		cs, ok := sess.servos[currentID]
		if !ok {
			return fmt.Errorf("servo %d not available", currentID)
		}
		return cs.SetID(ctx, targetID)
	})
}

// SetServoBaudRate changes a servo's baud rate.
func (h *ControllerHandle) SetServoBaudRate(ctx context.Context, id, baudRate int) error {
	return h.withSession(func(sess *busSession) error {
		cs, ok := sess.servos[id]
		if !ok {
			return fmt.Errorf("servo %d not available", id)
		}
		return cs.SetBaudRate(ctx, baudRate)
	})
}

// Release drops this handle's claim on the port, closing the shared bus and evicting the
// entry once this was the last holder. Idempotent, since rdk can Close a resource twice.
func (h *ControllerHandle) Release() {
	h.releaseOnce.Do(func() {
		e := h.entry
		e.mu.Lock()
		e.refCount--
		e.mu.Unlock()

		// Called with no lock held, and re-reads refCount itself, so a concurrent acquire
		// and release settle on one answer.
		e.teardownIfLast()
	})
}

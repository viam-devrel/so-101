package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/utils"
)

// isGripperServo checks if a servo ID is the gripper (servo 6)
func isGripperServo(servoID int) bool {
	return servoID == 6
}

var globalRegistry = NewControllerRegistry()

type SafeSoArmController struct {
	bus              *feetech.Bus
	group            *feetech.ServoGroup
	calibratedServos map[int]*CalibratedServo
	logger           logging.Logger
	calibration      SO101FullCalibration
	mu               sync.RWMutex
}

// writePositions writes goal positions to the servo group. When speed > 0 it commands
// every servo at the same goal velocity (independent-joint speed mode, steps/sec);
// speed <= 0 uses the legacy max-speed path, which the gripper component relies on (it
// always calls with speed 0).
func (s *SafeSoArmController) writePositions(ctx context.Context, rawPositions feetech.PositionMap, speed int) error {
	if speed > 0 {
		speeds := make(feetech.PositionMap, len(rawPositions))
		for id := range rawPositions {
			speeds[id] = speed
		}
		return s.group.SetPositionsWithSpeed(ctx, rawPositions, speeds)
	}
	return s.group.SetPositions(ctx, rawPositions)
}

func (s *SafeSoArmController) MoveToJointPositions(ctx context.Context, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	armServoIDs := []int{1, 2, 3, 4, 5}
	if len(jointAngles) != len(armServoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(armServoIDs), len(jointAngles))
	}

	// Convert radians to appropriate normalized values based on servo type
	rawPositions := make(map[int]int, len(jointAngles))
	for i, servoID := range armServoIDs {
		var normalizedValue float64

		// Arm servos: convert radians to degrees
		normalizedValue = utils.RadToDeg(jointAngles[i])

		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(normalizedValue)
		if err != nil {
			return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
		}
		rawPositions[servoID] = raw
	}

	return s.writePositions(ctx, rawPositions, speed)
}

func (s *SafeSoArmController) MoveServosToPositions(ctx context.Context, servoIDs []int, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(servoIDs) != len(jointAngles) {
		return fmt.Errorf("servo IDs and joint angles length mismatch")
	}

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

		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(normalizedValue)
		if err != nil {
			return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
		}
		rawPositions[servoID] = raw
	}

	return s.writePositions(ctx, rawPositions, speed)
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
func (s *SafeSoArmController) MoveServosWithProfiles(
	ctx context.Context,
	servoIDs []int,
	jointAngles []float64,
	profiles []ServoProfile,
) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(servoIDs) != len(jointAngles) || len(servoIDs) != len(profiles) {
		return fmt.Errorf("servo IDs (%d), angles (%d) and profiles (%d) must be the same length",
			len(servoIDs), len(jointAngles), len(profiles))
	}

	goals := make(map[int]feetech.GoalRequest, len(servoIDs))
	for i, servoID := range servoIDs {
		normalized := utils.RadToDeg(jointAngles[i])
		if isGripperServo(servoID) {
			normalized = (jointAngles[i]/math.Pi + 1.0) / 2.0 * 100.0
		}
		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(normalized)
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
	return s.group.SetGoals(ctx, goals)
}

// --- Single-servo operations backing the servo_* DoCommand family ---
//
// These exist so a gripper can drive one servo on this bus without holding a controller of
// its own. Each takes the shared mutex only for the duration of its bus transaction, and
// each denormalizes through the servo's own calibration rather than assuming a NormMode --
// servo 6 is Range100, but a 4-DOF arm may host a gripper on servo 5, which is Degrees.

// MoveServoPercent moves one servo to a position given as a percentage of its calibrated
// range. speed <= 0 uses the servo's configured speed.
func (s *SafeSoArmController) MoveServoPercent(ctx context.Context, id int, percent float64, speed int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	cs, ok := s.calibratedServos[id]
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
	return s.writePositions(ctx, feetech.PositionMap{id: raw}, speed)
}

// MoveServoRaw moves one servo to a raw encoder tick value, clamped to its calibrated range.
func (s *SafeSoArmController) MoveServoRaw(ctx context.Context, id, raw int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	cs, ok := s.calibratedServos[id]
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
	return s.writePositions(ctx, feetech.PositionMap{id: raw}, 0)
}

// ServoPositionPercent reads one servo's position as both a percentage of its calibrated
// range and the underlying raw tick value.
func (s *SafeSoArmController) ServoPositionPercent(ctx context.Context, id int) (float64, int, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	cs, ok := s.calibratedServos[id]
	if !ok {
		return 0, 0, fmt.Errorf("servo %d not available", id)
	}
	servo := s.group.ServoByID(id)
	if servo == nil {
		return 0, 0, fmt.Errorf("servo %d not available", id)
	}

	raw, err := servo.Position(ctx)
	if err != nil {
		return 0, 0, fmt.Errorf("failed to read position for servo %d: %w", id, err)
	}
	cal := cs.Calibration()
	if cal == nil {
		return 0, raw, fmt.Errorf("no calibration for servo %d", id)
	}
	normalized, err := cal.Normalize(raw)
	if err != nil {
		return 0, raw, fmt.Errorf("failed to normalize servo %d: %w", id, err)
	}
	return normalizedToPercent(cal, normalized), raw, nil
}

// StopServo halts a single servo. Unlike Stop, it leaves every other servo on the bus
// untouched, so stopping a gripper cannot kill an in-flight arm move.
func (s *SafeSoArmController) StopServo(ctx context.Context, id int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	cs, ok := s.calibratedServos[id]
	if !ok {
		return fmt.Errorf("servo %d not available", id)
	}
	return cs.SetVelocity(ctx, 0)
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

func (s *SafeSoArmController) GetJointPositions(ctx context.Context) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	servoIDs := []int{1, 2, 3, 4, 5, 6}
	positions := make([]float64, len(servoIDs))

	// Read arm positions using ServoGroup
	servoPositions, err := s.group.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to read servo positions: %w", err)
	}

	// Normalize arm positions (servos 1-5)
	for i := range 5 {
		servoId := servoIDs[i]
		cal := s.calibration.GetMotorCalibrationByID(servoId)
		normalized, err := cal.Normalize(servoPositions[servoId])
		if err != nil {
			return nil, fmt.Errorf("failed to normalize servo %d: %w", servoId, err)
		}
		// Convert degrees to radians
		positions[i] = utils.DegToRad(normalized)
	}

	// Normalize gripper position (servo 6)
	cal := s.calibration.GetMotorCalibrationByID(6)
	normalized, err := cal.Normalize(servoPositions[6])
	if err != nil {
		return nil, fmt.Errorf("failed to normalize gripper: %w", err)
	}
	// Gripper uses 0-100 range, convert to radians representation for API consistency
	// normalized is already 0-100, convert to [-π, +π] range
	positions[5] = (normalized/100.0*2.0 - 1.0) * math.Pi

	return positions, nil
}

func (s *SafeSoArmController) GetJointPositionsForServos(ctx context.Context, servoIDs []int) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	positions := make([]float64, len(servoIDs))

	rawPositions, err := s.group.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to get raw positions for servos: %w", err)
	}

	for i, servoID := range servoIDs {
		rawPos := rawPositions[servoID]
		cal := s.calibratedServos[servoID].calibration
		normalized, err := cal.Normalize(rawPos)
		if err != nil {
			return nil, fmt.Errorf("failed to normalize raw servo value for id %d: %w", servoID, err)
		}
		if isGripperServo(servoID) {
			positions[i] = (normalized/100.0*2.0 - 1.0) * math.Pi
		} else {
			positions[i] = utils.DegToRad(normalized)
		}

	}

	return positions, nil
}

func (s *SafeSoArmController) SetTorqueEnable(ctx context.Context, enable bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if enable {
		if err := s.group.EnableAll(ctx); err != nil {
			return fmt.Errorf("failed to set torque enable: %w", err)
		}
	} else {
		if err := s.group.DisableAll(ctx); err != nil {
			return fmt.Errorf("failed to set torque enable: %w", err)
		}
	}
	return nil
}

func (s *SafeSoArmController) Stop(ctx context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	for id, servo := range s.calibratedServos {
		if err := servo.SetVelocity(ctx, 0); err != nil {
			s.logger.Warnf("Failed to stop servo %d: %v", id, err)
		}
	}
	return nil
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
func (s *SafeSoArmController) WaitForServosToStop(ctx context.Context, servoIDs []int, timeoutMs int) error {
	s.mu.RLock()
	ids := make([]int, 0, len(servoIDs))
	servos := make([]*CalibratedServo, 0, len(servoIDs))
	for _, id := range servoIDs {
		if cs, ok := s.calibratedServos[id]; ok {
			ids = append(ids, id)
			servos = append(servos, cs)
		}
	}
	s.mu.RUnlock()

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
			s.logger.Warnf("WaitForServosToStop: servos %v still moving after %dms timeout", ids, timeoutMs)
			return nil
		}

		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func (s *SafeSoArmController) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.bus != nil {
		return s.bus.Close()
	}
	return nil
}

func (s *SafeSoArmController) Ping(ctx context.Context) error {
	s.mu.RLock()
	defer s.mu.RUnlock()

	for id, servo := range s.calibratedServos {
		if _, err := servo.Ping(ctx); err != nil {
			return fmt.Errorf("ping failed for servo %d: %w", id, err)
		}
	}
	return nil
}

// LoadForServos reads present load (signed, sign-magnitude decoded) for the given
// servos. One Load read per servo; at 50 Hz over 5 servos this is well within the
// 1 Mbaud bus budget. Future optimization: a single SyncRead of RegPresentLoad.
func (s *SafeSoArmController) LoadForServos(ctx context.Context, servoIDs []int) (map[int]int, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	out := make(map[int]int, len(servoIDs))
	for _, id := range servoIDs {
		servo := s.group.ServoByID(id)
		if servo == nil {
			return nil, fmt.Errorf("servo %d not available", id)
		}
		load, err := servo.Load(ctx)
		if err != nil {
			return nil, fmt.Errorf("failed to read load for servo %d: %w", id, err)
		}
		out[id] = load
	}
	return out, nil
}

// WriteServoRegister writes to a specific servo register by name
func (s *SafeSoArmController) WriteServoRegister(ctx context.Context, servoID int, registerName string, data []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	servo := s.group.ServoByID(servoID)
	if servo == nil {
		return fmt.Errorf("servo %d not available", servoID)
	}

	return servo.WriteRegister(ctx, registerName, data)
}

// ReadServoRegister reads a named register from a specific servo.
func (s *SafeSoArmController) ReadServoRegister(ctx context.Context, servoID int, registerName string) ([]byte, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	servo := s.group.ServoByID(servoID)
	if servo == nil {
		return nil, fmt.Errorf("servo %d not available", servoID)
	}

	return servo.ReadRegister(ctx, registerName)
}

func (s *SafeSoArmController) SetCalibration(calibration SO101FullCalibration) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Update calibration in each CalibratedServo
	for id := 1; id <= 6; id++ {
		motorCal := calibration.GetMotorCalibrationByID(id)
		appCal := &MotorCalibration{
			ID:           motorCal.ID,
			DriveMode:    motorCal.DriveMode,
			HomingOffset: motorCal.HomingOffset,
			RangeMin:     motorCal.RangeMin,
			RangeMax:     motorCal.RangeMax,
			NormMode:     motorCal.NormMode,
		}
		s.calibratedServos[id].UpdateCalibration(appCal)
	}

	s.calibration = calibration
	return nil
}

func (s *SafeSoArmController) GetCalibration() SO101FullCalibration {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.calibration
}

// getCalibrationForServo returns the calibration for a specific servo ID
func (s *SafeSoArmController) getCalibrationForServo(servoID int) *MotorCalibration {
	s.mu.RLock()
	defer s.mu.RUnlock()

	switch servoID {
	case 1:
		return s.calibration.ShoulderPan
	case 2:
		return s.calibration.ShoulderLift
	case 3:
		return s.calibration.ElbowFlex
	case 4:
		return s.calibration.WristFlex
	case 5:
		return s.calibration.WristRoll
	case 6:
		return s.calibration.Gripper
	default:
		return nil
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

func GetSharedController(config *SoArm101Config) (*SafeSoArmController, error) {
	return GetSharedControllerWithCalibration(config, DefaultSO101FullCalibration, false)
}

func GetSharedControllerWithCalibration(config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*SafeSoArmController, error) {
	return globalRegistry.GetController(config.Port, config, calibration, fromFile)
}

func ReleaseSharedController() {
	globalRegistry.releaseFromCaller()
}

func ForceCloseSharedController() error {
	globalRegistry.mu.RLock()
	portPaths := make([]string, 0, len(globalRegistry.entries))
	for portPath := range globalRegistry.entries {
		portPaths = append(portPaths, portPath)
	}
	globalRegistry.mu.RUnlock()

	var lastErr error
	for _, portPath := range portPaths {
		if err := globalRegistry.ForceCloseController(portPath); err != nil {
			lastErr = err
		}
	}
	return lastErr
}

func GetControllerStatus() (int64, bool, string) {
	globalRegistry.mu.RLock()
	defer globalRegistry.mu.RUnlock()

	var totalRefCount int64
	hasController := len(globalRegistry.entries) > 0
	configSummaries := make([]string, 0, len(globalRegistry.entries))

	for _, entry := range globalRegistry.entries {
		entry.mu.RLock()
		refCount := atomic.LoadInt64(&entry.refCount)
		totalRefCount += refCount

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

package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"

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

	// Use SetPositionsWithSpeed when speed is specified, otherwise use default SetPositions
	// Note: acceleration parameter not yet supported by feetech-servo library
	if speed > 0 {
		// Create speed map with the same speed for all servos
		speeds := make(map[int]int, len(rawPositions))
		for servoID := range rawPositions {
			speeds[servoID] = speed
		}
		return s.group.SetPositionsWithSpeed(ctx, rawPositions, speeds)
	}
	return s.group.SetPositions(ctx, rawPositions)
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

	// Use SetPositionsWithSpeed when speed is specified, otherwise use default SetPositions
	// Note: acceleration parameter not yet supported by feetech-servo library
	if speed > 0 {
		// Create speed map with the same speed for all servos
		speeds := make(map[int]int, len(rawPositions))
		for servoID := range rawPositions {
			speeds[servoID] = speed
		}
		return s.group.SetPositionsWithSpeed(ctx, rawPositions, speeds)
	}
	return s.group.SetPositions(ctx, rawPositions)
}

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

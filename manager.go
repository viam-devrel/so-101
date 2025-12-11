package so_arm

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
)

var globalRegistry = NewControllerRegistry()

type SafeSoArmController struct {
	bus          *feetech.Bus
	armGroup     *feetech.ServoGroup
	gripperGroup *feetech.ServoGroup
	servos       map[int]*CalibratedServo
	logger       logging.Logger
	calibration  SO101FullCalibration
	mu           sync.RWMutex
}

func (s *SafeSoArmController) MoveToJointPositions(ctx context.Context, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	armServoIDs := []int{1, 2, 3, 4, 5}
	if len(jointAngles) != len(armServoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(armServoIDs), len(jointAngles))
	}

	// Convert radians to degrees
	degrees := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		degrees[i] = angle * 180.0 / 3.14159265359
	}

	// Denormalize degrees to raw positions using calibration
	rawPositions := make([]int, len(degrees))
	for i, servoID := range armServoIDs {
		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(degrees[i])
		if err != nil {
			return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
		}
		rawPositions[i] = raw
	}

	// Use ServoGroup to write positions
	return s.armGroup.SetPositions(ctx, rawPositions)
}

func (s *SafeSoArmController) MoveServosToPositions(ctx context.Context, servoIDs []int, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(servoIDs) != len(jointAngles) {
		return fmt.Errorf("servo IDs and joint angles length mismatch")
	}

	// Determine which group to use
	isArmOnly := true
	isGripperOnly := len(servoIDs) == 1 && servoIDs[0] == 6

	for _, id := range servoIDs {
		if id == 6 {
			isArmOnly = false
		} else if id < 1 || id > 5 {
			return fmt.Errorf("invalid servo ID: %d", id)
		}
	}

	// Convert radians to degrees
	degrees := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		degrees[i] = angle * 180.0 / 3.14159265359
	}

	// Denormalize degrees to raw positions
	rawPositions := make([]int, len(degrees))
	for i, servoID := range servoIDs {
		cal := s.calibration.GetMotorCalibrationByID(servoID)
		raw, err := cal.Denormalize(degrees[i])
		if err != nil {
			return fmt.Errorf("failed to denormalize position for servo %d: %w", servoID, err)
		}
		rawPositions[i] = raw
	}

	// Use appropriate ServoGroup
	if isGripperOnly {
		return s.gripperGroup.SetPositions(ctx, rawPositions)
	} else if isArmOnly && len(servoIDs) == 5 {
		return s.armGroup.SetPositions(ctx, rawPositions)
	} else {
		// Mixed or partial - use individual servo writes
		for i, servoID := range servoIDs {
			if err := s.servos[servoID].SetPosition(ctx, degrees[i]); err != nil {
				return fmt.Errorf("failed to set position for servo %d: %w", servoID, err)
			}
		}
		return nil
	}
}

func (s *SafeSoArmController) GetJointPositions(ctx context.Context) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	servoIDs := []int{1, 2, 3, 4, 5, 6}
	positions := make([]float64, len(servoIDs))

	// Read arm positions using ServoGroup
	armPositions, err := s.armGroup.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to read arm positions: %w", err)
	}

	// Normalize arm positions
	for i := 0; i < 5; i++ {
		cal := s.calibration.GetMotorCalibrationByID(servoIDs[i])
		normalized, err := cal.Normalize(armPositions[i])
		if err != nil {
			return nil, fmt.Errorf("failed to normalize servo %d: %w", servoIDs[i], err)
		}
		// Convert degrees to radians
		positions[i] = normalized * 3.14159265359 / 180.0
	}

	// Read gripper position using ServoGroup
	gripperPositions, err := s.gripperGroup.Positions(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to read gripper position: %w", err)
	}

	// Normalize gripper position
	cal := s.calibration.GetMotorCalibrationByID(6)
	normalized, err := cal.Normalize(gripperPositions[0])
	if err != nil {
		return nil, fmt.Errorf("failed to normalize gripper: %w", err)
	}
	// Gripper uses 0-100 range, convert to radians for consistency
	positions[5] = normalized * 3.14159265359 / 180.0

	return positions, nil
}

func (s *SafeSoArmController) GetJointPositionsForServos(ctx context.Context, servoIDs []int) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	positions := make([]float64, len(servoIDs))

	// Use individual calibrated servo reads
	for i, servoID := range servoIDs {
		degrees, err := s.servos[servoID].Position(ctx)
		if err != nil {
			return nil, fmt.Errorf("failed to read servo %d: %w", servoID, err)
		}
		// Convert degrees to radians
		positions[i] = degrees * 3.14159265359 / 180.0
	}

	return positions, nil
}

func (s *SafeSoArmController) SetTorqueEnable(ctx context.Context, enable bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	for id, servo := range s.servos {
		if err := servo.SetTorqueEnabled(ctx, enable); err != nil {
			return fmt.Errorf("failed to set torque enable for servo %d: %w", id, err)
		}
	}
	return nil
}

func (s *SafeSoArmController) Stop(ctx context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	for id, servo := range s.servos {
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

	for id, servo := range s.servos {
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

	servo, exists := s.servos[servoID]
	if !exists {
		return fmt.Errorf("servo %d not available", servoID)
	}

	return servo.GetRawServo().WriteRegister(ctx, registerName, data)
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
		s.servos[id].UpdateCalibration(appCal)
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

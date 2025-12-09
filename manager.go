package so_arm

import (
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/hipsterbrown/feetech-servo"
	"go.viam.com/rdk/logging"
)

var globalRegistry = NewControllerRegistry()

type SafeSoArmController struct {
	bus         *feetech.Bus
	servos      map[int]*feetech.Servo
	logger      logging.Logger
	calibration SO101FullCalibration
	mu          sync.RWMutex
}

func (s *SafeSoArmController) MoveToJointPositions(jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	armServoIDs := []int{1, 2, 3, 4, 5}
	if len(jointAngles) != len(armServoIDs) {
		return fmt.Errorf("expected %d joint angles, got %d", len(armServoIDs), len(jointAngles))
	}

	var servos []*feetech.Servo
	for _, id := range armServoIDs {
		if servo, exists := s.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return fmt.Errorf("servo %d not available", id)
		}
	}

	positions := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		positions[i] = angle * 180.0 / 3.14159265359
	}

	return s.bus.SyncWritePositions(servos, positions, true)
}

func (s *SafeSoArmController) MoveServosToPositions(servoIDs []int, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(servoIDs) != len(jointAngles) {
		return fmt.Errorf("servo IDs and joint angles length mismatch")
	}

	var servos []*feetech.Servo
	for _, id := range servoIDs {
		if servo, exists := s.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return fmt.Errorf("servo %d not available", id)
		}
	}

	positions := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		positions[i] = angle * 180.0 / 3.14159265359
	}

	return s.bus.SyncWritePositions(servos, positions, true)
}

func (s *SafeSoArmController) GetJointPositions() ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	servoIDs := []int{1, 2, 3, 4, 5, 6}
	var servos []*feetech.Servo
	for _, id := range servoIDs {
		if servo, exists := s.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return nil, fmt.Errorf("servo %d not available", id)
		}
	}

	positionMap, err := s.bus.SyncReadPositions(servos, true)
	if err != nil {
		return nil, err
	}

	positions := make([]float64, len(servoIDs))
	for i, id := range servoIDs {
		if pos, exists := positionMap[id]; exists {
			positions[i] = pos * 3.14159265359 / 180.0
		} else {
			return nil, fmt.Errorf("no position data for servo %d", id)
		}
	}

	return positions, nil
}

func (s *SafeSoArmController) GetJointPositionsForServos(servoIDs []int) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	var servos []*feetech.Servo
	for _, id := range servoIDs {
		if servo, exists := s.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return nil, fmt.Errorf("servo %d not available", id)
		}
	}

	positionMap, err := s.bus.SyncReadPositions(servos, true)
	if err != nil {
		return nil, err
	}

	positions := make([]float64, len(servoIDs))
	for i, id := range servoIDs {
		if pos, exists := positionMap[id]; exists {
			positions[i] = pos * 3.14159265359 / 180.0
		} else {
			return nil, fmt.Errorf("no position data for servo %d", id)
		}
	}

	return positions, nil
}

func (s *SafeSoArmController) SetTorqueEnable(enable bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	for _, servo := range s.servos {
		if err := servo.SetTorqueEnable(enable); err != nil {
			return fmt.Errorf("failed to set torque enable for servo %d: %w", servo.ID, err)
		}
	}
	return nil
}

func (s *SafeSoArmController) Stop() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	for _, servo := range s.servos {
		if err := servo.WriteVelocity(0, false); err != nil {
			s.logger.Warnf("Failed to stop servo %d: %v", servo.ID, err)
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

func (s *SafeSoArmController) Ping() error {
	s.mu.RLock()
	defer s.mu.RUnlock()

	for _, servo := range s.servos {
		if _, err := servo.Ping(); err != nil {
			return fmt.Errorf("ping failed for servo %d: %w", servo.ID, err)
		}
	}
	return nil
}

// WriteServoRegister writes to a specific servo register by name
func (s *SafeSoArmController) WriteServoRegister(servoID int, registerName string, data []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	servo, exists := s.servos[servoID]
	if !exists {
		return fmt.Errorf("servo %d not available", servoID)
	}

	return servo.WriteRegisterByName(registerName, data)
}

func (s *SafeSoArmController) SetCalibration(calibration SO101FullCalibration) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	feetechCals := calibration.ToFeetechCalibrationMap()

	for id, cal := range feetechCals {
		s.bus.SetCalibration(id, cal)
	}

	s.calibration = calibration

	return nil
}

func (s *SafeSoArmController) GetCalibration() SO101FullCalibration {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.calibration
}

func (s *SafeSoArmController) getCalibrationForServo(servoID int) *feetech.MotorCalibration {
	cal := s.GetCalibration()
	return cal.GetMotorCalibrationByID(servoID)
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

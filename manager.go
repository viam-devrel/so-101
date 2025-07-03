package so_arm

import (
	"fmt"
	"sync"
	"sync/atomic"
)

var (
	globalController   *SafeSoArmController
	controllerMutex    sync.RWMutex
	refCount           int64
	lastError          error
	currentConfig      *SoArm101Config
	currentCalibration SO101FullCalibration
)

// SafeSoArmController wraps the low-level controller with thread-safe access
type SafeSoArmController struct {
	*SoArmController
	mu sync.RWMutex
}

// Thread-safe controller methods

func (s *SafeSoArmController) MoveToJointPositions(jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.SoArmController.MoveToJointPositions(jointAngles, speed, acc)
}

func (s *SafeSoArmController) MoveServosToPositions(servoIDs []int, jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.SoArmController.MoveServosToPositions(servoIDs, jointAngles, speed, acc)
}

func (s *SafeSoArmController) GetJointPositions() ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.SoArmController.GetJointPositions()
}

func (s *SafeSoArmController) GetJointPositionsForServos(servoIDs []int) ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.SoArmController.GetJointPositionsForServos(servoIDs)
}

func (s *SafeSoArmController) SetTorqueEnable(enable bool) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.SoArmController.SetTorqueEnable(enable)
}

func (s *SafeSoArmController) Stop() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.SoArmController.Stop()
}

func (s *SafeSoArmController) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.SoArmController.Close()
}

func (s *SafeSoArmController) Ping() error {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.SoArmController.Ping()
}

func (s *SafeSoArmController) SetCalibration(calibration SO101FullCalibration) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if err := s.SoArmController.SetCalibration(calibration); err != nil {
		return err
	}

	// Update the global calibration
	controllerMutex.Lock()
	currentCalibration = calibration
	controllerMutex.Unlock()

	return nil
}

func (s *SafeSoArmController) GetCalibration() SO101FullCalibration {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.SoArmController.GetCalibration()
}

// Compare configs for compatibility
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

// Compare calibrations for equality
func calibrationsEqual(a, b SO101FullCalibration) bool {
	return a.ShoulderPan == b.ShoulderPan &&
		a.ShoulderLift == b.ShoulderLift &&
		a.ElbowFlex == b.ElbowFlex &&
		a.WristFlex == b.WristFlex &&
		a.WristRoll == b.WristRoll &&
		a.Gripper == b.Gripper
}

// GetSharedController gets a shared controller using default calibration
func GetSharedController(config *SoArm101Config) (*SafeSoArmController, error) {
	return GetSharedControllerWithCalibration(config, DefaultSO101FullCalibration)
}

// GetSharedControllerWithCalibration gets a shared controller with specified calibration
func GetSharedControllerWithCalibration(config *SoArm101Config, calibration SO101FullCalibration) (*SafeSoArmController, error) {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	currentRefCount := atomic.LoadInt64(&refCount)

	if globalController == nil && lastError != nil {
		return nil, fmt.Errorf("cached controller creation error: %w", lastError)
	}

	if globalController != nil {
		if !configsEqual(currentConfig, config) {
			return nil, fmt.Errorf("conflict: existing controller uses different config (refCount: %d)", currentRefCount)
		}

		// Check if calibration is different - if so, update it
		if !calibrationsEqual(currentCalibration, calibration) {
			if config.Logger != nil {
				config.Logger.Info("Updating controller calibration")
			}
			if err := globalController.SoArmController.SetCalibration(calibration); err != nil {
				return nil, fmt.Errorf("failed to update controller calibration: %w", err)
			}
			currentCalibration = calibration
		}

		atomic.AddInt64(&refCount, 1)
		return globalController, nil
	}

	// Always initialize controller with all 6 servos, but let components specify which ones they use
	allServoIDs := []int{1, 2, 3, 4, 5, 6}

	controller, err := NewSoArmController(config.Port, config.Baudrate, allServoIDs, calibration, config.Logger)
	if err != nil {
		lastError = err
		return nil, err
	}

	globalController = &SafeSoArmController{
		SoArmController: controller,
	}
	currentConfig = config
	currentCalibration = calibration
	lastError = nil
	atomic.StoreInt64(&refCount, 1)

	return globalController, nil
}

func ReleaseSharedController() {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	currentRefCount := atomic.AddInt64(&refCount, -1)
	if currentRefCount <= 0 && globalController != nil {
		if err := globalController.Close(); err != nil && currentConfig != nil && currentConfig.Logger != nil {
			currentConfig.Logger.Warnf("error closing shared controller: %v", err)
		}
		globalController = nil
		currentConfig = nil
		currentCalibration = SO101FullCalibration{}
		atomic.StoreInt64(&refCount, 0)
		lastError = nil
	}
}

func ForceCloseSharedController() error {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	var err error
	if globalController != nil {
		err = globalController.Close()
		globalController = nil
		currentConfig = nil
		currentCalibration = SO101FullCalibration{}
		atomic.StoreInt64(&refCount, 0)
		lastError = nil
	}
	return err
}

func GetControllerStatus() (int64, bool, string) {
	controllerMutex.RLock()
	defer controllerMutex.RUnlock()

	currentRefCount := atomic.LoadInt64(&refCount)
	hasController := globalController != nil
	configSummary := ""

	if currentConfig != nil {
		calibrationInfo := "default"
		if currentCalibration.ShoulderPan.HomingOffset != DefaultSO101FullCalibration.ShoulderPan.HomingOffset {
			calibrationInfo = "custom"
		}
		configSummary = fmt.Sprintf("Serial: %s@%d, Calibration: %s",
			currentConfig.Port, currentConfig.Baudrate, calibrationInfo)
	}

	return currentRefCount, hasController, configSummary
}

// GetCurrentCalibration returns the current calibration being used
func GetCurrentCalibration() SO101FullCalibration {
	controllerMutex.RLock()
	defer controllerMutex.RUnlock()
	return currentCalibration
}

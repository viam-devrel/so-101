package so_arm

import (
	"fmt"
	"sync"
	"sync/atomic"
)

var (
	globalController *SafeSoArmController
	controllerMutex  sync.RWMutex
	refCount         int64 // Changed to int64 for atomic operations
	lastError        error
	currentConfig    *SoArm101Config
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

func (s *SafeSoArmController) GetJointPositions() ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.SoArmController.GetJointPositions()
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

// Shared controller manager
func GetSharedController(config *SoArm101Config) (*SafeSoArmController, error) {
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
		atomic.AddInt64(&refCount, 1)
		return globalController, nil
	}

	controller, err := NewSoArmController(config.Port, config.Baudrate, config.ServoIDs, config.Logger)
	if err != nil {
		lastError = err
		return nil, err
	}

	globalController = &SafeSoArmController{
		SoArmController: controller,
	}
	currentConfig = config
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
		atomic.StoreInt64(&refCount, 0)
		lastError = nil
	}
	return err
}

// Fixed function signature to avoid variable shadowing
func GetControllerStatus() (int64, bool, string) {
	controllerMutex.RLock()
	defer controllerMutex.RUnlock()

	currentRefCount := atomic.LoadInt64(&refCount)
	hasController := globalController != nil
	configSummary := ""

	if currentConfig != nil {
		configSummary = fmt.Sprintf("Serial: %s@%d", currentConfig.Port, currentConfig.Baudrate)
	}

	return currentRefCount, hasController, configSummary
}

package so_arm

import (
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo"
	"go.viam.com/rdk/logging"
)

var (
	globalBus           *feetech.Bus
	globalServos        map[int]*feetech.Servo
	controllerMutex     sync.RWMutex
	refCount            int64
	lastError           error
	currentConfig       *SoArm101Config
	currentCalibration  SO101FullCalibration
	feetechCalibrations map[int]*feetech.MotorCalibration
)

// SafeSoArmController wraps the feetech servo bus with thread-safe access
type SafeSoArmController struct {
	bus    *feetech.Bus
	servos map[int]*feetech.Servo
	logger logging.Logger
	mu     sync.RWMutex
}

// Thread-safe controller methods

func (s *SafeSoArmController) MoveToJointPositions(jointAngles []float64, speed, acc int) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Use the first 5 servos for arm movement
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

	// Convert positions to degrees for feetech-servo (it handles calibration internally)
	positions := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		// Convert from radians to degrees
		positions[i] = angle * 180.0 / 3.14159265359
	}

	return s.bus.SyncWritePositions(servos, positions, true) // true = use calibrated/normalized positions
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

	// Convert positions to degrees for feetech-servo
	positions := make([]float64, len(jointAngles))
	for i, angle := range jointAngles {
		positions[i] = angle * 180.0 / 3.14159265359
	}

	return s.bus.SyncWritePositions(servos, positions, true)
}

func (s *SafeSoArmController) GetJointPositions() ([]float64, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	// Get positions for servos 1-6
	servoIDs := []int{1, 2, 3, 4, 5, 6}
	var servos []*feetech.Servo
	for _, id := range servoIDs {
		if servo, exists := s.servos[id]; exists {
			servos = append(servos, servo)
		} else {
			return nil, fmt.Errorf("servo %d not available", id)
		}
	}

	positionMap, err := s.bus.SyncReadPositions(servos, true) // true = use calibrated/normalized positions
	if err != nil {
		return nil, err
	}

	// Convert map to ordered slice and degrees to radians
	positions := make([]float64, len(servoIDs))
	for i, id := range servoIDs {
		if pos, exists := positionMap[id]; exists {
			// Convert from degrees to radians
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

	// Convert map to ordered slice and degrees to radians
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

	// Enable/disable torque for all servos
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

	// Stop all servos by setting velocity to 0
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

func (s *SafeSoArmController) SetCalibration(calibration SO101FullCalibration) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Get feetech calibration map directly
	feetechCals := calibration.ToFeetechCalibrationMap()

	// Update calibrations in the bus
	for id, cal := range feetechCals {
		s.bus.SetCalibration(id, cal)
	}

	// Update global calibration
	controllerMutex.Lock()
	currentCalibration = calibration
	feetechCalibrations = feetechCals
	controllerMutex.Unlock()

	return nil
}

func (s *SafeSoArmController) GetCalibration() SO101FullCalibration {
	s.mu.RLock()
	defer s.mu.RUnlock()

	controllerMutex.RLock()
	defer controllerMutex.RUnlock()
	return currentCalibration
}

// Helper function to get calibration for a specific servo (for backward compatibility)
func (s *SafeSoArmController) getCalibrationForServo(servoID int) *feetech.MotorCalibration {
	cal := s.GetCalibration()
	return cal.GetMotorCalibrationByID(servoID)
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
func fullCalibrationsEqual(a, b SO101FullCalibration) bool {
	return a.Equal(b)
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

	if globalBus == nil && lastError != nil {
		return nil, fmt.Errorf("cached controller creation error: %w", lastError)
	}

	if globalBus != nil {
		if !configsEqual(currentConfig, config) {
			return nil, fmt.Errorf("conflict: existing controller uses different config (refCount: %d)", currentRefCount)
		}

		// Check if calibration is different - if so, update it
		if !fullCalibrationsEqual(currentCalibration, calibration) {
			if config.Logger != nil {
				config.Logger.Info("Updating controller calibration")
			}

			// Apply new calibration directly
			feetechCals := calibration.ToFeetechCalibrationMap()
			for id, cal := range feetechCals {
				globalBus.SetCalibration(id, cal)
			}
			currentCalibration = calibration
			feetechCalibrations = feetechCals
		}

		atomic.AddInt64(&refCount, 1)
		return &SafeSoArmController{
			bus:    globalBus,
			servos: globalServos,
			logger: config.Logger,
		}, nil
	}

	// Create new feetech-servo bus
	feetechCalibrations = calibration.ToFeetechCalibrationMap()

	config.Logger.Info("Calibration map: ", feetechCalibrations)

	busConfig := feetech.BusConfig{
		Port:         config.Port,
		Baudrate:     config.Baudrate,
		Protocol:     feetech.ProtocolV0, // SO-101 uses Protocol 0
		Timeout:      config.Timeout,
		Calibrations: feetechCalibrations,
	}

	if busConfig.Timeout == 0 {
		busConfig.Timeout = time.Second
	}
	if busConfig.Baudrate == 0 {
		busConfig.Baudrate = 1000000
	}

	bus, err := feetech.NewBus(busConfig)
	if err != nil {
		lastError = err
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}

	// Create servo instances for all 6 servos
	servos := make(map[int]*feetech.Servo)
	for id := 1; id <= 6; id++ {
		servo, err := bus.ServoWithModel(id, "sts3215") // SO-101 uses STS3215 servos
		if err != nil {
			bus.Close()
			lastError = err
			return nil, fmt.Errorf("failed to create servo %d: %w", id, err)
		}
		servos[id] = servo
	}

	globalBus = bus
	globalServos = servos
	currentConfig = config
	currentCalibration = calibration
	lastError = nil
	atomic.StoreInt64(&refCount, 1)

	if config.Logger != nil {
		config.Logger.Infof("Created new feetech servo bus with %d servos", len(servos))
	}

	return &SafeSoArmController{
		bus:    bus,
		servos: servos,
		logger: config.Logger,
	}, nil
}

func ReleaseSharedController() {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	currentRefCount := atomic.AddInt64(&refCount, -1)
	if currentRefCount <= 0 && globalBus != nil {
		if err := globalBus.Close(); err != nil && currentConfig != nil && currentConfig.Logger != nil {
			currentConfig.Logger.Warnf("error closing shared controller: %v", err)
		}
		globalBus = nil
		globalServos = nil
		currentConfig = nil
		currentCalibration = SO101FullCalibration{}
		feetechCalibrations = nil
		atomic.StoreInt64(&refCount, 0)
		lastError = nil
	}
}

func ForceCloseSharedController() error {
	controllerMutex.Lock()
	defer controllerMutex.Unlock()

	var err error
	if globalBus != nil {
		err = globalBus.Close()
		globalBus = nil
		globalServos = nil
		currentConfig = nil
		currentCalibration = SO101FullCalibration{}
		feetechCalibrations = nil
		atomic.StoreInt64(&refCount, 0)
		lastError = nil
	}
	return err
}

func GetControllerStatus() (int64, bool, string) {
	controllerMutex.RLock()
	defer controllerMutex.RUnlock()

	currentRefCount := atomic.LoadInt64(&refCount)
	hasController := globalBus != nil
	configSummary := ""

	if currentConfig != nil {
		calibrationInfo := "default"
		if currentCalibration.ShoulderPan != nil &&
			currentCalibration.ShoulderPan.HomingOffset != DefaultSO101FullCalibration.ShoulderPan.HomingOffset {
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

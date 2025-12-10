package so_arm

import (
	"context"
	"fmt"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

type ControllerEntry struct {
	controller  *SafeSoArmController
	config      *SoArm101Config
	calibration SO101FullCalibration
	refCount    int64 // Atomic reference counter
	lastError   error
	mu          sync.RWMutex
}

type ControllerRegistry struct {
	entries map[string]*ControllerEntry // port path -> entry
	mu      sync.RWMutex

	// For backward API compatibility - track which caller uses which port
	callerPorts map[uintptr]string // caller pointer -> port path
	callerMu    sync.RWMutex
}

func NewControllerRegistry() *ControllerRegistry {
	return &ControllerRegistry{
		entries:     make(map[string]*ControllerEntry),
		callerPorts: make(map[uintptr]string),
	}
}

func (r *ControllerRegistry) GetController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*SafeSoArmController, error) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if exists {
		return r.getExistingController(entry, config, calibration, fromFile)
	}

	return r.createNewController(portPath, config, calibration, fromFile)
}

func (r *ControllerRegistry) getExistingController(entry *ControllerEntry, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*SafeSoArmController, error) {
	entry.mu.Lock()
	defer entry.mu.Unlock()

	if entry.controller == nil {
		if entry.lastError != nil {
			return nil, fmt.Errorf("cached controller creation error: %w", entry.lastError)
		}
		return nil, fmt.Errorf("controller not available for port %s", entry.config.Port)
	}

	if !configsEqual(entry.config, config) {
		currentRefCount := atomic.LoadInt64(&entry.refCount)
		return nil, fmt.Errorf("conflict: existing controller uses different config (refCount: %d)", currentRefCount)
	}

	// Only update calibration if it's explicitly provided from a file
	// Skip calibration update when fromFile=false to avoid overwriting with defaults
	if fromFile && !fullCalibrationsEqual(entry.calibration, calibration) {
		if config.Logger != nil {
			config.Logger.Info("Updating controller calibration")
		}

		if entry.controller != nil {
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
				entry.controller.servos[id].calibration = appCal
			}
		}
		entry.calibration = calibration
	}

	atomic.AddInt64(&entry.refCount, 1)
	r.trackCaller(entry.config.Port)

	return &SafeSoArmController{
		bus:          entry.controller.bus,
		armGroup:     entry.controller.armGroup,
		gripperGroup: entry.controller.gripperGroup,
		servos:       entry.controller.servos,
		logger:       config.Logger,
		calibration:  entry.calibration,
	}, nil
}

func (r *ControllerRegistry) createNewController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*SafeSoArmController, error) {
	r.mu.Lock()
	defer r.mu.Unlock()

	if entry, exists := r.entries[portPath]; exists {
		return r.getExistingController(entry, config, calibration, fromFile)
	}

	entry := &ControllerEntry{
		config:      config,
		calibration: calibration,
	}

	feetechCalibrations := calibration.ToFeetechCalibrationMap()

	if config.Logger != nil {
		config.Logger.Info("Calibration map: ", feetechCalibrations)
	}

	busConfig := feetech.BusConfig{
		Port:     config.Port,
		BaudRate: config.Baudrate,
		Protocol: feetech.ProtocolSTS,
		Timeout:  config.Timeout,
	}

	if busConfig.Timeout == 0 {
		busConfig.Timeout = time.Second
	}
	if busConfig.BaudRate == 0 {
		busConfig.BaudRate = 1000000
	}

	bus, err := feetech.NewBus(busConfig)
	if err != nil {
		entry.lastError = err
		r.entries[portPath] = entry
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}

	// Create raw servo instances
	rawServos := make(map[int]*feetech.Servo)
	for id := 1; id <= 6; id++ {
		rawServos[id] = feetech.NewServo(bus, id, &feetech.ModelSTS3215)
	}

	// Create ServoGroups
	armGroup := feetech.NewServoGroup(bus,
		rawServos[1], rawServos[2], rawServos[3], rawServos[4], rawServos[5])
	gripperGroup := feetech.NewServoGroup(bus, rawServos[6])

	// Wrap servos with calibration
	calibratedServos := make(map[int]*CalibratedServo)
	for id := 1; id <= 6; id++ {
		motorCal := calibration.GetMotorCalibrationByID(id)

		// Convert SO101 MotorCalibration to our MotorCalibration type
		appCal := &MotorCalibration{
			ID:           motorCal.ID,
			DriveMode:    motorCal.DriveMode,
			HomingOffset: motorCal.HomingOffset,
			RangeMin:     motorCal.RangeMin,
			RangeMax:     motorCal.RangeMax,
			NormMode:     motorCal.NormMode,
		}

		calibratedServos[id] = NewCalibratedServo(rawServos[id], appCal)
	}

	// If using default calibration (not from file), try reading from servos
	finalCalibration := calibration
	if !fromFile {
		if config.Logger != nil {
			config.Logger.Info("No calibration file loaded, attempting to read from servo registers")
		}
		finalCalibration = ReadCalibrationFromServos(bus, config.ServoIDs, config.Logger)

		// Update calibrated servos with new calibration
		for id := 1; id <= 6; id++ {
			motorCal := finalCalibration.GetMotorCalibrationByID(id)
			appCal := &MotorCalibration{
				ID:           motorCal.ID,
				DriveMode:    motorCal.DriveMode,
				HomingOffset: motorCal.HomingOffset,
				RangeMin:     motorCal.RangeMin,
				RangeMax:     motorCal.RangeMax,
				NormMode:     motorCal.NormMode,
			}
			calibratedServos[id] = NewCalibratedServo(rawServos[id], appCal)
		}
	}

	entry.controller = &SafeSoArmController{
		bus:          bus,
		armGroup:     armGroup,
		gripperGroup: gripperGroup,
		servos:       calibratedServos,
		logger:       config.Logger,
		calibration:  finalCalibration,
	}
	// Update entry calibration after controller creation for consistency
	entry.calibration = finalCalibration
	entry.lastError = nil
	atomic.StoreInt64(&entry.refCount, 1)

	r.entries[portPath] = entry

	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Infof("Created new feetech servo bus with %d servos for port %s", len(calibratedServos), portPath)
	}

	return &SafeSoArmController{
		bus:          bus,
		armGroup:     armGroup,
		gripperGroup: gripperGroup,
		servos:       calibratedServos,
		logger:       config.Logger,
		calibration:  finalCalibration,
	}, nil
}

func (r *ControllerRegistry) ReleaseController(portPath string) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if !exists {
		return
	}

	entry.mu.Lock()
	defer entry.mu.Unlock()

	currentRefCount := atomic.AddInt64(&entry.refCount, -1)
	if currentRefCount <= 0 {
		if entry.controller != nil && entry.controller.bus != nil {
			if err := entry.controller.bus.Close(); err != nil && entry.config != nil && entry.config.Logger != nil {
				entry.config.Logger.Warnf("error closing shared controller for port %s: %v", portPath, err)
			}
		}

		r.mu.Lock()
		delete(r.entries, portPath)
		r.mu.Unlock()

		entry.controller = nil
		entry.config = nil
		entry.calibration = SO101FullCalibration{}
		atomic.StoreInt64(&entry.refCount, 0)
		entry.lastError = nil
	}
}

func (r *ControllerRegistry) ForceCloseController(portPath string) error {
	r.mu.Lock()
	entry, exists := r.entries[portPath]
	if exists {
		delete(r.entries, portPath)
	}
	r.mu.Unlock()

	if !exists {
		return nil
	}

	entry.mu.Lock()
	defer entry.mu.Unlock()

	var err error
	if entry.controller != nil {
		err = entry.controller.bus.Close()
		entry.controller = nil
		entry.config = nil
		entry.calibration = SO101FullCalibration{}
		atomic.StoreInt64(&entry.refCount, 0)
		entry.lastError = nil
	}

	return err
}

func (r *ControllerRegistry) GetControllerStatus(portPath string) (int64, bool, string) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if !exists {
		return 0, false, ""
	}

	entry.mu.RLock()
	defer entry.mu.RUnlock()

	currentRefCount := atomic.LoadInt64(&entry.refCount)
	hasController := entry.controller != nil
	configSummary := ""

	if entry.config != nil {
		calibrationInfo := "default"
		if entry.calibration.ShoulderPan != nil &&
			entry.calibration.ShoulderPan.HomingOffset != DefaultSO101FullCalibration.ShoulderPan.HomingOffset {
			calibrationInfo = "custom"
		}
		configSummary = fmt.Sprintf("Serial: %s@%d, Calibration: %s",
			entry.config.Port, entry.config.Baudrate, calibrationInfo)
	}

	return currentRefCount, hasController, configSummary
}

func (r *ControllerRegistry) GetCurrentCalibration(portPath string) SO101FullCalibration {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if !exists {
		return SO101FullCalibration{}
	}

	entry.mu.RLock()
	defer entry.mu.RUnlock()
	return entry.calibration
}

func (r *ControllerRegistry) trackCaller(portPath string) {
	pc, _, _, ok := runtime.Caller(3) // 3 levels up to get the actual caller
	if !ok {
		return
	}

	r.callerMu.Lock()
	r.callerPorts[pc] = portPath
	r.callerMu.Unlock()
}

func (r *ControllerRegistry) releaseFromCaller() {
	pc, _, _, ok := runtime.Caller(2) // 2 levels up to get the actual caller
	if !ok {
		return
	}

	r.callerMu.RLock()
	portPath, exists := r.callerPorts[pc]
	r.callerMu.RUnlock()

	if exists {
		r.ReleaseController(portPath)

		r.callerMu.Lock()
		delete(r.callerPorts, pc)
		r.callerMu.Unlock()
	}
}

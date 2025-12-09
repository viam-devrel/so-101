package so_arm

import (
	"fmt"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo"
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

		if entry.controller != nil && entry.controller.bus != nil {
			feetechCals := calibration.ToFeetechCalibrationMap()
			for id, cal := range feetechCals {
				entry.controller.bus.SetCalibration(id, cal)
			}
		}
		entry.calibration = calibration
	}

	atomic.AddInt64(&entry.refCount, 1)
	r.trackCaller(entry.config.Port)

	return &SafeSoArmController{
		bus:         entry.controller.bus,
		servos:      entry.controller.servos,
		logger:      config.Logger,
		calibration: entry.calibration,
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
		Port:         config.Port,
		Baudrate:     config.Baudrate,
		Protocol:     feetech.ProtocolV0,
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
		entry.lastError = err
		r.entries[portPath] = entry
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}

	servos := make(map[int]*feetech.Servo)
	for id := 1; id <= 6; id++ {
		servo, err := bus.ServoWithModel(id, "sts3215")
		if err != nil {
			bus.Close()
			entry.lastError = err
			r.entries[portPath] = entry
			return nil, fmt.Errorf("failed to create servo %d: %w", id, err)
		}
		servos[id] = servo
	}

	// If using default calibration (not from file), try reading from servos
	finalCalibration := calibration
	if !fromFile {
		if config.Logger != nil {
			config.Logger.Info("No calibration file loaded, attempting to read from servo registers")
		}
		finalCalibration = ReadCalibrationFromServos(bus, config.ServoIDs, config.Logger)

		// Set calibration on bus for normalization
		feetechCals := finalCalibration.ToFeetechCalibrationMap()
		for id, motorCal := range feetechCals {
			if motorCal != nil {
				bus.SetCalibration(id, motorCal)
			}
		}
	}

	entry.controller = &SafeSoArmController{
		bus:         bus,
		servos:      servos,
		logger:      config.Logger,
		calibration: finalCalibration,
	}
	// Update entry calibration after controller creation for consistency
	entry.calibration = finalCalibration
	entry.lastError = nil
	atomic.StoreInt64(&entry.refCount, 1)

	r.entries[portPath] = entry

	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Infof("Created new feetech servo bus with %d servos for port %s", len(servos), portPath)
	}

	return &SafeSoArmController{
		bus:         bus,
		servos:      servos,
		logger:      config.Logger,
		calibration: finalCalibration,
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

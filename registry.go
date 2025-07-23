package so_arm

import (
	"fmt"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo"
)

// ControllerEntry represents a registry entry for each unique serial port
type ControllerEntry struct {
	controller  *SafeSoArmController
	config      *SoArm101Config
	calibration SO101FullCalibration
	refCount    int64        // Atomic reference counter
	lastError   error        // Cached error state
	mu          sync.RWMutex // Protects this entry
}

// ControllerRegistry manages shared controllers indexed by serial port paths
type ControllerRegistry struct {
	entries map[string]*ControllerEntry // port path -> entry
	mu      sync.RWMutex                // Protects the map

	// For backward API compatibility - track which caller uses which port
	callerPorts map[uintptr]string // caller pointer -> port path
	callerMu    sync.RWMutex       // Protects caller tracking
}

// NewControllerRegistry creates a new controller registry
func NewControllerRegistry() *ControllerRegistry {
	return &ControllerRegistry{
		entries:     make(map[string]*ControllerEntry),
		callerPorts: make(map[uintptr]string),
	}
}

// GetController gets or creates a shared controller for the specified port and config
func (r *ControllerRegistry) GetController(portPath string, config *SoArm101Config, calibration SO101FullCalibration) (*SafeSoArmController, error) {
	// First check if entry exists (read lock)
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if exists {
		return r.getExistingController(entry, config, calibration)
	}

	return r.createNewController(portPath, config, calibration)
}

// getExistingController handles the case where a controller already exists for the port
func (r *ControllerRegistry) getExistingController(entry *ControllerEntry, config *SoArm101Config, calibration SO101FullCalibration) (*SafeSoArmController, error) {
	entry.mu.Lock()
	defer entry.mu.Unlock()

	// Check for cached error or nil controller
	if entry.controller == nil {
		if entry.lastError != nil {
			return nil, fmt.Errorf("cached controller creation error: %w", entry.lastError)
		}
		return nil, fmt.Errorf("controller not available for port %s", entry.config.Port)
	}

	// Verify config compatibility
	if !configsEqual(entry.config, config) {
		currentRefCount := atomic.LoadInt64(&entry.refCount)
		return nil, fmt.Errorf("conflict: existing controller uses different config (refCount: %d)", currentRefCount)
	}

	// Update calibration if different
	if !fullCalibrationsEqual(entry.calibration, calibration) {
		if config.Logger != nil {
			config.Logger.Info("Updating controller calibration")
		}

		// Apply new calibration directly to the bus
		if entry.controller != nil && entry.controller.bus != nil {
			feetechCals := calibration.ToFeetechCalibrationMap()
			for id, cal := range feetechCals {
				entry.controller.bus.SetCalibration(id, cal)
			}
		}
		entry.calibration = calibration
	}

	// Increment reference count and track caller
	atomic.AddInt64(&entry.refCount, 1)
	r.trackCaller(entry.config.Port)

	return &SafeSoArmController{
		bus:         entry.controller.bus,
		servos:      entry.controller.servos,
		logger:      config.Logger,
		calibration: entry.calibration,
	}, nil
}

// createNewController creates a new controller entry for the port
func (r *ControllerRegistry) createNewController(portPath string, config *SoArm101Config, calibration SO101FullCalibration) (*SafeSoArmController, error) {
	// Acquire write lock to create new entry
	r.mu.Lock()
	defer r.mu.Unlock()

	// Double-check in case another goroutine created it
	if entry, exists := r.entries[portPath]; exists {
		return r.getExistingController(entry, config, calibration)
	}

	// Create new entry
	entry := &ControllerEntry{
		config:      config,
		calibration: calibration,
	}

	// Create feetech-servo bus
	feetechCalibrations := calibration.ToFeetechCalibrationMap()

	if config.Logger != nil {
		config.Logger.Info("Calibration map: ", feetechCalibrations)
	}

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
		entry.lastError = err
		r.entries[portPath] = entry
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}

	// Create servo instances for all 6 servos
	servos := make(map[int]*feetech.Servo)
	for id := 1; id <= 6; id++ {
		servo, err := bus.ServoWithModel(id, "sts3215") // SO-101 uses STS3215 servos
		if err != nil {
			bus.Close()
			entry.lastError = err
			r.entries[portPath] = entry
			return nil, fmt.Errorf("failed to create servo %d: %w", id, err)
		}
		servos[id] = servo
	}

	// Initialize the entry
	entry.controller = &SafeSoArmController{
		bus:         bus,
		servos:      servos,
		logger:      config.Logger,
		calibration: calibration,
	}
	entry.lastError = nil
	atomic.StoreInt64(&entry.refCount, 1)

	// Store in registry
	r.entries[portPath] = entry

	// Track caller
	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Infof("Created new feetech servo bus with %d servos for port %s", len(servos), portPath)
	}

	return &SafeSoArmController{
		bus:         bus,
		servos:      servos,
		logger:      config.Logger,
		calibration: calibration,
	}, nil
}

// ReleaseController releases a controller reference for the specified port
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
		// Clean up the controller if it exists
		if entry.controller != nil && entry.controller.bus != nil {
			if err := entry.controller.bus.Close(); err != nil && entry.config != nil && entry.config.Logger != nil {
				entry.config.Logger.Warnf("error closing shared controller for port %s: %v", portPath, err)
			}
		}

		// Remove from registry
		r.mu.Lock()
		delete(r.entries, portPath)
		r.mu.Unlock()

		// Clear entry
		entry.controller = nil
		entry.config = nil
		entry.calibration = SO101FullCalibration{}
		atomic.StoreInt64(&entry.refCount, 0)
		entry.lastError = nil
	}
}

// ForceCloseController forces cleanup of a controller regardless of reference count
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

// GetControllerStatus returns status information for a specific port
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

// GetCurrentCalibration returns the current calibration for a specific port
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

// trackCaller tracks which caller is using which port for backward API compatibility
func (r *ControllerRegistry) trackCaller(portPath string) {
	// Get caller information
	pc, _, _, ok := runtime.Caller(3) // 3 levels up to get the actual caller
	if !ok {
		return
	}

	r.callerMu.Lock()
	r.callerPorts[pc] = portPath
	r.callerMu.Unlock()
}

// releaseFromCaller releases a controller based on caller context
func (r *ControllerRegistry) releaseFromCaller() {
	// Get caller information
	pc, _, _, ok := runtime.Caller(2) // 2 levels up to get the actual caller
	if !ok {
		return
	}

	r.callerMu.RLock()
	portPath, exists := r.callerPorts[pc]
	r.callerMu.RUnlock()

	if exists {
		r.ReleaseController(portPath)

		// Clean up caller tracking
		r.callerMu.Lock()
		delete(r.callerPorts, pc)
		r.callerMu.Unlock()
	}
}

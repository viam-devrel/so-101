package so_arm

import (
	"context"
	"fmt"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// canonicalPortKey resolves a serial-port path to a stable device identity so
// that two spellings of the same physical port — e.g. "/dev/ttyUSB0" and a
// "/dev/serial/by-id/usb-..." symlink pointing at it — map to a single shared
// controller instead of two feetech buses contending for one UART.
//
// It is best-effort: filepath.EvalSymlinks resolves symlinks and cleans the
// path, but if the path can't be resolved (device not present yet, or a
// platform without symlink semantics) the original string is returned so the
// caller still gets a usable key. The result is used as the registry map key
// and for port-equality comparisons; the bus is still opened with the caller's
// original config.Port, so an unresolved fallback preserves today's behavior.
func canonicalPortKey(port string) string {
	if resolved, err := filepath.EvalSymlinks(port); err == nil {
		return resolved
	}
	return port
}

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
	// Collapse different spellings of the same device (e.g. /dev/ttyUSB0 and a
	// /dev/serial/by-id symlink) to one registry key so both consumers share a
	// single bus instead of contending. Only the key is normalized -- the caller's
	// config is left untouched (it may be read concurrently via GetControllerStatus),
	// so configsEqual and trackCaller canonicalize port strings at comparison time.
	portPath = canonicalPortKey(portPath)

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
		configDiff := compareConfigs(entry.config, config)
		return nil, fmt.Errorf("conflict: existing controller uses different config (refCount: %d): %s",
			currentRefCount, configDiff)
	}

	// Only update calibration if it's explicitly provided from a file
	// Skip calibration update when fromFile=false to avoid overwriting with defaults
	if fromFile && !fullCalibrationsEqual(entry.calibration, calibration) {
		if config.Logger != nil {
			config.Logger.Info("Updating controller calibration")
		}

		if entry.controller != nil {
			// Update calibration in each CalibratedServo using thread-safe method
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
				entry.controller.calibratedServos[id].UpdateCalibration(appCal)
			}
		}
		entry.calibration = calibration
	}

	atomic.AddInt64(&entry.refCount, 1)
	// Track against the canonical key so releaseFromCaller can find the entry
	// even when this consumer spelled the port differently than the creator.
	r.trackCaller(canonicalPortKey(entry.config.Port))

	return &SafeSoArmController{
		bus:              entry.controller.bus,
		group:            entry.controller.group,
		calibratedServos: entry.controller.calibratedServos,
		logger:           config.Logger,
		calibration:      entry.calibration,
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
	group := feetech.NewServoGroup(bus,
		rawServos[1], rawServos[2], rawServos[3], rawServos[4], rawServos[5], rawServos[6])

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
		// Use background context for servo reading during controller creation
		ctx := context.Background()
		finalCalibration = ReadCalibrationFromServos(ctx, bus, config.ServoIDs, config.Logger)

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
		bus:              bus,
		group:            group,
		calibratedServos: calibratedServos,
		logger:           config.Logger,
		calibration:      finalCalibration,
	}
	// Update entry calibration after controller creation for consistency
	entry.calibration = finalCalibration
	entry.lastError = nil
	atomic.StoreInt64(&entry.refCount, 1)

	r.entries[portPath] = entry

	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Debugf("Created new feetech servo bus with %d servos for port %s", len(calibratedServos), portPath)
	}

	return &SafeSoArmController{
		bus:              bus,
		group:            group,
		calibratedServos: calibratedServos,
		logger:           config.Logger,
		calibration:      finalCalibration,
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
	portPath = canonicalPortKey(portPath)

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

// compareConfigs returns a string describing the differences between two configs
func compareConfigs(a, b *SoArm101Config) string {
	diffs := []string{}
	if a.Port != b.Port {
		diffs = append(diffs, fmt.Sprintf("port: %s vs %s", a.Port, b.Port))
	}
	if a.Baudrate != b.Baudrate {
		diffs = append(diffs, fmt.Sprintf("baudrate: %d vs %d", a.Baudrate, b.Baudrate))
	}
	if a.Timeout != b.Timeout {
		diffs = append(diffs, fmt.Sprintf("timeout: %v vs %v", a.Timeout, b.Timeout))
	}
	if len(diffs) == 0 {
		return "unknown differences"
	}
	return strings.Join(diffs, ", ")
}

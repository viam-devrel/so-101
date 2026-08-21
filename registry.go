package so_arm

import (
	"context"
	"fmt"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// ControllerEntry is the single owner of one port's bus. Every handle on that port reaches
// the bus through this entry's current session, so all holders share one state.
type ControllerEntry struct {
	config      *SoArm101Config
	calibration SO101FullCalibration // the seed newBusSession wraps servos in
	refCount    int64                // Atomic reference counter
	mu          sync.RWMutex         // guards bookkeeping: config, calibration, refCount

	port     string              // for error messages; no longer only in config
	registry *ControllerRegistry // for eviction at teardown
	logger   logging.Logger      // was reached via entry.config.Logger
	busMu    sync.RWMutex        // serializes bus work; the shared mutex problem 1 needed

	session atomic.Pointer[busSession] // nil == disconnected
	gen     atomic.Uint64              // monotonic session counter
}

// currentCalibration reads the session-rebuild seed.
func (e *ControllerEntry) currentCalibration() SO101FullCalibration {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.calibration
}

type ControllerRegistry struct {
	entries map[string]*ControllerEntry // port path -> entry
	mu      sync.RWMutex

	// For backward API compatibility - track which caller uses which port
	callerPorts map[uintptr]string // caller pointer -> port path
	callerMu    sync.RWMutex

	// busFactory opens a bus. Injectable so tests can simulate an unplug and replug
	// without serial hardware; production always uses feetech.NewBus.
	busFactory func(feetech.BusConfig) (*feetech.Bus, error)
}

func NewControllerRegistry() *ControllerRegistry {
	return &ControllerRegistry{
		entries:     make(map[string]*ControllerEntry),
		callerPorts: make(map[uintptr]string),
		busFactory:  feetech.NewBus,
	}
}

// AcquireController is the registry-scoped entry point. Tests use this one so they can
// drive an injected busFactory; production goes through the package-level delegate.
func (r *ControllerRegistry) AcquireController(
	owner resource.Name, config *SoArm101Config,
	calibration SO101FullCalibration, fromFile bool,
) (*ControllerHandle, error) {
	h, err := r.GetController(config.Port, config, calibration, fromFile)
	if err != nil {
		return nil, err
	}
	h.owner = owner
	// Seam: serial hotplug calls h.entry.resumeReconnect() here.
	//
	// It must be HERE and nowhere deeper. getExistingController holds entry.mu under a
	// defer for its whole body, and createNewController calls it while holding r.mu -- so
	// a seam placed in either would run under a lock that resumeReconnect's tryPin needs.
	return h, nil
}

func (r *ControllerRegistry) GetController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if exists {
		return r.getExistingController(entry, config, calibration, fromFile)
	}

	return r.createNewController(portPath, config, calibration, fromFile)
}

func (r *ControllerRegistry) getExistingController(entry *ControllerEntry, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	entry.mu.Lock()
	defer entry.mu.Unlock()

	if entry.session.Load() == nil {
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

		if sess := entry.session.Load(); sess != nil {
			// One place to write: the per-servo calibrations are the source of truth.
			for id := 1; id <= 6; id++ {
				sess.servos[id].UpdateCalibration(copyMotorCalibration(calibration.GetMotorCalibrationByID(id)))
			}
		}
		entry.calibration = calibration
	}

	atomic.AddInt64(&entry.refCount, 1)
	r.trackCaller(entry.config.Port)

	return &ControllerHandle{entry: entry, logger: config.Logger}, nil
}

func (r *ControllerRegistry) createNewController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	r.mu.Lock()
	defer r.mu.Unlock()

	if entry, exists := r.entries[portPath]; exists {
		return r.getExistingController(entry, config, calibration, fromFile)
	}

	entry := &ControllerEntry{
		config:      config,
		calibration: calibration,
		port:        portPath,
		registry:    r,
		logger:      config.Logger,
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

	bus, err := r.busFactory(busConfig)
	if err != nil {
		// Deliberately no entry: caching the failure here made the port permanently
		// unusable, which discarded viam-server's own 5s resource retry
		// (rdk robot/impl/local_robot.go:427).
		return nil, fmt.Errorf("failed to create feetech servo bus: %w", err)
	}

	// If using default calibration (not from file), try reading from servos. The session is
	// built afterwards so its servos carry whatever calibration was just read.
	finalCalibration := calibration
	if !fromFile {
		if config.Logger != nil {
			config.Logger.Info("No calibration file loaded, attempting to read from servo registers")
		}
		// Use background context for servo reading during controller creation
		ctx := context.Background()
		finalCalibration = ReadCalibrationFromServos(ctx, bus, config.ServoIDs, config.Logger)
	}

	entry.calibration = finalCalibration
	entry.session.Store(newBusSession(bus, finalCalibration, entry.gen.Add(1)))
	atomic.StoreInt64(&entry.refCount, 1)

	r.entries[portPath] = entry

	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Debugf("Created new feetech servo bus for port %s", portPath)
	}

	return &ControllerHandle{entry: entry, logger: config.Logger}, nil
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
		if sess := entry.session.Swap(nil); sess != nil && sess.bus != nil {
			if err := sess.bus.Close(); err != nil && entry.config != nil && entry.config.Logger != nil {
				entry.config.Logger.Warnf("error closing shared controller for port %s: %v", portPath, err)
			}
		}

		r.mu.Lock()
		delete(r.entries, portPath)
		r.mu.Unlock()

		entry.config = nil
		entry.calibration = SO101FullCalibration{}
		atomic.StoreInt64(&entry.refCount, 0)
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
	if sess := entry.session.Swap(nil); sess != nil {
		err = sess.bus.Close()
		entry.config = nil
		entry.calibration = SO101FullCalibration{}
		atomic.StoreInt64(&entry.refCount, 0)
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
	hasController := entry.session.Load() != nil
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

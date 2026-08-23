// Package controller is the ref-counted shared-serial-bus layer for the SO-101: one
// ControllerRegistry owns at most one ControllerEntry per serial port, and every component
// on that port (arm, gripper, calibration sensor) reaches the bus only through a
// ControllerHandle. It depends on internal/servo and internal/geometry, but not on the root
// module package, internal/planning, or internal/servocmd.
package controller

import (
	"context"
	"errors"
	"fmt"
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
	refCount    int                  // live handles; guarded by mu
	torndown    bool                 // once guard; also hides the entry from a new acquire
	mu          sync.RWMutex         // guards bookkeeping: config, calibration, refCount

	port     string              // for error messages; no longer only in config
	registry *ControllerRegistry // for eviction at teardown
	logger   logging.Logger      // was reached via entry.config.Logger
	busMu    sync.RWMutex        // serializes bus work; the shared mutex problem 1 needed

	session atomic.Pointer[busSession] // nil == disconnected
}

// teardownIfLast closes the bus and evicts the entry once no handles are left, at most once.
//
// Takes registry.mu before entry.mu, matching both acquire paths, and evicts in the same
// critical section that sets torndown -- otherwise an acquire could join a dead entry and
// the next one would open a second bus on the same port. The bus is closed after both locks
// are released; in-flight operations finish and later ones get feetech.ErrBusClosed.
func (e *ControllerEntry) teardownIfLast() {
	r := e.registry
	if r == nil {
		return // entries built by hand in tests
	}

	r.mu.Lock()
	e.mu.Lock()
	if e.torndown || e.refCount > 0 {
		e.mu.Unlock()
		r.mu.Unlock()
		return
	}
	e.torndown = true
	sess := e.session.Swap(nil)
	if r.entries[e.port] == e {
		delete(r.entries, e.port)
	}
	e.mu.Unlock()
	r.mu.Unlock()

	if sess != nil && sess.bus != nil {
		if err := sess.bus.Close(); err != nil && e.logger != nil {
			e.logger.Warnf("error closing shared bus for port %s: %v", e.port, err)
		}
	}
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

	// busFactory opens a bus. Injectable so tests can simulate an unplug and replug
	// without serial hardware; production always uses feetech.NewBus.
	busFactory func(feetech.BusConfig) (*feetech.Bus, error)
}

// Option configures a ControllerRegistry at construction. The field it touches
// (ControllerRegistry.busFactory) is unexported, so Option is the injection seam for a
// cross-package caller -- notably a test in another package that needs a fake-backed
// handle, such as the calibration sensor's tests in the root package. It is a function
// type rather than an exported struct field so the registry's internals stay private even
// across that boundary.
type Option func(*ControllerRegistry)

// WithBusFactory overrides how a registry opens a bus, so a caller can substitute a fake
// feetech.Transport (see feetech.BusConfig.Transport) for a real serial port. Production
// never calls this: NewControllerRegistry defaults busFactory to feetech.NewBus, and the
// process-wide registry behind the package-level AcquireController is built with no options.
func WithBusFactory(f func(feetech.BusConfig) (*feetech.Bus, error)) Option {
	return func(r *ControllerRegistry) { r.busFactory = f }
}

// NewControllerRegistry builds an independent registry. Prefer the package-level
// AcquireController: a registry built here does not share the process-wide port
// ownership the shared-bus design depends on, so two of them can open one serial
// port and contend for it. Exported for callers that must inject a bus factory.
func NewControllerRegistry(opts ...Option) *ControllerRegistry {
	r := &ControllerRegistry{
		entries:    make(map[string]*ControllerEntry),
		busFactory: feetech.NewBus,
	}
	for _, opt := range opts {
		opt(r)
	}
	return r
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
	return h, nil
}

// errEntryTornDown means the entry found in the map was already torn down, so the caller
// should build a fresh one rather than join it.
var errEntryTornDown = errors.New("controller entry torn down")

func (r *ControllerRegistry) GetController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if exists {
		h, err := r.getExistingController(entry, config, calibration, fromFile)
		if !errors.Is(err, errEntryTornDown) {
			return h, err
		}
	}

	return r.createNewController(portPath, config, calibration, fromFile)
}

func (r *ControllerRegistry) getExistingController(entry *ControllerEntry, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	entry.mu.Lock()
	defer entry.mu.Unlock()

	// Should be unreachable, since teardownIfLast evicts under the lock that sets this.
	// Kept so a regression surfaces here instead of as a dead handle.
	if entry.torndown {
		return nil, errEntryTornDown
	}

	if entry.session.Load() == nil {
		return nil, fmt.Errorf("controller not available for port %s", entry.config.Port)
	}

	if !configsEqual(entry.config, config) {
		currentRefCount := entry.refCount
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

	entry.refCount++

	return &ControllerHandle{entry: entry, logger: config.Logger}, nil
}

func (r *ControllerRegistry) createNewController(portPath string, config *SoArm101Config, calibration SO101FullCalibration, fromFile bool) (*ControllerHandle, error) {
	r.mu.Lock()
	defer r.mu.Unlock()

	if entry, exists := r.entries[portPath]; exists {
		h, err := r.getExistingController(entry, config, calibration, fromFile)
		if !errors.Is(err, errEntryTornDown) {
			return h, err
		}
		// Torn down under us: fall through and build a fresh entry below.
		delete(r.entries, portPath)
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
	entry.session.Store(newBusSession(bus, finalCalibration))
	entry.refCount = 1

	r.entries[portPath] = entry

	if config.Logger != nil {
		config.Logger.Debugf("Created new feetech servo bus for port %s", portPath)
	}

	return &ControllerHandle{entry: entry, logger: config.Logger}, nil
}

// ReleaseController drops one reference by port path. Delegates the close to
// teardownIfLast rather than doing it here, which is also what keeps the lock order
// consistent -- its old body took entry.mu before r.mu.
func (r *ControllerRegistry) ReleaseController(portPath string) {
	r.mu.RLock()
	entry, exists := r.entries[portPath]
	r.mu.RUnlock()

	if !exists {
		return
	}

	entry.mu.Lock()
	entry.refCount--
	entry.mu.Unlock()

	entry.teardownIfLast()
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
	entry.torndown = true // so a later teardownIfLast cannot close the bus twice
	sess := entry.session.Swap(nil)
	entry.config = nil
	entry.calibration = SO101FullCalibration{}
	entry.refCount = 0
	entry.mu.Unlock()

	var err error
	if sess != nil && sess.bus != nil {
		err = sess.bus.Close()
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

	currentRefCount := entry.refCount
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

	return int64(currentRefCount), hasController, configSummary
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

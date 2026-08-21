package so_arm

import (
	"context"
	"errors"
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
	refCount    int                  // live handles; guarded by mu, never atomics
	pins        int                  // in-flight users that are NOT handles
	torndown    bool                 // once guard; also hides the entry from a new acquire
	mu          sync.RWMutex         // guards bookkeeping: config, calibration, refCount

	port     string              // for error messages; no longer only in config
	registry *ControllerRegistry // for eviction at teardown
	logger   logging.Logger      // was reached via entry.config.Logger
	busMu    sync.RWMutex        // serializes bus work; the shared mutex problem 1 needed

	session atomic.Pointer[busSession] // nil == disconnected
	gen     atomic.Uint64              // monotonic session counter
}

// --- Entry lifecycle ---
//
// refCount counts live handles. pins counts in-flight users that are not handles: serial
// hotplug's reconnect goroutine, and any future mid-flight acquire. Whichever of the two
// reaches zero LAST performs the teardown. Release is idempotent via a per-handle once, so
// the releasing handle structurally cannot come back and finish the job -- the deferred
// close is an entry-level responsibility.
//
// pins ships built, tested, and deliberately unwired: nothing on the acquire path takes one,
// because getExistingController holds entry.mu for its whole body while tryPin locks the
// same mutex. It exists so hotplug's reconnect loop has a correct primitive to attach to.

// tryPin registers a non-handle user of this entry, or refuses once teardown has begun.
// The refusal is what keeps hotplug from starting a reconnect loop for a dying entry.
func (e *ControllerEntry) tryPin() bool {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.torndown {
		return false
	}
	e.pins++
	return true
}

// unpin drops a pin and tears the entry down if it was the last user.
func (e *ControllerEntry) unpin() {
	e.mu.Lock()
	e.pins--
	e.mu.Unlock()
	e.teardownIfLast()
}

// teardownIfLast closes the bus and evicts the entry, at most once.
//
// It takes registry.mu BEFORE entry.mu and evicts inside the same critical section that sets
// torndown. Both halves matter. Taking entry.mu first would invert the order against
// GetController, which holds r.mu and then entry.mu. And evicting after releasing the locks
// would leave a window where GetController finds a dead entry, raises its refCount, and
// hands back a permanently disconnected handle -- after which the eviction lands and the
// next acquire opens a SECOND bus on the same port.
//
// The bus is closed only after both locks are released: Close can block, and nothing else
// needs to wait behind it. In-flight bus ops are safe, since feetech.Bus.Close sets its
// closed flag under its own mutex -- they finish and later ones get ErrBusClosed.
func (e *ControllerEntry) teardownIfLast() {
	r := e.registry
	if r == nil {
		return // entries built by hand in tests
	}

	r.mu.Lock()
	e.mu.Lock()
	if e.torndown || e.refCount > 0 || e.pins > 0 {
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

	// Belt-and-braces: teardownIfLast evicts under the same lock that sets torndown, so
	// this should be unreachable. Keeping the check means a future refactor that reopens
	// that window fails loudly here instead of handing back a dead handle.
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
	r.trackCaller(entry.config.Port)

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
	entry.session.Store(newBusSession(bus, finalCalibration, entry.gen.Add(1)))
	entry.refCount = 1

	r.entries[portPath] = entry

	r.trackCaller(portPath)

	if config.Logger != nil {
		config.Logger.Debugf("Created new feetech servo bus for port %s", portPath)
	}

	return &ControllerHandle{entry: entry, logger: config.Logger}, nil
}

// ReleaseController drops one reference by port path.
//
// It delegates to teardownIfLast rather than closing the bus itself. Its old body held
// entry.mu and then r.mu -- the only reversed lock order in this package, against the
// r.mu-then-entry.mu that both acquire paths and teardownIfLast use. Keeping that body,
// even as a compatibility shim, would have been an ABBA deadlock waiting to happen.
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

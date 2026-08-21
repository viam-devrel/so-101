package so_arm

import (
	"errors"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
)

// Mock logger for testing
func testLogger() logging.Logger {
	return logging.NewLogger("registry-test")
}

// Test configuration factory
func testConfig(port string) *SoArm101Config {
	return &SoArm101Config{
		Port:     port,
		Baudrate: 1000000,
		ServoIDs: []int{1, 2, 3, 4, 5, 6},
		Timeout:  time.Second,
		Logger:   testLogger(),
	}
}

// TestRegistryCreation tests basic registry creation and initialization
func TestRegistryCreation(t *testing.T) {
	registry := NewControllerRegistry()

	if registry == nil {
		t.Fatal("NewControllerRegistry returned nil")
	}

	if registry.entries == nil {
		t.Fatal("Registry entries map not initialized")
	}

	if registry.callerPorts == nil {
		t.Fatal("Registry callerPorts map not initialized")
	}

	if len(registry.entries) != 0 {
		t.Fatal("Registry should start empty")
	}
}

// TestSingleControllerAccess tests basic controller access for a single port
func TestSingleControllerAccess(t *testing.T) {
	registry := NewControllerRegistry()
	config := testConfig("/dev/ttyUSB0")
	calibration := DefaultSO101FullCalibration

	// Skip this test if we can't create actual hardware connections
	// This is a unit test that should work without hardware
	t.Skip("Skipping hardware-dependent test")

	controller, err := registry.GetController(config.Port, config, calibration, false)
	if err != nil {
		t.Fatalf("Failed to get controller: %v", err)
	}

	if controller == nil {
		t.Fatal("Controller should not be nil")
	}

	// Verify registry state
	registry.mu.RLock()
	if len(registry.entries) != 1 {
		t.Fatalf("Expected 1 registry entry, got %d", len(registry.entries))
	}

	entry, exists := registry.entries[config.Port]
	if !exists {
		t.Fatal("Registry entry not found for port")
	}

	refCount := entry.refCount
	if refCount != 1 {
		t.Fatalf("Expected refCount 1, got %d", refCount)
	}
	registry.mu.RUnlock()

	// Release controller
	registry.ReleaseController(config.Port)

	// Verify cleanup
	registry.mu.RLock()
	if len(registry.entries) != 0 {
		t.Fatalf("Expected 0 registry entries after release, got %d", len(registry.entries))
	}
	registry.mu.RUnlock()
}

// TestMultiplePortsAccess tests concurrent access to different ports
func TestMultiplePortsAccess(t *testing.T) {
	registry := NewControllerRegistry()

	ports := []string{"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}
	var wg sync.WaitGroup
	var successCount int64

	// Try to get controllers for different ports concurrently
	for _, port := range ports {
		wg.Add(1)
		go func(p string) {
			defer wg.Done()

			config := testConfig(p)
			calibration := DefaultSO101FullCalibration

			// This will likely fail due to hardware, but we're testing the registry logic
			_, err := registry.GetController(p, config, calibration, false)
			if err == nil {
				atomic.AddInt64(&successCount, 1)
			}
			// Don't fail the test for hardware errors - we're testing registry logic
		}(port)
	}

	wg.Wait()

	// Verify registry can handle multiple ports (even if hardware fails)
	registry.mu.RLock()
	entriesCount := len(registry.entries)
	registry.mu.RUnlock()

	// Registry should have attempted to create entries for each port
	// Some may have failed due to hardware, but the structure should be there
	if entriesCount != len(ports) {
		t.Logf("Expected %d registry entries, got %d (some may have failed due to hardware)", len(ports), entriesCount)
	}
}

// TestSharedAccess tests multiple access to the same port
func TestSharedAccess(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/ttyUSB0"
	config := testConfig(port)

	// Skip hardware tests - focus on registry logic only
	t.Skip("Skipping hardware-dependent shared access test")

	const numGoroutines = 5
	var wg sync.WaitGroup
	var errorCount int64

	// Multiple goroutines try to get the same controller
	for i := 0; i < numGoroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()

			// This will fail due to hardware, but tests concurrent access
			_, err := registry.GetController(port, config, DefaultSO101FullCalibration, false)
			if err != nil {
				atomic.AddInt64(&errorCount, 1)
			}
		}()
	}

	wg.Wait()

	// All should fail due to hardware, but registry should handle concurrent access
	if errorCount != numGoroutines {
		t.Logf("Expected all %d attempts to fail due to hardware, got %d errors", numGoroutines, errorCount)
	}
}

// TestReferenceCountingLogic tests reference counting without hardware
func TestReferenceCountingLogic(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/ttyUSB0"
	config := testConfig(port)

	// Create a mock entry
	entry := &ControllerEntry{
		config:      config,
		calibration: DefaultSO101FullCalibration,
		port:        port,
		registry:    registry,
		refCount:    3, // Start with 3 references
	}
	registry.entries[port] = entry

	// Test decrement
	initialCount := entry.refCount
	if initialCount != 3 {
		t.Fatalf("Expected initial refCount 3, got %d", initialCount)
	}

	// Simulate releases
	entry.refCount--
	count1 := entry.refCount
	if count1 != 2 {
		t.Fatalf("Expected refCount 2 after first release, got %d", count1)
	}

	entry.refCount--
	count2 := entry.refCount
	if count2 != 1 {
		t.Fatalf("Expected refCount 1 after second release, got %d", count2)
	}

	entry.refCount--
	count3 := entry.refCount
	if count3 != 0 {
		t.Fatalf("Expected refCount 0 after third release, got %d", count3)
	}
}

// TestCleanupOnZeroRefs tests cleanup when reference count reaches zero
func TestCleanupOnZeroRefs(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/ttyUSB0"
	config := testConfig(port)

	// Create a mock entry with 1 reference - simulate a failed creation with error
	entry := &ControllerEntry{
		config:      config,
		calibration: DefaultSO101FullCalibration,
		port:        port,
		registry:    registry,
		refCount:    1,
	}
	registry.entries[port] = entry

	// Verify entry exists
	registry.mu.RLock()
	if len(registry.entries) != 1 {
		t.Fatalf("Expected 1 registry entry, got %d", len(registry.entries))
	}
	registry.mu.RUnlock()

	// Release the controller
	registry.ReleaseController(port)

	// Verify cleanup occurred
	registry.mu.RLock()
	if len(registry.entries) != 0 {
		t.Fatalf("Expected 0 registry entries after cleanup, got %d", len(registry.entries))
	}
	registry.mu.RUnlock()
}

// TestForceCloseController tests force closing controllers
func TestForceCloseController(t *testing.T) {
	registry := NewControllerRegistry()
	ports := []string{"/dev/ttyUSB0", "/dev/ttyUSB1"}

	// Create mock entries
	for _, port := range ports {
		config := testConfig(port)
		entry := &ControllerEntry{
			config:      config,
			calibration: DefaultSO101FullCalibration,
			port:        port,
			registry:    registry,
			refCount:    2, // Multiple references
		}
		registry.entries[port] = entry
	}

	// Verify entries exist
	registry.mu.RLock()
	if len(registry.entries) != 2 {
		t.Fatalf("Expected 2 registry entries, got %d", len(registry.entries))
	}
	registry.mu.RUnlock()

	// Force close one controller
	err := registry.ForceCloseController(ports[0])
	if err != nil {
		t.Fatalf("ForceCloseController failed: %v", err)
	}

	// Verify one entry was removed
	registry.mu.RLock()
	if len(registry.entries) != 1 {
		t.Fatalf("Expected 1 registry entry after force close, got %d", len(registry.entries))
	}

	// Verify the correct entry remains
	if _, exists := registry.entries[ports[1]]; !exists {
		t.Fatal("Wrong entry was removed")
	}
	registry.mu.RUnlock()
}

// TestConfigCompatibility tests configuration compatibility checking
func TestConfigCompatibility(t *testing.T) {
	config1 := testConfig("/dev/ttyUSB0")
	config2 := testConfig("/dev/ttyUSB0")
	config3 := testConfig("/dev/ttyUSB1") // Different port

	config2.Baudrate = 9600 // Different baudrate

	// Test equal configs
	if !configsEqual(config1, config1) {
		t.Fatal("Same config should be equal")
	}

	// Test different configs (same port, different settings)
	if configsEqual(config1, config2) {
		t.Fatal("Different configs should not be equal")
	}

	// Test different ports
	if configsEqual(config1, config3) {
		t.Fatal("Different port configs should not be equal")
	}

	// Test nil configs
	if !configsEqual(nil, nil) {
		t.Fatal("Both nil configs should be equal")
	}

	if configsEqual(config1, nil) {
		t.Fatal("Config and nil should not be equal")
	}
}

// TestCalibrationEquality tests calibration comparison
func TestCalibrationEquality(t *testing.T) {
	cal1 := DefaultSO101FullCalibration
	cal2 := DefaultSO101FullCalibration

	// Test equal calibrations
	if !fullCalibrationsEqual(cal1, cal2) {
		t.Fatal("Same calibrations should be equal")
	}

	// Test different calibrations - make a proper copy
	cal3 := DefaultSO101FullCalibration
	// Create a new ShoulderPan to avoid modifying the shared pointer
	newShoulderPan := *DefaultSO101FullCalibration.ShoulderPan
	newShoulderPan.HomingOffset = 100 // Different offset
	cal3.ShoulderPan = &newShoulderPan

	if fullCalibrationsEqual(cal1, cal3) {
		t.Fatal("Different calibrations should not be equal")
	}
}

// TestGetControllerStatus tests status reporting
func TestGetControllerStatus(t *testing.T) {
	registry := NewControllerRegistry()

	// Test empty registry
	refCount, hasController, summary := registry.GetControllerStatus("/dev/ttyUSB0")
	if refCount != 0 || hasController != false || summary != "" {
		t.Fatal("Empty registry should return zero values")
	}

	// Add a mock entry
	port := "/dev/ttyUSB0"
	config := testConfig(port)
	entry := &ControllerEntry{
		config:      config,
		calibration: DefaultSO101FullCalibration,
		port:        port,
		registry:    registry,
		refCount:    2,
	}
	registry.entries[port] = entry

	refCount, hasController, summary = registry.GetControllerStatus(port)
	if refCount != 2 {
		t.Fatalf("Expected refCount 2, got %d", refCount)
	}
	if hasController != false { // No actual controller
		t.Fatal("Expected hasController false")
	}
	if summary == "" {
		t.Fatal("Expected non-empty summary")
	}
}

// TestConcurrentRegistryAccess tests thread safety
func TestConcurrentRegistryAccess(t *testing.T) {
	registry := NewControllerRegistry()
	const numGoroutines = 10
	const numOperations = 100

	var wg sync.WaitGroup

	// Multiple goroutines performing registry operations concurrently
	for i := 0; i < numGoroutines; i++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()

			port := "/dev/ttyUSB0"
			config := testConfig(port)

			for j := 0; j < numOperations; j++ {
				// Try various registry operations (they will fail due to hardware, but test thread safety)
				registry.GetController(port, config, DefaultSO101FullCalibration, false)
				registry.GetControllerStatus(port)
				registry.GetCurrentCalibration(port)

				// Add small delay to increase chance of race conditions
				time.Sleep(time.Microsecond)
			}
		}(i)
	}

	wg.Wait()

	// Test should complete without race conditions or panics
	t.Log("Concurrent access test completed successfully")
}

// TestControllerUsesServoCalibrationWhenNoFile tests servo calibration fallback integration
func TestControllerUsesServoCalibrationWhenNoFile(t *testing.T) {
	// This would require hardware or extensive mocking
	// We'll verify the integration manually and via existing tests
	// The key is ensuring the code path is correct

	t.Skip("Integration test - requires hardware or mock bus setup")
}

// shortTimeoutConfig narrows testConfig's one-second timeout so a test that expects a
// failed read does not wait a full second per servo.
func shortTimeoutConfig(port string) *SoArm101Config {
	cfg := testConfig(port)
	cfg.Timeout = 50 * time.Millisecond
	return cfg
}

func TestRegistryUsesInjectedBusFactory(t *testing.T) {
	r := NewControllerRegistry()
	var calls int
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		calls++
		cfg.Transport = newFakeTransport()
		return feetech.NewBus(cfg)
	}

	_, err := r.GetController("/dev/fake0", shortTimeoutConfig("/dev/fake0"),
		DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	assert.Equal(t, 1, calls, "the registry must open buses through the factory")
}

func TestFailedOpenLeavesNoPoisonedEntry(t *testing.T) {
	r := NewControllerRegistry()
	var attempts int
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		attempts++
		if attempts == 1 {
			return nil, errors.New("no such device")
		}
		cfg.Transport = newFakeTransport()
		return feetech.NewBus(cfg)
	}
	cfg := shortTimeoutConfig("/dev/fake0")

	_, err := r.GetController("/dev/fake0", cfg, DefaultSO101FullCalibration, true)
	require.Error(t, err)

	r.mu.RLock()
	_, cached := r.entries["/dev/fake0"]
	r.mu.RUnlock()
	assert.False(t, cached, "a failed open must leave no entry for the retry to trip over")

	_, err = r.GetController("/dev/fake0", cfg, DefaultSO101FullCalibration, true)
	require.NoError(t, err, "the second attempt must reopen, not replay a cached error")
	assert.Equal(t, 2, attempts)
}

func TestReleaseClosesTheBusOnlyAfterTheLastHolder(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)
	entry := h1.entry

	h1.Release()
	assert.NotNil(t, entry.session.Load(), "one holder left; the bus must stay open")
	assert.Equal(t, 0, ft.closeCount())

	h2.Release()
	r.mu.RLock()
	_, present := r.entries["/dev/fake0"]
	r.mu.RUnlock()
	assert.False(t, present, "the last release must evict the entry")
	assert.Equal(t, 1, ft.closeCount())
}

func TestReleaseIsIdempotent(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)
	h.Release()
	h.Release() // a double Close must not double-decrement into a negative refCount
	assert.Equal(t, 1, ft.closeCount())

	h2 := acquireTestHandle(t, r)
	assert.NotNil(t, h2.entry.session.Load(), "a fresh acquire must open a fresh entry")
}

func TestTryPinRefusesOnceTeardownHasBegun(t *testing.T) {
	ft := newFakeTransport()
	_, h := testRegistryAndHandle(t, ft)
	entry := h.entry

	h.Release() // last holder: teardown runs
	assert.False(t, entry.tryPin(), "a pin after teardown must be refused, not granted")
}

func TestTeardownRunsExactlyOnce(t *testing.T) {
	ft := newFakeTransport()
	_, h := testRegistryAndHandle(t, ft)
	entry := h.entry

	h.Release()
	entry.teardownIfLast() // a redundant call must be inert
	entry.teardownIfLast()
	assert.Equal(t, 1, ft.closeCount())
}

func TestAPinKeepsTheEntryAliveAcrossTheLastRelease(t *testing.T) {
	ft := newFakeTransport()
	_, h := testRegistryAndHandle(t, ft)
	entry := h.entry

	require.True(t, entry.tryPin(), "a pin on a live entry must be granted")

	h.Release()
	assert.Equal(t, 0, ft.closeCount(),
		"the pin holder is still using the entry; teardown must wait for it")
	assert.NotNil(t, entry.session.Load())

	entry.unpin()
	assert.Equal(t, 1, ft.closeCount(), "the last decrement performs the teardown")
}

func TestAcquireNeverJoinsATornDownEntry(t *testing.T) {
	ft := newFakeTransport()
	r, h := testRegistryAndHandle(t, ft)
	torn := h.entry

	h.Release()
	h2 := acquireTestHandle(t, r)
	assert.NotSame(t, torn, h2.entry,
		"joining a torn-down entry would hand back a permanently dead handle")
	assert.NotNil(t, h2.entry.session.Load())
}

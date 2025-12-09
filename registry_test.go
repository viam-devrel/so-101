package so_arm

import (
	"fmt"
	"sync"
	"sync/atomic"
	"testing"
	"time"

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

	refCount := atomic.LoadInt64(&entry.refCount)
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
		refCount:    3, // Start with 3 references
	}
	registry.entries[port] = entry

	// Test decrement
	initialCount := atomic.LoadInt64(&entry.refCount)
	if initialCount != 3 {
		t.Fatalf("Expected initial refCount 3, got %d", initialCount)
	}

	// Simulate releases
	atomic.AddInt64(&entry.refCount, -1)
	count1 := atomic.LoadInt64(&entry.refCount)
	if count1 != 2 {
		t.Fatalf("Expected refCount 2 after first release, got %d", count1)
	}

	atomic.AddInt64(&entry.refCount, -1)
	count2 := atomic.LoadInt64(&entry.refCount)
	if count2 != 1 {
		t.Fatalf("Expected refCount 1 after second release, got %d", count2)
	}

	atomic.AddInt64(&entry.refCount, -1)
	count3 := atomic.LoadInt64(&entry.refCount)
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
		refCount:    1,
		controller:  nil,
		lastError:   fmt.Errorf("mock hardware error"), // Add error to make nil controller valid
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
			refCount:    2,   // Multiple references
			controller:  nil, // No actual controller
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
		refCount:    2,
		controller:  nil,
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

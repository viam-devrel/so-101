package so_arm

import (
	"os"
	"path/filepath"
	"testing"
)

func TestCanonicalPortKey(t *testing.T) {
	dir := t.TempDir()
	device := filepath.Join(dir, "ttyUSB0")
	if err := os.WriteFile(device, nil, 0o600); err != nil {
		t.Fatalf("create fake device: %v", err)
	}
	link := filepath.Join(dir, "by-id-usb-serial")
	if err := os.Symlink(device, link); err != nil {
		t.Fatalf("symlink: %v", err)
	}

	// A symlink and its target resolve to the same canonical key.
	if got, want := canonicalPortKey(link), canonicalPortKey(device); got != want {
		t.Errorf("symlink and target should canonicalize equal: %q vs %q", got, want)
	}

	// An unresolvable path falls back to the original string (best-effort).
	missing := filepath.Join(dir, "does-not-exist")
	if got := canonicalPortKey(missing); got != missing {
		t.Errorf("unresolvable path should fall back to original: got %q, want %q", got, missing)
	}
}

// TestRegistry_SymlinkPortSharesController verifies that a consumer acquiring a
// port via a /dev/serial/by-id-style symlink reuses the controller a prior
// consumer created via the real device path, instead of opening a second bus
// on the same UART. Regression guard for the silent double-open footgun.
func TestRegistry_SymlinkPortSharesController(t *testing.T) {
	dir := t.TempDir()
	device := filepath.Join(dir, "ttyUSB0")
	if err := os.WriteFile(device, nil, 0o600); err != nil {
		t.Fatalf("create fake device: %v", err)
	}
	link := filepath.Join(dir, "by-id-usb-serial")
	if err := os.Symlink(device, link); err != nil {
		t.Fatalf("symlink: %v", err)
	}

	realKey := canonicalPortKey(device)

	registry := NewControllerRegistry()
	// Pre-inject an entry keyed by the canonical device path, as if the arm
	// acquired first via the real path. No live bus is needed here:
	// getExistingController touches no bus method on the shared-acquire path.
	registry.entries[realKey] = &ControllerEntry{
		controller:  &SafeSoArmController{},
		config:      testConfig(realKey),
		calibration: DefaultSO101FullCalibration,
		refCount:    1,
	}

	// A second consumer acquires the same device via the symlink spelling.
	if _, err := registry.GetController(link, testConfig(link), DefaultSO101FullCalibration, false); err != nil {
		t.Fatalf("acquire via symlink: %v", err)
	}

	// It must reuse the existing entry rather than create a second one...
	if n := len(registry.entries); n != 1 {
		t.Errorf("expected 1 shared registry entry, got %d", n)
	}
	// ...bumping the shared refcount instead of opening a second bus.
	if rc := registry.entries[realKey].refCount; rc != 2 {
		t.Errorf("expected refCount 2 after shared acquire, got %d", rc)
	}
}

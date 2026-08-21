package so_arm

import (
	"syscall"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
)

// testRegistry returns a registry whose bus factory opens every bus over `ft` -- the SAME
// fake the test holds.
//
// This is the most important detail in these tests. A factory that creates a fresh fake per
// call would make every reconnect test pass vacuously: a reopen would get a brand-new
// healthy transport no matter what ft.unplug() did.
//
// The factory also FAILS THE OPEN while the fake is unplugged. That models production, where
// serial.Open on a vanished device node errors -- feetech.NewBus with a non-nil Transport
// never fails on its own, so without this a fresh acquire during an outage would publish a
// live session over a dead port, which cannot happen with real hardware.
func testRegistry(ft *fakeTransport) *ControllerRegistry {
	r := NewControllerRegistry()
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		ft.noteOpen()
		if ft.isUnplugged() {
			return nil, syscall.ENXIO
		}
		cfg.Transport = ft
		return feetech.NewBus(cfg)
	}
	return r
}

// acquireTestHandle acquires one handle on /dev/fake0 and releases it at cleanup.
func acquireTestHandle(t *testing.T, r *ControllerRegistry) *ControllerHandle {
	t.Helper()
	owner := arm.Named("test-arm-" + t.Name())
	h, err := r.AcquireController(owner, shortTimeoutConfig("/dev/fake0"),
		DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	t.Cleanup(h.Release)
	return h
}

func testRegistryAndHandle(t *testing.T, ft *fakeTransport) (*ControllerRegistry, *ControllerHandle) {
	t.Helper()
	r := testRegistry(ft)
	return r, acquireTestHandle(t, r)
}

func testHandle(t *testing.T, ft *fakeTransport) *ControllerHandle {
	t.Helper()
	_, h := testRegistryAndHandle(t, ft)
	return h
}

// cloneCalibration deep-copies a calibration. SO101FullCalibration's six fields are all
// *MotorCalibration (config.go:33-40), so `cal := DefaultSO101FullCalibration` shares every
// pointer with the package-level default -- mutating the copy corrupts that default for
// every later test in the process, including TestGetControllerStatus, which branches on
// DefaultSO101FullCalibration.ShoulderPan.HomingOffset.
func cloneCalibration(src SO101FullCalibration) SO101FullCalibration {
	return SO101FullCalibration{
		ShoulderPan:  copyMotorCalibration(src.ShoulderPan),
		ShoulderLift: copyMotorCalibration(src.ShoulderLift),
		ElbowFlex:    copyMotorCalibration(src.ElbowFlex),
		WristFlex:    copyMotorCalibration(src.WristFlex),
		WristRoll:    copyMotorCalibration(src.WristRoll),
		Gripper:      copyMotorCalibration(src.Gripper),
	}
}

func TestWithSessionRunsAgainstTheLiveSession(t *testing.T) {
	h := testHandle(t, newFakeTransport())

	var seen *busSession
	require.NoError(t, h.withSession(func(s *busSession) error {
		seen = s
		return nil
	}))
	require.NotNil(t, seen)
	assert.NotNil(t, seen.bus)
	assert.Len(t, seen.servos, 6)
}

func TestWithSessionFailsFastWhenDisconnected(t *testing.T) {
	h := testHandle(t, newFakeTransport())
	h.entry.session.Store(nil)

	start := time.Now()
	err := h.withSession(func(*busSession) error {
		t.Fatal("the operation must not run without a session")
		return nil
	})
	assert.ErrorIs(t, err, ErrPortDisconnected)
	assert.Less(t, time.Since(start), 10*time.Millisecond,
		"a disconnected call must not pay a serial timeout")
}

func TestAllHoldersShareOneControllerState(t *testing.T) {
	ft := newFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)

	require.Same(t, h1.entry, h2.entry, "one port must mean one entry")
	assert.Same(t, h1.entry.session.Load(), h2.entry.session.Load(),
		"both holders must see the same session -- problem 1 was each getting a private copy")
}

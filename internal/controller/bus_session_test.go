package controller

import (
	"syscall"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"

	"so_arm/internal/testfake"
)

// testRegistry returns a registry whose bus factory opens every bus over the caller's fake,
// and fails the open while that fake is unplugged.
//
// Both details matter: a factory handing out a fresh fake per call would let a test pass
// against a transport it never touched, and feetech.NewBus with a non-nil Transport never
// fails on its own, so without the unplugged check a test could open a working bus over a
// port that is supposed to be gone.
func testRegistry(ft *testfake.FakeTransport) *ControllerRegistry {
	r := NewControllerRegistry()
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		ft.NoteOpen()
		if ft.IsUnplugged() {
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

func testRegistryAndHandle(t *testing.T, ft *testfake.FakeTransport) (*ControllerRegistry, *ControllerHandle) {
	t.Helper()
	r := testRegistry(ft)
	return r, acquireTestHandle(t, r)
}

func testHandle(t *testing.T, ft *testfake.FakeTransport) *ControllerHandle {
	t.Helper()
	_, h := testRegistryAndHandle(t, ft)
	return h
}

// cloneCalibration deep-copies a calibration. The six fields are all pointers, so a plain
// struct copy of DefaultSO101FullCalibration shares them with the package-level default and
// mutating it corrupts every later test in the process.
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
	h := testHandle(t, testfake.NewFakeTransport())

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
	h := testHandle(t, testfake.NewFakeTransport())
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
	ft := testfake.NewFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)

	require.Same(t, h1.entry, h2.entry, "one port must mean one entry")
	assert.Same(t, h1.entry.session.Load(), h2.entry.session.Load(),
		"both holders must see the same session -- problem 1 was each getting a private copy")
}

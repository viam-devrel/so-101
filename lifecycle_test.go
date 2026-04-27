package so_arm

import (
	"context"
	"errors"
	"sync/atomic"
	"testing"
)

// TestController_PostCloseReturnsSentinel verifies that every gated controller
// method returns ErrControllerClosed when the controller's closed flag is set,
// rather than panicking or hitting the (closed) bus. Table-driven so adding a
// new method to the gated list without an entry here is a visible omission.
func TestController_PostCloseReturnsSentinel(t *testing.T) {
	ctx := context.Background()
	cases := []struct {
		name string
		call func(*SafeSoArmController) error
	}{
		{"Ping", func(c *SafeSoArmController) error { return c.Ping(ctx) }},
		{"SetTorqueEnable", func(c *SafeSoArmController) error { return c.SetTorqueEnable(ctx, true) }},
		{"Stop", func(c *SafeSoArmController) error { return c.Stop(ctx) }},
		{"MoveToJointPositions", func(c *SafeSoArmController) error {
			return c.MoveToJointPositions(ctx, []float64{0, 0, 0, 0, 0}, 0, 0)
		}},
		{"MoveServosToPositions", func(c *SafeSoArmController) error {
			return c.MoveServosToPositions(ctx, []int{1}, []float64{0}, 0, 0)
		}},
		{"WriteServoRegister", func(c *SafeSoArmController) error {
			return c.WriteServoRegister(ctx, 1, "goal_position", []byte{0, 0})
		}},
		{"SetCalibration", func(c *SafeSoArmController) error {
			return c.SetCalibration(SO101FullCalibration{})
		}},
		{"GetJointPositions", func(c *SafeSoArmController) error {
			_, err := c.GetJointPositions(ctx)
			return err
		}},
		{"GetJointPositionsForServos", func(c *SafeSoArmController) error {
			_, err := c.GetJointPositionsForServos(ctx, []int{1})
			return err
		}},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			bus, _ := newMockBus(t)
			ctrl := &SafeSoArmController{
				bus:    bus,
				logger: newTestLogger(t),
			}
			ctrl.closed.Store(true)

			if err := tc.call(ctrl); !errors.Is(err, ErrControllerClosed) {
				t.Errorf("%s after close: expected ErrControllerClosed, got %v", tc.name, err)
			}
		})
	}
}

// TestRegistry_SamePointerForSamePort verifies that two callers acquiring a
// controller for the same port receive the *same* *SafeSoArmController, so
// that close-state propagates correctly across all consumers.
func TestRegistry_SamePointerForSamePort(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/test-port"
	cfg := testConfig(port)

	// Inject a pre-built entry so we don't need a real bus.
	bus, _ := newMockBus(t)
	ctrl := &SafeSoArmController{
		bus:    bus,
		logger: cfg.Logger,
	}
	registry.entries[port] = &ControllerEntry{
		controller:  ctrl,
		config:      cfg,
		calibration: DefaultSO101FullCalibration,
		refCount:    0,
	}

	first, err := registry.GetController(port, cfg, DefaultSO101FullCalibration, false)
	if err != nil {
		t.Fatalf("first GetController: %v", err)
	}
	second, err := registry.GetController(port, cfg, DefaultSO101FullCalibration, false)
	if err != nil {
		t.Fatalf("second GetController: %v", err)
	}

	if first != second {
		t.Errorf("expected same pointer for same port; got %p and %p", first, second)
	}
	if first != ctrl {
		t.Errorf("expected cached controller pointer to be returned")
	}
}

// TestRegistry_ReleaseClosesAllConsumers verifies that ReleaseController
// at refcount zero closes the bus and sets the closed flag on the shared
// controller, so other holders observe ErrControllerClosed on next call.
func TestRegistry_ReleaseClosesAllConsumers(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/test-port"
	cfg := testConfig(port)

	bus, _ := newMockBus(t)
	ctrl := &SafeSoArmController{
		bus:    bus,
		logger: cfg.Logger,
	}
	registry.entries[port] = &ControllerEntry{
		controller:  ctrl,
		config:      cfg,
		calibration: DefaultSO101FullCalibration,
		refCount:    2, // simulate arm + gripper both holding
	}

	// First release: refcount drops to 1, controller stays alive.
	registry.ReleaseController(port)
	if ctrl.closed.Load() {
		t.Fatalf("controller closed prematurely at refcount > 0")
	}

	// Second release: refcount drops to 0, controller closes.
	registry.ReleaseController(port)
	if !ctrl.closed.Load() {
		t.Errorf("expected controller.closed=true after final release")
	}
	if err := ctrl.Ping(t.Context()); !errors.Is(err, ErrControllerClosed) {
		t.Errorf("Ping after final release: expected ErrControllerClosed, got %v", err)
	}
}

// TestRegistry_ExplicitPortReleaseDecrementsRefcount verifies that callers
// can release a controller by passing the port path directly, with no
// dependence on runtime.Caller PC tracking.
func TestRegistry_ExplicitPortReleaseDecrementsRefcount(t *testing.T) {
	registry := NewControllerRegistry()
	port := "/dev/test-port"
	cfg := testConfig(port)
	bus, _ := newMockBus(t)
	registry.entries[port] = &ControllerEntry{
		controller: &SafeSoArmController{bus: bus, logger: cfg.Logger},
		config:     cfg,
		refCount:   3,
	}

	registry.ReleaseController(port)

	got := atomic.LoadInt64(&registry.entries[port].refCount)
	if got != 2 {
		t.Errorf("expected refCount=2 after release, got %d", got)
	}
}

// TestRegistry_ReleaseUnknownPortIsNoop verifies that releasing a port that
// was never registered does not panic and does not affect other entries.
func TestRegistry_ReleaseUnknownPortIsNoop(t *testing.T) {
	registry := NewControllerRegistry()
	registry.ReleaseController("/dev/never-existed")
}

// File-scope signature assertions: these guarantee the ctx-threading helpers
// keep their (ctx context.Context) parameter. Dropping ctx would fail to
// compile here — caught at build time, not at test runtime.
var (
	_ func(context.Context) error      = (*so101)(nil).doServoInitialization
	_ func(context.Context) error      = (*so101)(nil).diagnoseConnection
	_ func(context.Context) error      = (*so101)(nil).verifyServoConfig
	_ func(context.Context) error      = (*so101)(nil).initializeServos
	_ func(context.Context, int) error = (*so101)(nil).initializeServosWithRetry
)

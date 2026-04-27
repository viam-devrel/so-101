package so_arm

import (
	"context"
	"errors"
	"testing"
)

// TestController_PostCloseReturnsSentinel verifies that calling any controller
// method after the bus has been closed returns ErrControllerClosed rather than
// a panic or a serial-port error.
func TestController_PostCloseReturnsSentinel(t *testing.T) {
	bus, _ := newMockBus(t)

	ctrl := &SafeSoArmController{
		bus:    bus,
		logger: newTestLogger(t),
	}
	ctrl.closed.Store(true)

	if err := ctrl.Ping(context.Background()); !errors.Is(err, ErrControllerClosed) {
		t.Errorf("Ping after close: expected ErrControllerClosed, got %v", err)
	}
	if err := ctrl.SetTorqueEnable(context.Background(), true); !errors.Is(err, ErrControllerClosed) {
		t.Errorf("SetTorqueEnable after close: expected ErrControllerClosed, got %v", err)
	}
	if _, err := ctrl.GetJointPositionsForServos(context.Background(), []int{1}); !errors.Is(err, ErrControllerClosed) {
		t.Errorf("GetJointPositionsForServos after close: expected ErrControllerClosed, got %v", err)
	}
}

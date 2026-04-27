package so_arm

import (
	"context"
	"errors"
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

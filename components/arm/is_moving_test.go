package arm

import (
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"

	"so_arm/internal/controller"
	"so_arm/internal/testfake"
)

// testArmHandle builds a *controller.ControllerHandle around a fake feetech.Transport, the
// same bypass calibration_recording_test.go uses: NewSO101 acquires through the process-wide
// registry, which needs real serial hardware, so this builds its own registry instead.
func testArmHandle(t *testing.T, ft feetech.Transport) *controller.ControllerHandle {
	t.Helper()
	reg := controller.NewControllerRegistry(
		controller.WithBusFactory(func(cfg feetech.BusConfig) (*feetech.Bus, error) {
			cfg.Transport = ft
			return feetech.NewBus(cfg)
		}),
	)
	h, err := reg.AcquireController(
		arm.Named("test-arm-"+t.Name()),
		&controller.SoArm101Config{Port: "/dev/fake0", Timeout: 50 * time.Millisecond},
		controller.DefaultSO101FullCalibration, true,
	)
	require.NoError(t, err)
	t.Cleanup(h.Release)
	return h
}

// The atomic intent flag is a fast path: when set, IsMoving must return true without any
// bus traffic. Unplugging the transport turns any bus traffic into an EIO failure, so a
// passing call here is a genuine no-traffic assertion, not just a return-value check.
func TestHardwareArmIsMovingFastPathShortCircuits(t *testing.T) {
	ft := testfake.NewFakeTransport()
	s := &so101{controller: testArmHandle(t, ft), armServoIDs: []int{1, 2, 3, 4, 5}}
	s.isMoving.Store(true)
	ft.Unplug()

	moving, err := s.IsMoving(context.Background())
	require.NoError(t, err)
	assert.True(t, moving)
}

func TestHardwareArmIsMovingQueriesHardwareWhenFlagFalse(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(3, feetech.RegMoving.Address, []byte{1})
	s := &so101{controller: testArmHandle(t, ft), armServoIDs: []int{1, 2, 3, 4, 5}}

	moving, err := s.IsMoving(context.Background())
	require.NoError(t, err)
	assert.True(t, moving, "servo 3 is a requested joint and reports moving")
}

func TestHardwareArmIsMovingReturnsFalseWhenAllStopped(t *testing.T) {
	s := &so101{controller: testArmHandle(t, testfake.NewFakeTransport()), armServoIDs: []int{1, 2, 3, 4, 5}}

	moving, err := s.IsMoving(context.Background())
	require.NoError(t, err)
	assert.False(t, moving)
}

// A hot or clamped joint answers with a condition flag; the arm must still know it is moving.
func TestHardwareArmIsMovingSurvivesAConditionFlag(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(3, feetech.RegMoving.Address, []byte{1})
	s := &so101{controller: testArmHandle(t, ft), armServoIDs: []int{1, 2, 3, 4, 5}}
	ft.SetStatus(3, feetech.ErrOverload)

	moving, err := s.IsMoving(context.Background())
	require.NoError(t, err)
	assert.True(t, moving)
}

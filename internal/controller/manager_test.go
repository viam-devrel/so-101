package controller

import (
	"context"
	"sync"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

func TestCalibrationHasASingleSourceOfTruth(t *testing.T) {
	ft := testfake.NewFakeTransport()
	r, h1 := testRegistryAndHandle(t, ft)
	h2 := acquireTestHandle(t, r)

	updated := cloneCalibration(DefaultSO101FullCalibration)
	updated.Gripper.RangeMin = 1234
	require.NoError(t, h1.SetCalibration(updated))

	assert.Equal(t, 1234, h1.entry.calibration.Gripper.RangeMin,
		"the session-rebuild seed must track SetCalibration, or a replug undoes it")

	// The read path and the write path must agree, through either holder.
	assert.Equal(t, 1234, h2.GetCalibration().Gripper.RangeMin)
	assert.Equal(t, 1234, h2.entry.session.Load().servos[6].Calibration().RangeMin,
		"the per-servo calibration is the one source of truth; nothing may hold a second copy")
}

func TestCalibrationUpdateDoesNotMutateTheDefault(t *testing.T) {
	before := DefaultSO101FullCalibration.Gripper.RangeMin

	h := testHandle(t, testfake.NewFakeTransport())
	updated := cloneCalibration(DefaultSO101FullCalibration)
	updated.Gripper.RangeMin = 999
	require.NoError(t, h.SetCalibration(updated))

	assert.Equal(t, before, DefaultSO101FullCalibration.Gripper.RangeMin,
		"a calibration update must not reach through shared pointers into the package default")
}

func TestConcurrentCalibrationUpdateAndReadIsRaceFree(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	var wg sync.WaitGroup
	for i := 0; i < 20; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			c := cloneCalibration(DefaultSO101FullCalibration)
			c.Gripper.RangeMin = 1000 + i
			_ = h.SetCalibration(c)
		}(i)
		wg.Add(1)
		go func() {
			defer wg.Done()
			_ = h.GetCalibration()
		}()
		wg.Add(1)
		go func() {
			defer wg.Done()
			_, _ = h.GetJointPositionsForServos(context.Background(), []int{1, 2, 3, 4, 5})
		}()
	}
	wg.Wait()
}

func TestSyncReadPositionsDecodesInsideTheHandle(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(1, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(2048))
	h := testHandle(t, ft)

	positions, err := h.SyncReadPositions(context.Background(), []int{1})
	require.NoError(t, err)
	assert.Equal(t, 2048, positions[1],
		"decoding belongs inside the handle; exposing Protocol() would re-leak the bus")
}

func TestPingServoReachesOneServo(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	model, err := h.PingServo(context.Background(), 3)
	require.NoError(t, err)
	assert.Equal(t, testfake.FakeModelNumber, model)
}

func TestServoPresentReportsConfiguredServos(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	assert.True(t, h.ServoPresent(6))
	assert.False(t, h.ServoPresent(9), "motorSetupVerify needs absent-vs-unresponsive")
}

func TestServosMovingReportsFalseWhenAllStopped(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	moving, err := h.ServosMoving(context.Background(), []int{1, 2, 3})
	require.NoError(t, err)
	assert.False(t, moving)
}

func TestServosMovingReportsTrueWhenOneIsMoving(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(2, feetech.RegMoving.Address, []byte{1})
	h := testHandle(t, ft)

	moving, err := h.ServosMoving(context.Background(), []int{1, 2, 3})
	require.NoError(t, err)
	assert.True(t, moving)
}

func TestServosMovingOnlyConsultsRequestedIDs(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(6, feetech.RegMoving.Address, []byte{1}) // gripper moving, not requested
	h := testHandle(t, ft)

	moving, err := h.ServosMoving(context.Background(), []int{1, 2, 3, 4, 5})
	require.NoError(t, err)
	assert.False(t, moving, "a moving servo outside ids must not make the batch read true")
}

func TestWaitForServosToStopReturnsOnceAllStopped(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	err := h.WaitForServosToStop(context.Background(), []int{1, 2}, 200)
	require.NoError(t, err)
}

func TestWaitForServosToStopTimesOutWithoutError(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(1, feetech.RegMoving.Address, []byte{1})
	h := testHandle(t, ft)

	err := h.WaitForServosToStop(context.Background(), []int{1}, 60)
	assert.NoError(t, err, "a still-moving servo must time out best-effort, not fail")
}

func TestWaitForServosToStopHonorsContextCancellation(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(1, feetech.RegMoving.Address, []byte{1})
	h := testHandle(t, ft)

	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	err := h.WaitForServosToStop(ctx, []int{1}, 5000)
	assert.ErrorIs(t, err, context.Canceled)
}

func TestWaitForServosToStopFailsFastWhileDisconnected(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	h.entry.session.Store(nil)

	err := h.WaitForServosToStop(context.Background(), []int{1}, 200)
	assert.ErrorIs(t, err, ErrPortDisconnected)
}

func TestWaitForServosToStopIgnoresIDsNotOnTheBus(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	err := h.WaitForServosToStop(context.Background(), []int{1, 42}, 200)
	require.NoError(t, err)
}

func TestHandleOperationsFailFastWhileDisconnected(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	h.entry.session.Store(nil)

	_, err := h.SyncReadPositions(context.Background(), []int{1})
	assert.ErrorIs(t, err, ErrPortDisconnected)
	_, err = h.PingServo(context.Background(), 1)
	assert.ErrorIs(t, err, ErrPortDisconnected)
	assert.ErrorIs(t, h.SetTorqueEnable(context.Background(), true), ErrPortDisconnected)
	assert.False(t, h.ServoPresent(1), "no session means no servos")
}

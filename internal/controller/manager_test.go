package controller

import (
	"context"
	"sync"
	"testing"
	"time"

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

func TestAnyServoMovingReportsFalseWhenAllStopped(t *testing.T) {
	h := testHandle(t, testfake.NewFakeTransport())
	moving, _, err := h.AnyServoMoving(context.Background(), []int{1, 2, 3})
	require.NoError(t, err)
	assert.False(t, moving)
}

func TestAnyServoMovingReportsTrueWhenOneIsMoving(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(2, feetech.RegMoving.Address, []byte{1})
	h := testHandle(t, ft)

	moving, _, err := h.AnyServoMoving(context.Background(), []int{1, 2, 3})
	require.NoError(t, err)
	assert.True(t, moving)
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

// testArmIDs is the arm servo set; manager.go has a function-local armServoIDs.
var testArmIDs = []int{1, 2, 3, 4, 5}

// conditionTestHandles builds two handles over identical register files; only the second
// has servos 3 and 6 setting a condition flag. Every read path must return the same data
// from both, and an error from neither.
func conditionTestHandles(t *testing.T) (clean, flagged *ControllerHandle) {
	t.Helper()
	build := func(flag bool) *ControllerHandle {
		ft := testfake.NewFakeTransport()
		for id := 1; id <= 5; id++ {
			ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(2000+100*id))
		}
		ft.SetRegister(6, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(2648))
		// The fake serves a read from the bytes stored at the requested address only, so
		// GetJointPositionsAndMovingForServos's single 11-byte read at Present_Position
		// needs its Moving bit inside that blob; a 2-byte read of the same address still
		// gets the position word (value() copies then truncates).
		state := make([]byte, stateReadWidth)
		copy(state, testfake.EncodeWordLE(2300))
		state[stateMovingOffset] = 1
		ft.SetRegister(3, feetech.RegPresentPosition.Address, state)
		ft.SetRegister(3, feetech.RegMoving.Address, []byte{1}) // AnyServoMoving reads 66 directly
		ft.SetRegister(3, feetech.RegPresentLoad.Address, testfake.EncodeWordLE(404))
		h := testHandle(t, ft)
		if flag { // after acquire, so the flag only ever meets the read paths under test
			ft.SetStatus(3, feetech.ErrOverload)
			ft.SetStatus(6, feetech.ErrOverheat)
		}
		return h
	}
	return build(false), build(true)
}

func TestServoReadsSurviveAConditionFlag(t *testing.T) {
	ctx := context.Background()
	clean, flagged := conditionTestHandles(t)

	t.Run("ServoPositionPercent", func(t *testing.T) {
		wantPct, wantRaw, wantCond, err := clean.ServoPositionPercent(ctx, 6)
		require.NoError(t, err)
		assert.Zero(t, wantCond)

		pct, raw, cond, err := flagged.ServoPositionPercent(ctx, 6)
		require.NoError(t, err, "a condition must not be reported as a failed read")
		assert.Equal(t, wantRaw, raw)
		assert.Equal(t, wantPct, pct)
		assert.Equal(t, feetech.ErrOverheat, cond, "and the condition must be reported alongside")
	})

	t.Run("AnyServoMoving", func(t *testing.T) {
		moving, cond, err := clean.AnyServoMoving(ctx, testArmIDs)
		require.NoError(t, err)
		assert.True(t, moving)
		assert.Zero(t, cond)

		moving, cond, err = flagged.AnyServoMoving(ctx, testArmIDs)
		require.NoError(t, err)
		assert.True(t, moving, "the Moving bit is valid and must survive")
		assert.Equal(t, feetech.ErrOverload, cond)
	})
}

// The other half of the policy: a request-rejection flag means the servo did not answer the
// question, and the read must still fail.
func TestServoReadsStillFailOnARejectionFlag(t *testing.T) {
	ctx := context.Background()
	ft := testfake.NewFakeTransport()
	h := testHandle(t, ft)
	ft.SetStatus(6, feetech.ErrChecksum)

	_, _, _, err := h.ServoPositionPercent(ctx, 6)
	require.Error(t, err)
	_, _, err = h.AnyServoMoving(ctx, []int{6})
	require.Error(t, err)
}

// A clamping jaw sets overload while answering normally. The wait must end when the servo
// stops, not when the caller's timeout expires.
func TestWaitForServosToStopEndsOnAStoppedServoDespiteACondition(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(6, feetech.RegMoving.Address, []byte{0})
	h := testHandle(t, ft)
	ft.SetStatus(6, feetech.ErrOverload)

	start := time.Now()
	require.NoError(t, h.WaitForServosToStop(context.Background(), []int{6}, 5000))
	assert.Less(t, time.Since(start), 2*time.Second, "must not wait out the 5s timeout")
}

package controller

import (
	"context"
	"sync"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// syncWriteLog records the start address of every SYNC_WRITE, in order.
type syncWriteLog struct {
	*testfake.FakeTransport
	mu    sync.Mutex
	addrs []byte
}

func (l *syncWriteLog) Write(b []byte) (int, error) {
	if len(b) >= 7 && b[0] == 0xFF && b[1] == 0xFF && b[4] == 0x83 {
		l.mu.Lock()
		l.addrs = append(l.addrs, b[5])
		l.mu.Unlock()
	}
	return l.FakeTransport.Write(b)
}

func (l *syncWriteLog) order() []byte {
	l.mu.Lock()
	defer l.mu.Unlock()
	return append([]byte(nil), l.addrs...)
}

func readWord(t *testing.T, h *ControllerHandle, id int, reg string) int {
	t.Helper()
	b, err := h.ReadServoRegister(context.Background(), id, reg)
	require.NoError(t, err)
	if len(b) == 1 {
		return int(b[0])
	}
	return int(b[0]) | int(b[1])<<8
}

func TestEnableTorqueHeldSeedsGoalBeforeEnabling(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for id := 1; id <= 3; id++ {
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(1000+id))
		ft.SetRegister(id, feetech.RegGoalPosition.Address, testfake.EncodeWordLE(3000))
	}
	log := &syncWriteLog{FakeTransport: ft}
	h := testHandleOver(t, log)

	require.NoError(t, h.EnableTorqueHeld(context.Background(), []int{1, 2, 3}))

	require.Equal(t, []byte{feetech.RegGoalPosition.Address, feetech.RegTorqueEnable.Address}, log.order())
	for id := 1; id <= 3; id++ {
		require.Equal(t, 1000+id, readWord(t, h, id, "goal_position"))
		require.Equal(t, 1, readWord(t, h, id, "torque_enable"))
	}
	require.Zero(t, ft.WriteCount(4, feetech.RegTorqueEnable.Address), "servo outside ids untouched")
}

func TestEnableTorqueHeldLeavesTorqueOffOnBadRead(t *testing.T) {
	cases := map[string]func(ft *testfake.FakeTransport) []int{
		"missing servo": func(ft *testfake.FakeTransport) []int { return []int{1, 7} },
		"out of range": func(ft *testfake.FakeTransport) []int {
			ft.SetRegister(1, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(5000))
			return []int{1}
		},
	}
	for name, setup := range cases {
		t.Run(name, func(t *testing.T) {
			ft := testfake.NewFakeTransport()
			ids := setup(ft)
			h := testHandle(t, ft)
			require.Error(t, h.EnableTorqueHeld(context.Background(), ids))
			require.Zero(t, ft.WriteCount(1, feetech.RegTorqueEnable.Address))
			require.Zero(t, ft.WriteCount(1, feetech.RegGoalPosition.Address))
		})
	}
}

func TestSetTorqueEnableAndSetServoTorqueSeed(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for id := 1; id <= 6; id++ {
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(1500+id))
	}
	h := testHandle(t, ft)

	require.NoError(t, h.SetTorqueEnable(context.Background(), true))
	for id := 1; id <= 6; id++ {
		require.Equal(t, 1500+id, readWord(t, h, id, "goal_position"))
	}

	ft.SetRegister(6, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(2222))
	require.NoError(t, h.SetServoTorque(context.Background(), 6, true))
	require.Equal(t, 2222, readWord(t, h, 6, "goal_position"))
}

func TestDisableTorqueTouchesOnlyIDs(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for id := 1; id <= 6; id++ {
		ft.SetRegister(id, feetech.RegTorqueEnable.Address, []byte{1})
	}
	h := testHandle(t, ft)
	require.NoError(t, h.DisableTorque(context.Background(), []int{1, 2, 3, 4, 5}))
	for id := 1; id <= 5; id++ {
		require.Equal(t, 0, readWord(t, h, id, "torque_enable"))
	}
	require.Equal(t, 1, readWord(t, h, 6, "torque_enable"))
}

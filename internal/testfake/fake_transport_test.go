package testfake

import (
	"context"
	"syscall"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestFakeTransportAnswersPing(t *testing.T) {
	ft := NewFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	model, err := bus.Ping(context.Background(), 1)
	require.NoError(t, err)
	assert.Equal(t, 777, model, "fake should report the STS3215 model number")
}

func TestFakeTransportFailsWritesWhenUnplugged(t *testing.T) {
	ft := NewFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	ft.Unplug()
	_, err = bus.Ping(context.Background(), 1)
	require.Error(t, err)
	assert.ErrorIs(t, err, syscall.EIO, "an unplugged port surfaces its errno through the write path")

	ft.Replug()
	_, err = bus.Ping(context.Background(), 1)
	assert.NoError(t, err)
}

func TestFakeTransportIsSilentForAbsentServos(t *testing.T) {
	ft := NewFakeTransport()
	delete(ft.registers, 4)
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	_, err = bus.Ping(context.Background(), 4)
	assert.ErrorIs(t, err, feetech.ErrNoResponse,
		"a silent servo must be distinguishable from an unplugged port")
}

func TestWriteCountTracksRegisterWrites(t *testing.T) {
	ft := NewFakeTransport()
	if got := ft.WriteCount(1, feetech.RegPGain.Address); got != 0 {
		t.Fatalf("expected 0 writes before any traffic, got %d", got)
	}
	// Seeding a register is not a write.
	ft.SetRegister(1, feetech.RegPGain.Address, []byte{32})
	if got := ft.WriteCount(1, feetech.RegPGain.Address); got != 0 {
		t.Fatalf("SetRegister must not count as a write, got %d", got)
	}
}

// Goal writes reach the bus as SYNC_WRITE, so without this a test cannot see what speed or
// acceleration was commanded.
func TestSyncWriteLandsInTheRegisterStore(t *testing.T) {
	ft := NewFakeTransport()
	// [address, dataLen, id, data..., id, data...]
	params := []byte{feetech.RegAcceleration.Address, 2, 1, 0x11, 0x22, 3, 0x33, 0x44}
	ft.mu.Lock()
	ft.reply(append([]byte{0xFF, 0xFF, 0xFE, byte(len(params) + 2), 0x83}, append(params, 0)...))
	ft.mu.Unlock()

	if got := ft.value(1, feetech.RegAcceleration.Address, 1); got[0] != 0x11 {
		t.Fatalf("servo 1 acc: got %#x, want 0x11", got[0])
	}
	if got := ft.value(3, feetech.RegAcceleration.Address, 1); got[0] != 0x33 {
		t.Fatalf("servo 3 acc: got %#x, want 0x33", got[0])
	}
}

// An overloaded STS3215 answers normally with its status byte set. The fake must produce the
// same shape so the controller's tolerance can be tested without hardware.
func TestFakeTransportReportsAConditionFlagWithValidData(t *testing.T) {
	ft := NewFakeTransport()
	ft.SetRegister(2, feetech.RegPresentPosition.Address, EncodeWordLE(1500))
	ft.SetStatus(2, feetech.ErrOverload)
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	data, err := bus.ReadRegister(context.Background(), 2, feetech.RegPresentPosition.Address, 2)
	flags, ok := feetech.ConditionStatus(err)
	require.True(t, ok, "the flag must arrive as a condition, not a failure: %v", err)
	assert.Equal(t, feetech.ErrOverload, flags)
	assert.Equal(t, 1500, int(bus.Protocol().DecodeWord(data)), "the payload must still be live")

	// Unflagged servos on the same bus are unaffected.
	_, err = bus.ReadRegister(context.Background(), 1, feetech.RegPresentPosition.Address, 2)
	require.NoError(t, err)
}

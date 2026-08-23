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

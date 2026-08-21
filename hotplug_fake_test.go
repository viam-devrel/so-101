package so_arm

import (
	"context"
	"sync"
	"syscall"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestFakeTransportAnswersPing(t *testing.T) {
	ft := newFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	model, err := bus.Ping(context.Background(), 1)
	require.NoError(t, err)
	assert.Equal(t, 777, model, "fake should report the STS3215 model number")
}

func TestFakeTransportFailsWritesWhenUnplugged(t *testing.T) {
	ft := newFakeTransport()
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	ft.unplug()
	_, err = bus.Ping(context.Background(), 1)
	require.Error(t, err)
	assert.ErrorIs(t, err, syscall.EIO, "an unplugged port surfaces its errno through the write path")

	ft.replug()
	_, err = bus.Ping(context.Background(), 1)
	assert.NoError(t, err)
}

func TestFakeTransportIsSilentForAbsentServos(t *testing.T) {
	ft := newFakeTransport()
	delete(ft.registers, 4)
	bus, err := feetech.NewBus(feetech.BusConfig{Transport: ft, Timeout: 50 * time.Millisecond})
	require.NoError(t, err)
	t.Cleanup(func() { _ = bus.Close() })

	_, err = bus.Ping(context.Background(), 4)
	assert.ErrorIs(t, err, feetech.ErrNoResponse,
		"a silent servo must be distinguishable from an unplugged port")
}

// fakeTransport answers real Feetech protocol packets from an in-memory register file,
// and can be flipped to fail every write with syscall.EIO the way an unplugged port does
// (go.bug.st/serial's unixPort.Write returns the raw errno; serial_unix.go:112-118).
//
// Safe for concurrent use, unlike transports.MockTransport and transports.Script -- the
// reconnect loop drives a bus from its own goroutine, so -race demands it.
type fakeTransport struct {
	mu        sync.Mutex
	proto     *feetech.Protocol
	registers map[int]map[byte][]byte // servo id -> address -> value
	pending   []byte                  // queued response bytes
	unplugged bool
	writes    int
	opens     int
	closes    int
}

const fakeModelNumber = 777 // ModelSTS3215's model number

func newFakeTransport() *fakeTransport {
	ft := &fakeTransport{
		proto:     feetech.NewProtocol(feetech.ProtocolSTS),
		registers: make(map[int]map[byte][]byte),
	}
	for id := 1; id <= 6; id++ {
		ft.registers[id] = map[byte][]byte{
			feetech.RegModelNumber.Address: ft.proto.EncodeWord(fakeModelNumber),
		}
	}
	return ft
}

func (ft *fakeTransport) unplug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = true
	ft.pending = nil
}

func (ft *fakeTransport) replug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = false
}

func (ft *fakeTransport) isUnplugged() bool {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.unplugged
}

// noteOpen/openCount count reopen attempts, which is the only observable that moves while
// the port is down -- writes fail before they are counted.
func (ft *fakeTransport) noteOpen() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.opens++
}

func (ft *fakeTransport) openCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.opens
}

// writeCount reports how many packets the fake has accepted, so a test can assert that a
// reconnect loop stopped touching the port.
func (ft *fakeTransport) writeCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.writes
}

func (ft *fakeTransport) Write(p []byte) (int, error) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if ft.unplugged {
		return 0, syscall.EIO
	}
	ft.writes++
	ft.reply(p)
	return len(p), nil
}

func (ft *fakeTransport) Read(p []byte) (int, error) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if ft.unplugged {
		return 0, syscall.EIO
	}
	if len(ft.pending) == 0 {
		return 0, nil // no data yet; the bus retries until its deadline
	}
	n := copy(p, ft.pending)
	ft.pending = ft.pending[n:]
	return n, nil
}

func (ft *fakeTransport) Close() error {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.closes++
	return nil
}
func (ft *fakeTransport) SetReadTimeout(time.Duration) error { return nil }
func (ft *fakeTransport) Flush() error                       { return nil }

// reply parses one request and queues its response. Called with ft.mu held.
func (ft *fakeTransport) reply(req []byte) {
	if len(req) < 6 || req[0] != 0xFF || req[1] != 0xFF {
		return
	}
	id, instruction, params := int(req[2]), req[4], req[5:len(req)-1]

	switch instruction {
	case 0x01: // PING -- status only
		ft.queue(id, nil)
	case 0x02: // READ [address, length]
		if len(params) < 2 {
			return
		}
		ft.queue(id, ft.value(id, params[0], int(params[1])))
	case 0x03, 0x04: // WRITE / REG_WRITE [address, data...]
		if len(params) < 1 {
			return
		}
		if regs, ok := ft.registers[id]; ok {
			regs[params[0]] = append([]byte(nil), params[1:]...)
		}
		ft.queue(id, nil)
	case 0x82: // SYNC_READ [address, length, ids...]
		if len(params) < 2 {
			return
		}
		address, length := params[0], int(params[1])
		for _, raw := range params[2:] {
			ft.queue(int(raw), ft.value(int(raw), address, length))
		}
	case 0x83: // SYNC_WRITE -- no response on the wire
	}
}

// value returns length bytes for a register, zero-filled when unset.
func (ft *fakeTransport) value(id int, address byte, length int) []byte {
	out := make([]byte, length)
	if regs, ok := ft.registers[id]; ok {
		copy(out, regs[address])
	}
	return out
}

func (ft *fakeTransport) queue(id int, params []byte) {
	if _, known := ft.registers[id]; !known {
		return // absent servo: silence, which the bus reports as ErrNoResponse
	}
	ft.pending = append(ft.pending,
		ft.proto.Encode(feetech.Packet{ID: byte(id), Instruction: 0, Parameters: params})...)
}

var _ feetech.Transport = (*fakeTransport)(nil)

// setRegister seeds a register so a read returns a known value.
func (ft *fakeTransport) setRegister(id int, address byte, value []byte) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if regs, ok := ft.registers[id]; ok {
		regs[address] = append([]byte(nil), value...)
	}
}

// closeCount reports how many times the bus over this fake has been closed, so a teardown
// test can assert it happened exactly once.
func (ft *fakeTransport) closeCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.closes
}

// encodeWordLE encodes a 16-bit register value the way the STS protocol expects.
func encodeWordLE(v int) []byte {
	return feetech.NewProtocol(feetech.ProtocolSTS).EncodeWord(uint16(v))
}

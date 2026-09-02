// Package testfake holds test doubles shared across package boundaries: they live in a
// regular package rather than _test.go files because several packages' tests need them.
// It must not import internal/controller or the root package -- controller's own
// in-package tests import this, so either edge would be an import cycle.
package testfake

import (
	"sync"
	"syscall"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// FakeModelNumber is the model number FakeTransport reports for every servo (ModelSTS3215's
// real model number).
const FakeModelNumber = 777

// FakeTransport answers real Feetech protocol packets from an in-memory register file, and
// can be flipped to fail writes with syscall.EIO the way an unplugged port does.
//
// Concurrency-safe, unlike transports.MockTransport and transports.Script, so tests can
// drive a bus from more than one goroutine under -race.
type FakeTransport struct {
	mu        sync.Mutex
	proto     *feetech.Protocol
	registers map[int]map[byte][]byte // servo id -> address -> value
	regWrites map[int]map[byte]int    // servo id -> address -> write count
	pending   []byte                  // queued response bytes
	unplugged bool
	writes    int
	opens     int
	closes    int
}

// NewFakeTransport builds a fake transport seeded with all six SO-101 servos present and
// answering FakeModelNumber.
func NewFakeTransport() *FakeTransport {
	ft := &FakeTransport{
		proto:     feetech.NewProtocol(feetech.ProtocolSTS),
		registers: make(map[int]map[byte][]byte),
		regWrites: make(map[int]map[byte]int),
	}
	for id := 1; id <= 6; id++ {
		ft.registers[id] = map[byte][]byte{
			feetech.RegModelNumber.Address: ft.proto.EncodeWord(FakeModelNumber),
		}
	}
	return ft
}

// Unplug makes every subsequent write fail with syscall.EIO, the way an unplugged serial
// port does.
func (ft *FakeTransport) Unplug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = true
	ft.pending = nil
}

// Replug reverses Unplug.
func (ft *FakeTransport) Replug() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.unplugged = false
}

// IsUnplugged reports whether the fake is currently simulating an unplugged port.
func (ft *FakeTransport) IsUnplugged() bool {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.unplugged
}

// NoteOpen records a reopen attempt, which is the
// only observable that moves while the port is down -- writes fail before they are counted.
func (ft *FakeTransport) NoteOpen() {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.opens++
}

func (ft *FakeTransport) Write(p []byte) (int, error) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if ft.unplugged {
		return 0, syscall.EIO
	}
	ft.writes++
	ft.reply(p)
	return len(p), nil
}

func (ft *FakeTransport) Read(p []byte) (int, error) {
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

func (ft *FakeTransport) Close() error {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	ft.closes++
	return nil
}
func (ft *FakeTransport) SetReadTimeout(time.Duration) error { return nil }
func (ft *FakeTransport) Flush() error                       { return nil }

// reply parses one request and queues its response. Called with ft.mu held.
func (ft *FakeTransport) reply(req []byte) {
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
			if _, ok := ft.regWrites[id]; !ok {
				ft.regWrites[id] = map[byte]int{}
			}
			ft.regWrites[id][params[0]]++
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
	case 0x83: // SYNC_WRITE -- broadcast, no response on the wire
		// Params are [address, dataLen, id, data..., id, data...]. A sync write covers a
		// RANGE of registers (a goal write is acceleration, position, time and velocity in
		// one 7-byte blob), so store the blob at `address` AND every suffix of it at the
		// address it starts from. value() copies min(len, stored) bytes, so a later read of
		// any register inside the range returns its own bytes rather than the blob's head.
		if len(params) < 2 {
			return
		}
		address, n := params[0], int(params[1])
		for i := 2; i+1+n <= len(params); i += 1 + n {
			regs, ok := ft.registers[int(params[i])]
			if !ok {
				continue
			}
			blob := params[i+1 : i+1+n]
			for off := 0; off < n; off++ {
				regs[address+byte(off)] = append([]byte(nil), blob[off:]...)
			}
		}
	}
}

// value returns length bytes for a register, zero-filled when unset.
func (ft *FakeTransport) value(id int, address byte, length int) []byte {
	out := make([]byte, length)
	if regs, ok := ft.registers[id]; ok {
		copy(out, regs[address])
	}
	return out
}

func (ft *FakeTransport) queue(id int, params []byte) {
	if _, known := ft.registers[id]; !known {
		return // absent servo: silence, which the bus reports as ErrNoResponse
	}
	ft.pending = append(ft.pending,
		ft.proto.Encode(feetech.Packet{ID: byte(id), Instruction: 0, Parameters: params})...)
}

var _ feetech.Transport = (*FakeTransport)(nil)

// SetRegister seeds a register so a read returns a known value.
func (ft *FakeTransport) SetRegister(id int, address byte, value []byte) {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	if regs, ok := ft.registers[id]; ok {
		regs[address] = append([]byte(nil), value...)
	}
}

// CloseCount reports how many times the bus over this fake has been closed, so a teardown
// test can assert it happened exactly once.
func (ft *FakeTransport) CloseCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.closes
}

// PacketCount reports how many request packets have been written to the transport, so a
// test can assert a code path's bus traffic rather than only its effect -- a skipped read
// leaves no trace in the register store.
func (ft *FakeTransport) PacketCount() int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.writes
}

// WriteCount reports how many WRITE packets have landed on one register, so a test can
// assert a read-first guard actually skipped a redundant write -- reading the value back
// cannot distinguish "already correct" from "written again".
func (ft *FakeTransport) WriteCount(id int, address byte) int {
	ft.mu.Lock()
	defer ft.mu.Unlock()
	return ft.regWrites[id][address]
}

// EncodeWordLE encodes a 16-bit register value the way the STS protocol expects.
func EncodeWordLE(v int) []byte {
	return feetech.NewProtocol(feetech.ProtocolSTS).EncodeWord(uint16(v))
}

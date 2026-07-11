package so_arm

import (
	"io"
	"sync"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
)

// scriptedMockTransport is a custom Transport (not feetech.MockTransport) whose
// Read responses are queued per-request. We use a custom type rather than the
// upstream MockTransport because tests in PR2-PR5 need to script multiple
// round-trips with different responses, and MockTransport's single ReadData
// buffer doesn't support that pattern cleanly.
type scriptedMockTransport struct {
	mu        sync.Mutex
	written   []byte
	responses [][]byte
	closed    bool
}

func (m *scriptedMockTransport) Read(p []byte) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.responses) == 0 {
		// Match feetech.MockTransport semantics: returning io.EOF on an empty
		// queue lets feetech.Bus.readRawBytesLocked fall into its 1ms-sleep
		// retry path instead of busy-spinning under the bus mutex. Important
		// for PR5's planned 100Hz calibration-sensor reader.
		return 0, io.EOF
	}
	resp := m.responses[0]
	n := copy(p, resp)
	if n >= len(resp) {
		m.responses = m.responses[1:]
	} else {
		m.responses[0] = resp[n:]
	}
	return n, nil
}

func (m *scriptedMockTransport) Write(p []byte) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.written = append(m.written, p...)
	return len(p), nil
}

func (m *scriptedMockTransport) Close() error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.closed = true
	return nil
}

func (m *scriptedMockTransport) SetReadTimeout(timeout time.Duration) error {
	return nil
}

func (m *scriptedMockTransport) Flush() error {
	// No-op: tests that need to drop unconsumed responses should clear
	// m.responses explicitly. Mirroring SerialTransport.Flush would risk
	// silently swallowing scripted frames between operations.
	return nil
}

// queueResponse appends a raw frame to the mock's response queue.
func (m *scriptedMockTransport) queueResponse(b []byte) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.responses = append(m.responses, b)
}

// reset clears the written-data buffer (responses queue is preserved).
func (m *scriptedMockTransport) resetWritten() {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.written = nil
}

// snapshotWritten returns a copy of all bytes written since the last reset.
func (m *scriptedMockTransport) snapshotWritten() []byte {
	m.mu.Lock()
	defer m.mu.Unlock()
	out := make([]byte, len(m.written))
	copy(out, m.written)
	return out
}

// pingResponse builds the ping ACK frame for a given servo ID using the STS protocol.
// Frame: 0xFF 0xFF <id> 0x02 0x00 <checksum>
func pingResponse(servoID byte) []byte {
	checksum := ^(servoID + 0x02 + 0x00)
	return []byte{0xFF, 0xFF, servoID, 0x02, 0x00, checksum}
}

// newMockBus returns a *feetech.Bus backed by a scriptedMockTransport.
// Caller is responsible for queuing responses before issuing bus operations.
func newMockBus(t *testing.T) (*feetech.Bus, *scriptedMockTransport) {
	t.Helper()
	mock := &scriptedMockTransport{}
	bus, err := feetech.NewBus(feetech.BusConfig{
		Transport: mock,
		Protocol:  feetech.ProtocolSTS,
		Timeout:   100 * time.Millisecond,
	})
	if err != nil {
		t.Fatalf("newMockBus: NewBus failed: %v", err)
	}
	t.Cleanup(func() { _ = bus.Close() })
	return bus, mock
}

// newTestLogger returns a logger that discards output unless the test fails.
func newTestLogger(t *testing.T) logging.Logger {
	t.Helper()
	return logging.NewTestLogger(t)
}

func TestMockBus_PingRoundTrip(t *testing.T) {
	bus, mock := newMockBus(t)
	// Bus.Ping does two round trips: a ping ACK, then a model-number register read.
	// Queue both responses so the call completes.
	mock.queueResponse(pingResponse(1))
	// Model number 777 (0x0309) read response: FF FF <id> <len=4> <err=0> <lo> <hi> <chk>
	mock.queueResponse([]byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x09, 0x03, 0xEE})

	if _, err := bus.Ping(t.Context(), 1); err != nil {
		t.Fatalf("ping failed: %v", err)
	}

	written := mock.snapshotWritten()
	if len(written) < 6 {
		t.Fatalf("expected ping packet to be written, got %d bytes: %X", len(written), written)
	}
	// Ping packet: 0xFF 0xFF <id=1> <length=2> <instruction=0x01> <checksum>
	if written[0] != 0xFF || written[1] != 0xFF || written[2] != 0x01 {
		t.Errorf("malformed ping packet: %X", written[:6])
	}
}

package controller

import (
	"context"
	"fmt"
	"sort"
	"sync"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"

	"so_arm/internal/testfake"
)

// This file answers a concrete question: does polling AnyServoMoving (the hardware IsMoving
// path, one SyncRead of feetech.RegMoving batched across servos) disrupt the position
// reads/writes that share the same serial bus?
//
// Two independent locks are in play: entry.busMu (internal/controller/bus_session.go) only
// arbitrates writes vs. reads at this package's level -- withSessionRead takes RLock, so two
// concurrent reads (a position SyncRead and a moving-poll SyncRead) are NOT serialized by
// busMu alone. The actual exclusion is one level down, in *feetech.Bus itself: every one of
// its methods (SyncRead included) takes bus.mu for the request+response round trip, and
// sendPacketLocked's enforceCommandGap (a real time.Sleep, MinCommandGap defaults to 1ms and
// this module never overrides it -- see registry.go's feetech.BusConfig literal) runs WHILE
// THAT LOCK IS HELD. So no interleaving/corruption is possible, but a caller queued behind an
// in-flight transaction pays that transaction's full command-gap sleep, not just its own.

// txTracker wraps testfake.FakeTransport to record request/response byte boundaries, so a
// test can show a position SyncRead's bytes are never split by a Moving SyncRead's bytes.
// Pattern mirrors packetLogTransport in components/calibration/torque_reenable_test.go:
// wrap the fake in the test file rather than touch internal/testfake itself.
type txTracker struct {
	*testfake.FakeTransport

	mu           sync.Mutex
	pendingBytes int    // response bytes still owed for the in-flight transaction
	pendingKind  string // "position", "moving", or "other"
	violations   []string
}

func (t *txTracker) Write(p []byte) (int, error) {
	t.mu.Lock()
	if len(p) >= 7 && p[0] == 0xFF && p[1] == 0xFF && p[4] == 0x82 { // SYNC_READ
		params := p[5 : len(p)-1]
		if len(params) >= 2 {
			address, length := params[0], int(params[1])
			numIDs := len(params) - 2
			kind := "other"
			switch address {
			case feetech.RegPresentPosition.Address:
				kind = "position"
			case feetech.RegMoving.Address:
				kind = "moving"
			}
			if t.pendingBytes > 0 {
				t.violations = append(t.violations, fmt.Sprintf(
					"a %s request was sent while %d response bytes from a prior %s request were still unread",
					kind, t.pendingBytes, t.pendingKind))
			}
			t.pendingBytes = numIDs * (6 + length)
			t.pendingKind = kind
		}
	}
	t.mu.Unlock()

	return t.FakeTransport.Write(p)
}

func (t *txTracker) Read(p []byte) (int, error) {
	n, err := t.FakeTransport.Read(p)
	if n > 0 {
		t.mu.Lock()
		t.pendingBytes -= n
		if t.pendingBytes < 0 {
			t.pendingBytes = 0
		}
		t.mu.Unlock()
	}
	return n, err
}

func (t *txTracker) Violations() []string {
	t.mu.Lock()
	defer t.mu.Unlock()
	return append([]string(nil), t.violations...)
}

// trackedTestHandle acquires a handle whose bus runs over a txTracker-wrapped fake, so the
// caller can inspect byte-level transaction boundaries. Mirrors testHandle/testRegistry in
// manager_test.go, but needs its own registry since those take a bare *testfake.FakeTransport.
func trackedTestHandle(t *testing.T, ft *testfake.FakeTransport) (*ControllerHandle, *txTracker) {
	t.Helper()
	tracker := &txTracker{FakeTransport: ft}
	r := NewControllerRegistry()
	r.busFactory = func(cfg feetech.BusConfig) (*feetech.Bus, error) {
		cfg.Transport = tracker
		return feetech.NewBus(cfg)
	}
	owner := arm.Named("test-arm-" + t.Name())
	h, err := r.AcquireController(owner, shortTimeoutConfig("/dev/fake-"+t.Name()),
		DefaultSO101FullCalibration, true)
	require.NoError(t, err)
	t.Cleanup(h.Release)
	return h, tracker
}

// TestConcurrentAnyServoMovingDoesNotInterleaveOrCorruptPositionReads is the correctness
// contract: hammer position reads and Moving polls concurrently (no cadence limiter -- this
// is a stress test, not a cadence simulation) and confirm two things that must hold
// regardless of timing: every position read still returns exactly its seeded value, and the
// tracker never observes one transaction's request bytes sent before a prior transaction's
// response bytes were fully drained.
func TestConcurrentAnyServoMovingDoesNotInterleaveOrCorruptPositionReads(t *testing.T) {
	ft := testfake.NewFakeTransport()
	armIDs := []int{1, 2, 3, 4, 5}
	seeded := map[int]int{1: 1000, 2: 1500, 3: 2000, 4: 2500, 5: 3000}
	for id, pos := range seeded {
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(pos))
	}

	h, tracker := trackedTestHandle(t, ft)
	ctx := context.Background()
	const iterations = 200

	var wg sync.WaitGroup
	pollerDone := make(chan struct{})
	wg.Add(1)
	go func() {
		defer wg.Done()
		for {
			select {
			case <-pollerDone:
				return
			default:
			}
			_, err := h.AnyServoMoving(ctx, armIDs)
			assert.NoError(t, err)
		}
	}()

	for i := 0; i < iterations; i++ {
		positions, err := h.SyncReadPositions(ctx, armIDs)
		require.NoError(t, err)
		for id, want := range seeded {
			assert.Equal(t, want, positions[id],
				"position read must return the seeded value even under a concurrent AnyServoMoving poll")
		}
	}
	close(pollerDone)
	wg.Wait()

	assert.Empty(t, tracker.Violations(),
		"a position SyncRead's request/response must never be split by a AnyServoMoving SyncRead's bytes")
}

// latencyStats returns the mean and median of a set of durations.
func latencyStats(samples []time.Duration) (mean, median time.Duration) {
	if len(samples) == 0 {
		return 0, 0
	}
	var total time.Duration
	sorted := append([]time.Duration(nil), samples...)
	for _, d := range sorted {
		total += d
	}
	sort.Slice(sorted, func(i, j int) bool { return sorted[i] < sorted[j] })
	return total / time.Duration(len(sorted)), sorted[len(sorted)/2]
}

// TestBusContentionTiming measures, rather than asserts, the cost a concurrent AnyServoMoving
// poll adds to position reads on the shared bus. It logs raw numbers instead of asserting
// wall-clock thresholds, since those are inherently noisy in CI; the only assertions here are
// that every call still succeeds. See the file doc comment for the mechanism (feetech.Bus's
// own mutex, held across each transaction's 1ms MinCommandGap sleep) that these numbers are
// expected to reflect: a queued caller pays the gap of whatever transaction is ahead of it,
// not just its own.
func TestBusContentionTiming(t *testing.T) {
	ft := testfake.NewFakeTransport()
	h, _ := trackedTestHandle(t, ft)
	ctx := context.Background()
	armIDs := []int{1, 2, 3, 4, 5}

	const n = 150
	// 50Hz: the fast end of the teleop setpoint-streaming rate this module targets
	// (30-50Hz); WaitForServosToStop itself only polls at 20Hz (every 50ms).
	const pollInterval = 20 * time.Millisecond

	measure := func() []time.Duration {
		out := make([]time.Duration, n)
		for i := 0; i < n; i++ {
			start := time.Now()
			_, err := h.SyncReadPositions(ctx, armIDs)
			out[i] = time.Since(start)
			require.NoError(t, err)
		}
		return out
	}

	baseline := measure()

	stop := make(chan struct{})
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		ticker := time.NewTicker(pollInterval)
		defer ticker.Stop()
		for {
			select {
			case <-stop:
				return
			case <-ticker.C:
				_, _ = h.AnyServoMoving(ctx, armIDs)
			}
		}
	}()
	contended := measure()
	close(stop)
	wg.Wait()

	movingAlone := make([]time.Duration, 50)
	for i := range movingAlone {
		start := time.Now()
		_, err := h.AnyServoMoving(ctx, armIDs)
		movingAlone[i] = time.Since(start)
		require.NoError(t, err)
	}

	baseMean, baseMedian := latencyStats(baseline)
	contMean, contMedian := latencyStats(contended)
	movMean, movMedian := latencyStats(movingAlone)

	t.Logf("SyncReadPositions baseline (n=%d, no concurrent poll):        mean=%v median=%v", n, baseMean, baseMedian)
	t.Logf("SyncReadPositions under AnyServoMoving poll (n=%d, every %v):   mean=%v median=%v", n, pollInterval, contMean, contMedian)
	t.Logf("Contention delta:                                             mean=%v median=%v", contMean-baseMean, contMedian-baseMedian)
	t.Logf("AnyServoMoving alone (n=%d):                                    mean=%v median=%v", len(movingAlone), movMean, movMedian)
}

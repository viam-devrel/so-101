package controller_test

import (
	"context"
	"fmt"
	"os"
	"sort"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"

	"so_arm/internal/controller"
)

// Hardware-only latency measurement for the AnyServoMoving poll added for issue #22.
// Skipped unless SO101_PORT names a real serial port:
//
//	SO101_PORT=/dev/cu.usbmodemXXXX go test ./internal/controller -run HardwareBusLatency -v
//
// Read-only: it issues position and Moving reads and never writes a servo register, so it
// cannot move the arm or change its torque state.

const (
	latencyWarmup  = 20
	latencySamples = 300
	teleopCadence  = 20 * time.Millisecond // 50 Hz, the teleop setpoint rate
)

type latencyStats struct {
	mean, median, p95, max time.Duration
	errs                   int
	lastErr                error
}

func summarize(samples []time.Duration) latencyStats {
	sorted := append([]time.Duration(nil), samples...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i] < sorted[j] })
	var total time.Duration
	for _, s := range sorted {
		total += s
	}
	return latencyStats{
		mean:   total / time.Duration(len(sorted)),
		median: sorted[len(sorted)/2],
		p95:    sorted[(len(sorted)*95)/100],
		max:    sorted[len(sorted)-1],
	}
}

func (s latencyStats) String() string {
	out := "mean=" + s.mean.String() + " median=" + s.median.String() +
		" p95=" + s.p95.String() + " max=" + s.max.String()
	if s.errs > 0 {
		out += fmt.Sprintf("  ERRORS=%d/%d (%.1f%%) last=%v",
			s.errs, latencySamples, 100*float64(s.errs)/float64(latencySamples), s.lastErr)
	}
	return out
}

// timeCalls runs f latencySamples times, timing the calls that succeed and counting the
// ones that do not. Read failures are the measurement here, not a reason to abort: under
// heavy bus contention they are exactly what we are trying to quantify. Warmup still
// requires success, so a disconnected arm fails loudly instead of reporting 100% errors.
func timeCalls(t *testing.T, f func() error) latencyStats {
	t.Helper()
	for i := 0; i < latencyWarmup; i++ {
		require.NoError(t, f())
	}
	samples := make([]time.Duration, 0, latencySamples)
	errs, lastErr := 0, error(nil)
	for i := 0; i < latencySamples; i++ {
		start := time.Now()
		err := f()
		elapsed := time.Since(start)
		if err != nil {
			errs++
			lastErr = err
			continue
		}
		samples = append(samples, elapsed)
	}
	require.NotEmpty(t, samples, "every call failed: %v", lastErr)
	stats := summarize(samples)
	stats.errs, stats.lastErr = errs, lastErr
	return stats
}

func TestHardwareBusLatency(t *testing.T) {
	port := os.Getenv("SO101_PORT")
	if port == "" {
		t.Skip("set SO101_PORT to run against real hardware")
	}

	logger := logging.NewTestLogger(t)
	allIDs := []int{1, 2, 3, 4, 5, 6}
	armIDs := []int{1, 2, 3, 4, 5}

	// fromFile=true with the default calibration so acquiring does not also run
	// ReadCalibrationFromServos -- this test measures the bus, not controller startup.
	reg := controller.NewControllerRegistry()
	h, err := reg.AcquireController(
		arm.Named("latency-test"),
		&controller.SoArm101Config{Port: port, ServoIDs: allIDs, Timeout: time.Second, Logger: logger},
		controller.DefaultSO101FullCalibration, true,
	)
	require.NoError(t, err)
	t.Cleanup(h.Release)

	ctx := context.Background()
	require.NoError(t, h.Ping(ctx), "no servos answered on %s", port)

	positions, err := h.SyncReadPositions(ctx, allIDs)
	require.NoError(t, err)
	t.Logf("servos answering: %d of %d, ticks=%v", len(positions), len(allIDs), positions)

	// 1. Position SyncRead alone -- the transaction every move and every JointPositions does.
	posAlone := timeCalls(t, func() error {
		_, err := h.SyncReadPositions(ctx, allIDs)
		return err
	})
	t.Logf("SyncReadPositions(6) alone:            %s", posAlone)

	// 2. AnyServoMoving alone -- the new IsMoving transaction.
	movingAlone := timeCalls(t, func() error {
		_, err := h.AnyServoMoving(ctx, armIDs)
		return err
	})
	t.Logf("AnyServoMoving(5) alone:               %s", movingAlone)

	// 3. The pre-batching cost: one Moving read per servo, as WaitForServosToStop used to do.
	perServo := timeCalls(t, func() error {
		for _, id := range armIDs {
			if _, err := h.ReadServoRegister(ctx, id, "moving"); err != nil {
				return err
			}
		}
		return nil
	})
	t.Logf("per-servo Moving reads (5, old path):  %s", perServo)

	readPositions := func() error {
		_, err := h.SyncReadPositions(ctx, allIDs)
		return err
	}

	// 4. Position reads with a concurrent Moving poller at the teleop setpoint rate.
	stop := startPoller(t, h, armIDs, teleopCadence)
	posUnder50Hz := timeCalls(t, readPositions)
	stop()
	t.Logf("SyncReadPositions(6) under 50Hz poll:  %s  (delta mean %s)",
		posUnder50Hz, posUnder50Hz.mean-posAlone.mean)

	// 5. Worst case: a poller with no cadence at all, i.e. permanent bus contention.
	stop = startPoller(t, h, armIDs, 0)
	posUnhinged := timeCalls(t, readPositions)
	stop()
	t.Logf("SyncReadPositions(6) under max poll:   %s  (delta mean %s)",
		posUnhinged, posUnhinged.mean-posAlone.mean)
}

// startPoller runs a background AnyServoMoving poller on the same bus at the given cadence
// (0 = as fast as the bus allows). Call the returned stop func before the next measurement:
// a poller left running would contaminate every step after its own.
func startPoller(t *testing.T, h *controller.ControllerHandle, ids []int, cadence time.Duration) func() {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		for ctx.Err() == nil {
			// Keep polling through errors: bailing here would silently remove the very
			// contention this poller exists to apply.
			_, _ = h.AnyServoMoving(ctx, ids)
			if cadence > 0 {
				select {
				case <-ctx.Done():
				case <-time.After(cadence):
				}
			}
		}
	}()
	stopped := false
	stop := func() {
		if stopped {
			return
		}
		stopped = true
		cancel()
		<-done
	}
	t.Cleanup(stop)
	return stop
}

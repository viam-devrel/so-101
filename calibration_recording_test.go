package so_arm

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
)

// testRecordingSensor builds a sensor field-by-field around a fake-backed handle.
// NewSO101CalibrationSensor acquires through the process-wide registry, which needs real
// serial hardware.
func testRecordingSensor(t *testing.T, ft *fakeTransport) *so101CalibrationSensor {
	t.Helper()
	h := testHandle(t, ft)
	servoIDs := []int{1, 2, 3, 4, 5, 6}

	joints := make(map[int]*JointCalibrationData, len(servoIDs))
	for _, id := range servoIDs {
		joints[id] = &JointCalibrationData{
			RecordedMin: math.MaxInt32,
			RecordedMax: math.MinInt32,
		}
	}

	return &so101CalibrationSensor{
		name:       sensor.Named("test-calibration"),
		logger:     logging.NewTestLogger(t),
		cfg:        &SO101CalibrationSensorConfig{ServoIDs: servoIDs},
		controller: h,
		state:      StateHomingPosition,
		joints:     joints,
		servoNames: map[int]string{1: "shoulder_pan"},
	}
}

func TestClosingDuringRecordingJoinsTheGoroutine(t *testing.T) {
	cs := testRecordingSensor(t, newFakeTransport())

	cs.mu.Lock()
	_, err := cs.startRangeRecording(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)

	// Let the 10ms ticker actually get into a cycle, so Close has something to join.
	time.Sleep(50 * time.Millisecond)

	cs.mu.RLock()
	done := cs.recordingDone
	cs.mu.RUnlock()
	require.NotNil(t, done, "a running recording must publish a done channel to join")

	closed := make(chan error, 1)
	go func() { closed <- cs.Close(context.Background()) }()

	select {
	case err := <-closed:
		require.NoError(t, err)
	case <-time.After(5 * time.Second):
		t.Fatal("Close deadlocked: it is holding cs.mu across the join, and the recording " +
			"goroutine needs that lock every tick")
	}

	select {
	case <-done:
	default:
		t.Fatal("Close returned before the recording goroutine exited, so Release would " +
			"close the bus under a live read")
	}
}

func TestStopRangeRecordingLetsTheGoroutineExit(t *testing.T) {
	cs := testRecordingSensor(t, newFakeTransport())

	cs.mu.Lock()
	_, err := cs.startRangeRecording(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)

	cs.mu.RLock()
	done := cs.recordingDone
	cs.mu.RUnlock()
	require.NotNil(t, done)

	cs.mu.Lock()
	// Reports invalid ranges, since nothing moved. This test is about the goroutine.
	_, _ = cs.stopRangeRecording(context.Background())
	cs.mu.Unlock()

	// stopRangeRecording does not join: the port stays open, so there is nothing to guard
	// against. The goroutine must still exit on its own.
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("recording goroutine outlived stop_range_recording")
	}

	assert.NoError(t, cs.Close(context.Background()), "Close after stop must not hang")
}

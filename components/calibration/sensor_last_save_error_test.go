package calibration

import (
	"context"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// TestLastSaveErrorSurfacesOnFailureAndClearsOnRetrySuccess is the regression test for the
// other loose end left by the "Retry Save can never succeed" fix: saveCalibration now keeps
// StateCompleted on failure (see workflow.go), so the durable readings().error field (which
// setState only ever populates for StateError) goes empty on a save failure. This drives a
// real save_calibration failure through DoCommand -- the seam that owns lastSaveError -- and
// asserts Readings() surfaces it as last_save_error, then that a subsequent successful retry
// clears it again.
func TestLastSaveErrorSurfacesOnFailureAndClearsOnRetrySuccess(t *testing.T) {
	failing := &failNWritesToServo{
		FakeTransport: testfake.NewFakeTransport(),
		targetServo:   3,
		failsLeft:     1,
	}

	cs := testRecordingSensor(t, failing)
	cs.state = StateCompleted
	cs.cfg.CalibrationFile = filepath.Join(t.TempDir(), "so101_calibration.json")

	for id, joint := range cs.joints {
		joint.HomingOffset = id
		joint.RangeMin = 500 + id*10
		joint.RangeMax = 3500 + id*10
		joint.IsCompleted = true
	}

	ctx := context.Background()

	// First attempt fails on servo 3.
	_, err := cs.DoCommand(ctx, map[string]any{"command": "save_calibration"})
	require.Error(t, err)

	readings, rErr := cs.Readings(ctx, nil)
	require.NoError(t, rErr)
	lastSaveErr, ok := readings["last_save_error"].(string)
	require.True(t, ok, "a save failure must durably surface last_save_error in Readings")
	assert.NotEmpty(t, lastSaveErr)
	assert.Contains(t, readings, "calibration_state")
	assert.Equal(t, StateCompleted.String(), readings["calibration_state"],
		"the state must still be StateCompleted, per the fix this field complements")

	// A save failure must not fabricate a StateError-only error field. The positive control
	// below keeps that assertion honest: readings.error really is emitted, just only for
	// StateError, so its absence above is about the state and not a key Readings never sets.
	_, hasError := readings["error"]
	assert.False(t, hasError, "readings.error is reserved for StateError, not a completed-state save failure")

	func() {
		cs.mu.Lock()
		saved := cs.state
		cs.state = StateError
		cs.errorMsg = "positive control"
		cs.mu.Unlock()
		defer func() {
			cs.mu.Lock()
			cs.state = saved
			cs.errorMsg = ""
			cs.mu.Unlock()
		}()

		errReadings, err := cs.Readings(ctx, nil)
		require.NoError(t, err)
		assert.Equal(t, "positive control", errReadings["error"])
	}()

	// Retry succeeds once the transport recovers; the stale failure must not survive.
	_, retryErr := cs.DoCommand(ctx, map[string]any{"command": "save_calibration"})
	require.NoError(t, retryErr)

	readingsAfter, rErr2 := cs.Readings(ctx, nil)
	require.NoError(t, rErr2)
	_, stillPresent := readingsAfter["last_save_error"]
	assert.False(t, stillPresent, "a successful save must clear last_save_error")
}

// TestLastSaveErrorClearedByStartResetAndAbort pins the other half of the contract: a stale
// save failure from a prior session must not haunt the next one. start and reset are reachable
// from a save failure's resting state (StateCompleted); abort is not (it is rejected there),
// so its subtest seeds the flag at StateStarted instead -- what is under test is the dispatch
// rule that a *successful* abort clears it, not the reachability of that pairing.
func TestLastSaveErrorClearedByStartResetAndAbort(t *testing.T) {
	ctx := context.Background()

	t.Run("start clears it", func(t *testing.T) {
		cs := testRecordingSensor(t, testfake.NewFakeTransport())
		cs.state = StateCompleted
		cs.lastSaveError = "a previous save_calibration attempt failed"

		_, err := cs.DoCommand(ctx, map[string]any{"command": "start"})
		require.NoError(t, err)

		readings, rErr := cs.Readings(ctx, nil)
		require.NoError(t, rErr)
		_, present := readings["last_save_error"]
		assert.False(t, present, "start must clear a stale last_save_error")
	})

	t.Run("reset clears it", func(t *testing.T) {
		cs := testRecordingSensor(t, testfake.NewFakeTransport())
		cs.state = StateCompleted
		cs.lastSaveError = "a previous save_calibration attempt failed"

		_, err := cs.DoCommand(ctx, map[string]any{"command": "reset"})
		require.NoError(t, err)

		readings, rErr := cs.Readings(ctx, nil)
		require.NoError(t, rErr)
		_, present := readings["last_save_error"]
		assert.False(t, present, "reset must clear a stale last_save_error")
	})

	t.Run("abort clears it", func(t *testing.T) {
		cs := testRecordingSensor(t, testfake.NewFakeTransport())
		// StateStarted, not StateCompleted: abort is rejected at completed so it would not
		// clear anything there (see abortCalibration's guard).
		cs.state = StateStarted
		cs.lastSaveError = "a previous save_calibration attempt failed"

		_, err := cs.DoCommand(ctx, map[string]any{"command": "abort"})
		require.NoError(t, err)

		readings, rErr := cs.Readings(ctx, nil)
		require.NoError(t, rErr)
		_, present := readings["last_save_error"]
		assert.False(t, present, "abort must clear a stale last_save_error")
	})
}

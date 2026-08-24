package calibration

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// allCalibrationStates lists every CalibrationState, for tests that must hold across all of
// them rather than pick one representative.
var allCalibrationStates = []CalibrationState{
	StateIdle, StateStarted, StateHomingPosition, StateRangeRecording, StateCompleted, StateError,
}

// TestResetIsUnguardedFromEveryState is the claim FIX 1 relies on: resetCalibration has no
// state check, so it must succeed and land at StateIdle no matter where it starts. This
// verifies that claim directly rather than trusting the read of workflow.go.
func TestResetIsUnguardedFromEveryState(t *testing.T) {
	for _, start := range allCalibrationStates {
		t.Run(start.String(), func(t *testing.T) {
			cs := testRecordingSensor(t, testfake.NewFakeTransport())
			cs.state = start
			cs.errorMsg = "some prior error"
			for _, joint := range cs.joints {
				joint.HomingOffset = 7
				joint.RangeMin = 111
				joint.RangeMax = 222
				joint.RecordedMin = 111
				joint.RecordedMax = 222
				joint.IsCompleted = true
			}

			cs.mu.Lock()
			result, err := cs.resetCalibration(context.Background())
			cs.mu.Unlock()

			require.NoError(t, err, "reset must be accepted from state %s", start)
			assert.Equal(t, true, result["success"])
			assert.Equal(t, StateIdle, cs.state)
			assert.Empty(t, cs.errorMsg)
			for id, joint := range cs.joints {
				assert.Equal(t, 0, joint.HomingOffset, "servo %d homing offset", id)
				assert.Equal(t, 0, joint.RangeMin, "servo %d range min", id)
				assert.Equal(t, 4095, joint.RangeMax, "servo %d range max", id)
				assert.Equal(t, math.MaxInt32, joint.RecordedMin, "servo %d recorded min", id)
				assert.Equal(t, math.MinInt32, joint.RecordedMax, "servo %d recorded max", id)
				assert.False(t, joint.IsCompleted, "servo %d is_completed", id)
			}
		})
	}
}

// TestAvailableCommandsAlwaysAdvertiseReset pins FIX 1 at the Readings layer: since reset is
// accepted from every state, it must be advertised from every state too.
func TestAvailableCommandsAlwaysAdvertiseReset(t *testing.T) {
	for _, state := range allCalibrationStates {
		t.Run(state.String(), func(t *testing.T) {
			cs := testRecordingSensor(t, testfake.NewFakeTransport())
			cs.state = state

			readings, err := cs.Readings(context.Background(), nil)
			require.NoError(t, err)

			cmds, ok := readings["available_commands"].([]any)
			require.True(t, ok, "available_commands must be a []any")
			assert.Contains(t, cmds, "reset", "state %s must advertise reset", state)
		})
	}
}

// TestAbortRejectedAtCompleted is the regression guard for FIX 2: a completed recording is
// waiting on save_calibration, so abort must refuse rather than silently discard it.
func TestAbortRejectedAtCompleted(t *testing.T) {
	cs := testRecordingSensor(t, testfake.NewFakeTransport())
	cs.state = StateCompleted
	for id, joint := range cs.joints {
		joint.HomingOffset = id
		joint.RangeMin = 500 + id*10
		joint.RangeMax = 3500 + id*10
		joint.IsCompleted = true
	}
	cs.positionHistory = []map[int]int{{1: 1000}, {1: 1001}}

	cs.mu.Lock()
	result, err := cs.abortCalibration(context.Background())
	cs.mu.Unlock()

	require.Error(t, err, "abort must be rejected at StateCompleted")
	assert.Equal(t, false, result["success"])
	assert.Equal(t, StateCompleted, cs.state, "the guard must not itself change state")

	// The whole point: the recorded ranges from range recording must survive the rejected
	// abort attempt untouched.
	for id, joint := range cs.joints {
		assert.Equal(t, id, joint.HomingOffset, "servo %d homing offset", id)
		assert.Equal(t, 500+id*10, joint.RangeMin, "servo %d range min", id)
		assert.Equal(t, 3500+id*10, joint.RangeMax, "servo %d range max", id)
		assert.True(t, joint.IsCompleted, "servo %d is_completed", id)
	}
	assert.Len(t, cs.positionHistory, 2, "recorded position history must survive the rejected abort")

	// The other half of the contract: what is rejected must not be advertised.
	readings, rErr := cs.Readings(context.Background(), nil)
	require.NoError(t, rErr)
	cmds, ok := readings["available_commands"].([]any)
	require.True(t, ok)
	assert.NotContains(t, cmds, "abort", "completed must not advertise a command it rejects")
}

// TestAbortAtIdleIsNoopSuccess: idle has nothing to abort, so this treats it as a harmless
// no-op rather than an error -- but the response must not read as though it performed an
// action, so callers (and users) can tell nothing changed.
func TestAbortAtIdleIsNoopSuccess(t *testing.T) {
	cs := testRecordingSensor(t, testfake.NewFakeTransport())
	cs.state = StateIdle
	cs.lastInstruction = "some earlier instruction"

	cs.mu.Lock()
	result, err := cs.abortCalibration(context.Background())
	cs.mu.Unlock()

	require.NoError(t, err, "abort at idle must not error")
	assert.Equal(t, true, result["success"])
	assert.Equal(t, StateIdle, cs.state)
	msg, _ := result["message"].(string)
	assert.NotContains(t, msg, "aborted", "the message must not claim an abort happened")
}

// TestAbortStillAcceptedAtAdvertisedStates pins the unchanged half of FIX 2: abort must keep
// working normally from every state it is actually advertised for.
func TestAbortStillAcceptedAtAdvertisedStates(t *testing.T) {
	for _, state := range []CalibrationState{StateStarted, StateHomingPosition, StateRangeRecording} {
		t.Run(state.String(), func(t *testing.T) {
			cs := testRecordingSensor(t, testfake.NewFakeTransport())
			cs.state = state

			cs.mu.Lock()
			result, err := cs.abortCalibration(context.Background())
			cs.mu.Unlock()

			require.NoError(t, err)
			assert.Equal(t, true, result["success"])
			assert.Equal(t, StateIdle, cs.state)
		})
	}
}

// TestResetDuringLiveRecordingDoesNotDeadlock is the safety proof behind advertising reset in
// StateRangeRecording. DoCommand holds cs.mu for the whole dispatch while recordPositions
// takes it every 10ms tick, so a reset that waited on the goroutine would deadlock exactly
// the way Close's comment describes. resetCalibration must only cancel, never join.
func TestResetDuringLiveRecordingDoesNotDeadlock(t *testing.T) {
	// A deliberately slow transport, so the recording goroutine is almost always *mid-cycle*
	// (past its select, inside SyncReadPositions or waiting on cs.mu) when the reset lands.
	// With an instant transport the goroutine is nearly always parked in its select, where
	// cancel alone frees it and a joining reset would pass by luck -- see Close's comment.
	cs := testRecordingSensor(t, &slowTransport{
		FakeTransport: testfake.NewFakeTransport(),
		delay:         8 * time.Millisecond,
	})

	cs.mu.Lock()
	_, err := cs.startRangeRecording(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)
	require.Equal(t, StateRangeRecording, cs.state)

	// Let the 10ms ticker get into a cycle, so the goroutine is actually contending for cs.mu.
	time.Sleep(60 * time.Millisecond)

	cs.mu.RLock()
	done := cs.recordingDone
	cs.mu.RUnlock()
	require.NotNil(t, done, "a running recording must publish a done channel")

	// Through DoCommand deliberately: the lock scope is what makes this a hazard.
	returned := make(chan error, 1)
	go func() {
		_, e := cs.DoCommand(context.Background(), map[string]any{"command": "reset"})
		returned <- e
	}()

	select {
	case e := <-returned:
		require.NoError(t, e)
	case <-time.After(5 * time.Second):
		t.Fatal("reset deadlocked during range_recording: DoCommand holds cs.mu for the whole " +
			"dispatch and the recording goroutine needs that lock every tick, so resetCalibration " +
			"must cancel without joining")
	}

	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("recording goroutine outlived reset")
	}

	cs.mu.RLock()
	assert.Equal(t, StateIdle, cs.state)
	assert.False(t, cs.recordingActive)
	cs.mu.RUnlock()

	assert.NoError(t, cs.Close(context.Background()), "Close after reset must not hang")
}

// slowTransport delays every read so a bus round trip outlasts the recorder's 10ms tick.
type slowTransport struct {
	*testfake.FakeTransport
	delay time.Duration
}

func (s *slowTransport) Read(p []byte) (int, error) {
	time.Sleep(s.delay)
	return s.FakeTransport.Read(p)
}

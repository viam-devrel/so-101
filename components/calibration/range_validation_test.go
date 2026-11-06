package calibration

import (
	"context"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
)

func TestRecordedRangeProblem(t *testing.T) {
	assert.Empty(t, recordedRangeProblem(700, 3400))
	assert.Empty(t, recordedRangeProblem(0, 4095))

	// The encoder wrap seen in the field: wrist_flex homed near one end of its travel.
	assert.Contains(t, recordedRangeProblem(2, 32973),
		"recorded range [2, 32973] is outside the encoder's [0, 4095]: re-home with this joint at the middle of its travel")

	for name, r := range map[string][2]int{
		"negative min": {-1, 2000},
		"above max":    {100, 4096},
	} {
		assert.Contains(t, recordedRangeProblem(r[0], r[1]), "outside the encoder", name)
	}
	for name, r := range map[string][2]int{
		"empty span":  {2000, 2000},
		"inverted":    {3000, 1000},
		"never moved": {math.MaxInt32, math.MinInt32},
	} {
		assert.Contains(t, recordedRangeProblem(r[0], r[1]), "recorded no range", name)
	}
}

func rangeTestSensor(t *testing.T, ranges map[int][2]int) *so101CalibrationSensor {
	names := map[int]string{1: "shoulder_pan", 4: "wrist_flex"}
	joints := map[int]*JointCalibrationData{}
	for id, r := range ranges {
		joints[id] = &JointCalibrationData{ID: id, Name: names[id], RecordedMin: r[0], RecordedMax: r[1]}
	}
	return &so101CalibrationSensor{
		logger: logging.NewTestLogger(t),
		state:  StateRangeRecording,
		joints: joints,
	}
}

// A wrapped range must land in StateError, which offers reset, not StateCompleted, which
// advertises save_calibration.
func TestStopRangeRecordingRejectsAWrappedRange(t *testing.T) {
	cs := rangeTestSensor(t, map[int][2]int{1: {700, 3400}, 4: {2, 32973}})
	res, err := cs.stopRangeRecording(context.Background())
	require.Error(t, err)
	assert.Equal(t, false, res["success"])
	assert.Equal(t, StateError, cs.state)
	assert.Contains(t, err.Error(), "servo 4 (wrist_flex) recorded range [2, 32973]")
	assert.NotContains(t, err.Error(), "shoulder_pan")
	assert.Contains(t, cs.lastInstruction, "Reset and calibrate again")
}

func TestStopRangeRecordingAcceptsValidRanges(t *testing.T) {
	cs := rangeTestSensor(t, map[int][2]int{1: {700, 3400}, 4: {0, 4095}})
	_, err := cs.stopRangeRecording(context.Background())
	require.NoError(t, err)
	assert.Equal(t, StateCompleted, cs.state)
	assert.Equal(t, 4095, cs.joints[4].RangeMax)
}

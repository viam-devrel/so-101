package calibration

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestValidateRecordedRanges(t *testing.T) {
	good := func() map[int]*JointCalibrationData {
		return map[int]*JointCalibrationData{
			1: {ID: 1, Name: "shoulder_pan", RangeMin: 700, RangeMax: 3400},
			4: {ID: 4, Name: "wrist_flex", RangeMin: 0, RangeMax: 4095},
		}
	}
	require.NoError(t, validateRecordedRanges(good()))

	// The encoder wrap seen in the field: wrist_flex homed near one end of its travel.
	joints := good()
	joints[4].RangeMin, joints[4].RangeMax = 2, 32973
	err := validateRecordedRanges(joints)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "servo 4 (wrist_flex) recorded range [2, 32973]")
	assert.Contains(t, err.Error(), "re-home with this joint at the middle of its travel, then record again")
	assert.NotContains(t, err.Error(), "shoulder_pan")

	for name, r := range map[string][2]int{
		"negative min": {-1, 2000},
		"empty span":   {2000, 2000},
		"inverted":     {3000, 1000},
		"above max":    {100, 4096},
	} {
		joints := good()
		joints[1].RangeMin, joints[1].RangeMax = r[0], r[1]
		err := validateRecordedRanges(joints)
		require.Error(t, err, name)
		assert.Contains(t, err.Error(), "servo 1 (shoulder_pan)", name)
	}
}

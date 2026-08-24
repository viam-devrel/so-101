package calibration

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// decodeSignMagnitude11 mirrors readInt16Register's decode (internal/controller/config.go),
// which is the read path this write path must round-trip through.
func decodeSignMagnitude11(raw int) int {
	const signBit = 11
	direction := (raw >> signBit) & 1
	magnitude := raw & ((1 << signBit) - 1)
	if direction != 0 {
		return -magnitude
	}
	return magnitude
}

// TestWriteHomingOffsetEncodesSignMagnitude is the regression test for writeHomingOffset:
// it used to ignore its homingOffset argument entirely and write the constant 128 to
// torque_enable. It must now write the actual offset, sign-magnitude encoded at
// feetech.RegPositionOffset's bit 11, to position_offset -- the same register and encoding
// readInt16Register (and so ReadCalibrationFromServos, the no-file fallback) already expect.
func TestWriteHomingOffsetEncodesSignMagnitude(t *testing.T) {
	cases := []struct {
		name   string
		offset int
	}{
		{"positive", 47},
		{"negative", -103},
		{"zero", 0},
		{"largest positive", 2047},
		{"largest negative", -2047},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cs := testRecordingSensor(t, testfake.NewFakeTransport())

			require.NoError(t, cs.writeHomingOffset(context.Background(), 1, tc.offset))

			data, err := cs.controller.ReadServoRegister(context.Background(), 1, "position_offset")
			require.NoError(t, err)
			require.Len(t, data, 2)

			raw := int(data[0]) | int(data[1])<<8 // little-endian, matching writeMinPositionLimit etc.
			assert.Equal(t, tc.offset, decodeSignMagnitude11(raw))
		})
	}
}

// TestWriteHomingOffsetRejectsOverflow guards the sign-bit boundary: a magnitude of 2048
// would collide with feetech.RegPositionOffset's own sign bit (2048 == 1<<11) and silently
// decode back as 0, so it must be rejected rather than written.
func TestWriteHomingOffsetRejectsOverflow(t *testing.T) {
	cs := testRecordingSensor(t, testfake.NewFakeTransport())

	err := cs.writeHomingOffset(context.Background(), 1, 2048)
	assert.Error(t, err)
}

// TestSetHomingPositionPersistsOffsetToRegister exercises the real caller: setHomingPosition
// computes homingOffset = currentRawPos - 2047, and the fix's contract is that this exact
// value round-trips through the servo register afterward.
func TestSetHomingPositionPersistsOffsetToRegister(t *testing.T) {
	ft := testfake.NewFakeTransport()
	// Servo 1 sits at raw position 1980, so the expected homing offset is 1980-2047 = -67.
	ft.SetRegister(1, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(1980))

	cs := testRecordingSensor(t, ft)
	cs.state = StateStarted

	cs.mu.Lock()
	result, err := cs.setHomingPosition(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)
	assert.Equal(t, StateHomingPosition, cs.state)

	homingOffsets, ok := result["homing_offsets"].(map[string]any)
	require.True(t, ok)
	assert.Equal(t, -67, homingOffsets["1"])

	data, err := cs.controller.ReadServoRegister(context.Background(), 1, "position_offset")
	require.NoError(t, err)
	raw := int(data[0]) | int(data[1])<<8
	assert.Equal(t, -67, decodeSignMagnitude11(raw), "the register must hold the same offset reported back to the caller")
}

// TestSetHomingPositionClampsUnencodableOffset covers the one pose whose computed offset
// (currentRawPos - 2047) does not fit position_offset's 11-bit magnitude: a joint sitting at
// raw 4095 yields 2048, which collides with the sign bit and would decode back as 0. That is
// a legal pose, so set_homing must trim the single tick and carry on rather than failing the
// workflow -- while encodePositionOffset stays strict for anything wider.
func TestSetHomingPositionClampsUnencodableOffset(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(1, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(4095))

	cs := testRecordingSensor(t, ft)
	cs.state = StateStarted

	cs.mu.Lock()
	result, err := cs.setHomingPosition(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err, "a joint at the top of the encoder must not fail set_homing")
	assert.Equal(t, StateHomingPosition, cs.state)

	homingOffsets, ok := result["homing_offsets"].(map[string]any)
	require.True(t, ok)
	assert.Equal(t, 2047, homingOffsets["1"], "2048 must be trimmed to the largest encodable magnitude")

	data, err := cs.controller.ReadServoRegister(context.Background(), 1, "position_offset")
	require.NoError(t, err)
	raw := int(data[0]) | int(data[1])<<8
	assert.Equal(t, 2047, decodeSignMagnitude11(raw))
}

package calibration

import (
	"context"
	"path/filepath"
	"syscall"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
	"so_arm/internal/testfake"
)

// failNWritesToServo wraps a FakeTransport and fails the first n raw WRITE/REG_WRITE packets
// addressed to one servo ID with syscall.EIO (the same failure Unplug simulates), then lets
// every later packet through untouched. It lets a test reproduce a save_calibration that
// fails partway through the min/max-limit loop and then succeeds on retry.
type failNWritesToServo struct {
	*testfake.FakeTransport
	targetServo int
	failsLeft   int
}

func (f *failNWritesToServo) Write(p []byte) (int, error) {
	if f.failsLeft > 0 && isWriteToServo(p, f.targetServo) {
		f.failsLeft--
		return 0, syscall.EIO
	}
	return f.FakeTransport.Write(p)
}

// isWriteToServo reports whether p is a raw Feetech WRITE or REG_WRITE packet addressed to
// servoID. Packet layout: 0xFF 0xFF id length instruction params... checksum.
func isWriteToServo(p []byte, servoID int) bool {
	if len(p) < 5 || p[0] != 0xFF || p[1] != 0xFF {
		return false
	}
	id, instruction := int(p[2]), p[4]
	return id == servoID && (instruction == 0x03 || instruction == 0x04)
}

// decodeWord decodes a little-endian 2-byte Feetech register value, the format
// writeMinPositionLimit/writeMaxPositionLimit encode and FakeTransport stores verbatim.
func decodeWord(data []byte) int {
	return int(feetech.NewProtocol(feetech.ProtocolSTS).DecodeWord(data))
}

// TestSaveCalibrationFailureKeepsStateCompletedAndRetrySucceeds is the regression test for
// the "Retry Save can never succeed" bug: saveCalibration used to move to StateError on any
// write failure, and its own guard then refused every retry ("calibration not completed").
// This drives a real failure through a fake transport (no hardware needed, since
// feetech.BusConfig.Transport is exported) and asserts the state stays retryable and that a
// second, successful call actually reaches every servo and the file.
func TestSaveCalibrationFailureKeepsStateCompletedAndRetrySucceeds(t *testing.T) {
	failing := &failNWritesToServo{
		FakeTransport: testfake.NewFakeTransport(),
		targetServo:   3,
		failsLeft:     1,
	}

	cs := testRecordingSensor(t, failing)
	cs.state = StateCompleted
	cs.cfg.CalibrationFile = filepath.Join(t.TempDir(), "so101_calibration.json")

	// Seed every joint with a distinct, already-recorded range, as range recording would
	// have left them.
	for id, joint := range cs.joints {
		joint.HomingOffset = id
		joint.RangeMin = 500 + id*10
		joint.RangeMax = 3500 + id*10
		joint.IsCompleted = true
	}

	ctx := context.Background()

	// First attempt: servo 3's write fails.
	cs.mu.Lock()
	_, err := cs.saveCalibration(ctx)
	cs.mu.Unlock()
	require.Error(t, err, "the injected servo-3 failure must surface as the DoCommand error")
	assert.Equal(t, StateCompleted, cs.state,
		"a save failure must stay at StateCompleted so save_calibration remains retryable")

	// Retry: same in-memory calibration, transport no longer failing.
	cs.mu.Lock()
	result, retryErr := cs.saveCalibration(ctx)
	cs.mu.Unlock()
	require.NoError(t, retryErr, "the retry must succeed once the transport recovers")
	assert.Equal(t, true, result["success"])
	assert.Equal(t, StateIdle, cs.state, "a successful save must still reach StateIdle")
	assert.Equal(t, cs.cfg.CalibrationFile, result["calibration_file"])
	assert.Equal(t, len(cs.joints), result["joints_calibrated"])

	// Every servo -- not just the one that failed the first time -- must carry the in-memory
	// calibration after the retry: the loop rewrites all of them unconditionally rather than
	// resuming only where it left off.
	for id, joint := range cs.joints {
		minReg, err := cs.controller.ReadServoRegister(ctx, id, "min_angle_limit")
		require.NoError(t, err)
		maxReg, err := cs.controller.ReadServoRegister(ctx, id, "max_angle_limit")
		require.NoError(t, err)
		assert.Equal(t, joint.RangeMin, decodeWord(minReg), "servo %d min limit", id)
		assert.Equal(t, joint.RangeMax, decodeWord(maxReg), "servo %d max limit", id)
	}

	// The file must reflect the same calibration that ended up on the servos, proving the
	// two never permanently disagree once a retry succeeds.
	saved, err := controller.LoadFullCalibrationFromFile(cs.cfg.CalibrationFile, cs.logger)
	require.NoError(t, err)
	require.NotNil(t, saved.ElbowFlex)
	assert.Equal(t, cs.joints[3].RangeMin, saved.ElbowFlex.RangeMin)
	assert.Equal(t, cs.joints[3].RangeMax, saved.ElbowFlex.RangeMax)
	assert.Equal(t, cs.joints[3].HomingOffset, saved.ElbowFlex.HomingOffset)
}

// TestSaveCalibrationGuardStillRejectsWrongState pins the unchanged half of the contract:
// saveCalibration must still refuse to run from any state other than StateCompleted (this
// fix only had to stop that guard from being reached on a save's OWN failure path).
func TestSaveCalibrationGuardStillRejectsWrongState(t *testing.T) {
	cs := testRecordingSensor(t, testfake.NewFakeTransport())
	cs.state = StateIdle

	cs.mu.Lock()
	_, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()

	require.Error(t, err)
	assert.Equal(t, StateIdle, cs.state, "the guard must not itself change state")
}

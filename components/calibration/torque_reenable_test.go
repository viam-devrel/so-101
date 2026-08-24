package calibration

import (
	"context"
	"fmt"
	"path/filepath"
	"sync"
	"syscall"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/controller"
	"so_arm/internal/testfake"
)

// TestSaveCalibrationReenablesTorqueOnSuccess is the regression test for "torque is never
// re-enabled": a successful save_calibration must leave the arm holding position, not limp,
// matching what the setup app's Next Steps / Complete screens already tell the user.
func TestSaveCalibrationReenablesTorqueOnSuccess(t *testing.T) {
	cs := completedSensorForSave(t, testfake.NewFakeTransport())

	cs.mu.Lock()
	result, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()

	require.NoError(t, err)
	assert.Equal(t, true, result["success"])
	assert.Equal(t, true, result["torque_enabled"], "a successful save must re-enable torque")
	assert.NotContains(t, result, "torque_enable_error")
	assert.Equal(t, StateIdle, cs.state, "a successful save must still reach StateIdle")
}

// TestSaveCalibrationTorqueReenableFailureStaysIdleAndSucceeds covers the failure branch of
// the same fix: the calibration file and servo registers are already durably saved by the
// time torque re-enable runs, so a re-enable failure must NOT revert to StateCompleted (that
// would resurrect the "Retry Save" deadlock -- save_calibration would then look retryable but
// retrying redoes work that already succeeded) and must NOT surface as a command error
// (success:false is what the app's sendCommand treats as a thrown error). The caller instead
// learns about it via torque_enabled/torque_enable_error.
func TestSaveCalibrationTorqueReenableFailureStaysIdleAndSucceeds(t *testing.T) {
	failing := &packetLogTransport{FakeTransport: testfake.NewFakeTransport(), failTorqueEnable: true}
	cs := completedSensorForSave(t, failing)

	cs.mu.Lock()
	result, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()

	require.NoError(t, err, "a torque-only failure must not surface as the command error")
	assert.Equal(t, true, result["success"], "the calibration save itself still succeeded")
	assert.Equal(t, false, result["torque_enabled"])
	assert.NotEmpty(t, result["torque_enable_error"])
	assert.Equal(t, StateIdle, cs.state,
		"a successful save must reach StateIdle even when the torque re-enable that follows it fails")

	// The calibration must actually be durable despite the torque failure: reload from file.
	saved, err := controller.LoadFullCalibrationFromFile(cs.cfg.CalibrationFile, cs.logger)
	require.NoError(t, err)
	require.NotNil(t, saved.ElbowFlex)
	assert.Equal(t, cs.joints[3].RangeMin, saved.ElbowFlex.RangeMin)
	assert.Equal(t, cs.joints[3].RangeMax, saved.ElbowFlex.RangeMax)
}

// packetLogTransport records the ordering of the two packet kinds this fix is about: the
// per-servo WRITE (instruction 0x03) of RegGoalPosition, and the group SYNC_WRITE (0x83) of
// RegTorqueEnable -- the packet EnableAll/DisableAll send. Packet layout matches
// isWriteToServo's comment in calibration_save_retry_test.go, except a SYNC_WRITE's ID field
// is the broadcast ID and its parameters are [address, dataLen, id, data...]... rather than a
// single servo's payload. Everything else passes through untouched.
//
// failSyncRead fails the present-position SyncRead, so a test can drive the "could not read
// positions" branch; failTorqueEnable fails only the torque SYNC_WRITE, so a test can drive a
// save whose file and register writes land but whose post-save torque re-enable does not.
type packetLogTransport struct {
	*testfake.FakeTransport
	mu               sync.Mutex
	events           []string
	failSyncRead     bool
	failTorqueEnable bool
}

func (p *packetLogTransport) Write(b []byte) (int, error) {
	if len(b) >= 7 && b[0] == 0xFF && b[1] == 0xFF {
		id, instruction, params := int(b[2]), b[4], b[5:len(b)-1]
		switch {
		case instruction == 0x03 && params[0] == feetech.RegGoalPosition.Address && len(params) == 3:
			p.record(fmt.Sprintf("goal:%d=%d", id, int(params[1])|int(params[2])<<8))
		case instruction == 0x83 && params[0] == feetech.RegTorqueEnable.Address:
			if p.failTorqueEnable {
				return 0, syscall.EIO
			}
			p.record("torque_enable")
		case instruction == 0x82 && params[0] == feetech.RegPresentPosition.Address:
			if p.failSyncRead {
				return 0, syscall.EIO
			}
		}
	}
	return p.FakeTransport.Write(b)
}

func (p *packetLogTransport) record(ev string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.events = append(p.events, ev)
}

func (p *packetLogTransport) log() []string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return append([]string(nil), p.events...)
}

func completedSensorForSave(t *testing.T, ft feetech.Transport) *so101CalibrationSensor {
	t.Helper()
	cs := testRecordingSensor(t, ft)
	cs.state = StateCompleted
	cs.cfg.CalibrationFile = filepath.Join(t.TempDir(), "so101_calibration.json")
	for id, joint := range cs.joints {
		joint.HomingOffset = id
		joint.RangeMin = 500 + id*10
		joint.RangeMax = 3500 + id*10
		joint.IsCompleted = true
	}
	return cs
}

// TestSaveCalibrationSeedsGoalPositionsBeforeEnablingTorque is the safety contract of the
// post-save torque re-enable. Torque enable is a bare SyncWrite to RegTorqueEnable, so the
// firmware immediately drives toward whatever Goal_Position holds -- stale after a hand-moved
// calibration, and reinterpreted through the position_offset set_homing wrote. Every servo's
// live position must therefore reach Goal_Position BEFORE that sync write, or enabling torque
// snaps the arm.
func TestSaveCalibrationSeedsGoalPositionsBeforeEnablingTorque(t *testing.T) {
	ft := testfake.NewFakeTransport()
	want := map[int]int{1: 1000, 2: 1500, 3: 2000, 4: 2500, 5: 3000, 6: 2100}
	for id, pos := range want {
		ft.SetRegister(id, feetech.RegPresentPosition.Address, testfake.EncodeWordLE(pos))
	}
	logged := &packetLogTransport{FakeTransport: ft}

	cs := completedSensorForSave(t, logged)
	cs.mu.Lock()
	result, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()
	require.NoError(t, err)
	assert.Equal(t, true, result["torque_enabled"])

	events := logged.log()
	torqueAt := -1
	for i, ev := range events {
		if ev == "torque_enable" {
			torqueAt = i
			break
		}
	}
	require.NotEqual(t, -1, torqueAt, "torque must actually be enabled; packets seen: %v", events)

	for _, id := range cs.cfg.ServoIDs {
		want := fmt.Sprintf("goal:%d=%d", id, want[id])
		require.Contains(t, events[:torqueAt], want,
			"servo %d's live position must be written to goal_position before torque is enabled; packets seen: %v",
			id, events)
	}
}

// TestSaveCalibrationLeavesTorqueOffWhenPositionsUnreadable covers the other half of the same
// contract: if the arm's live positions cannot be read, torque must stay OFF rather than be
// enabled against a stale goal. The calibration is already durable, so the command still
// succeeds and still lands in StateIdle -- only torque_enabled reports the shortfall.
func TestSaveCalibrationLeavesTorqueOffWhenPositionsUnreadable(t *testing.T) {
	logged := &packetLogTransport{FakeTransport: testfake.NewFakeTransport(), failSyncRead: true}

	cs := completedSensorForSave(t, logged)
	cs.mu.Lock()
	result, err := cs.saveCalibration(context.Background())
	cs.mu.Unlock()

	require.NoError(t, err)
	assert.Equal(t, true, result["success"])
	assert.Equal(t, false, result["torque_enabled"])
	assert.NotEmpty(t, result["torque_enable_error"])
	assert.Equal(t, StateIdle, cs.state)
	assert.NotContains(t, logged.log(), "torque_enable",
		"torque must not be enabled when the positions it would hold could not be read")

	saved, err := controller.LoadFullCalibrationFromFile(cs.cfg.CalibrationFile, cs.logger)
	require.NoError(t, err)
	require.NotNil(t, saved.ElbowFlex)
	assert.Equal(t, cs.joints[3].RangeMin, saved.ElbowFlex.RangeMin)
}

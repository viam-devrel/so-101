package arm

import (
	"context"
	"syscall"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"

	"so_arm/internal/servo"
	"so_arm/internal/testfake"
)

func gainsTestArm(t *testing.T, ft feetech.Transport, cfg *SO101ArmConfig) *so101 {
	t.Helper()
	return &so101{
		logger:      logging.NewTestLogger(t),
		controller:  testArmHandle(t, ft),
		armServoIDs: []int{1, 2, 3, 4, 5},
		cfg:         cfg,
		initCtx:     context.Background(),
	}
}

func seedGains(ft *testfake.FakeTransport, p, d, i byte, ids ...int) {
	for _, id := range ids {
		ft.SetRegister(id, feetech.RegPGain.Address, []byte{p})
		ft.SetRegister(id, feetech.RegDGain.Address, []byte{d})
		ft.SetRegister(id, feetech.RegIGain.Address, []byte{i})
	}
}

func TestServoInitializationWritesTheTunedGains(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5, 6)
	s := gainsTestArm(t, ft, &SO101ArmConfig{})

	require.NoError(t, s.initializeServos())

	assert.Equal(t, byte(48), readServoGain(t, s, 2, "p_gain"))
	assert.Equal(t, byte(64), readServoGain(t, s, 2, "d_gain"))
	assert.Equal(t, byte(2), readServoGain(t, s, 2, "i_gain"))
	assert.Equal(t, byte(4), readServoGain(t, s, 3, "i_gain"))
	// Servo 1 is the only P that is not 48, servo 5 the only D that is not 64: a
	// transcription slip in either is caught by nothing else.
	assert.Equal(t, byte(32), readServoGain(t, s, 1, "p_gain"))
	assert.Equal(t, byte(48), readServoGain(t, s, 5, "d_gain"))
}

// orderRecordingTransport records the order of gain writes to one servo at the wire level.
type orderRecordingTransport struct {
	*testfake.FakeTransport
	id   int
	seen []byte
}

func (o *orderRecordingTransport) Write(p []byte) (int, error) {
	// Packet: [0xFF, 0xFF, id, len, instruction, params..., checksum]. WRITE is 0x03.
	if len(p) >= 7 && p[0] == 0xFF && p[1] == 0xFF && int(p[2]) == o.id && p[4] == 0x03 &&
		(p[5] == feetech.RegPGain.Address || p[5] == feetech.RegDGain.Address ||
			p[5] == feetech.RegIGain.Address) {
		o.seen = append(o.seen, p[5])
	}
	return o.FakeTransport.Write(p)
}

// A non-zero I is only stable against a raised D, so any other order leaves a window where a
// bus failure strands the servo above its stable I. Final register values are
// order-independent, so this needs the wire.
func TestApplyServoGainsWritesDampingBeforeIntegral(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5, 6)
	rec := &orderRecordingTransport{FakeTransport: ft, id: 3}

	require.NoError(t, gainsTestArm(t, rec, &SO101ArmConfig{}).initializeServos())

	assert.Equal(t, []byte{
		feetech.RegDGain.Address, feetech.RegIGain.Address, feetech.RegPGain.Address,
	}, rec.seen, "gains must be written D, then I, then P")
}

func TestServoInitializationRespectsDisableServoGains(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5)
	s := gainsTestArm(t, ft, &SO101ArmConfig{DisableServoGains: true})

	require.NoError(t, s.initializeServos())

	for _, id := range []int{1, 2, 3, 4, 5} {
		for _, reg := range gainRegisters() {
			assert.Zero(t, ft.WriteCount(id, reg), "servo %d register %d must not be written", id, reg)
		}
	}
}

// The invariant is "no servo outside this arm's own ids", not "not servo 6": a 4-DOF arm
// configured with servo_ids [1,2,3,4] hosts a gripper on servo 5, and DefaultArmGains[5]
// exists.
func TestApplyServoGainsTouchesOnlyTheArmsOwnServos(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5, 6)
	s := gainsTestArm(t, ft, &SO101ArmConfig{})
	s.armServoIDs = []int{1, 2, 3, 4}

	require.NoError(t, s.initializeServos())

	for _, id := range []int{5, 6} {
		for _, reg := range gainRegisters() {
			assert.Zero(t, ft.WriteCount(id, reg), "servo %d is not this arm's, register %d", id, reg)
		}
	}
}

// EEPROM has finite write endurance and initializeServosWithRetry runs this up to 3 times.
func TestApplyServoGainsSkipsNoOpWrites(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for id, g := range servo.DefaultArmGains {
		seedGains(ft, byte(g.P), byte(g.D), byte(g.I), id)
	}
	s := gainsTestArm(t, ft, &SO101ArmConfig{})

	require.NoError(t, s.initializeServos())

	for _, id := range []int{1, 2, 3, 4, 5} {
		for _, reg := range gainRegisters() {
			assert.Zero(t, ft.WriteCount(id, reg),
				"servo %d register %d was already correct", id, reg)
		}
	}
}

// failingWriteTransport wraps a FakeTransport and fails exactly the FIRST (servo, register)
// WRITE packet at the wire level -- everything else (PING, SYNC_READ, reads, the EEPROM
// unlock/relock writes to the lock register, writes to any other register or servo, and a
// later write to the SAME register once failsLeft is exhausted) passes through untouched.
//
// The failure is injected on the WRITE, not the read: writeGainIfChanged deliberately falls
// through to the write when a read fails, so a read failure lands the gain anyway and would
// make this test vacuous. Failing only once, not forever, is what lets the test both trigger
// the warn-and-continue path during init AND read the register back afterward. The ping and
// position read go over the SAME transport and must keep succeeding -- that is the proof
// this exercises the real contract rather than a transport that is simply broken.
type failingWriteTransport struct {
	*testfake.FakeTransport
	id        int
	address   byte
	failsLeft int
}

func (f *failingWriteTransport) Write(p []byte) (int, error) {
	// Packet: [0xFF, 0xFF, id, len, instruction, params..., checksum]. WRITE is
	// instruction 0x03 with params [address, data...].
	if f.failsLeft > 0 && len(p) >= 7 && p[0] == 0xFF && p[1] == 0xFF &&
		int(p[2]) == f.id && p[4] == 0x03 && p[5] == f.address {
		f.failsLeft--
		return 0, syscall.EIO
	}
	return f.FakeTransport.Write(p)
}

// A gain write failing must not fail initialization: the arm works at stock gains. Only
// servo 2's d_gain WRITE fails; ping (PING) and the position read (SYNC_READ) use different
// instructions and are unaffected, so their continued success is the proof this exercises
// the real warn-and-continue path and not a transport that is simply broken outright.
func TestApplyServoGainsFailureDoesNotFailInit(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5, 6)
	failing := &failingWriteTransport{FakeTransport: ft, id: 2, address: feetech.RegDGain.Address, failsLeft: 1}
	s := gainsTestArm(t, failing, &SO101ArmConfig{})

	require.NoError(t, s.initializeServos())

	// Servo 2's d_gain write failed, so it must be left exactly as found.
	assert.Equal(t, byte(32), readServoGain(t, s, 2, "d_gain"), "servo 2 d_gain must be left as found after a write failure")
	// Terminal for the rest of servo 2's gains on purpose: 48/32/4 is the combination the
	// write order exists to prevent, and it would persist in EEPROM with no retry.
	assert.Equal(t, byte(0), readServoGain(t, s, 2, "i_gain"), "servo 2 i_gain must not land over a stock d_gain")
	assert.Equal(t, byte(32), readServoGain(t, s, 2, "p_gain"), "servo 2 p_gain must not land over a stock d_gain")
	// It must NOT cascade to any other servo.
	assert.Equal(t, byte(64), readServoGain(t, s, 3, "d_gain"), "servo 3 d_gain must still land")
	assert.Equal(t, byte(4), readServoGain(t, s, 3, "i_gain"), "servo 3 i_gain must still land")
}

func gainRegisters() []byte {
	return []byte{feetech.RegPGain.Address, feetech.RegDGain.Address, feetech.RegIGain.Address}
}

func readServoGain(t *testing.T, s *so101, id int, name string) byte {
	t.Helper()
	got, err := s.controller.ReadServoRegister(context.Background(), id, name)
	require.NoError(t, err)
	require.Len(t, got, 1)
	return got[0]
}

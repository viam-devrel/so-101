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

// failingReadTransport wraps a FakeTransport and fails exactly the FIRST (servo, register)
// READ packet at the wire level -- everything else (PING, SYNC_READ, WRITE/SYNC_WRITE, reads
// of any other register or servo, and a later re-read of the SAME register once failsLeft is
// exhausted) passes through untouched. Failing only once, not forever, is what lets a test
// both trigger the warn-and-continue path during init AND read the register back afterward
// to assert it was left alone. This is what makes TestApplyServoGainsFailureDoesNotFailInit a
// real test of the contract rather than a vacuous one: the ping and position read below go
// over the SAME transport and must keep succeeding.
type failingReadTransport struct {
	*testfake.FakeTransport
	id        int
	address   byte
	failsLeft int
}

func (f *failingReadTransport) Write(p []byte) (int, error) {
	// Packet: [0xFF, 0xFF, id, len, instruction, params..., checksum]. READ is
	// instruction 0x02 with params [address, length].
	if f.failsLeft > 0 && len(p) >= 7 && p[0] == 0xFF && p[1] == 0xFF &&
		int(p[2]) == f.id && p[4] == 0x02 && p[5] == f.address {
		f.failsLeft--
		return 0, syscall.EIO
	}
	return f.FakeTransport.Write(p)
}

// A gain write failing must not fail initialization: the arm works at stock gains. Only
// servo 2's d_gain READ fails (via failingReadTransport); ping (PING packets) and the
// position read (a SYNC_READ) use different instructions and are unaffected, so their
// continued success is the proof this exercises the real warn-and-continue path and not a
// transport that is simply broken outright.
func TestApplyServoGainsFailureDoesNotFailInit(t *testing.T) {
	ft := testfake.NewFakeTransport()
	seedGains(ft, 32, 32, 0, 1, 2, 3, 4, 5, 6)
	failing := &failingReadTransport{FakeTransport: ft, id: 2, address: feetech.RegDGain.Address, failsLeft: 1}
	s := gainsTestArm(t, failing, &SO101ArmConfig{})

	require.NoError(t, s.initializeServos())

	// Servo 2's d_gain read failed, so writeGainIfChanged bailed out before writing --
	// it must be left exactly as found, not stranded at some other value.
	assert.Equal(t, byte(32), readServoGain(t, s, 2, "d_gain"), "servo 2 d_gain must be left as found after a read failure")
	// The failure must not cascade to the rest of servo 2's own gains...
	assert.Equal(t, byte(2), readServoGain(t, s, 2, "i_gain"), "servo 2 i_gain must still land")
	assert.Equal(t, byte(48), readServoGain(t, s, 2, "p_gain"), "servo 2 p_gain must still land")
	// ...or to any other servo.
	assert.Equal(t, byte(64), readServoGain(t, s, 3, "d_gain"), "servo 3 d_gain must still land")
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

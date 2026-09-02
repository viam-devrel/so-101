package arm

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// Compliance must zero the INTEGRAL term, not just lower P. With tuned gains in EEPROM the
// integrator accumulates the standing error while a hand holds the arm off-target and then
// pushes back on release; P=8 does not touch that path.
func TestApplyComplianceZeroesIGain(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ids := []int{1, 2, 3, 4, 5}
	for _, id := range ids {
		ft.SetRegister(id, feetech.RegPGain.Address, []byte{48})
		ft.SetRegister(id, feetech.RegIGain.Address, []byte{2})
		ft.SetRegister(id, feetech.RegDGain.Address, []byte{64})
		ft.SetRegister(id, feetech.RegTorqueLimit.Address, testfake.EncodeWordLE(1000))
	}
	io := &controllerManualIO{controller: testArmHandle(t, ft), ids: ids}

	require.NoError(t, io.applyCompliance(context.Background(), 8, 50))

	for _, id := range ids {
		assert.Equal(t, byte(0), readReg(t, io, id, "i_gain"), "servo %d i_gain during compliance", id)
		assert.Equal(t, byte(64), readReg(t, io, id, "d_gain"), "servo %d d_gain must be left alone", id)
	}

	require.NoError(t, io.restoreCompliance(context.Background()))

	for _, id := range ids {
		assert.Equal(t, byte(2), readReg(t, io, id, "i_gain"), "servo %d i_gain restored", id)
		assert.Equal(t, byte(48), readReg(t, io, id, "p_gain"), "servo %d p_gain restored", id)
	}
}

// p_gain 0 is the documented opt-out for gain writes (manualIO's interface comment). i_gain
// is a gain register, so it must honour the same opt-out.
func TestApplyComplianceHonoursThePGainOptOut(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for _, id := range []int{1, 2} {
		ft.SetRegister(id, feetech.RegPGain.Address, []byte{48})
		ft.SetRegister(id, feetech.RegIGain.Address, []byte{4})
		ft.SetRegister(id, feetech.RegTorqueLimit.Address, testfake.EncodeWordLE(1000))
	}
	io := &controllerManualIO{controller: testArmHandle(t, ft), ids: []int{1, 2}}

	require.NoError(t, io.applyCompliance(context.Background(), 0, 50))
	// Restore too: it writes whatever it captured, so a register the apply never touched
	// must not be written back from a zero value -- p_gain 0 is no position control at all.
	require.NoError(t, io.restoreCompliance(context.Background()))

	for _, id := range []int{1, 2} {
		assert.Equal(t, byte(4), readReg(t, io, id, "i_gain"), "servo %d i_gain must be untouched", id)
		assert.Equal(t, byte(48), readReg(t, io, id, "p_gain"), "servo %d p_gain must be untouched", id)
		assert.Zero(t, ft.WriteCount(id, feetech.RegIGain.Address))
		assert.Zero(t, ft.WriteCount(id, feetech.RegPGain.Address))
	}
}

// These are EEPROM registers and manual mode is entered and left repeatedly. A servo already
// holding the target value must not be written.
func TestComplianceSkipsNoOpGainWrites(t *testing.T) {
	ft := testfake.NewFakeTransport()
	// Already at the compliance values: p_gain 8, i_gain 0.
	ft.SetRegister(1, feetech.RegPGain.Address, []byte{8})
	ft.SetRegister(1, feetech.RegIGain.Address, []byte{0})
	ft.SetRegister(1, feetech.RegTorqueLimit.Address, testfake.EncodeWordLE(1000))
	io := &controllerManualIO{controller: testArmHandle(t, ft), ids: []int{1}}

	require.NoError(t, io.applyCompliance(context.Background(), 8, 50))
	require.NoError(t, io.restoreCompliance(context.Background()))

	assert.Zero(t, ft.WriteCount(1, feetech.RegPGain.Address), "p_gain was already 8")
	assert.Zero(t, ft.WriteCount(1, feetech.RegIGain.Address), "i_gain was already 0")
}

func readReg(t *testing.T, io *controllerManualIO, id int, name string) byte {
	t.Helper()
	got, err := io.controller.ReadServoRegister(context.Background(), id, name)
	require.NoError(t, err)
	require.Len(t, got, 1)
	return got[0]
}

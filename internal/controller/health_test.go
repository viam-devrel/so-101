package controller

import (
	"context"
	"testing"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"so_arm/internal/testfake"
)

// A servo outside its voltage limits sets the voltage flag on every reply but still returns
// valid data. Treating that as a failed read is what left manual mode at full stiffness (the
// p_gain read failed, so compliance was never applied) on a real leader arm.
func TestReadServoRegisterKeepsThePayloadUnderAConditionFlag(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(2, feetech.RegPGain.Address, []byte{32})
	ft.SetStatus(2, feetech.ErrVoltage)
	h := testHandle(t, ft)

	got, err := h.ReadServoRegister(context.Background(), 2, "p_gain")
	require.NoError(t, err)
	assert.Equal(t, []byte{32}, got)
}

func TestServoHealthReportsVoltageAgainstLimits(t *testing.T) {
	ft := testfake.NewFakeTransport()
	for id := 1; id <= 3; id++ {
		ft.SetRegister(id, feetech.RegPresentVoltage.Address, []byte{50}) // 5.0 V
		ft.SetRegister(id, feetech.RegMinVoltage.Address, []byte{40})
		ft.SetRegister(id, feetech.RegMaxVoltage.Address, []byte{80})
	}
	ft.SetRegister(2, feetech.RegMinVoltage.Address, []byte{60}) // 6.0 V: 5.0 is under it
	ft.SetStatus(2, feetech.ErrVoltage)
	h := testHandle(t, ft)

	health, err := h.ServoHealth(context.Background(), []int{3, 1, 2})
	require.NoError(t, err)
	require.Len(t, health, 3)
	assert.Equal(t, []int{1, 2, 3}, []int{health[0].ID, health[1].ID, health[2].ID})

	s2 := health[1]
	assert.Empty(t, s2.Errors, "a condition flag must not lose the readings")
	assert.InDelta(t, 5.0, s2.PresentVoltage, 1e-9)
	assert.InDelta(t, 6.0, s2.MinVoltage, 1e-9)
	assert.InDelta(t, 8.0, s2.MaxVoltage, 1e-9)
	assert.Equal(t, []interface{}{"voltage"}, s2.Map()["flags"])
	assert.Zero(t, health[0].Flags)
	assert.Empty(t, health[0].Map()["flags"])
}

func TestConditionDetailNamesTheVoltageAndLimits(t *testing.T) {
	ft := testfake.NewFakeTransport()
	ft.SetRegister(2, feetech.RegPresentVoltage.Address, []byte{50})
	ft.SetRegister(2, feetech.RegMinVoltage.Address, []byte{60})
	ft.SetRegister(2, feetech.RegMaxVoltage.Address, []byte{84})
	ft.SetStatus(2, feetech.ErrVoltage)
	h := testHandle(t, ft)

	var detail string
	require.NoError(t, h.withSessionRead(func(sess *busSession) error {
		detail = conditionDetail(context.Background(), sess.bus, 2, feetech.ErrVoltage)
		return nil
	}))
	assert.Contains(t, detail, "5.0 V")
	assert.Contains(t, detail, "6.0-8.4 V")

	require.NoError(t, h.withSessionRead(func(sess *busSession) error {
		assert.Empty(t, conditionDetail(context.Background(), sess.bus, 2, feetech.ErrOverload),
			"only the voltage flag gets voltage detail")
		return nil
	}))
}

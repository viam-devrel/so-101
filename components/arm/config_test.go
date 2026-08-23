package arm

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestSO101ArmConfigValidateMeshRatios(t *testing.T) {
	// Out-of-range ratio is rejected.
	bad := &SO101ArmConfig{Port: "/dev/null", UseURDF: true, MeshDecimationRatios: []float64{1.5}}
	_, _, err := bad.Validate("")
	require.Error(t, err)
	require.Contains(t, err.Error(), "mesh_decimation_ratios")

	// Valid ratios pass.
	ok := &SO101ArmConfig{Port: "/dev/null", UseURDF: true, MeshDecimationRatios: []float64{0.1, 0.2}}
	_, _, err = ok.Validate("")
	require.NoError(t, err)
}

// TestArmConfigValidateToleranceRanges pins that Validate rejects out-of-range
// orientation_tolerance_deg/position_tolerance_mm by delegating to
// planning.ValidateGoalCloudTolerances -- see internal/planning's own tests for the
// exhaustive range/NaN cases; this only pins the wiring.
func TestArmConfigValidateToleranceRanges(t *testing.T) {
	base := func() *SO101ArmConfig { return &SO101ArmConfig{Port: "/dev/null"} }

	for name, mutate := range map[string]func(*SO101ArmConfig){
		"negative orientation": func(c *SO101ArmConfig) { c.OrientationToleranceDeg = -1 },
		"orientation over 180": func(c *SO101ArmConfig) { c.OrientationToleranceDeg = 181 },
		"NaN orientation":      func(c *SO101ArmConfig) { c.OrientationToleranceDeg = math.NaN() },
		"negative position":    func(c *SO101ArmConfig) { c.PositionToleranceMM = -0.1 },
		"NaN position":         func(c *SO101ArmConfig) { c.PositionToleranceMM = math.NaN() },
	} {
		t.Run(name, func(t *testing.T) {
			c := base()
			mutate(c)
			_, _, err := c.Validate("")
			assert.Error(t, err)
		})
	}

	t.Run("valid bounds accepted", func(t *testing.T) {
		c := base()
		c.OrientationToleranceDeg, c.PositionToleranceMM = 180, 75
		_, _, err := c.Validate("")
		assert.NoError(t, err)
	})
}

func TestManualModeConfigValidate(t *testing.T) {
	// Out-of-range deadband is rejected.
	bad := &SO101ArmConfig{Port: "/dev/null", ManualMode: &ManualModeConfig{PosDeadbandDeg: -1}}
	if _, _, err := bad.Validate(""); err == nil {
		t.Fatal("expected error for negative pos_deadband_deg, got nil")
	}

	// Valid scalar config passes.
	ok := &SO101ArmConfig{Port: "/dev/null", ManualMode: &ManualModeConfig{
		PosDeadbandDeg: 2, LoopHz: 50, PGain: 16, TorqueLimit: 300,
	}}
	if _, _, err := ok.Validate(""); err != nil {
		t.Fatalf("expected valid config, got %v", err)
	}

	// Out-of-range torque_limit is rejected.
	badTorque := &SO101ArmConfig{Port: "/dev/null", ManualMode: &ManualModeConfig{TorqueLimit: 2000}}
	if _, _, err := badTorque.Validate(""); err == nil {
		t.Fatal("expected error for out-of-range torque_limit, got nil")
	}

	// Nil manual_mode is valid.
	none := &SO101ArmConfig{Port: "/dev/null"}
	if _, _, err := none.Validate(""); err != nil {
		t.Fatalf("expected nil manual_mode to be valid, got %v", err)
	}
}

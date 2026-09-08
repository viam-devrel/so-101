package servo

import "testing"

// The calibration workflow's setHomingPosition writes each servo's position_offset so the
// homed pose reads raw tick HomingTick (2047), regardless of where RangeMin/RangeMax later
// land. NormModeDegrees's kinematic zero must be that same tick, not the range midpoint --
// otherwise an asymmetric recorded range drifts the arm's zero off the pose the user homed at.
func TestNormModeDegreesZeroIsHomingTickNotRangeMidpoint(t *testing.T) {
	// Asymmetric range: midpoint is 1969.5, not HomingTick.
	cal := &MotorCalibration{RangeMin: 866, RangeMax: 3073, NormMode: NormModeDegrees}

	got, err := cal.Normalize(HomingTick)
	if err != nil {
		t.Fatalf("Normalize: %v", err)
	}
	if got != 0 {
		t.Errorf("Normalize(HomingTick) = %v, want 0", got)
	}

	raw, err := cal.Denormalize(0)
	if err != nil {
		t.Fatalf("Denormalize: %v", err)
	}
	if raw != HomingTick {
		t.Errorf("Denormalize(0) = %v, want %v", raw, HomingTick)
	}
}

func TestNormModeDegreesRoundTrip(t *testing.T) {
	cal := &MotorCalibration{RangeMin: 866, RangeMax: 3073, NormMode: NormModeDegrees}

	for _, raw := range []int{900, HomingTick, 2500, 3000} {
		norm, err := cal.Normalize(raw)
		if err != nil {
			t.Fatalf("Normalize(%d): %v", raw, err)
		}
		back, err := cal.Denormalize(norm)
		if err != nil {
			t.Fatalf("Denormalize(%v): %v", norm, err)
		}
		if back != raw {
			t.Errorf("round trip raw=%d: normalized=%v, denormalized back=%d", raw, norm, back)
		}
	}
}

// Denormalize must still clamp to RangeMin/RangeMax in degrees mode -- only the center moved.
func TestNormModeDegreesDenormalizeStillClampsToRange(t *testing.T) {
	cal := &MotorCalibration{RangeMin: 866, RangeMax: 3073, NormMode: NormModeDegrees}

	raw, err := cal.Denormalize(1000)
	if err != nil {
		t.Fatalf("Denormalize: %v", err)
	}
	if raw != cal.RangeMax {
		t.Errorf("Denormalize(1000) = %d, want clamped to RangeMax %d", raw, cal.RangeMax)
	}

	raw, err = cal.Denormalize(-1000)
	if err != nil {
		t.Fatalf("Denormalize: %v", err)
	}
	if raw != cal.RangeMin {
		t.Errorf("Denormalize(-1000) = %d, want clamped to RangeMin %d", raw, cal.RangeMin)
	}
}

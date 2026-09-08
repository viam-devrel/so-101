package servo

import "testing"

// Zero must be HomingTick, not the range midpoint (1969.5 here).
func TestNormModeDegreesZeroIsHomingTickNotRangeMidpoint(t *testing.T) {
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

	// The gripper's percent mode still spans the recorded range.
	grip := &MotorCalibration{RangeMin: 2047, RangeMax: 3467, NormMode: NormModeRange100}
	if pct, _ := grip.Normalize(2047); pct != 0 {
		t.Errorf("Range100 Normalize(RangeMin) = %v, want 0", pct)
	}
	if pct, _ := grip.Normalize(3467); pct != 100 {
		t.Errorf("Range100 Normalize(RangeMax) = %v, want 100", pct)
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

package so_arm

import "testing"

func TestDegPerSecSqToAccUnits(t *testing.T) {
	tests := []struct {
		name  string
		degs2 float64
		want  int
	}{
		// (degs2 * 4096/360 - 386) / 108, rounded, clamped to [1,50].
		{"shipped default 500 stays below the clamp", 500, 49},
		{"old documented default 100", 100, 7},
		{"config minimum 50", 50, 2},
		{"600 would clamp, which is why it is not the default", 600, 50},
		{"unreachable low value clamps to the floor, not to 0", 10, 1},
		{"zero clamps to the floor, never to 0 (0 means UNLIMITED)", 0, 1},
		{"absurdly high clamps to the knee", 100000, 50},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := degPerSecSqToAccUnits(tt.degs2); got != tt.want {
				t.Errorf("degPerSecSqToAccUnits(%v) = %d, want %d", tt.degs2, got, tt.want)
			}
		})
	}
}

func TestCoordinatedProfiles(t *testing.T) {
	const (
		refSpeed = 50.0  // deg/s
		refAccel = 500.0 // deg/s^2
	)

	t.Run("equal travels give equal profiles", func(t *testing.T) {
		got := coordinatedProfiles([]float64{10, 10, 10}, refSpeed, refAccel, nil)
		if len(got) != 3 {
			t.Fatalf("len = %d, want 3", len(got))
		}
		for i, p := range got {
			if p != got[0] {
				t.Errorf("joint %d = %+v, want %+v (equal travels must scale equally)", i, p, got[0])
			}
		}
		// The reference joint runs at the full reference.
		if want := degPerSecToStepsPerSec(refSpeed); got[0].speedSteps != want {
			t.Errorf("speedSteps = %d, want %d", got[0].speedSteps, want)
		}
	})

	t.Run("4:1 travel gives 4:1 speed and acceleration", func(t *testing.T) {
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, nil)
		// k = 1.0 and 0.25.
		if want := degPerSecToStepsPerSec(50); got[0].speedSteps != want {
			t.Errorf("long joint speed = %d, want %d", got[0].speedSteps, want)
		}
		if want := degPerSecToStepsPerSec(12.5); got[1].speedSteps != want {
			t.Errorf("short joint speed = %d, want %d", got[1].speedSteps, want)
		}
		if want := degPerSecSqToAccUnits(500); got[0].accUnits != want {
			t.Errorf("long joint acc = %d, want %d", got[0].accUnits, want)
		}
		if want := degPerSecSqToAccUnits(125); got[1].accUnits != want {
			t.Errorf("short joint acc = %d, want %d", got[1].accUnits, want)
		}
	})

	t.Run("direction does not matter, only magnitude", func(t *testing.T) {
		fwd := coordinatedProfiles([]float64{40, 10}, refSpeed, refAccel, nil)
		rev := coordinatedProfiles([]float64{-40, -10}, refSpeed, refAccel, nil)
		for i := range fwd {
			if fwd[i] != rev[i] {
				t.Errorf("joint %d: forward %+v != reverse %+v", i, fwd[i], rev[i])
			}
		}
	})

	t.Run("all joints stationary is a no-op", func(t *testing.T) {
		if got := coordinatedProfiles([]float64{0, 0, 0}, refSpeed, refAccel, nil); got != nil {
			t.Errorf("got %+v, want nil (nothing to command)", got)
		}
	})

	t.Run("a single moving joint runs at the full reference", func(t *testing.T) {
		got := coordinatedProfiles([]float64{0, 20, 0}, refSpeed, refAccel, nil)
		if want := degPerSecToStepsPerSec(refSpeed); got[1].speedSteps != want {
			t.Errorf("moving joint speed = %d, want %d", got[1].speedSteps, want)
		}
	})

	t.Run("stationary joints never round into the 0 sentinels", func(t *testing.T) {
		// Speed 0 means MAXIMUM and Acc 0 means UNLIMITED, so a k=0 joint must floor to 1
		// on both, not to 0. Harmless on the wire because its goal equals its position.
		got := coordinatedProfiles([]float64{20, 0}, refSpeed, refAccel, nil)
		if got[1].speedSteps != minSpeedSteps {
			t.Errorf("stationary speedSteps = %d, want %d (0 would mean MAX SPEED)",
				got[1].speedSteps, minSpeedSteps)
		}
		if got[1].accUnits != minAccUnits {
			t.Errorf("stationary accUnits = %d, want %d (0 would mean UNLIMITED)",
				got[1].accUnits, minAccUnits)
		}
	})

	t.Run("a tiny travel floors instead of rounding into the sentinels", func(t *testing.T) {
		// k = 0.0001: speed scales to 0.005 deg/s and acceleration to 0.05 deg/s^2, both
		// of which round to 0 without the floors.
		got := coordinatedProfiles([]float64{100, 0.01}, refSpeed, refAccel, nil)
		if got[1].speedSteps < minSpeedSteps {
			t.Errorf("speedSteps = %d, want >= %d", got[1].speedSteps, minSpeedSteps)
		}
		if got[1].accUnits < minAccUnits {
			t.Errorf("accUnits = %d, want >= %d", got[1].accUnits, minAccUnits)
		}
	})

	t.Run("the shipped default leaves the reference joint UNCLAMPED", func(t *testing.T) {
		// This is why the default is 500 and not 600. If the reference joint (k=1) clamps
		// at maxAccUnits while lower-k joints keep their exact scaled value, coordination
		// silently breaks at the default -- the reference arrives late and every joint
		// above k~0.85 collapses to the same register value.
		got := coordinatedProfiles([]float64{40, 10}, refSpeed, defaultAccelDegsPerSecSq, nil)
		if got[0].accUnits >= maxAccUnits {
			t.Errorf("reference joint accUnits = %d, which is at the clamp of %d; "+
				"the default acceleration is too high and coordination is broken",
				got[0].accUnits, maxAccUnits)
		}
	})
}

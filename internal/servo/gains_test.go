package servo

import "testing"

func TestDefaultArmGainsInvariants(t *testing.T) {
	if len(DefaultArmGains) != 5 {
		t.Fatalf("expected exactly the 5 arm servos, got %d entries", len(DefaultArmGains))
	}
	for _, id := range []int{1, 2, 3, 4, 5} {
		g, ok := DefaultArmGains[id]
		if !ok {
			t.Fatalf("servo %d missing from the tuned gain table", id)
		}
		// P=64 oscillated at every D tested (report finding 3), which is why nothing here
		// may exceed 48.
		if g.P > 48 {
			t.Fatalf("servo %d: P=%d is above the measured stability ceiling of 48", id, g.P)
		}
		// Finding 4: raising D is what makes a non-zero I usable.
		if g.I > 0 && g.D < 48 {
			t.Fatalf("servo %d: I=%d needs D>=48 to be stable, got D=%d", id, g.I, g.D)
		}
	}
	if _, ok := DefaultArmGains[6]; ok {
		t.Fatal("servo 6 (gripper) is untested and must not carry a tuned default")
	}
}

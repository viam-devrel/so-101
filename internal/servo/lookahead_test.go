package servo

import (
	"math"
	"testing"
)

// The fixed 2.0 this replaced was right only below about 45 deg/s at the default
// acceleration, which is why a slow recording replayed smoothly and a fast one did not.
func TestLookaheadDegFor(t *testing.T) {
	for _, tc := range []struct {
		name       string
		speed, acc float64
		want       float64
		why        string
	}{{
		// Measured: a recorded wave replays at 58.8 deg/s, its deceleration ramp is
		// 3.45 deg, and a lookahead of 2.0 released the next goal only once the servo was
		// already inside that ramp -- ten velocity dips a second at 10 Hz. Raising it to 4
		// removed the jerk on hardware; the formula independently lands there.
		name: "a fast move clears its deceleration ramp", speed: 58.8, acc: 500, want: 4.15,
		why: "must exceed the 3.45 deg ramp",
	}, {
		// A slow move's ramp is far under the droop, so the droop floor governs.
		name: "a slow move keeps the droop floor", speed: 25.2, acc: 500, want: DefaultLookaheadDeg,
		why: "0.64 deg ramp is below the 2.0 droop floor",
	}, {
		name: "the shipped default profile", speed: 50, acc: 500, want: 3.0,
		why: "50^2/(2*500) * 1.2",
	}, {
		// Below the droop the dwell is never satisfied and every waypoint burns its full
		// timeout -- the failure the floor exists to prevent.
		name:  "the slowest configurable speed still clears the droop",
		speed: minSpeedDegsPerSec, acc: 500, want: DefaultLookaheadDeg,
		why: "a tiny ramp must not produce an unsatisfiable lookahead",
	}, {
		// The Acc register floors at ~43 deg/s^2, so a lower request is silently raised by
		// the hardware and the real ramp is shorter than the configured value implies.
		name:  "acceleration below the register floor uses the floor",
		speed: 50, acc: 1, want: 1.2 * 50 * 50 / (2 * AccFloorDegsPerSecSq),
		why: "must not model a ramp the hardware cannot deliver",
	}, {
		name: "capped at the maximum", speed: maxSpeedDegsPerSec * 10, acc: 500,
		want: MaxLookaheadDeg, why: "clamped",
	}, {
		name: "a non-positive speed falls back to the floor", speed: 0, acc: 500,
		want: DefaultLookaheadDeg, why: "no ramp to model",
	}} {
		t.Run(tc.name, func(t *testing.T) {
			got := LookaheadDegFor(tc.speed, tc.acc)
			if math.Abs(got-tc.want) > 0.01 {
				t.Errorf("LookaheadDegFor(%g, %g) = %.3f, want %.3f (%s)",
					tc.speed, tc.acc, got, tc.want, tc.why)
			}
		})
	}
}

// Both bounds are load-bearing: drop either and one regime breaks silently.
func TestLookaheadDegForNeverDropsBelowTheDroopFloor(t *testing.T) {
	for v := 1.0; v <= maxSpeedDegsPerSec; v += 1.0 {
		if got := LookaheadDegFor(v, MaxAccelDegsPerSecSq); got < DefaultLookaheadDeg {
			t.Fatalf("LookaheadDegFor(%g) = %.3f, below the droop floor %.1f", v, got, DefaultLookaheadDeg)
		}
	}
}

func TestLookaheadDegForGrowsWithTheSquareOfSpeed(t *testing.T) {
	// Doubling the speed quadruples the ramp; that is the whole reason a fixed value cannot
	// serve both a fast and a slow move.
	slow, fast := LookaheadDegFor(50, 500), LookaheadDegFor(100, 500)
	if ratio := fast / slow; math.Abs(ratio-4.0) > 0.01 {
		t.Errorf("doubling the speed scaled the lookahead by %.2f, want 4.00", ratio)
	}
}

// The lookahead is derived from BOTH speed and acceleration, so acceleration has to track
// MoveOptions the same way speed does. It did not: speed resolved through
// ResolveSpeedDegsPerSec while acceleration silently kept the component default, so a caller
// that LOWERED acceleration got a longer real deceleration ramp and a lookahead still sized
// for the shorter one -- the next goal released after the servo had begun decelerating.
func TestResolveAccelDegsPerSecSq(t *testing.T) {
	const dflt = 500.0
	for _, tc := range []struct {
		name       string
		maxAccRads float64
		want       float64
	}{
		{"unset keeps the configured default", 0, dflt},
		{"negative keeps the configured default", -1, dflt},
		{"a lower request is honoured", math.Pi, 180},
		{"below the register's floor clamps up", 0.1, MinAccelDegsPerSecSq},
		{"above the register's ceiling clamps down", 100, MaxAccelDegsPerSecSq},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if got := ResolveAccelDegsPerSecSq(tc.maxAccRads, dflt); math.Abs(got-tc.want) > 0.01 {
				t.Errorf("ResolveAccelDegsPerSecSq(%g, %g) = %.2f, want %.2f",
					tc.maxAccRads, dflt, got, tc.want)
			}
		})
	}
}

func TestLowerAccelerationWidensTheLookahead(t *testing.T) {
	// Halving the acceleration doubles the deceleration ramp, so the lookahead must grow to
	// match. If acceleration were ignored, both of these would be equal and the slower-
	// stopping arm would be paced as though it stopped on a dime.
	fast := LookaheadDegFor(80, MaxAccelDegsPerSecSq)
	slow := LookaheadDegFor(80, MaxAccelDegsPerSecSq/2)
	if slow <= fast {
		t.Fatalf("lookahead at half the acceleration = %.2f, want greater than %.2f", slow, fast)
	}
	if ratio := slow / fast; math.Abs(ratio-2.0) > 0.01 {
		t.Errorf("halving acceleration scaled the lookahead by %.2f, want 2.00", ratio)
	}
}

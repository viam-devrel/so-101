package so_arm

import (
	"math"
	"testing"
)

func approx(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

// At rest (load == bias), goal must not move and bias must not drift.
func TestManualStep_AtRestHolds(t *testing.T) {
	p := manualParams{deadband: 10, gain: 0.5, biasAlpha: 0.1, dt: 0.02}
	st := jointState{bias: 40, goal: 0.3}
	out := manualStep(st, 40 /*load*/, [2]float64{-3, 3}, p)
	if !approx(out.goal, 0.3) {
		t.Fatalf("goal moved at rest: %v", out.goal)
	}
	if !approx(out.bias, 40) {
		t.Fatalf("bias drifted at rest: %v", out.bias)
	}
	if out.moved {
		t.Fatal("moved should be false at rest")
	}
}

// Load within deadband of bias: no motion, but bias tracks toward load.
func TestManualStep_WithinDeadbandTracksBias(t *testing.T) {
	p := manualParams{deadband: 10, gain: 0.5, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 0.3}
	out := manualStep(st, 45 /*load, excess=5 < 10*/, [2]float64{-3, 3}, p)
	if out.moved {
		t.Fatal("should not move within deadband")
	}
	// bias += 0.5*(45-40) = 42.5
	if !approx(out.bias, 42.5) {
		t.Fatalf("bias should track to 42.5, got %v", out.bias)
	}
}

// Push beyond deadband moves goal in push direction; bias is frozen while moving.
func TestManualStep_PushMovesGoalAndFreezesBias(t *testing.T) {
	p := manualParams{deadband: 10, gain: 0.5, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 0.3}
	out := manualStep(st, 60 /*load, excess=20*/, [2]float64{-3, 3}, p)
	// goal += gain * (excess - deadband) * dt = 0.5 * (20-10) * 0.02 = 0.1
	if !approx(out.goal, 0.4) {
		t.Fatalf("goal should move to 0.4, got %v", out.goal)
	}
	if !approx(out.bias, 40) {
		t.Fatalf("bias must be frozen while pushing, got %v", out.bias)
	}
	if !out.moved {
		t.Fatal("moved should be true")
	}
}

// Negative push moves goal negative.
func TestManualStep_NegativePush(t *testing.T) {
	p := manualParams{deadband: 10, gain: 0.5, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 0.3}
	out := manualStep(st, 20 /*excess=-20*/, [2]float64{-3, 3}, p)
	// goal += 0.5 * (-20 - (-10)) * 0.02 = -0.1
	if !approx(out.goal, 0.2) {
		t.Fatalf("goal should move to 0.2, got %v", out.goal)
	}
}

// Goal is clamped to joint limits (soft wall).
func TestManualStep_ClampsToLimit(t *testing.T) {
	p := manualParams{deadband: 10, gain: 50, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 0.95}
	out := manualStep(st, 200 /*big push up*/, [2]float64{-1, 1}, p)
	if !approx(out.goal, 1.0) {
		t.Fatalf("goal must be clamped to 1.0, got %v", out.goal)
	}
}

// A large negative push against a joint near its lower limit clamps to the lower limit.
func TestManualStep_LowerBoundClamp(t *testing.T) {
	p := manualParams{deadband: 10, gain: 50, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: -0.95}
	out := manualStep(st, -160 /*big push down, excess=-200*/, [2]float64{-1, 1}, p)
	if !approx(out.goal, -1.0) {
		t.Fatalf("goal must be clamped to -1.0, got %v", out.goal)
	}
}

// Goal already at the limit, push further into the wall: no movement, so the caller skips the write.
func TestManualStep_ClampSaturationNotMoved(t *testing.T) {
	p := manualParams{deadband: 10, gain: 50, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 1.0}
	out := manualStep(st, 200 /*big push up into the wall*/, [2]float64{-1, 1}, p)
	if out.moved {
		t.Fatal("moved must be false when already saturated at the limit")
	}
	if !approx(out.goal, 1.0) {
		t.Fatalf("goal must stay at 1.0, got %v", out.goal)
	}
}

// Exactly at the deadband edge: takes the push branch (rest guard is strict <), drive==0,
// so goal does not move, moved is false, and bias is frozen (NOT low-pass tracked).
func TestManualStep_ExactlyAtDeadband(t *testing.T) {
	p := manualParams{deadband: 10, gain: 0.5, biasAlpha: 0.5, dt: 0.02}
	st := jointState{bias: 40, goal: 0.3}
	out := manualStep(st, 50 /*excess exactly +10 == deadband*/, [2]float64{-3, 3}, p)
	if out.moved {
		t.Fatal("moved must be false when drive is exactly zero at the deadband edge")
	}
	if !approx(out.goal, 0.3) {
		t.Fatalf("goal must not move at the deadband edge, got %v", out.goal)
	}
	if !approx(out.bias, 40) {
		t.Fatalf("bias must be frozen (not tracked) in the push branch, got %v", out.bias)
	}
}

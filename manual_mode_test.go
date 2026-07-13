package so_arm

import (
	"context"
	"math"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
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

// fakeIO records writes and lets a test drive loads.
type fakeIO struct {
	mu        sync.Mutex
	ids       []int
	pos       map[int]float64
	loads     map[int]int
	limits    map[int][2]float64
	pgainSet  map[int]int
	pgainRest bool
	writes    int
}

func (f *fakeIO) servoIDs() []int { return f.ids }
func (f *fakeIO) positions(ctx context.Context) (map[int]float64, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	cp := map[int]float64{}
	for k, v := range f.pos {
		cp[k] = v
	}
	return cp, nil
}
func (f *fakeIO) loadsFor(ctx context.Context) (map[int]int, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	cp := map[int]int{}
	for k, v := range f.loads {
		cp[k] = v
	}
	return cp, nil
}
func (f *fakeIO) limitsFor() map[int][2]float64 { return f.limits }
func (f *fakeIO) writeGoals(ctx context.Context, goals map[int]float64) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.writes++
	for k, v := range goals {
		f.pos[k] = v // simulate the servo reaching goal
	}
	return nil
}
func (f *fakeIO) reducePGain(ctx context.Context, v int) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.pgainSet = map[int]int{}
	for _, id := range f.ids {
		f.pgainSet[id] = v
	}
	return nil
}
func (f *fakeIO) restorePGain(ctx context.Context) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.pgainRest = true
	return nil
}

func newFakeIO() *fakeIO {
	return &fakeIO{
		ids:    []int{1, 2},
		pos:    map[int]float64{1: 0, 2: 0},
		loads:  map[int]int{1: 0, 2: 0},
		limits: map[int][2]float64{1: {-3, 3}, 2: {-3, 3}},
	}
}

// Enter starts the loop; exit tears it down and restores pgain.
func TestManualSession_EnterExit(t *testing.T) {
	io := newFakeIO()
	p := manualParams{deadband: 5, gain: 1, biasAlpha: 0.1, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 12, logging.NewTestLogger(t))
	sess.start()
	if !sess.running() {
		t.Fatal("session should be running")
	}
	sess.stop() // blocks until loop exits
	if sess.running() {
		t.Fatal("session should be stopped")
	}
	io.mu.Lock()
	defer io.mu.Unlock()
	if io.pgainSet[1] != 12 {
		t.Fatalf("pgain should have been reduced to 12, got %v", io.pgainSet[1])
	}
	if !io.pgainRest {
		t.Fatal("pgain should have been restored on stop")
	}
}

// A sustained push should drive the goal (and thus written position) in that direction.
func TestManualSession_PushDrivesGoal(t *testing.T) {
	io := newFakeIO()
	io.mu.Lock()
	io.loads[1] = 100 // steady hard push on joint 1
	io.mu.Unlock()
	p := manualParams{deadband: 5, gain: 1, biasAlpha: 0.02, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, logging.NewTestLogger(t))
	sess.start()
	time.Sleep(150 * time.Millisecond)
	sess.stop()
	io.mu.Lock()
	defer io.mu.Unlock()
	if io.pos[1] <= 0 {
		t.Fatalf("joint 1 goal should have advanced under push, got %v", io.pos[1])
	}
	if io.pos[2] != 0 {
		t.Fatalf("joint 2 (no push) should not move, got %v", io.pos[2])
	}
}

// status() returns per-joint telemetry while running.
func TestManualSession_Status(t *testing.T) {
	io := newFakeIO()
	p := manualParams{deadband: 5, gain: 1, biasAlpha: 0.1, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, logging.NewTestLogger(t))
	sess.start()
	time.Sleep(50 * time.Millisecond)
	joints := sess.statusJoints()
	sess.stop()
	if len(joints) != 2 {
		t.Fatalf("expected 2 joint status entries, got %d", len(joints))
	}
}

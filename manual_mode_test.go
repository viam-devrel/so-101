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

func TestManualStep_AtGoalHolds(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.3, 0.3, [2]float64{-3, 3}, p)
	if moved || !approx(ng, 0.3) {
		t.Fatalf("at goal must hold: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_WithinDeadbandHolds(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.3, 0.33, [2]float64{-3, 3}, p) // dev 0.03 < 0.05
	if moved || !approx(ng, 0.3) {
		t.Fatalf("within deadband must hold: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_PushPositiveFollows(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.3, 0.5, [2]float64{-3, 3}, p) // dev 0.2 > 0.05
	if !moved || !approx(ng, 0.45) {                        // 0.5 - 0.05
		t.Fatalf("positive push must follow to 0.45: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_PushNegativeFollows(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.3, 0.1, [2]float64{-3, 3}, p) // dev -0.2
	if !moved || !approx(ng, 0.15) {                        // 0.1 + 0.05
		t.Fatalf("negative push must follow to 0.15: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_ExactlyAtDeadbandHolds(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.3, 0.35, [2]float64{-3, 3}, p) // dev exactly 0.05, strict > => hold
	if moved || !approx(ng, 0.3) {
		t.Fatalf("exactly at deadband must hold: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_ClampUpper(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(0.95, 1.5, [2]float64{-1, 1}, p) // would follow to 1.45, clamp to 1.0
	if !moved || !approx(ng, 1.0) {
		t.Fatalf("must clamp to upper limit 1.0: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_ClampLower(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	ng, moved := manualStep(-0.95, -1.5, [2]float64{-1, 1}, p) // would follow to -1.45, clamp to -1.0
	if !moved || !approx(ng, -1.0) {
		t.Fatalf("must clamp to lower limit -1.0: ng=%v moved=%v", ng, moved)
	}
}

func TestManualStep_ClampSaturationNotMoved(t *testing.T) {
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	// goal already at the upper limit, hand pushes further into the wall
	ng, moved := manualStep(1.0, 1.5, [2]float64{-1, 1}, p)
	if moved || !approx(ng, 1.0) {
		t.Fatalf("goal saturated at limit must not move: ng=%v moved=%v", ng, moved)
	}
}

// fakeIO records writes and lets a test drive loads and a persistent "held" position
// that simulates an operator backdriving a joint (writeGoals cannot erase it).
type fakeIO struct {
	mu                 sync.Mutex
	ids                []int
	pos                map[int]float64
	held               map[int]float64
	loads              map[int]int
	limits             map[int][2]float64
	pGainApplied       int
	torqueApplied      int
	complianceRestored bool
	writes             int
}

func (f *fakeIO) servoIDs() []int { return f.ids }
func (f *fakeIO) positions(ctx context.Context) (map[int]float64, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	cp := map[int]float64{}
	for _, id := range f.ids {
		if h, ok := f.held[id]; ok {
			cp[id] = h
		} else {
			cp[id] = f.pos[id]
		}
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
func (f *fakeIO) applyCompliance(ctx context.Context, pGain, torqueLimit int) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.pGainApplied = pGain
	f.torqueApplied = torqueLimit
	return nil
}
func (f *fakeIO) restoreCompliance(ctx context.Context) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.complianceRestored = true
	return nil
}

func newFakeIO() *fakeIO {
	return &fakeIO{
		ids:    []int{1, 2},
		pos:    map[int]float64{1: 0, 2: 0},
		held:   map[int]float64{},
		loads:  map[int]int{1: 0, 2: 0},
		limits: map[int][2]float64{1: {-3, 3}, 2: {-3, 3}},
	}
}

// Enter starts the loop; exit tears it down and restores compliance.
func TestManualSession_EnterExit(t *testing.T) {
	io := newFakeIO()
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 12, 300, logging.NewTestLogger(t))
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
	if io.pGainApplied != 12 {
		t.Fatalf("p_gain should have been applied as 12, got %v", io.pGainApplied)
	}
	if io.torqueApplied != 300 {
		t.Fatalf("torque_limit should have been applied as 300, got %v", io.torqueApplied)
	}
	if !io.complianceRestored {
		t.Fatal("compliance should have been restored on stop")
	}
}

// A sustained backdrive should pull the goal (and thus written position) toward the hand,
// trailing by the deadband.
func TestManualSession_BackdriveFollowsGoal(t *testing.T) {
	io := newFakeIO()
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, 0, logging.NewTestLogger(t))
	sess.start() // goals seeded to pos = 0
	io.mu.Lock()
	io.held[1] = 0.5 // operator backdrives joint 1 to 0.5 and holds it there
	io.mu.Unlock()
	time.Sleep(150 * time.Millisecond)
	sess.stop()
	io.mu.Lock()
	defer io.mu.Unlock()
	if io.pos[1] <= 0.4 { // goal should have followed toward held - deadband (0.45)
		t.Fatalf("joint 1 goal should follow the backdrive, got %v", io.pos[1])
	}
	if io.pos[2] != 0 { // untouched joint holds
		t.Fatalf("joint 2 should not move, got %v", io.pos[2])
	}
}

// A load present at entry with no position backdrive must not cause any motion (load is
// telemetry only, not part of the control law).
func TestManualSession_EnterHoldsWithoutBackdrive(t *testing.T) {
	io := newFakeIO()
	io.mu.Lock()
	io.loads[1] = 100 // gravity-holding load present at entry
	io.pos[1] = 0.5
	io.mu.Unlock()
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, 0, logging.NewTestLogger(t))
	sess.start()
	time.Sleep(120 * time.Millisecond)
	sess.stop()
	io.mu.Lock()
	defer io.mu.Unlock()
	if io.pos[1] != 0.5 {
		t.Fatalf("joint must not move without a position backdrive, got %v", io.pos[1])
	}
}

// status() returns per-joint telemetry while running.
func TestManualSession_Status(t *testing.T) {
	io := newFakeIO()
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, 0, logging.NewTestLogger(t))
	sess.start()
	time.Sleep(50 * time.Millisecond)
	joints := sess.statusJoints()
	sess.stop()
	if len(joints) != 2 {
		t.Fatalf("expected 2 joint status entries, got %d", len(joints))
	}
}

// statusJoints() reports the last-observed present load per joint (telemetry only), even
// when the joint stays put because it was never backdriven.
func TestManualSession_StatusReportsLoad(t *testing.T) {
	io := newFakeIO()
	io.mu.Lock()
	io.loads[1] = 30 // present load, but no position backdrive
	io.mu.Unlock()
	p := manualParams{posDeadband: 0.05, dt: 0.02}
	sess := newManualSession(context.Background(), io, p, 0, 0, logging.NewTestLogger(t))
	sess.start()
	time.Sleep(60 * time.Millisecond)
	joints := sess.statusJoints()
	sess.stop()
	var found bool
	for _, j := range joints {
		if j.Servo == 1 {
			found = true
			if j.Load != 30 {
				t.Fatalf("expected load 30 for servo 1, got %d", j.Load)
			}
		}
	}
	if !found {
		t.Fatal("servo 1 not in status")
	}
	io.mu.Lock()
	defer io.mu.Unlock()
	if io.pos[1] != 0 {
		t.Fatalf("joint with load but no backdrive must not move, got %v", io.pos[1])
	}
}

// TestResolveManualParams_BenchValidatedDefaults pins the out-of-the-box tuning to the
// values validated on the bench: easy to guide by hand while holding a fully extended arm.
func TestResolveManualParams_BenchValidatedDefaults(t *testing.T) {
	p, pGain, torqueLimit := resolveManualParams(nil, nil)
	if !approx(p.posDeadband, 0.5*math.Pi/180.0) {
		t.Fatalf("default pos_deadband_deg must be 0.5, got %v rad", p.posDeadband)
	}
	if !approx(p.dt, 1.0/50.0) {
		t.Fatalf("default loop rate must be 50 Hz, got dt=%v", p.dt)
	}
	if pGain != 8 {
		t.Fatalf("default p_gain must be 8, got %d", pGain)
	}
	if torqueLimit != 50 {
		t.Fatalf("default torque_limit must be 50, got %d", torqueLimit)
	}
}

// Config values must win over the defaults, including for the compliance registers.
func TestResolveManualParams_ConfigOverridesDefaults(t *testing.T) {
	cfg := &ManualModeConfig{PosDeadbandDeg: 3, LoopHz: 25, PGain: 32, TorqueLimit: 400}
	p, pGain, torqueLimit := resolveManualParams(cfg, nil)
	if !approx(p.posDeadband, 3*math.Pi/180.0) || !approx(p.dt, 1.0/25.0) {
		t.Fatalf("config loop params ignored: %+v", p)
	}
	if pGain != 32 || torqueLimit != 400 {
		t.Fatalf("config compliance ignored: p_gain=%d torque_limit=%d", pGain, torqueLimit)
	}
}

// An explicit 0 override is the escape hatch for leaving a compliance register untouched,
// since a config zero is indistinguishable from "absent" and falls back to the default.
func TestResolveManualParams_ZeroOverrideDisablesCompliance(t *testing.T) {
	overrides := map[string]interface{}{"p_gain": float64(0), "torque_limit": float64(0)}
	_, pGain, torqueLimit := resolveManualParams(nil, overrides)
	if pGain != 0 || torqueLimit != 0 {
		t.Fatalf("explicit zero must disable compliance writes: p_gain=%d torque_limit=%d", pGain, torqueLimit)
	}
}

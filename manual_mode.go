package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/logging"
)

// manualParams are the resolved loop tuning constants.
type manualParams struct {
	posDeadband float64 // radians; how far a joint must be backdriven from its goal before the goal follows
	dt          float64 // seconds per tick (loop rate)
}

// jointState is the per-joint state carried between ticks.
type jointState struct {
	goal   float64 // commanded position, radians
	actual float64 // last observed actual position, radians (telemetry)
	load   float64 // last observed present load (telemetry only)
}

// manualStep runs one position-deviation admittance step for a single joint. The servo
// holds `goal`; when the joint is backdriven past posDeadband, the goal follows the hand,
// trailing by the deadband (which leaves a small restoring force so the joint holds against
// gravity). Returns the new goal (clamped to limit) and whether it changed.
func manualStep(goal, actual float64, limit [2]float64, p manualParams) (newGoal float64, moved bool) {
	dev := actual - goal
	ng := goal
	if dev > p.posDeadband {
		ng = actual - p.posDeadband
	} else if dev < -p.posDeadband {
		ng = actual + p.posDeadband
	}
	if ng < limit[0] {
		ng = limit[0]
	}
	if ng > limit[1] {
		ng = limit[1]
	}
	return ng, ng != goal
}

// manualIO is the seam between the manual-mode loop and the hardware controller.
type manualIO interface {
	servoIDs() []int
	positions(ctx context.Context) (map[int]float64, error) // radians
	loadsFor(ctx context.Context) (map[int]int, error)      // signed load
	limitsFor() map[int][2]float64                          // radians, by servo id
	writeGoals(ctx context.Context, goals map[int]float64) error
	reducePGain(ctx context.Context, v int) error // no-op if v == 0
	restorePGain(ctx context.Context) error
}

const maxConsecutiveLoopErrors = 10

type manualSession struct {
	io     manualIO
	params manualParams
	pgain  int
	logger logging.Logger

	ctx    context.Context
	cancel context.CancelFunc
	done   chan struct{}

	mu    sync.Mutex
	state map[int]jointState
	run   bool
}

func newManualSession(parent context.Context, io manualIO, p manualParams, pgain int, logger logging.Logger) *manualSession {
	ctx, cancel := context.WithCancel(parent)
	return &manualSession{
		io: io, params: p, pgain: pgain, logger: logger,
		ctx: ctx, cancel: cancel, done: make(chan struct{}),
		state: map[int]jointState{},
	}
}

func (m *manualSession) running() bool { m.mu.Lock(); defer m.mu.Unlock(); return m.run }

// start seeds goals at the current pose, reduces stiffness, and launches the loop.
func (m *manualSession) start() {
	// Freeze goals at current measured positions so entry causes no motion.
	pos, err := m.io.positions(m.ctx)
	if err != nil {
		m.logger.Warnf("manual mode: failed to read start positions: %v", err)
		pos = map[int]float64{}
	}
	m.mu.Lock()
	for _, id := range m.io.servoIDs() {
		p := pos[id]
		m.state[id] = jointState{goal: p, actual: p}
	}
	m.run = true
	m.mu.Unlock()

	if m.pgain > 0 {
		if err := m.io.reducePGain(m.ctx, m.pgain); err != nil {
			m.logger.Warnf("manual mode: failed to reduce p_gain: %v", err)
		}
	}
	go m.loop()
}

// stop cancels the loop and waits for it to exit; the loop's deferred cleanup
// restores stiffness and flips running() to false (single cleanup owner).
func (m *manualSession) stop() {
	m.cancel()
	<-m.done
}

func (m *manualSession) loop() {
	defer func() {
		if err := m.io.restorePGain(context.Background()); err != nil {
			m.logger.Warnf("manual mode: failed to restore p_gain: %v", err)
		}
		m.mu.Lock()
		m.run = false
		m.mu.Unlock()
		close(m.done)
	}()
	interval := time.Duration(m.params.dt * float64(time.Second))
	if interval <= 0 {
		interval = 20 * time.Millisecond
	}
	ticker := time.NewTicker(interval)
	defer ticker.Stop()

	errCount := 0
	for {
		select {
		case <-m.ctx.Done():
			return
		case <-ticker.C:
		}
		if err := m.tick(); err != nil {
			errCount++
			m.logger.Warnf("manual mode tick error (%d): %v", errCount, err)
			if errCount >= maxConsecutiveLoopErrors {
				m.logger.Errorf("manual mode: too many consecutive errors, exiting loop")
				return
			}
			continue
		}
		errCount = 0
	}
}

func (m *manualSession) tick() error {
	pos, err := m.io.positions(m.ctx)
	if err != nil {
		return err
	}
	loads, _ := m.io.loadsFor(m.ctx) // telemetry only; a load read failure must not stop control
	limits := m.io.limitsFor()

	goals := map[int]float64{}
	m.mu.Lock()
	for id, st := range m.state {
		lim, ok := limits[id]
		if !ok {
			lim = [2]float64{-math.Pi, math.Pi}
		}
		actual := pos[id]
		ng, moved := manualStep(st.goal, actual, lim, m.params)
		st.goal = ng
		st.actual = actual
		if loads != nil {
			st.load = float64(loads[id])
		}
		m.state[id] = st
		if moved {
			goals[id] = ng
		}
	}
	m.mu.Unlock()

	if len(goals) > 0 {
		return m.io.writeGoals(m.ctx, goals)
	}
	return nil
}

// manualJointStatus is one joint's live telemetry for Status().
type manualJointStatus struct {
	Servo     int     `json:"servo"`
	ActualDeg float64 `json:"actual_deg"`
	GoalDeg   float64 `json:"goal_deg"`
	DevDeg    float64 `json:"dev_deg"`
	Load      int     `json:"load"`
}

func (m *manualSession) statusJoints() []manualJointStatus {
	m.mu.Lock()
	defer m.mu.Unlock()
	const rad2deg = 180.0 / math.Pi
	out := make([]manualJointStatus, 0, len(m.state))
	for _, id := range m.io.servoIDs() {
		st := m.state[id]
		out = append(out, manualJointStatus{
			Servo:     id,
			ActualDeg: st.actual * rad2deg,
			GoalDeg:   st.goal * rad2deg,
			DevDeg:    (st.actual - st.goal) * rad2deg,
			Load:      int(st.load),
		})
	}
	return out
}

// controllerManualIO adapts SafeSoArmController to manualIO for a set of arm servos.
type controllerManualIO struct {
	controller *SafeSoArmController
	ids        []int
	limits     map[int][2]float64
	origPGain  map[int]int // captured lazily on reducePGain for restore
}

func newControllerManualIO(c *SafeSoArmController, ids []int, limits [][2]float64) *controllerManualIO {
	lm := map[int][2]float64{}
	for i, id := range ids {
		if i < len(limits) {
			lm[id] = limits[i]
		}
	}
	return &controllerManualIO{controller: c, ids: ids, limits: lm, origPGain: map[int]int{}}
}

func (a *controllerManualIO) servoIDs() []int { return a.ids }

func (a *controllerManualIO) positions(ctx context.Context) (map[int]float64, error) {
	vals, err := a.controller.GetJointPositionsForServos(ctx, a.ids)
	if err != nil {
		return nil, err
	}
	out := map[int]float64{}
	for i, id := range a.ids {
		if i < len(vals) {
			out[id] = vals[i]
		}
	}
	return out, nil
}

func (a *controllerManualIO) loadsFor(ctx context.Context) (map[int]int, error) {
	return a.controller.LoadForServos(ctx, a.ids)
}

func (a *controllerManualIO) limitsFor() map[int][2]float64 { return a.limits }

func (a *controllerManualIO) writeGoals(ctx context.Context, goals map[int]float64) error {
	ids := make([]int, 0, len(goals))
	rads := make([]float64, 0, len(goals))
	for id, g := range goals {
		ids = append(ids, id)
		rads = append(rads, g)
	}
	// speed=0, acc=0 => plain goal write at default speed (small per-tick deltas).
	return a.controller.MoveServosToPositions(ctx, ids, rads, 0, 0)
}

func (a *controllerManualIO) reducePGain(ctx context.Context, v int) error {
	// Capture every servo's original p_gain BEFORE changing anything, so a later
	// restore always covers every servo we touch. If any read fails, change nothing
	// (manual mode then runs at full stiffness — safe: the arm still holds).
	orig := make(map[int]int, len(a.ids))
	for _, id := range a.ids {
		cur, err := a.controller.ReadServoRegister(ctx, id, "p_gain")
		if err != nil {
			return fmt.Errorf("failed to read p_gain for servo %d: %w", id, err)
		}
		if len(cur) != 1 {
			return fmt.Errorf("unexpected p_gain width %d for servo %d", len(cur), id)
		}
		orig[id] = int(cur[0])
	}
	a.origPGain = orig
	for _, id := range a.ids {
		if err := a.controller.WriteServoRegister(ctx, id, "p_gain", []byte{byte(v)}); err != nil {
			return err
		}
	}
	return nil
}

func (a *controllerManualIO) restorePGain(ctx context.Context) error {
	for id, v := range a.origPGain {
		if err := a.controller.WriteServoRegister(ctx, id, "p_gain", []byte{byte(v)}); err != nil {
			return err
		}
	}
	return nil
}

// manualDefaults returns the built-in tuning; conservative for a stable first bring-up.
func manualDefaults() manualParams {
	return manualParams{posDeadband: 2.0 * math.Pi / 180.0, dt: 1.0 / 50.0} // 2° deadband, 50 Hz
}

// resolveManualParams merges config + inline overrides over defaults.
// Guarantees dt > 0 (LoopHz override only applies when > 0; default is 1/50).
func resolveManualParams(cfg *ManualModeConfig, overrides map[string]interface{}) (manualParams, int) {
	p := manualDefaults()
	pgain := 0
	if cfg != nil {
		if cfg.PosDeadbandDeg > 0 {
			p.posDeadband = cfg.PosDeadbandDeg * math.Pi / 180.0
		}
		if cfg.LoopHz > 0 {
			p.dt = 1.0 / cfg.LoopHz
		}
		if cfg.PGain > 0 {
			pgain = cfg.PGain
		}
	}
	if v, ok := overrides["pos_deadband_deg"].(float64); ok && v > 0 {
		p.posDeadband = v * math.Pi / 180.0
	}
	if v, ok := overrides["loop_hz"].(float64); ok && v > 0 {
		p.dt = 1.0 / v
	}
	if v, ok := overrides["p_gain"].(float64); ok && v >= 0 && v <= 255 {
		pgain = int(v)
	}
	return p, pgain
}

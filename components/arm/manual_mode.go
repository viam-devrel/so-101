package arm

import (
	"context"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
)

// exitTimer schedules a one-shot automatic exit from manual mode.
type exitTimer struct {
	mu sync.Mutex
	t  *time.Timer
}

func (e *exitTimer) schedule(d time.Duration, exit func()) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
	}
	e.t = time.AfterFunc(d, exit)
}

func (e *exitTimer) cancel() {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
		e.t = nil
	}
}

// Properties returns which optional features the arm supports.
func (s *so101) Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: true, SupportCartesianCommands: true}, nil
}

// ManualMode reports whether the admittance manual-mode session is running.
func (s *so101) ManualMode(ctx context.Context, extra map[string]interface{}) (bool, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.manual != nil && s.manual.running(), nil
}

// SetManualMode enters or exits manual mode. When enabledFor > 0, manual mode
// auto-exits after that duration (any explicit exit path cancels the timer).
func (s *so101) SetManualMode(ctx context.Context, on bool, enabledFor time.Duration, extra map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.manualExit.cancel()
	if !on {
		s.exitManualLocked("SetManualMode(false)")
		return nil
	}
	s.enterManualLocked(extra)
	if enabledFor > 0 {
		s.manualExit.schedule(enabledFor, s.autoExitManualMode)
	}
	return nil
}

// autoExitManualMode runs from the exitTimer when a SetManualMode deadline elapses.
func (s *so101) autoExitManualMode() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.exitManualLocked("manual mode duration elapsed")
}

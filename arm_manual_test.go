package so_arm

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
)

func TestStatusReportsAutoWhenNotManual(t *testing.T) {
	s := &so101{}
	out, err := s.Status(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if out["mode"] != "auto" {
		t.Fatalf("expected auto, got %v", out["mode"])
	}
	if _, present := out["manual_mode"]; present {
		t.Fatal("manual_mode should be absent in auto")
	}
}

func TestExitManualLockedTearsDown(t *testing.T) {
	io := newFakeIO()
	s := &so101{logger: logging.NewTestLogger(t)}
	s.manual = newManualSession(context.Background(), io, manualDefaults(), 0, s.logger)
	s.manual.start()
	s.mu.Lock()
	s.exitManualLocked("test")
	s.mu.Unlock()
	if s.manual != nil {
		t.Fatal("session should be cleared after exitManualLocked")
	}
	io.mu.Lock()
	defer io.mu.Unlock()
	if !io.pgainRest {
		t.Fatal("p_gain should be restored on teardown")
	}
}

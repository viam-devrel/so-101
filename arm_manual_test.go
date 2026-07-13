package so_arm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"google.golang.org/protobuf/types/known/structpb"
)

// TestStatusIsProtoSerializableInManualMode guards against regressing the wire format:
// the robot's GetStatus path serializes the arm's Status() via structpb.NewStruct, which
// rejects any slice that isn't []interface{} (e.g. a []manualJointStatus). Status() must
// therefore emit only structpb-native types.
func TestStatusIsProtoSerializableInManualMode(t *testing.T) {
	io := newFakeIO()
	s := &so101{logger: logging.NewTestLogger(t)}
	s.manual = newManualSession(context.Background(), io, manualDefaults(), 0, 0, s.logger)
	s.manual.start()
	defer func() {
		s.mu.Lock()
		s.exitManualLocked("test cleanup")
		s.mu.Unlock()
	}()
	time.Sleep(40 * time.Millisecond)

	out, err := s.Status(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if out["mode"] != "manual" {
		t.Fatalf("expected manual, got %v", out["mode"])
	}
	if _, err := structpb.NewStruct(out); err != nil {
		t.Fatalf("Status output must be structpb-serializable, got: %v", err)
	}
}

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
	s.manual = newManualSession(context.Background(), io, manualDefaults(), 0, 0, s.logger)
	s.manual.start()
	s.mu.Lock()
	s.exitManualLocked("test")
	s.mu.Unlock()
	if s.manual != nil {
		t.Fatal("session should be cleared after exitManualLocked")
	}
	io.mu.Lock()
	defer io.mu.Unlock()
	if !io.complianceRestored {
		t.Fatal("p_gain should be restored on teardown")
	}
}

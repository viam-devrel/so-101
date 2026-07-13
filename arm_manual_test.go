package so_arm

import (
	"context"
	"testing"
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

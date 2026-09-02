package arm

import (
	"context"
	"fmt"

	"so_arm/internal/controller"
)

// writeGainIfChanged writes a one-byte gain register only when it does not already hold the
// target value.
//
// p_gain, d_gain and i_gain are all EEPROM registers (addresses 21-23) with finite write
// endurance, and every write costs three bus transactions for the unlock/write/relock dance.
// Both callers rewrite the same values repeatedly -- manual mode on every entry and exit,
// arm init on every Reconfigure -- so the skip is the wear guard, not an optimization.
func writeGainIfChanged(ctx context.Context, h *controller.ControllerHandle, id int, name string, want byte) error {
	cur, err := h.ReadServoRegister(ctx, id, name)
	if err != nil {
		return fmt.Errorf("read %s servo %d: %w", name, id, err)
	}
	if len(cur) != 1 {
		return fmt.Errorf("unexpected %s width %d for servo %d", name, len(cur), id)
	}
	if cur[0] == want {
		return nil
	}
	return h.WriteServoRegister(ctx, id, name, []byte{want})
}

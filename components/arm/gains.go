package arm

import (
	"context"

	"go.viam.com/rdk/logging"

	"so_arm/internal/controller"
	"so_arm/internal/servo"
)

// writeGainIfChanged writes a one-byte gain register unless it can prove the write would
// change nothing. These are EEPROM registers and both callers rewrite the same values
// repeatedly, so the skip is a wear guard.
//
// A FAILED read falls through to the write: it cannot prove the write is unnecessary, and
// skipping one on restoreCompliance's path leaves a joint soft in EEPROM. See CLAUDE.md,
// "Compliance zeroes i_gain".
func writeGainIfChanged(ctx context.Context, h *controller.ControllerHandle, id int, name string, want byte) error {
	if cur, err := h.ReadServoRegister(ctx, id, name); err == nil && len(cur) == 1 && cur[0] == want {
		return nil
	}
	return h.WriteServoRegister(ctx, id, name, []byte{want})
}

// applyServoGains writes the tuned P/D/I table to the arm's own servos.
//
// D, then I, then P, and a failed write BREAKS out for that servo: a non-zero I is only
// stable against a raised D, so damping-first makes every prefix of the sequence at least as
// stable as the state before it -- but only if a failure is terminal, or {I,P}-without-D
// strands a gravity-loaded joint on a combination measured to oscillate. Never returns an
// error; the arm works at stock gains. See docs/arm.md, "Servo gains".
func applyServoGains(ctx context.Context, h *controller.ControllerHandle, ids []int, gains map[int]servo.Gains, logger logging.Logger) {
	for _, id := range ids {
		g, ok := gains[id]
		if !ok {
			continue
		}
		for _, w := range []struct {
			name string
			want byte
		}{{"d_gain", g.D}, {"i_gain", g.I}, {"p_gain", g.P}} {
			if err := writeGainIfChanged(ctx, h, id, w.name, w.want); err != nil {
				logger.Warnf("servo %d: %s write failed; leaving it and this servo's "+
					"remaining gains as found: %v", id, w.name, err)
				break
			}
		}
	}
}

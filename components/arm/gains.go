package arm

import (
	"context"

	"go.viam.com/rdk/logging"

	"so_arm/internal/controller"
	"so_arm/internal/servo"
)

// writeGainIfChanged writes a one-byte gain register unless it can prove the write would
// change nothing.
//
// p_gain, d_gain and i_gain are all EEPROM registers (addresses 21-23) with finite write
// endurance, and every write costs three bus transactions for the unlock/write/relock dance.
// Both callers rewrite the same values repeatedly -- manual mode on every entry and exit,
// arm init on every Reconfigure -- so the skip is the wear guard, not an optimization.
//
// A read that FAILS therefore falls through to the write rather than returning: it cannot
// prove the write is unnecessary, and the wear guard is not worth the alternative. On
// restoreCompliance's path, skipping the write leaves a joint at p_gain 8 / i_gain 0 in
// EEPROM -- soft, across power cycles -- and a bare register read failing outright is a
// transient this hardware really produces, with nothing retrying it.
func writeGainIfChanged(ctx context.Context, h *controller.ControllerHandle, id int, name string, want byte) error {
	if cur, err := h.ReadServoRegister(ctx, id, name); err == nil && len(cur) == 1 && cur[0] == want {
		return nil
	}
	return h.WriteServoRegister(ctx, id, name, []byte{want})
}

// applyServoGains writes the tuned P/D/I table to the arm's own servos, skipping registers
// that already hold the target value.
//
// The write ORDER is fixed at D, I, P and is not incidental: a non-zero I is only stable
// against a raised D (measured -- joint 2 oscillated at D=32/I=4 and was stable at
// D=48/I=4), so damping first means every prefix of the sequence is at least as stable as
// the state before it. A partial failure then cannot strand a gravity-loaded joint on a
// combination measured to oscillate, in EEPROM.
//
// It NEVER returns an error: the arm works at stock gains, so a bus problem here must not
// prevent it from coming up.
func applyServoGains(ctx context.Context, h *controller.ControllerHandle, ids []int, gains map[int]servo.Gains, logger logging.Logger) {
	for _, id := range ids {
		g, ok := gains[id]
		if !ok {
			continue
		}
		for _, w := range []struct {
			name string
			want int
		}{{"d_gain", g.D}, {"i_gain", g.I}, {"p_gain", g.P}} {
			if err := writeGainIfChanged(ctx, h, id, w.name, byte(w.want)); err != nil {
				logger.Warnf("servo %d: leaving %s as found: %v", id, w.name, err)
			}
		}
	}
}

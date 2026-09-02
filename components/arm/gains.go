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
// it can prove already hold the target value.
//
// The write ORDER is fixed at D, I, P and is not incidental: a non-zero I is only stable
// against a raised D (measured -- joint 2 oscillated at D=32/I=4 and was stable at
// D=48/I=4), so damping first means every prefix of the sequence is at least as stable as
// the state before it. A partial failure then cannot strand a gravity-loaded joint on a
// combination measured to oscillate, in EEPROM.
//
// The prefix argument only holds if a failure is TERMINAL for that servo, hence the break:
// without it the reachable states are all eight subsets rather than the four prefixes, and
// {I,P}-without-D is 48/32/4 on servo 3 -- precisely the state this ordering exists to
// prevent. It is load-bearing for a second reason since writeGainIfChanged began writing
// through an unreadable register: all three writes are now attempted even when the reads
// fail, so the break is the only thing bounding the failure set. Per servo, not per call --
// one servo's bus trouble must not skip the rest.
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

package so_arm

import (
	"errors"
	"fmt"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// ErrPortDisconnected is returned by every bus operation while the port is down, in place
// of stalling a second per servo on serial timeouts.
var ErrPortDisconnected = errors.New("serial port disconnected")

// busSession is one open bus and everything derived from it. Immutable once published: a
// reconnect builds a new one and swaps it in, so an in-flight caller keeps using the
// session it loaded rather than observing a half-swapped one.
type busSession struct {
	bus    *feetech.Bus
	group  *feetech.ServoGroup
	servos map[int]*CalibratedServo
	gen    uint64
}

// newBusSession assembles a session around an open bus, wrapping each servo in the
// calibration the entry already holds.
func newBusSession(bus *feetech.Bus, cal SO101FullCalibration, gen uint64) *busSession {
	raw := make(map[int]*feetech.Servo, 6)
	servos := make(map[int]*CalibratedServo, 6)
	for id := 1; id <= 6; id++ {
		raw[id] = feetech.NewServo(bus, id, &feetech.ModelSTS3215)
		servos[id] = NewCalibratedServo(raw[id], copyMotorCalibration(cal.GetMotorCalibrationByID(id)))
	}
	return &busSession{
		bus:    bus,
		group:  feetech.NewServoGroup(bus, raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]),
		servos: servos,
		gen:    gen,
	}
}

// copyMotorCalibration detaches a calibration from whatever struct it arrived in, so a
// session's servos cannot be mutated through the caller's copy.
func copyMotorCalibration(src *MotorCalibration) *MotorCalibration {
	if src == nil {
		return nil
	}
	return &MotorCalibration{
		ID:           src.ID,
		DriveMode:    src.DriveMode,
		HomingOffset: src.HomingOffset,
		RangeMin:     src.RangeMin,
		RangeMax:     src.RangeMax,
		NormMode:     src.NormMode,
	}
}

// Two mutexes, deliberately. entry.busMu serializes bus WORK; entry.mu guards bookkeeping
// (refCount, pins, torndown, calibration). Keeping them apart is what stops a one-second
// serial timeout from blocking a Release, an acquire, or a teardown.
//
// Lock order: busMu -> entry.mu. Never the reverse.

// withSession runs a bus WRITE. Exclusive, matching the s.mu.Lock() paths this replaced.
func (h *ControllerHandle) withSession(f func(*busSession) error) error {
	h.entry.busMu.Lock()
	defer h.entry.busMu.Unlock()
	return h.runLocked(f)
}

// withSessionRead runs a bus READ. Shared, matching the s.mu.RLock() paths this replaced.
//
// The distinction is worth preserving even though feetech.Bus serializes each individual
// transaction internally (its own b.mu): what the controller lock buys is MULTI-transaction
// atomicity, so a write sequence is not interleaved with reads.
func (h *ControllerHandle) withSessionRead(f func(*busSession) error) error {
	h.entry.busMu.RLock()
	defer h.entry.busMu.RUnlock()
	return h.runLocked(f)
}

func (h *ControllerHandle) runLocked(f func(*busSession) error) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	return h.entry.report(sess.gen, f(sess))
}

// report is where a bus error will become a connection verdict once serial hotplug lands.
// Until then it is the identity function.
//
// It is called with busMu held, so it must never take busMu. Taking entry.mu IS legal and
// hotplug's version does -- that is the declared busMu -> entry.mu order, not a violation.
func (e *ControllerEntry) report(gen uint64, err error) error { return err }

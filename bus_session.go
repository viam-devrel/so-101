package so_arm

import (
	"errors"
	"fmt"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// ErrPortDisconnected is returned by bus operations when the port has been torn down,
// instead of stalling a second per servo on serial timeouts.
var ErrPortDisconnected = errors.New("serial port disconnected")

// busSession is one open bus and everything derived from it. Treated as immutable once
// published so a holder that loaded it never sees it change underneath.
type busSession struct {
	bus    *feetech.Bus
	group  *feetech.ServoGroup
	servos map[int]*CalibratedServo
}

func newBusSession(bus *feetech.Bus, cal SO101FullCalibration) *busSession {
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
	}
}

// copyMotorCalibration detaches a calibration from the caller's struct, so the caller
// cannot mutate a servo's calibration after handing it over.
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

// withSession runs a bus write, excluding all other bus work on this port.
func (h *ControllerHandle) withSession(f func(*busSession) error) error {
	h.entry.busMu.Lock()
	defer h.entry.busMu.Unlock()
	return h.run(f)
}

// withSessionRead runs a bus read. Reads share the lock; the exclusion that matters is
// against writes, since a write sequence must not be interleaved with reads. Individual
// transactions are already serialized inside feetech.Bus.
func (h *ControllerHandle) withSessionRead(f func(*busSession) error) error {
	h.entry.busMu.RLock()
	defer h.entry.busMu.RUnlock()
	return h.run(f)
}

func (h *ControllerHandle) run(f func(*busSession) error) error {
	sess := h.entry.session.Load()
	if sess == nil {
		return fmt.Errorf("%w: %s", ErrPortDisconnected, h.entry.port)
	}
	return f(sess)
}

package controller

import (
	"context"
	"fmt"
	"sort"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// ServoHealth is one servo's supply and condition, as reported by servo_health. Voltages are
// volts, temperature degrees C. A field the servo did not answer for is left at zero and
// named in Errors.
type ServoHealth struct {
	ID             int
	PresentVoltage float64
	MinVoltage     float64
	MaxVoltage     float64
	TemperatureC   int
	MaxTempC       int
	Flags          feetech.StatusError
	Errors         []string
}

// Map is a structpb-compatible form for DoCommand.
func (h ServoHealth) Map() map[string]interface{} {
	flags := []interface{}{}
	for _, f := range []struct {
		bit  feetech.StatusError
		name string
	}{
		{feetech.ErrVoltage, "voltage"},
		{feetech.ErrAngleLimit, "angle_limit"},
		{feetech.ErrOverheat, "overheat"},
		{feetech.ErrOverload, "overload"},
	} {
		if h.Flags&f.bit != 0 {
			flags = append(flags, f.name)
		}
	}
	errs := make([]interface{}, len(h.Errors))
	for i, e := range h.Errors {
		errs[i] = e
	}
	return map[string]interface{}{
		"servo_id":          h.ID,
		"present_voltage_v": h.PresentVoltage,
		"min_voltage_v":     h.MinVoltage,
		"max_voltage_v":     h.MaxVoltage,
		"temperature_c":     h.TemperatureC,
		"max_temperature_c": h.MaxTempC,
		"flags":             flags,
		"errors":            errs,
	}
}

// readByte reads a one-byte register, keeping the value when the only error is a condition
// flag, and returns the flag it saw.
func readByte(ctx context.Context, dev *feetech.Servo, name string) (int, feetech.StatusError, error) {
	data, err := dev.ReadRegister(ctx, name)
	flags, err := tolerateCondition(err)
	if err != nil {
		return 0, 0, err
	}
	if len(data) != 1 {
		return 0, flags, fmt.Errorf("%s: expected 1 byte, got %d", name, len(data))
	}
	return int(data[0]), flags, nil
}

func readHealth(ctx context.Context, bus *feetech.Bus, id int) ServoHealth {
	dev := feetech.NewServo(bus, id, &feetech.ModelSTS3215)
	h := ServoHealth{ID: id}
	read := func(name string) int {
		v, flags, err := readByte(ctx, dev, name)
		h.Flags |= flags
		if err != nil {
			h.Errors = append(h.Errors, fmt.Sprintf("%s: %v", name, err))
		}
		return v
	}
	h.PresentVoltage = float64(read("present_voltage")) / 10
	h.MinVoltage = float64(read("min_voltage")) / 10
	h.MaxVoltage = float64(read("max_voltage")) / 10
	h.TemperatureC = read("present_temp")
	h.MaxTempC = read("max_temp")
	return h
}

// ServoHealth reads supply voltage, voltage limits, temperature and condition flags from each
// servo. Read-only: it changes no register.
func (h *ControllerHandle) ServoHealth(ctx context.Context, servoIDs []int) ([]ServoHealth, error) {
	ids := append([]int(nil), servoIDs...)
	sort.Ints(ids)
	var out []ServoHealth
	err := h.withSessionRead(func(sess *busSession) error {
		for _, id := range ids {
			out = append(out, readHealth(ctx, sess.bus, id))
		}
		return nil
	})
	return out, err
}

// conditionDetail turns a bare condition flag into something actionable for the log: for a
// voltage flag, the voltage the servo sees against the limits it holds. Best effort; it
// returns "" if the extra reads fail. Caller holds the bus.
func conditionDetail(ctx context.Context, bus *feetech.Bus, id int, flags feetech.StatusError) string {
	if flags&feetech.ErrVoltage == 0 || bus == nil {
		return ""
	}
	hl := readHealth(ctx, bus, id)
	if len(hl.Errors) > 0 {
		return ""
	}
	return fmt.Sprintf(": supply %.1f V is outside this servo's limits %.1f-%.1f V "+
		"(check the power supply, or the servo's min/max voltage registers)",
		hl.PresentVoltage, hl.MinVoltage, hl.MaxVoltage)
}

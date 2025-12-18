package so_arm

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

// Normalization modes
const (
	NormModeRaw       = 0 // Raw servo values (0-4095 for STS3215)
	NormModeRange100  = 1 // Normalized to 0-100 range
	NormModeRangeM100 = 2 // Normalized to -100 to +100 range
	NormModeDegrees   = 3 // Normalized to -180° to +180° range
)

// MotorCalibration defines calibration parameters for a servo motor
type MotorCalibration struct {
	ID           int `json:"id"`
	DriveMode    int `json:"drive_mode"`
	HomingOffset int `json:"homing_offset"`
	RangeMin     int `json:"range_min"`
	RangeMax     int `json:"range_max"`
	NormMode     int `json:"norm_mode,omitempty"`
}

// Normalize converts a raw servo position to normalized value
func (c *MotorCalibration) Normalize(rawValue int) (float64, error) {
	var normalized float64

	// First normalize to the target range without considering drive mode
	switch c.NormMode {
	case NormModeRaw:
		normalized = float64(rawValue)

	case NormModeRange100:
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		normalized = float64(rawValue-c.RangeMin) / float64(c.RangeMax-c.RangeMin) * 100.0
		normalized = math.Max(0, math.Min(100, normalized))

	case NormModeRangeM100:
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		normalized = (float64(rawValue) - center) / halfRange * 100.0
		normalized = math.Max(-100, math.Min(100, normalized))

	case NormModeDegrees:
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		maxResolution := float64(4095)
		normalized = (float64(rawValue) - center) * 360 / maxResolution

	default:
		return 0, fmt.Errorf("unknown normalization mode: %d", c.NormMode)
	}

	// Apply drive mode inversion to the normalized value
	if c.DriveMode != 0 {
		switch c.NormMode {
		case NormModeRaw:
			center := float64(c.RangeMin+c.RangeMax) / 2.0
			normalized = 2*center - normalized
		case NormModeRange100:
			normalized = 100.0 - normalized
		case NormModeRangeM100:
			normalized = -normalized
		case NormModeDegrees:
			normalized = -normalized
		}
	}

	return normalized, nil
}

// Denormalize converts normalized value back to raw servo position
func (c *MotorCalibration) Denormalize(normalizedValue float64) (int, error) {
	// Apply drive mode inversion to the normalized value first
	adjustedValue := normalizedValue
	if c.DriveMode != 0 {
		switch c.NormMode {
		case NormModeRaw:
			center := float64(c.RangeMin+c.RangeMax) / 2.0
			adjustedValue = 2*center - normalizedValue
		case NormModeRange100:
			adjustedValue = 100.0 - normalizedValue
		case NormModeRangeM100:
			adjustedValue = -normalizedValue
		case NormModeDegrees:
			adjustedValue = -normalizedValue
		}
	}

	var rawValue int

	switch c.NormMode {
	case NormModeRaw:
		rawValue = int(math.Round(adjustedValue))

	case NormModeRange100:
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		clamped := math.Max(0, math.Min(100, adjustedValue))
		rawValue = int(math.Round(clamped/100.0*float64(c.RangeMax-c.RangeMin) + float64(c.RangeMin)))

	case NormModeRangeM100:
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		clamped := math.Max(-100, math.Min(100, adjustedValue))
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		rawValue = int(math.Round(center + clamped/100.0*halfRange))

	case NormModeDegrees:
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		maxResolution := float64(4095)
		rawValue = int((adjustedValue * maxResolution / 360) + center)

	default:
		return 0, fmt.Errorf("unknown normalization mode: %d", c.NormMode)
	}

	// Clamp to servo limits
	if rawValue < c.RangeMin {
		rawValue = c.RangeMin
	}
	if rawValue > c.RangeMax {
		rawValue = c.RangeMax
	}

	return rawValue, nil
}

// Validate checks if the calibration parameters are valid
func (c *MotorCalibration) Validate() error {
	if c.ID < 0 || c.ID > 253 {
		return fmt.Errorf("invalid servo ID: %d", c.ID)
	}

	if c.RangeMin >= c.RangeMax {
		return fmt.Errorf("invalid range: min (%d) must be less than max (%d)", c.RangeMin, c.RangeMax)
	}

	if c.RangeMin < 0 || c.RangeMax > 4095 {
		return fmt.Errorf("range values must be between 0-4095, got min=%d max=%d", c.RangeMin, c.RangeMax)
	}

	if c.NormMode < NormModeRaw || c.NormMode > NormModeDegrees {
		return fmt.Errorf("invalid normalization mode: %d", c.NormMode)
	}

	return nil
}

// CalibratedServo wraps a feetech.Servo with calibration support
type CalibratedServo struct {
	servo       *feetech.Servo
	calibration *MotorCalibration
	mu          sync.RWMutex
}

// NewCalibratedServo creates a new calibrated servo wrapper
func NewCalibratedServo(servo *feetech.Servo, calibration *MotorCalibration) *CalibratedServo {
	return &CalibratedServo{
		servo:       servo,
		calibration: calibration,
	}
}

// Position reads the current position and returns normalized value
func (cs *CalibratedServo) Position(ctx context.Context) (float64, error) {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	rawPos, err := cs.servo.Position(ctx)
	if err != nil {
		return 0, fmt.Errorf("failed to read position: %w", err)
	}

	normalized, err := cs.calibration.Normalize(rawPos)
	if err != nil {
		return 0, fmt.Errorf("failed to normalize position: %w", err)
	}

	return normalized, nil
}

// SetPosition sets the servo position from normalized value
func (cs *CalibratedServo) SetPosition(ctx context.Context, normalized float64) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	raw, err := cs.calibration.Denormalize(normalized)
	if err != nil {
		return fmt.Errorf("failed to denormalize position: %w", err)
	}

	if err := cs.servo.SetPosition(ctx, raw); err != nil {
		return fmt.Errorf("failed to set position: %w", err)
	}

	return nil
}

// SetPositionWithSpeed sets position with speed control
func (cs *CalibratedServo) SetPositionWithSpeed(ctx context.Context, normalized float64, speed int) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	raw, err := cs.calibration.Denormalize(normalized)
	if err != nil {
		return fmt.Errorf("failed to denormalize position: %w", err)
	}

	if err := cs.servo.SetPositionWithSpeed(ctx, raw, speed); err != nil {
		return fmt.Errorf("failed to set position with speed: %w", err)
	}

	return nil
}

// Enable enables the servo torque
func (cs *CalibratedServo) Enable(ctx context.Context) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.Enable(ctx)
}

// Disable disables the servo torque
func (cs *CalibratedServo) Disable(ctx context.Context) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.Disable(ctx)
}

// SetTorqueEnabled sets the torque enable state
func (cs *CalibratedServo) SetTorqueEnabled(ctx context.Context, enable bool) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.SetTorqueEnabled(ctx, enable)
}

// Moving checks if servo is currently moving
func (cs *CalibratedServo) Moving(ctx context.Context) (bool, error) {
	cs.mu.RLock()
	defer cs.mu.RUnlock()
	return cs.servo.Moving(ctx)
}

// Load reads the current load on the servo
// Returns signed value: positive = clockwise load, negative = counter-clockwise
func (cs *CalibratedServo) Load(ctx context.Context) (int, error) {
	cs.mu.RLock()
	defer cs.mu.RUnlock()
	return cs.servo.Load(ctx)
}

// Ping pings the servo
func (cs *CalibratedServo) Ping(ctx context.Context) (int, error) {
	cs.mu.RLock()
	defer cs.mu.RUnlock()
	return cs.servo.Ping(ctx)
}

// DetectModel detects the servo model
func (cs *CalibratedServo) DetectModel(ctx context.Context) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.DetectModel(ctx)
}

// Model returns the servo model
func (cs *CalibratedServo) Model() *feetech.Model {
	cs.mu.RLock()
	defer cs.mu.RUnlock()
	return cs.servo.Model()
}

// SetID sets the servo ID
func (cs *CalibratedServo) SetID(ctx context.Context, newID int) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.SetID(ctx, newID)
}

// SetBaudRate sets the servo baud rate
func (cs *CalibratedServo) SetBaudRate(ctx context.Context, baudRate int) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.SetBaudRate(ctx, baudRate)
}

// SetVelocity sets the servo velocity
func (cs *CalibratedServo) SetVelocity(ctx context.Context, vel int) error {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.servo.SetVelocity(ctx, vel)
}

// GetRawServo returns the underlying feetech.Servo (for ServoGroup creation)
func (cs *CalibratedServo) GetRawServo() *feetech.Servo {
	return cs.servo
}

// UpdateCalibration safely updates the calibration data
func (cs *CalibratedServo) UpdateCalibration(calibration *MotorCalibration) {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	cs.calibration = calibration
}

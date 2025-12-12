package so_arm

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.viam.com/rdk/logging"
)

type SoArm101Config struct {
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	ServoIDs []int `json:"servo_ids,omitempty"`

	Timeout time.Duration `json:"timeout,omitempty"`

	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`

	CalibrationFile string `json:"calibration_file,omitempty"`

	// Not serialized
	Logger logging.Logger `json:"-"`
}

type SO101FullCalibration struct {
	ShoulderPan  *MotorCalibration `json:"shoulder_pan"`
	ShoulderLift *MotorCalibration `json:"shoulder_lift"`
	ElbowFlex    *MotorCalibration `json:"elbow_flex"`
	WristFlex    *MotorCalibration `json:"wrist_flex"`
	WristRoll    *MotorCalibration `json:"wrist_roll"`
	Gripper      *MotorCalibration `json:"gripper"`
}

var DefaultSO101FullCalibration = SO101FullCalibration{
	ShoulderPan: &MotorCalibration{
		ID: 1, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeDegrees,
	},
	ShoulderLift: &MotorCalibration{
		ID: 2, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeDegrees,
	},
	ElbowFlex: &MotorCalibration{
		ID: 3, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeDegrees,
	},
	WristFlex: &MotorCalibration{
		ID: 4, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeDegrees,
	},
	WristRoll: &MotorCalibration{
		ID: 5, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeDegrees,
	},
	Gripper: &MotorCalibration{
		ID: 6, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: NormModeRange100, // 0-100% for gripper
	},
}

// Validate ensures all parts of the config are valid
func (cfg *SoArm101Config) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	if len(cfg.ServoIDs) == 0 {
		cfg.ServoIDs = []int{1, 2, 3, 4, 5}
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000
	}

	return nil, nil, nil
}

// LoadCalibration loads calibration from file or returns default calibration
// Returns (calibration, fromFile) where fromFile indicates if loaded from file
func (cfg *SoArm101Config) LoadCalibration(logger logging.Logger) (SO101FullCalibration, bool) {
	if cfg.CalibrationFile == "" {
		if logger != nil {
			logger.Debug("No calibration file specified, using default calibration")
		}
		return DefaultSO101FullCalibration, false
	}

	// Handle relative paths using VIAM_MODULE_DATA
	if !filepath.IsAbs(cfg.CalibrationFile) {
		moduleDataDir := os.Getenv("VIAM_MODULE_DATA")
		if moduleDataDir == "" {
			moduleDataDir = "/tmp" // Fallback if VIAM_MODULE_DATA not set
		}
		cfg.CalibrationFile = filepath.Join(moduleDataDir, cfg.CalibrationFile)
	}

	calibration, err := LoadFullCalibrationFromFile(cfg.CalibrationFile, logger)
	if err != nil {
		if logger != nil {
			logger.Warnf("Failed to load calibration from %s: %v, using default calibration", cfg.CalibrationFile, err)
		}
		return DefaultSO101FullCalibration, false
	}

	if logger != nil {
		logger.Debugf("Successfully loaded calibration from %s", cfg.CalibrationFile)
	}
	return calibration, true
}

// Maintains backward compatibility with existing calibration files
type CalibrationFileFormat struct {
	ShoulderPan  *CalibrationEntry `json:"shoulder_pan"`
	ShoulderLift *CalibrationEntry `json:"shoulder_lift"`
	ElbowFlex    *CalibrationEntry `json:"elbow_flex"`
	WristFlex    *CalibrationEntry `json:"wrist_flex"`
	WristRoll    *CalibrationEntry `json:"wrist_roll"`
	Gripper      *CalibrationEntry `json:"gripper"`
}

type CalibrationEntry struct {
	ID           int `json:"id"`
	DriveMode    int `json:"drive_mode"`
	HomingOffset int `json:"homing_offset"`
	RangeMin     int `json:"range_min"`
	RangeMax     int `json:"range_max"`
	NormMode     int `json:"norm_mode,omitempty"`
}

// ToMotorCalibration converts CalibrationEntry to MotorCalibration
func (ce *CalibrationEntry) ToMotorCalibration() *MotorCalibration {
	normMode := ce.NormMode
	if normMode == 0 {
		if ce.ID == 6 {
			normMode = NormModeRange100
		} else {
			normMode = NormModeDegrees
		}
	}

	return &MotorCalibration{
		ID:           ce.ID,
		DriveMode:    ce.DriveMode,
		HomingOffset: ce.HomingOffset,
		RangeMin:     ce.RangeMin,
		RangeMax:     ce.RangeMax,
		NormMode:     normMode,
	}
}

// FromMotorCalibration converts MotorCalibration to CalibrationEntry
func FromMotorCalibration(mc *MotorCalibration) *CalibrationEntry {
	return &CalibrationEntry{
		ID:           mc.ID,
		DriveMode:    mc.DriveMode,
		HomingOffset: mc.HomingOffset,
		RangeMin:     mc.RangeMin,
		RangeMax:     mc.RangeMax,
		NormMode:     mc.NormMode,
	}
}

// LoadFullCalibrationFromFile loads and validates full calibration from a JSON file
func LoadFullCalibrationFromFile(filePath string, logger logging.Logger) (SO101FullCalibration, error) {
	data, err := os.ReadFile(filePath)
	if err != nil {
		return SO101FullCalibration{}, fmt.Errorf("failed to read calibration file: %w", err)
	}

	var fileFormat CalibrationFileFormat
	if err := json.Unmarshal(data, &fileFormat); err != nil {
		return SO101FullCalibration{}, fmt.Errorf("failed to parse calibration JSON: %w", err)
	}

	convertOrDefault := func(entry *CalibrationEntry, defaultCal *MotorCalibration) *MotorCalibration {
		if entry != nil {
			return entry.ToMotorCalibration()
		}
		return defaultCal
	}

	calibration := SO101FullCalibration{
		ShoulderPan:  convertOrDefault(fileFormat.ShoulderPan, DefaultSO101FullCalibration.ShoulderPan),
		ShoulderLift: convertOrDefault(fileFormat.ShoulderLift, DefaultSO101FullCalibration.ShoulderLift),
		ElbowFlex:    convertOrDefault(fileFormat.ElbowFlex, DefaultSO101FullCalibration.ElbowFlex),
		WristFlex:    convertOrDefault(fileFormat.WristFlex, DefaultSO101FullCalibration.WristFlex),
		WristRoll:    convertOrDefault(fileFormat.WristRoll, DefaultSO101FullCalibration.WristRoll),
		Gripper:      convertOrDefault(fileFormat.Gripper, DefaultSO101FullCalibration.Gripper),
	}

	if err := ValidateFullCalibration(calibration, logger); err != nil {
		return SO101FullCalibration{}, fmt.Errorf("calibration validation failed: %w", err)
	}

	return calibration, nil
}

// SaveFullCalibrationToFile saves calibration to a JSON file
func SaveFullCalibrationToFile(filePath string, calibration SO101FullCalibration) error {
	convertOrNil := func(mc *MotorCalibration) *CalibrationEntry {
		if mc != nil {
			return FromMotorCalibration(mc)
		}
		return nil
	}

	fileFormat := CalibrationFileFormat{
		ShoulderPan:  convertOrNil(calibration.ShoulderPan),
		ShoulderLift: convertOrNil(calibration.ShoulderLift),
		ElbowFlex:    convertOrNil(calibration.ElbowFlex),
		WristFlex:    convertOrNil(calibration.WristFlex),
		WristRoll:    convertOrNil(calibration.WristRoll),
		Gripper:      convertOrNil(calibration.Gripper),
	}

	data, err := json.MarshalIndent(fileFormat, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal calibration: %w", err)
	}

	if err := os.WriteFile(filePath, data, 0644); err != nil {
		return fmt.Errorf("failed to write calibration file: %w", err)
	}

	return nil
}

// ValidateFullCalibration validates that all calibration values are reasonable
func ValidateFullCalibration(cal SO101FullCalibration, logger logging.Logger) error {
	joints := []struct {
		name   string
		config *MotorCalibration
	}{
		{"shoulder_pan", cal.ShoulderPan},
		{"shoulder_lift", cal.ShoulderLift},
		{"elbow_flex", cal.ElbowFlex},
		{"wrist_flex", cal.WristFlex},
		{"wrist_roll", cal.WristRoll},
		{"gripper", cal.Gripper},
	}

	for _, joint := range joints {
		if joint.config == nil {
			return fmt.Errorf("joint %s: calibration is nil", joint.name)
		}
		if err := joint.config.Validate(); err != nil {
			return fmt.Errorf("joint %s: %w", joint.name, err)
		}
	}

	if logger != nil {
		logger.Debug("Full calibration validation passed")
	}

	return nil
}

// GetMotorCalibrationByID returns the motor calibration for a specific servo ID
func (cal SO101FullCalibration) GetMotorCalibrationByID(servoID int) *MotorCalibration {
	switch servoID {
	case 1:
		return cal.ShoulderPan
	case 2:
		return cal.ShoulderLift
	case 3:
		return cal.ElbowFlex
	case 4:
		return cal.WristFlex
	case 5:
		return cal.WristRoll
	case 6:
		return cal.Gripper
	default:
		return nil
	}
}

// ToFeetechCalibrationMap converts SO101FullCalibration to a map for feetech-servo
func (cal SO101FullCalibration) ToFeetechCalibrationMap() map[int]*MotorCalibration {
	return map[int]*MotorCalibration{
		1: cal.ShoulderPan,
		2: cal.ShoulderLift,
		3: cal.ElbowFlex,
		4: cal.WristFlex,
		5: cal.WristRoll,
		6: cal.Gripper,
	}
}

// FromFeetechCalibrationMap creates SO101FullCalibration from a feetech calibration map
func FromFeetechCalibrationMap(calibrations map[int]*MotorCalibration) SO101FullCalibration {
	getOrDefault := func(id int, defaultCal *MotorCalibration) *MotorCalibration {
		if mc, exists := calibrations[id]; exists && mc != nil {
			return mc
		}
		return defaultCal
	}

	return SO101FullCalibration{
		ShoulderPan:  getOrDefault(1, DefaultSO101FullCalibration.ShoulderPan),
		ShoulderLift: getOrDefault(2, DefaultSO101FullCalibration.ShoulderLift),
		ElbowFlex:    getOrDefault(3, DefaultSO101FullCalibration.ElbowFlex),
		WristFlex:    getOrDefault(4, DefaultSO101FullCalibration.WristFlex),
		WristRoll:    getOrDefault(5, DefaultSO101FullCalibration.WristRoll),
		Gripper:      getOrDefault(6, DefaultSO101FullCalibration.Gripper),
	}
}

func (cal SO101FullCalibration) Equal(other SO101FullCalibration) bool {
	return calibrationsEqual(cal.ShoulderPan, other.ShoulderPan) &&
		calibrationsEqual(cal.ShoulderLift, other.ShoulderLift) &&
		calibrationsEqual(cal.ElbowFlex, other.ElbowFlex) &&
		calibrationsEqual(cal.WristFlex, other.WristFlex) &&
		calibrationsEqual(cal.WristRoll, other.WristRoll) &&
		calibrationsEqual(cal.Gripper, other.Gripper)
}

func calibrationsEqual(a, b *MotorCalibration) bool {
	if a == nil && b == nil {
		return true
	}
	if a == nil || b == nil {
		return false
	}
	return a.ID == b.ID &&
		a.DriveMode == b.DriveMode &&
		a.HomingOffset == b.HomingOffset &&
		a.RangeMin == b.RangeMin &&
		a.RangeMax == b.RangeMax &&
		a.NormMode == b.NormMode
}

// getNormModeForServo returns the appropriate NormMode for a servo ID
// Servo 6 (gripper) uses 0-100 range, servos 1-5 (arm) use degrees
func getNormModeForServo(servoID int) int {
	if servoID == 6 {
		return NormModeRange100 // Gripper uses 0-100%
	}
	return NormModeDegrees // Arm servos use degrees
}

// readUint16Register reads a 2-byte register from servo and decodes as uint16
func readUint16Register(ctx context.Context, servo *feetech.Servo, registerName string) (uint16, error) {
	data, err := servo.ReadRegister(ctx, registerName)
	if err != nil {
		return 0, err
	}
	if len(data) != 2 {
		return 0, fmt.Errorf("expected 2 bytes for %s, got %d", registerName, len(data))
	}
	return uint16(data[0]) | (uint16(data[1]) << 8), nil
}

// readInt16Register reads a 2-byte register and decodes as signed int16
func readInt16Register(ctx context.Context, servo *feetech.Servo, registerName string) (int, error) {
	data, err := servo.ReadRegister(ctx, registerName)
	if err != nil {
		return 0, err
	}
	if len(data) != 2 {
		return 0, fmt.Errorf("expected 2 bytes for %s, got %d", registerName, len(data))
	}

	// Decode as uint16 first
	raw := uint16(data[0]) | (uint16(data[1]) << 8)

	// Check if this is a sign-magnitude encoded value (bit 15 is sign bit for STS3215)
	// For homing_offset, sign bit is at position 11 (12-bit value)
	signBit := 11
	directionBit := (raw >> uint(signBit)) & 1
	magnitudeMask := uint16((1 << uint(signBit)) - 1)
	magnitude := int(raw & magnitudeMask)

	if directionBit != 0 {
		return -magnitude, nil
	}
	return magnitude, nil
}

// ReadCalibrationFromServos attempts to read calibration from servo registers
// Returns a complete calibration with successfully-read values and defaults for failures
// Never returns an error - worst case is all defaults
func ReadCalibrationFromServos(
	ctx context.Context,
	bus *feetech.Bus,
	servoIDs []int,
	logger logging.Logger,
) SO101FullCalibration {
	if bus == nil {
		if logger != nil {
			logger.Warn("Cannot read servo calibration: bus is nil")
		}
		return DefaultSO101FullCalibration
	}

	successCount := 0
	calibrations := make(map[int]*MotorCalibration)

	for _, servoID := range servoIDs {
		// Create servo instance for reading
		servo := feetech.NewServo(bus, servoID, &feetech.ModelSTS3215)

		// Try reading registers - updated method names
		homingOffset, offsetErr := readInt16Register(ctx, servo, "position_offset")
		minLimit, minErr := readUint16Register(ctx, servo, "min_angle_limit")
		maxLimit, maxErr := readUint16Register(ctx, servo, "max_angle_limit")

		// Check if we got valid data
		if offsetErr == nil && minErr == nil && maxErr == nil {
			// Validate range limits are within servo resolution
			if minLimit < maxLimit && maxLimit <= 4095 {
				calibrations[servoID] = &MotorCalibration{
					ID:           servoID,
					DriveMode:    0,
					HomingOffset: homingOffset,
					RangeMin:     int(minLimit),
					RangeMax:     int(maxLimit),
					NormMode:     getNormModeForServo(servoID),
				}
				successCount++
				if logger != nil {
					logger.Debugf("Successfully read calibration from servo %d: offset=%d, range=%d-%d",
						servoID, homingOffset, minLimit, maxLimit)
				}
				continue
			} else {
				if logger != nil {
					logger.Warnf("Servo %d: invalid range values (min=%d, max=%d), using defaults",
						servoID, minLimit, maxLimit)
				}
			}
		} else {
			if logger != nil {
				logger.Warnf("Servo %d: failed to read registers, using defaults (offset_err=%v, min_err=%v, max_err=%v)",
					servoID, offsetErr, minErr, maxErr)
			}
		}
	}

	if logger != nil {
		logger.Debugf("Calibration loaded from servos: %d/%d successful", successCount, len(servoIDs))
	}

	return FromFeetechCalibrationMap(calibrations)
}

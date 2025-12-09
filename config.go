package so_arm

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/hipsterbrown/feetech-servo"
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
	ShoulderPan  *feetech.MotorCalibration `json:"shoulder_pan"`
	ShoulderLift *feetech.MotorCalibration `json:"shoulder_lift"`
	ElbowFlex    *feetech.MotorCalibration `json:"elbow_flex"`
	WristFlex    *feetech.MotorCalibration `json:"wrist_flex"`
	WristRoll    *feetech.MotorCalibration `json:"wrist_roll"`
	Gripper      *feetech.MotorCalibration `json:"gripper"`
}

var DefaultSO101FullCalibration = SO101FullCalibration{
	ShoulderPan: &feetech.MotorCalibration{
		ID: 1, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeDegrees,
	},
	ShoulderLift: &feetech.MotorCalibration{
		ID: 2, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeDegrees,
	},
	ElbowFlex: &feetech.MotorCalibration{
		ID: 3, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeDegrees,
	},
	WristFlex: &feetech.MotorCalibration{
		ID: 4, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeDegrees,
	},
	WristRoll: &feetech.MotorCalibration{
		ID: 5, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeDegrees,
	},
	Gripper: &feetech.MotorCalibration{
		ID: 6, DriveMode: 0, HomingOffset: 0,
		RangeMin: 500, RangeMax: 3500,
		NormMode: feetech.NormModeRange100, // 0-100% for gripper
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
		logger.Infof("Successfully loaded calibration from %s", cfg.CalibrationFile)
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

// ToMotorCalibration converts CalibrationEntry to feetech.MotorCalibration
func (ce *CalibrationEntry) ToMotorCalibration() *feetech.MotorCalibration {
	normMode := ce.NormMode
	if normMode == 0 {
		if ce.ID == 6 {
			normMode = feetech.NormModeRange100
		} else {
			normMode = feetech.NormModeDegrees
		}
	}

	return &feetech.MotorCalibration{
		ID:           ce.ID,
		DriveMode:    ce.DriveMode,
		HomingOffset: ce.HomingOffset,
		RangeMin:     ce.RangeMin,
		RangeMax:     ce.RangeMax,
		NormMode:     normMode,
	}
}

// FromMotorCalibration converts feetech.MotorCalibration to CalibrationEntry
func FromMotorCalibration(mc *feetech.MotorCalibration) *CalibrationEntry {
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

	convertOrDefault := func(entry *CalibrationEntry, defaultCal *feetech.MotorCalibration) *feetech.MotorCalibration {
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
	convertOrNil := func(mc *feetech.MotorCalibration) *CalibrationEntry {
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
		config *feetech.MotorCalibration
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
func (cal SO101FullCalibration) GetMotorCalibrationByID(servoID int) *feetech.MotorCalibration {
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
func (cal SO101FullCalibration) ToFeetechCalibrationMap() map[int]*feetech.MotorCalibration {
	return map[int]*feetech.MotorCalibration{
		1: cal.ShoulderPan,
		2: cal.ShoulderLift,
		3: cal.ElbowFlex,
		4: cal.WristFlex,
		5: cal.WristRoll,
		6: cal.Gripper,
	}
}

// FromFeetechCalibrationMap creates SO101FullCalibration from a feetech calibration map
func FromFeetechCalibrationMap(calibrations map[int]*feetech.MotorCalibration) SO101FullCalibration {
	getOrDefault := func(id int, defaultCal *feetech.MotorCalibration) *feetech.MotorCalibration {
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

func calibrationsEqual(a, b *feetech.MotorCalibration) bool {
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
		return feetech.NormModeRange100 // Gripper uses 0-100%
	}
	return feetech.NormModeDegrees // Arm servos use degrees
}

// readUint16Register reads a 2-byte register from servo and decodes as uint16
func readUint16Register(servo *feetech.Servo, registerName string) (uint16, error) {
	data, err := servo.ReadRegisterByName(registerName)
	if err != nil {
		return 0, err
	}
	if len(data) != 2 {
		return 0, fmt.Errorf("expected 2 bytes for %s, got %d", registerName, len(data))
	}
	// Little-endian decode (LSB first)
	return uint16(data[0]) | (uint16(data[1]) << 8), nil
}

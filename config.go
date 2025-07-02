package so_arm

import (
	"encoding/json"
	"fmt"
	"go.viam.com/rdk/logging"
	"os"
	"path/filepath"
	"time"
)

// SoArm101Config represents the configuration for the SO-101 arm component
type SoArm101Config struct {
	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Servo configuration
	ServoIDs []int `json:"servo_ids,omitempty"`

	// Common configuration
	Timeout time.Duration `json:"timeout,omitempty"`

	// Motion configuration
	SpeedDegsPerSec        float32 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`

	// Calibration configuration
	CalibrationFile string `json:"calibration_file,omitempty"`

	// Logger for debugging (not serialized)
	Logger logging.Logger `json:"-"`
}

// Validate ensures all parts of the config are valid
func (cfg *SoArm101Config) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	if len(cfg.ServoIDs) == 0 {
		// Set default servo IDs if not specified (arm joints only)
		cfg.ServoIDs = []int{1, 2, 3, 4, 5}
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Default baudrate
	}

	// Validate calibration file if provided
	if cfg.CalibrationFile != "" {
		if !filepath.IsAbs(cfg.CalibrationFile) {
			return nil, nil, fmt.Errorf("calibration_file must be an absolute path, got: %s", cfg.CalibrationFile)
		}

		// Check if file exists and is readable
		if _, err := os.Stat(cfg.CalibrationFile); err != nil {
			return nil, []string{fmt.Sprintf("calibration file not accessible: %v (will use defaults)", err)}, nil
		}
	}

	return nil, nil, nil
}

// LoadCalibration loads calibration from file or returns default calibration
func (cfg *SoArm101Config) LoadCalibration(logger logging.Logger) SO101Calibration {
	if cfg.CalibrationFile == "" {
		if logger != nil {
			logger.Debug("No calibration file specified, using default calibration")
		}
		return DefaultSO101Calibration
	}

	calibration, err := LoadCalibrationFromFile(cfg.CalibrationFile, logger)
	if err != nil {
		if logger != nil {
			logger.Warnf("Failed to load calibration from %s: %v, using default calibration", cfg.CalibrationFile, err)
		}
		return DefaultSO101Calibration
	}

	if logger != nil {
		logger.Infof("Successfully loaded calibration from %s", cfg.CalibrationFile)
	}
	return calibration
}

// CalibrationFileFormat represents the JSON structure for calibration files
type CalibrationFileFormat struct {
	ShoulderPan  SO101JointCalibration `json:"shoulder_pan"`
	ShoulderLift SO101JointCalibration `json:"shoulder_lift"`
	ElbowFlex    SO101JointCalibration `json:"elbow_flex"`
	WristFlex    SO101JointCalibration `json:"wrist_flex"`
	WristRoll    SO101JointCalibration `json:"wrist_roll"`
	Gripper      SO101JointCalibration `json:"gripper"`
}

// LoadCalibrationFromFile loads and validates calibration from a JSON file
func LoadCalibrationFromFile(filePath string, logger logging.Logger) (SO101Calibration, error) {
	// Read file
	data, err := os.ReadFile(filePath)
	if err != nil {
		return SO101Calibration{}, fmt.Errorf("failed to read calibration file: %w", err)
	}

	// Parse JSON
	var fileFormat CalibrationFileFormat
	if err := json.Unmarshal(data, &fileFormat); err != nil {
		return SO101Calibration{}, fmt.Errorf("failed to parse calibration JSON: %w", err)
	}

	// Convert to internal format (excluding gripper since arm only has 5 joints)
	calibration := SO101Calibration{
		ShoulderPan:  fileFormat.ShoulderPan,
		ShoulderLift: fileFormat.ShoulderLift,
		ElbowFlex:    fileFormat.ElbowFlex,
		WristFlex:    fileFormat.WristFlex,
		WristRoll:    fileFormat.WristRoll,
	}

	// Validate calibration
	if err := ValidateCalibration(calibration, logger); err != nil {
		return SO101Calibration{}, fmt.Errorf("calibration validation failed: %w", err)
	}

	return calibration, nil
}

// ValidateCalibration validates that calibration values are reasonable
func ValidateCalibration(cal SO101Calibration, logger logging.Logger) error {
	joints := []struct {
		name   string
		config SO101JointCalibration
	}{
		{"shoulder_pan", cal.ShoulderPan},
		{"shoulder_lift", cal.ShoulderLift},
		{"elbow_flex", cal.ElbowFlex},
		{"wrist_flex", cal.WristFlex},
		{"wrist_roll", cal.WristRoll},
	}

	for _, joint := range joints {
		if err := validateJointCalibration(joint.name, joint.config); err != nil {
			return err
		}
	}

	if logger != nil {
		logger.Debug("Calibration validation passed")
	}

	return nil
}

// validateJointCalibration validates a single joint's calibration
func validateJointCalibration(jointName string, cal SO101JointCalibration) error {
	// Validate servo ID range (1-6 for typical setups)
	if cal.ID < 1 || cal.ID > 6 {
		return fmt.Errorf("joint %s: invalid servo ID %d, must be 1-6", jointName, cal.ID)
	}

	// Validate drive mode (typically 0 or 1)
	if cal.DriveMode < 0 || cal.DriveMode > 1 {
		return fmt.Errorf("joint %s: invalid drive_mode %d, must be 0 or 1", jointName, cal.DriveMode)
	}

	// Validate position range
	if cal.RangeMin < 0 || cal.RangeMax > 4095 {
		return fmt.Errorf("joint %s: position range [%d, %d] outside valid servo range [0, 4095]",
			jointName, cal.RangeMin, cal.RangeMax)
	}

	if cal.RangeMin >= cal.RangeMax {
		return fmt.Errorf("joint %s: range_min (%d) must be less than range_max (%d)",
			jointName, cal.RangeMin, cal.RangeMax)
	}

	// Validate range size (should be reasonable, at least 500 steps for meaningful movement)
	rangeSize := cal.RangeMax - cal.RangeMin
	if rangeSize < 500 {
		return fmt.Errorf("joint %s: range size %d is too small (< 500), check range_min/range_max values",
			jointName, rangeSize)
	}

	// Validate homing offset is reasonable (shouldn't be extremely large)
	if cal.HomingOffset < -4095 || cal.HomingOffset > 4095 {
		return fmt.Errorf("joint %s: homing_offset %d is outside reasonable range [-4095, 4095]",
			jointName, cal.HomingOffset)
	}

	return nil
}

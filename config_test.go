package so_arm

import (
	"path/filepath"
	"testing"

	"go.viam.com/rdk/logging"
)

func TestLoadCalibrationFromFile(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("returns fromFile=true when file exists", func(t *testing.T) {
		// Create temp file with calibration
		tmpDir := t.TempDir()
		calibFile := filepath.Join(tmpDir, "test_calibration.json")
		err := SaveFullCalibrationToFile(calibFile, DefaultSO101FullCalibration)
		if err != nil {
			t.Fatalf("Failed to create test calibration file: %v", err)
		}

		cfg := &SoArm101Config{
			CalibrationFile: calibFile,
		}

		cal, fromFile := cfg.LoadCalibration(logger)

		if !fromFile {
			t.Error("Expected fromFile=true when loading from existing file")
		}
		if !cal.Equal(DefaultSO101FullCalibration) {
			t.Error("Expected calibration to match saved values")
		}
	})

	t.Run("returns fromFile=false when no file configured", func(t *testing.T) {
		cfg := &SoArm101Config{}

		cal, fromFile := cfg.LoadCalibration(logger)

		if fromFile {
			t.Error("Expected fromFile=false when no file configured")
		}
		if !cal.Equal(DefaultSO101FullCalibration) {
			t.Error("Expected default calibration")
		}
	})

	t.Run("returns fromFile=false when file doesn't exist", func(t *testing.T) {
		cfg := &SoArm101Config{
			CalibrationFile: "/nonexistent/path/calibration.json",
		}

		cal, fromFile := cfg.LoadCalibration(logger)

		if fromFile {
			t.Error("Expected fromFile=false when file doesn't exist")
		}
		if !cal.Equal(DefaultSO101FullCalibration) {
			t.Error("Expected default calibration")
		}
	})
}

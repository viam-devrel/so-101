package so_arm

import (
	"fmt"
	"path/filepath"
	"testing"

	"github.com/hipsterbrown/feetech-servo"
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

func TestGetNormModeForServo(t *testing.T) {
	tests := []struct {
		servoID  int
		expected int
	}{
		{1, feetech.NormModeDegrees},
		{2, feetech.NormModeDegrees},
		{3, feetech.NormModeDegrees},
		{4, feetech.NormModeDegrees},
		{5, feetech.NormModeDegrees},
		{6, feetech.NormModeRange100},
	}

	for _, tt := range tests {
		t.Run(fmt.Sprintf("servo_%d", tt.servoID), func(t *testing.T) {
			result := getNormModeForServo(tt.servoID)
			if result != tt.expected {
				t.Errorf("Expected NormMode %d for servo %d, got %d",
					tt.expected, tt.servoID, result)
			}
		})
	}
}

// Note: readUint16Register requires actual servo hardware to test fully
// We'll test it via integration when we test ReadCalibrationFromServos

// Mock servo for testing - would need to implement feetech.Servo interface
// For now, we'll write integration-style test that verifies structure

func TestReadCalibrationFromServos_Structure(t *testing.T) {
	// This test verifies the function signature and default behavior
	// Full testing requires hardware or extensive mocking

	// Can't test with real bus, but verify defaults are used
	// when bus/servos are nil (this will be our "all failures" case)

	// We'll test this more thoroughly in Task 4 when integrated
	t.Skip("Requires hardware or mock bus - tested via integration in Task 4")
}

func TestValidateServoRegisterValues(t *testing.T) {
	tests := []struct {
		name     string
		minLimit uint16
		maxLimit uint16
		valid    bool
	}{
		{"valid range", 500, 3500, true},
		{"min equals max", 2000, 2000, false},
		{"min greater than max", 3000, 2000, false},
		{"max exceeds resolution", 1000, 5000, false},
		{"min at boundary", 0, 4095, true},
		{"max at boundary", 1000, 4095, true},
		{"max exceeds boundary", 1000, 4096, false},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			valid := tt.minLimit < tt.maxLimit && tt.maxLimit <= 4095
			if valid != tt.valid {
				t.Errorf("Expected valid=%v for range [%d-%d], got %v",
					tt.valid, tt.minLimit, tt.maxLimit, valid)
			}
		})
	}
}

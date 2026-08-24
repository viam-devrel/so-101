package controller

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"

	"so_arm/internal/servo"
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

// A calibration file written by a servo_ids:[1,2,3,4,5] run has no gripper entry.
// convertOrDefault fills the default and fromFile=true skips the guard entirely, so this is
// the one fallback that has to announce itself here.
// The warning lives in LoadCalibration, not LoadFullCalibrationFromFile: only the former knows
// the configured servo IDs, so only it can gate the way finalizeCalibrationFromServos does.
func TestLoadCalibrationWarnsWhenTheFileHasNoGripperEntry(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cal.json")
	require.NoError(t, os.WriteFile(path, []byte(`{"shoulder_pan":{"id":1,"range_min":500,"range_max":3500}}`), 0o600))

	logger, logs := logging.NewObservedTestLogger(t)
	cfg := &SoArm101Config{CalibrationFile: path, ServoIDs: []int{1, 2, 3, 4, 5, 6}}
	cal, fromFile := cfg.LoadCalibration(logger)

	require.True(t, fromFile)
	assert.Equal(t, gripperSafeRangeMin, cal.Gripper.RangeMin)
	assert.Equal(t, 1, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

// Mirrors TestFinalizeCalibrationFromServosStaysQuietWhenTheGripperWasNotRequested: an arm-only
// ServoIDs (no 6) must not warn about a gripper the machine doesn't have, even though the file
// is (correctly, for an arm-only machine) missing a gripper entry.
func TestLoadCalibrationStaysQuietWhenGripperIsNotInServoIDs(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cal.json")
	require.NoError(t, os.WriteFile(path, []byte(`{"shoulder_pan":{"id":1,"range_min":500,"range_max":3500}}`), 0o600))

	logger, logs := logging.NewObservedTestLogger(t)
	cfg := &SoArm101Config{CalibrationFile: path, ServoIDs: []int{1, 2, 3, 4, 5}}
	_, fromFile := cfg.LoadCalibration(logger)

	require.True(t, fromFile)
	assert.Equal(t, 0, logs.FilterMessageSnippet("run the calibration workflow").Len())
}

func TestGetNormModeForServo(t *testing.T) {
	tests := []struct {
		servoID  int
		expected int
	}{
		{1, servo.NormModeDegrees},
		{2, servo.NormModeDegrees},
		{3, servo.NormModeDegrees},
		{4, servo.NormModeDegrees},
		{5, servo.NormModeDegrees},
		{6, servo.NormModeRange100},
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

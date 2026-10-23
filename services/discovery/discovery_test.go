package discovery

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/logging"
)

func TestFilterCandidatePorts(t *testing.T) {
	tests := []struct {
		name     string
		ports    []string
		expected []string
	}{
		{
			name:     "Linux USB ports",
			ports:    []string{"/dev/ttyUSB0", "/dev/ttyS0", "/dev/ttyACM0", "/dev/null"},
			expected: []string{"/dev/ttyUSB0", "/dev/ttyACM0"},
		},
		{
			name:     "macOS USB ports",
			ports:    []string{"/dev/tty.usbmodem123", "/dev/tty.Bluetooth", "/dev/tty.usbserial-AB", "/dev/cu.usbmodem5A7A0594021", "/dev/cu.usbserial-123"},
			expected: []string{"/dev/tty.usbmodem123", "/dev/tty.usbserial-AB", "/dev/cu.usbmodem5A7A0594021", "/dev/cu.usbserial-123"},
		},
		{
			name:     "Windows COM ports",
			ports:    []string{"COM3", "COM10", "LPT1", "PRN"},
			expected: []string{"COM3", "COM10"},
		},
		{
			name:     "Empty list",
			ports:    []string{},
			expected: []string{},
		},
		{
			name:     "No matching ports",
			ports:    []string{"/dev/null", "/dev/zero"},
			expected: []string{},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := filterCandidatePorts(tt.ports)
			assert.Equal(t, tt.expected, result)
		})
	}
}

func TestExtractPortSuffix(t *testing.T) {
	tests := []struct {
		name     string
		portPath string
		expected string
	}{
		{
			name:     "Linux ttyUSB",
			portPath: "/dev/ttyUSB0",
			expected: "ttyUSB0",
		},
		{
			name:     "Linux ttyACM",
			portPath: "/dev/ttyACM1",
			expected: "ttyACM1",
		},
		{
			name:     "macOS usbmodem",
			portPath: "/dev/tty.usbmodem14201",
			expected: "usbmodem14201",
		},
		{
			name:     "macOS usbserial",
			portPath: "/dev/tty.usbserial-AB123",
			expected: "usbserial-AB123",
		},
		{
			name:     "Windows COM port",
			portPath: "COM3",
			expected: "COM3",
		},
		{
			name:     "Windows double digit COM",
			portPath: "COM10",
			expected: "COM10",
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := extractPortSuffix(tt.portPath)
			assert.Equal(t, tt.expected, result)
		})
	}
}

func TestFindCalibrationFile(t *testing.T) {
	// Create temp directory for test files
	tempDir := t.TempDir()

	tests := []struct {
		name       string
		setup      func(string) // Setup function to create test files
		portSuffix string
		expected   string
	}{
		{
			name: "Port-specific file exists",
			setup: func(dir string) {
				os.WriteFile(filepath.Join(dir, "ttyUSB0_calibration.json"), []byte("{}"), 0644)
				os.WriteFile(filepath.Join(dir, "so101_calibration.json"), []byte("{}"), 0644)
			},
			portSuffix: "ttyUSB0",
			expected:   "ttyUSB0_calibration.json",
		},
		{
			name: "Only default file exists",
			setup: func(dir string) {
				os.WriteFile(filepath.Join(dir, "so101_calibration.json"), []byte("{}"), 0644)
			},
			portSuffix: "ttyUSB0",
			expected:   "so101_calibration.json",
		},
		{
			name: "No calibration files",
			setup: func(dir string) {
				// No files created
			},
			portSuffix: "ttyUSB0",
			expected:   "",
		},
	}

	logger := logging.NewTestLogger(t)

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			testDir := filepath.Join(tempDir, tt.name)
			os.MkdirAll(testDir, 0755)
			tt.setup(testDir)

			result := findCalibrationFile(testDir, tt.portSuffix, logger)
			assert.Equal(t, tt.expected, result)
		})
	}
}

func TestEnumerateSerialPorts(t *testing.T) {
	// This is a system-dependent test - just verify it doesn't panic and returns a slice
	ports := enumerateSerialPorts()
	assert.NotNil(t, ports)
	// Ports list can be empty on systems without serial devices
	t.Logf("Found %d serial ports", len(ports))
}

func TestStablePort(t *testing.T) {
	dir := t.TempDir()
	byID := filepath.Join(dir, "by-id")
	require.NoError(t, os.Mkdir(byID, 0o755))
	acm1 := filepath.Join(dir, "ttyACM1")
	acm2 := filepath.Join(dir, "ttyACM2")
	require.NoError(t, os.WriteFile(acm1, nil, 0o600))
	require.NoError(t, os.WriteFile(acm2, nil, 0o600))
	link := filepath.Join(byID, "usb-1a86_USB_Single_Serial_5C4C126186-if00")
	require.NoError(t, os.Symlink(acm1, link))

	path, suffix := stablePort(byID, acm1)
	assert.Equal(t, link, path)
	assert.Equal(t, "5C4C126186", suffix)

	path, suffix = stablePort(byID, acm2)
	assert.Equal(t, acm2, path, "no by-id link: keep the tty")
	assert.Equal(t, "ttyACM2", suffix)

	path, suffix = stablePort(filepath.Join(dir, "missing"), acm1)
	assert.Equal(t, acm1, path, "no by-id dir (macOS, Windows): keep the tty")
	assert.Equal(t, "ttyACM1", suffix)
}

func TestGenerateConfigsGivesArmAndSensorTheSameCalibrationFile(t *testing.T) {
	dis := &so101Discovery{logger: logging.NewTestLogger(t)}
	cfgs := dis.generateConfigs("/dev/serial/by-id/x", "5C4C126186", true, true, "5C4C126186_calibration.json")
	require.Len(t, cfgs, 3)
	for _, c := range cfgs {
		if c.Name == "so101-gripper-5C4C126186" {
			continue
		}
		assert.Equal(t, "5C4C126186_calibration.json", c.Attributes["calibration_file"], c.Name)
		assert.Equal(t, "/dev/serial/by-id/x", c.Attributes["port"], c.Name)
	}
}

// discovery.go
package so_arm

import (
	"os"
	"path/filepath"
	"strings"

	"go.viam.com/rdk/logging"
)

// filterCandidatePorts filters serial ports by platform-specific naming patterns
func filterCandidatePorts(ports []string) []string {
	candidates := []string{}
	for _, port := range ports {
		if isCandidatePort(port) {
			candidates = append(candidates, port)
		}
	}
	return candidates
}

// isCandidatePort checks if a port matches SO-101 serial port patterns
func isCandidatePort(port string) bool {
	// Linux: /dev/ttyUSB*, /dev/ttyACM*
	if strings.HasPrefix(port, "/dev/ttyUSB") || strings.HasPrefix(port, "/dev/ttyACM") {
		return true
	}
	// macOS: /dev/tty.usbmodem*, /dev/tty.usbserial*
	if strings.HasPrefix(port, "/dev/tty.usbmodem") || strings.HasPrefix(port, "/dev/tty.usbserial") {
		return true
	}
	// Windows: COM*
	if strings.HasPrefix(port, "COM") {
		return true
	}
	return false
}

// extractPortSuffix extracts a friendly suffix from port path for naming
// /dev/ttyUSB0 -> "ttyUSB0"
// COM3 -> "COM3"
// /dev/tty.usbmodem123 -> "usbmodem123"
func extractPortSuffix(portPath string) string {
	base := filepath.Base(portPath)

	// For macOS /dev/tty.usb* ports, strip the "tty." prefix
	if strings.HasPrefix(base, "tty.usb") {
		return strings.TrimPrefix(base, "tty.")
	}

	return base
}

// findCalibrationFile searches for calibration files in moduleDataDir
// Tries port-specific file first, then falls back to default
// Returns just the filename (not full path) or empty string if not found
func findCalibrationFile(moduleDataDir, portSuffix string, logger logging.Logger) string {
	// Try port-specific file first: ttyUSB0_calibration.json
	portSpecific := filepath.Join(moduleDataDir, portSuffix+"_calibration.json")
	if _, err := os.Stat(portSpecific); err == nil {
		logger.Debugf("Found port-specific calibration file: %s", filepath.Base(portSpecific))
		return filepath.Base(portSpecific)
	}

	// Try default file: so101_calibration.json
	defaultFile := filepath.Join(moduleDataDir, "so101_calibration.json")
	if _, err := os.Stat(defaultFile); err == nil {
		logger.Debugf("Found default calibration file: so101_calibration.json")
		return "so101_calibration.json"
	}

	logger.Debug("No calibration file found")
	return ""
}

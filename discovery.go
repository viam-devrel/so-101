// discovery.go
package so_arm

import (
	"path/filepath"
	"strings"
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

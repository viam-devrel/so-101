// discovery.go
package so_arm

import (
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

// discovery_test.go
package so_arm

import (
	"testing"

	"github.com/stretchr/testify/assert"
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
			ports:    []string{"/dev/tty.usbmodem123", "/dev/tty.Bluetooth", "/dev/tty.usbserial-AB"},
			expected: []string{"/dev/tty.usbmodem123", "/dev/tty.usbserial-AB"},
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

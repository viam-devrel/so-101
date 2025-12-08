// discovery.go
package so_arm

import (
	"context"
	"os"
	"path/filepath"
	"strings"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/discovery"
)

var SO101DiscoveryModel = resource.NewModel("devrel", "so101", "discovery")

func init() {
	resource.RegisterService(
		discovery.API,
		SO101DiscoveryModel,
		resource.Registration[discovery.Service, *SO101DiscoveryConfig]{
			Constructor: newSO101Discovery,
		})
}

// SO101DiscoveryConfig is the configuration for the discovery service
type SO101DiscoveryConfig struct {
	// Empty for now - could add port filters or baudrate options later
}

// Validate ensures the config is valid
func (cfg *SO101DiscoveryConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

// so101Discovery implements the discovery service
type so101Discovery struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable
	logger logging.Logger
}

// newSO101Discovery creates a new SO-101 discovery service
func newSO101Discovery(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (discovery.Service, error) {
	_, err := resource.NativeConfig[*SO101DiscoveryConfig](conf)
	if err != nil {
		return nil, err
	}

	return &so101Discovery{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}, nil
}

// DiscoverResources scans for SO-101 arms on serial ports and returns component configurations
func (dis *so101Discovery) DiscoverResources(ctx context.Context, extra map[string]any) ([]resource.Config, error) {
	// TODO: Implementation will be added in later tasks
	return nil, nil
}

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

// discovery.go
package so_arm

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
	"go.bug.st/serial/enumerator"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
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
	dis.logger.Info("Starting SO-101 discovery")

	// Phase 1: Enumerate all serial ports
	allPorts := enumerateSerialPorts()
	dis.logger.Debugf("Found %d total serial ports", len(allPorts))

	// Phase 2: Filter to candidate ports
	candidates := filterCandidatePorts(allPorts)
	dis.logger.Debugf("Filtered to %d candidate ports", len(candidates))

	// Phase 3: Validate each port and generate configs
	var allConfigs []resource.Config
	for _, portPath := range candidates {
		// Check context cancellation
		select {
		case <-ctx.Done():
			dis.logger.Info("Discovery cancelled")
			return allConfigs, ctx.Err()
		default:
		}

		portConfigs := dis.discoverPort(ctx, portPath)
		allConfigs = append(allConfigs, portConfigs...)
	}

	if len(allConfigs) == 0 {
		dis.logger.Info("No SO-101 arms discovered")
	} else {
		dis.logger.Infof("Discovered %d component configurations", len(allConfigs))
	}

	return allConfigs, nil
}

// discoverPort validates a single port and generates component configurations
func (dis *so101Discovery) discoverPort(ctx context.Context, portPath string) []resource.Config {
	portSuffix := extractPortSuffix(portPath)
	dis.logger.Debugf("Checking port %s", portPath)

	// Try to open port and ping servos
	hasArm, hasGripper := dis.pingServos(portPath)

	if !hasArm && !hasGripper {
		dis.logger.Debugf("No SO-101 servos detected on %s", portPath)
		return nil
	}

	dis.logger.Infof("Discovered SO-101 on %s (arm: %v, gripper: %v)", portPath, hasArm, hasGripper)

	// Find calibration file
	moduleDataDir := os.Getenv("VIAM_MODULE_DATA")
	if moduleDataDir == "" {
		moduleDataDir = "/tmp"
	}
	calibrationFile := findCalibrationFile(moduleDataDir, portSuffix, dis.logger)

	// Generate component configs
	return dis.generateConfigs(portPath, portSuffix, hasArm, hasGripper, calibrationFile)
}

// pingServos attempts to ping servo 1 and servo 6 on the given port
// Returns (hasArm, hasGripper)
func (dis *so101Discovery) pingServos(portPath string) (bool, bool) {
	ctx := context.Background()

	busConfig := feetech.BusConfig{
		Port:     portPath,
		BaudRate: 1000000,
		Protocol: feetech.ProtocolSTS,
		Timeout:  500 * time.Millisecond,
	}

	bus, err := feetech.NewBus(busConfig)
	if err != nil {
		dis.logger.Debugf("Failed to open port %s: %v", portPath, err)
		return false, false
	}
	defer bus.Close()

	// Ping servo 1 (arm)
	servo1 := feetech.NewServo(bus, 1, &feetech.ModelSTS3215)
	hasArm := false
	if _, err := servo1.Ping(ctx); err == nil {
		hasArm = true
	}

	// Ping servo 6 (gripper)
	servo6 := feetech.NewServo(bus, 6, &feetech.ModelSTS3215)
	hasGripper := false
	if _, err := servo6.Ping(ctx); err == nil {
		hasGripper = true
	}

	return hasArm, hasGripper
}

// generateConfigs creates component configurations based on discovered servos
func (dis *so101Discovery) generateConfigs(
	portPath, portSuffix string,
	hasArm, hasGripper bool,
	calibrationFile string,
) []resource.Config {
	var configs []resource.Config

	// Generate arm config if servo 1 responded
	if hasArm {
		attrs := map[string]interface{}{
			"port": portPath,
		}
		if calibrationFile != "" {
			attrs["calibration_file"] = calibrationFile
		}

		configs = append(configs, resource.Config{
			Name:       "so101-arm-" + portSuffix,
			API:        arm.API,
			Model:      SO101Model,
			Attributes: attrs,
		})
	}

	// Generate gripper config if servo 6 responded
	if hasGripper {
		attrs := map[string]interface{}{
			"port": portPath,
		}
		if calibrationFile != "" {
			attrs["calibration_file"] = calibrationFile
		}

		configs = append(configs, resource.Config{
			Name:       "so101-gripper-" + portSuffix,
			API:        gripper.API,
			Model:      SO101GripperModel,
			Attributes: attrs,
		})
	}

	// Always generate calibration sensor if either servo responded
	if hasArm || hasGripper {
		configs = append(configs, resource.Config{
			Name:  "so101-calibration-" + portSuffix,
			API:   sensor.API,
			Model: SO101CalibrationSensorModel,
			Attributes: map[string]interface{}{
				"port": portPath,
			},
		})
	}

	return configs
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
	// macOS: /dev/tty.usbmodem*, /dev/tty.usbserial*, /dev/cu.usbmodem*, /dev/cu.usbserial*
	if strings.HasPrefix(port, "/dev/tty.usbmodem") || strings.HasPrefix(port, "/dev/tty.usbserial") || strings.HasPrefix(port, "/dev/cu.usbmodem") || strings.HasPrefix(port, "/dev/cu.usbserial") {
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
	if strings.HasPrefix(base, "cu.usb") {
		return strings.TrimPrefix(base, "cu.")
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

// enumerateSerialPorts returns a list of all serial ports on the system
func enumerateSerialPorts() []string {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return []string{}
	}

	var portPaths []string
	for _, port := range ports {
		portPaths = append(portPaths, port.Name)
	}
	return portPaths
}

package so_arm

import (
	"fmt"
	"time"
	"go.viam.com/rdk/logging"
)

// SoArm101Config represents the configuration for the SO-ARM controller
type SoArm101Config struct {
	// Serial configuration
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Servo configuration
	ServoIDs []int `json:"servo_ids,omitempty"`

	// Common configuration
	Timeout time.Duration `json:"timeout,omitempty"`

	// Logger for debugging (not serialized)
	Logger logging.Logger `json:"-"`
}

// Validate ensures all parts of the config are valid
func (cfg *SoArm101Config) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}
	
	if len(cfg.ServoIDs) == 0 {
		// Set default servo IDs if not specified
		cfg.ServoIDs = []int{1, 2, 3, 4, 5, 6}
	}
	
	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000 // Default baudrate
	}
	
	return nil, nil, nil
}

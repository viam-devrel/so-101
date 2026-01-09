package so_arm

import (
	"context"
	"encoding/binary"
)

// configureServosOptimal configures all servos with optimal settings
// Similar to LeRobot's configure() and configure_motors() methods
func configureServosOptimal(ctx context.Context, controller *SafeSoArmController, logger interface{ Debugf(string, ...interface{}) }) error {
	// Helper functions to encode values to little-endian bytes
	encodeU8 := func(value uint8) []byte {
		return []byte{value}
	}
	encodeU16 := func(value uint16) []byte {
		buf := make([]byte, 2)
		binary.LittleEndian.PutUint16(buf, value)
		return buf
	}

	if logger != nil {
		logger.Debugf("Configuring servos with optimal settings (LeRobot-style)")
	}

	// Get protocol version to check for maximum_acceleration support
	proto := controller.bus.Protocol()
	protocolVersion := proto.Version()

	// Configure each servo (1-6)
	for servoID := 1; servoID <= 6; servoID++ {
		// 1. Reduce return delay time from 500µs (default 250) to minimum 2µs (value 0)
		// This speeds up communication with servos
		if err := controller.WriteServoRegister(ctx, servoID, "response_delay", encodeU8(0)); err != nil {
			if logger != nil {
				logger.Debugf("Failed to set response_delay for servo %d: %v", servoID, err)
			}
		}

		// 2. Set maximum acceleration to 254 (only for protocol version 0 - STS)
		if protocolVersion == 0 {
			if err := controller.WriteServoRegister(ctx, servoID, "max_acceleration", encodeU8(254)); err != nil {
				if logger != nil {
					logger.Debugf("Failed to set max_acceleration for servo %d: %v", servoID, err)
				}
			}
		}

		// 3. Set default acceleration to 254
		if err := controller.WriteServoRegister(ctx, servoID, "acceleration", encodeU8(254)); err != nil {
			if logger != nil {
				logger.Debugf("Failed to set acceleration for servo %d: %v", servoID, err)
			}
		}

		// 4. Set PID coefficients to reduce shakiness
		// p_gain: lower from default 32 to 16 to reduce shakiness
		if err := controller.WriteServoRegister(ctx, servoID, "p_gain", encodeU8(16)); err != nil {
			if logger != nil {
				logger.Debugf("Failed to set p_gain for servo %d: %v", servoID, err)
			}
		}

		// i_gain: set to 0 (default)
		if err := controller.WriteServoRegister(ctx, servoID, "i_gain", encodeU8(0)); err != nil {
			if logger != nil {
				logger.Debugf("Failed to set i_gain for servo %d: %v", servoID, err)
			}
		}

		// d_gain: set to 32 (default)
		if err := controller.WriteServoRegister(ctx, servoID, "d_gain", encodeU8(32)); err != nil {
			if logger != nil {
				logger.Debugf("Failed to set d_gain for servo %d: %v", servoID, err)
			}
		}

		// 5. Special configuration for gripper (servo 6)
		if servoID == 6 {
			// Set max torque limit to 50% to avoid burnout
			if err := controller.WriteServoRegister(ctx, servoID, "max_torque", encodeU16(500)); err != nil {
				if logger != nil {
					logger.Debugf("Failed to set max_torque for gripper: %v", err)
				}
			}

			// Set protection current to 50% of max to avoid burnout
			if err := controller.WriteServoRegister(ctx, servoID, "protection_current", encodeU16(250)); err != nil {
				if logger != nil {
					logger.Debugf("Failed to set protection_current for gripper: %v", err)
				}
			}

			// Set overload torque to 25% when overloaded
			if err := controller.WriteServoRegister(ctx, servoID, "overload_torque", encodeU8(25)); err != nil {
				if logger != nil {
					logger.Debugf("Failed to set overload_torque for gripper: %v", err)
				}
			}
		}
	}

	if logger != nil {
		logger.Debugf("Servo configuration complete (protocol version: %d)", protocolVersion)
	}

	return nil
}

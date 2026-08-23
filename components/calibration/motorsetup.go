package calibration

import (
	"context"
	"fmt"

	feetech "github.com/hipsterbrown/feetech-servo/feetech"
)

// MotorSetupConfig represents the target configuration for SO-101 motors
type MotorSetupConfig struct {
	Name     string `json:"name"`
	TargetID int    `json:"target_id"`
	Model    string `json:"model"`
}

// SO101MotorConfigs defines the standard SO-101 motor configuration
// Processed in reverse order to avoid ID conflicts during assignment
var SO101MotorConfigs = []MotorSetupConfig{
	{"gripper", 6, "sts3215"},
	{"wrist_roll", 5, "sts3215"},
	{"wrist_flex", 4, "sts3215"},
	{"elbow_flex", 3, "sts3215"},
	{"shoulder_lift", 2, "sts3215"},
	{"shoulder_pan", 1, "sts3215"},
}

// motorSetupDiscover discovers a single motor connected to the bus
// Parameters: motor_name (string) - name of motor to discover
func (cs *so101CalibrationSensor) motorSetupDiscover(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	motorName, ok := cmd["motor_name"].(string)
	if !ok {
		return nil, fmt.Errorf("motor_name parameter required")
	}

	// Find motor config
	var motorConfig *MotorSetupConfig
	for _, config := range SO101MotorConfigs {
		if config.Name == motorName {
			motorConfig = &config
			break
		}
	}
	if motorConfig == nil {
		return nil, fmt.Errorf("unknown motor name: %s", motorName)
	}

	cs.setupStatus = fmt.Sprintf("Discovering %s motor...", motorName)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Try to discover the servo using controller's bus
	discoveredServo, foundBaudrate, err := cs.discoverOneMotor(ctx, motorConfig.Model)
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Failed to discover %s: %v", motorName, err)
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	modelName := "unknown"
	if discoveredServo.Model != nil {
		modelName = discoveredServo.Model.Name
	}
	cs.setupStatus = fmt.Sprintf("Found %s: ID %d, Model %s, Baudrate %d",
		motorName, discoveredServo.ID, modelName, foundBaudrate)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	discModelName := "unknown"
	if discoveredServo.Model != nil {
		discModelName = discoveredServo.Model.Name
	}

	return map[string]any{
		"success":        true,
		"motor_name":     motorName,
		"current_id":     discoveredServo.ID,
		"target_id":      motorConfig.TargetID,
		"model":          discModelName,
		"found_baudrate": foundBaudrate,
		"status":         cs.setupStatus,
	}, nil
}

// motorSetupAssignID assigns the target ID to a discovered motor
// Parameters: motor_name (string), current_id (int), target_id (int), current_baudrate (int)
func (cs *so101CalibrationSensor) motorSetupAssignID(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	motorName, ok := cmd["motor_name"].(string)
	if !ok {
		return nil, fmt.Errorf("motor_name parameter required")
	}

	currentID, ok := cmd["current_id"].(float64) // JSON numbers are float64
	if !ok {
		return nil, fmt.Errorf("current_id parameter required")
	}

	targetID, ok := cmd["target_id"].(float64)
	if !ok {
		return nil, fmt.Errorf("target_id parameter required")
	}

	currentBaudrate, ok := cmd["current_baudrate"].(float64)
	if !ok {
		return nil, fmt.Errorf("current_baudrate parameter required")
	}

	cs.setupInProgress = true
	cs.setupStatus = fmt.Sprintf("Configuring %s motor...", motorName)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Create a temporary connection at the current baudrate for configuration
	err := cs.assignMotorIDAndBaudrate(int(currentID), int(targetID), int(currentBaudrate), 1000000)
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Failed to configure %s: %v", motorName, err)
		cs.setupInProgress = false
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	cs.setupStatus = fmt.Sprintf("Successfully configured %s (ID: %d)", motorName, int(targetID))
	cs.setupInProgress = false
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success":      true,
		"motor_name":   motorName,
		"old_id":       int(currentID),
		"new_id":       int(targetID),
		"new_baudrate": 1000000,
		"status":       cs.setupStatus,
	}, nil
}

// motorSetupVerify verifies that all SO-101 motors are properly configured
func (cs *so101CalibrationSensor) motorSetupVerify(ctx context.Context) (map[string]any, error) {
	cs.setupStatus = "Verifying SO-101 motor configuration..."
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Expected motor configuration
	expectedMotors := map[int]string{
		1: "shoulder_pan",
		2: "shoulder_lift",
		3: "elbow_flex",
		4: "wrist_flex",
		5: "wrist_roll",
		6: "gripper",
	}

	results := make(map[string]any)
	allGood := true

	// Check each expected motor
	for id, name := range expectedMotors {
		if cs.controller.ServoPresent(id) {
			// Try to ping the servo
			_, err := cs.controller.PingServo(ctx, id)
			if err != nil {
				results[name] = map[string]any{
					"id":     id,
					"status": "not_responding",
					"error":  err.Error(),
				}
				allGood = false
			} else {
				// Auto-detect model
				if modelName, err := cs.controller.DetectServoModel(ctx, id); err != nil {
					results[name] = map[string]any{
						"id":     id,
						"status": "model_detection_failed",
						"error":  err.Error(),
					}
				} else {
					results[name] = map[string]any{
						"id":     id,
						"status": "ok",
						"model":  modelName,
					}
				}
			}
		} else {
			results[name] = map[string]any{
				"id":     id,
				"status": "not_found",
				"error":  "servo not in controller",
			}
			allGood = false
		}
	}

	if allGood {
		cs.setupStatus = "✅ All SO-101 motors verified successfully"
	} else {
		cs.setupStatus = "⚠️ Some motors failed verification"
	}

	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success": allGood,
		"motors":  results,
		"status":  cs.setupStatus,
	}, nil
}

// motorSetupScanBus scans the entire bus for connected servos
func (cs *so101CalibrationSensor) motorSetupScanBus(ctx context.Context) (map[string]any, error) {
	cs.setupStatus = "Scanning servo bus for connected motors..."
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	// Discover sweeps every bus ID (1..253) and reliably returns all responders;
	// each absent ID costs one BusConfig.PingTimeout, so a sparse bus takes a few seconds.
	discovered, err := cs.controller.Discover(ctx)
	if err != nil {
		cs.setupStatus = fmt.Sprintf("Bus scan failed: %v", err)
		return map[string]any{"success": false, "error": cs.setupStatus}, err
	}

	// Process results
	foundServos := make([]map[string]any, 0)
	expectedMotors := map[int]string{1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
	unexpectedCount := 0

	for _, servo := range discovered {
		modelName := "unknown"
		if servo.Model != nil {
			modelName = servo.Model.Name
		}

		servoInfo := map[string]any{
			"id":    servo.ID,
			"model": modelName,
		}

		if expectedName, isExpected := expectedMotors[servo.ID]; isExpected {
			servoInfo["expected_name"] = expectedName
			servoInfo["status"] = "expected"
		} else {
			servoInfo["status"] = "unexpected"
			unexpectedCount++
		}

		foundServos = append(foundServos, servoInfo)
	}

	cs.setupStatus = fmt.Sprintf("Found %d servos (%d unexpected)", len(discovered), unexpectedCount)
	cs.logger.Infof("Motor setup: %s", cs.setupStatus)

	return map[string]any{
		"success":          true,
		"servos_found":     len(discovered),
		"unexpected_count": unexpectedCount,
		"servos":           foundServos,
		"status":           cs.setupStatus,
	}, nil
}

// motorSetupResetStatus resets the motor setup status
func (cs *so101CalibrationSensor) motorSetupResetStatus(ctx context.Context) (map[string]any, error) {
	cs.setupInProgress = false
	cs.currentSetupStep = 0
	cs.setupStatus = "Motor setup status reset"

	return map[string]any{
		"success": true,
		"status":  cs.setupStatus,
	}, nil
}

// Helper function to discover a single motor using the bus Discover method
func (cs *so101CalibrationSensor) discoverOneMotor(ctx context.Context, expectedModel string) (*feetech.FoundServo, int, error) {
	// Since we're using a shared controller, we need to work with the existing bus.
	// Discover sweeps all bus IDs, so a lone motor is found whatever ID it carries —
	// and finding more than one responder is a real error, not a collision artifact.
	discovered, err := cs.controller.Discover(ctx)
	if err != nil {
		return nil, 0, fmt.Errorf("discovery failed: %w", err)
	}

	if len(discovered) == 0 {
		return nil, 0, fmt.Errorf("no servos found")
	}

	if len(discovered) > 1 {
		return nil, 0, fmt.Errorf("multiple servos found (%d) - connect only one motor", len(discovered))
	}

	servo := discovered[0]
	if servo.Model == nil || servo.Model.Name != expectedModel {
		actualModel := "unknown"
		if servo.Model != nil {
			actualModel = servo.Model.Name
		}
		return nil, 0, fmt.Errorf("model mismatch: expected %s, found %s", expectedModel, actualModel)
	}

	return &servo, cs.cfg.Baudrate, nil // Return current baudrate as "found" baudrate
}

// Helper function to assign motor ID and baudrate
func (cs *so101CalibrationSensor) assignMotorIDAndBaudrate(currentID, targetID, currentBaudrate, targetBaudrate int) error {
	// Get the servo instance
	if !cs.controller.ServoPresent(currentID) {
		return fmt.Errorf("servo with ID %d not found in controller", currentID)
	}

	// Create context for operations
	ctx := context.Background()

	// Ping to verify communication
	if _, err := cs.controller.PingServo(ctx, currentID); err != nil {
		return fmt.Errorf("failed to ping servo: %w", err)
	}

	// Set target ID if different from current
	if currentID != targetID {
		if err := cs.controller.SetServoID(ctx, currentID, targetID); err != nil {
			return fmt.Errorf("failed to set servo ID: %w", err)
		}
		cs.logger.Infof("Updated servo ID from %d to %d", currentID, targetID)
	}

	// Set target baudrate if different from current
	if currentBaudrate != targetBaudrate {
		if err := cs.controller.SetServoBaudRate(ctx, currentID, targetBaudrate); err != nil {
			return fmt.Errorf("failed to set baudrate: %w", err)
		}
		cs.logger.Infof("Updated servo baudrate to %d", targetBaudrate)
	}

	return nil
}

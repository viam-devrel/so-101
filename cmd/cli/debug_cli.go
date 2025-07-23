package main

import (
	"context"
	"time"

	soarm "so_arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

func main() {
	ctx := context.Background()
	logger := logging.NewLogger("soarm-debug")
	deps := resource.Dependencies{}

	// Configuration for SO-101 Leader arm
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem5A4B0464471",
		Baudrate:            1000000, // Use same baudrate as working simple test
		Timeout:             5 * time.Second,
		DefaultSpeed:        500,
		DefaultAcceleration: 25,
		ServoIDs:            []int{1, 2, 3, 4, 5},
		Mode:                "leader",
		ScaleFactor:         1.0,
		SyncRate:            20,
	}

	// Create SO-101 Leader arm
	leaderArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-leader"), cfg, logger)
	if err != nil {
		panic(err)
	}
	defer leaderArm.Close(ctx)

	logger.Info("SO-101 Leader arm initialized successfully")

	// Test individual servo commands using DoCommand
	logger.Info("Testing individual servo commands...")

	// Test 1: Try to ping servos
	result, err := leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "ping_servos",
	})
	if err != nil {
		logger.Errorf("Ping servos failed: %v", err)
	} else {
		logger.Infof("Ping servos result: %+v", result)
	}

	// Test 2: Try to disable and re-enable torque
	logger.Info("Testing torque control...")

	// Disable torque
	result, err = leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "set_torque",
		"enable":  false,
	})
	if err != nil {
		logger.Errorf("Disable torque failed: %v", err)
	} else {
		logger.Infof("Disable torque result: %+v", result)
	}

	time.Sleep(1 * time.Second)

	// Enable torque
	result, err = leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "set_torque",
		"enable":  true,
	})
	if err != nil {
		logger.Errorf("Enable torque failed: %v", err)
	} else {
		logger.Infof("Enable torque result: %+v", result)
	}

	// Test 3: Try very small movements
	logger.Info("Testing small base movement...")

	// Move base just 5 degrees (0.087 radians)
	smallPositions := []referenceframe.Input{
		{Value: 0.087}, // Base: 5 degrees
		{Value: 0.0},   // Shoulder
		{Value: 0.0},   // Elbow
		{Value: 0.0},   // Wrist pitch
		{Value: 0.0},   // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, smallPositions, map[string]interface{}{
		"speed":        100, // Very slow
		"acceleration": 10,  // Very gentle
	})
	if err != nil {
		logger.Errorf("Small movement failed: %v", err)
	} else {
		logger.Info("Small movement command sent")
	}

	logger.Info("Waiting 5 seconds to observe movement...")
	time.Sleep(5 * time.Second)

	// Test 4: Check if servos are receiving commands
	logger.Info("Checking servo status...")

	// Try to check motion parameters (this command exists in our implementation)
	result, err = leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "set_motion_params",
		"speed":   500.0,
	})
	if err != nil {
		logger.Errorf("Set motion params failed: %v", err)
	} else {
		logger.Infof("Motion params set: %+v", result)
	}

	logger.Info("Debug test completed. Check if the arm moved at all during the test.")
}

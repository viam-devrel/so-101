package main

import (
	"context"
	"time"

	soarm "arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("soarm-cli")
	deps := resource.Dependencies{}

	// Configuration for SO-101 Leader arm
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem5A4B0464471", // Adjust to your serial port
		Baudrate:            1000000,                        // Standard SO-ARM baudrate
		Timeout:             5 * time.Second,
		DefaultSpeed:        1000,                 // Mid-range speed
		DefaultAcceleration: 50,                   // Mid-range acceleration
		ServoIDs:            []int{1, 2, 3, 4, 5}, // Default servo IDs
		Mode:                "leader",             // Set as leader
		ScaleFactor:         1.0,
		SyncRate:            20, // 20 Hz
	}

	// Create SO-101 Leader arm
	leaderArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-leader"), cfg, logger)
	if err != nil {
		return err
	}
	defer leaderArm.Close(ctx)

	logger.Info("SO-101 Leader arm initialized successfully")

	// Skip initial position reading if it's problematic
	logger.Info("Skipping initial position check due to servo communication issues")

	// Start with movement tests directly
	logger.Info("Starting movement tests...")

	// Test Movement 1: Move to home position (all joints at 0 radians)
	logger.Info("Test 1: Moving to home position...")
	homePositions := []referenceframe.Input{
		{Value: 0.0}, // Base
		{Value: 0.0}, // Shoulder
		{Value: 0.0}, // Elbow
		{Value: 0.0}, // Wrist pitch
		{Value: 0.0}, // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, homePositions, map[string]interface{}{
		"speed":        500, // Medium speed
		"acceleration": 30,  // Gentle acceleration
	})
	if err != nil {
		logger.Errorf("Failed to move to home position: %v", err)
	} else {
		logger.Info("Successfully moved to home position")
	}

	time.Sleep(3 * time.Second) // Wait for movement to complete

	// Test Movement 2: Move base joint to 45 degrees (π/4 radians)
	logger.Info("Test 2: Moving base joint to 45 degrees...")
	baseRotatePositions := []referenceframe.Input{
		{Value: 0.7854}, // Base: 45 degrees in radians
		{Value: 0.0},    // Shoulder
		{Value: 0.0},    // Elbow
		{Value: 0.0},    // Wrist pitch
		{Value: 0.0},    // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, baseRotatePositions, nil)
	if err != nil {
		logger.Errorf("Failed to rotate base: %v", err)
	} else {
		logger.Info("Successfully rotated base to 45 degrees")
	}

	time.Sleep(3 * time.Second)

	// Test Movement 3: Simple reaching motion
	logger.Info("Test 3: Performing reaching motion...")
	reachPositions := []referenceframe.Input{
		{Value: 0.0},     // Base: straight ahead
		{Value: 0.5236},  // Shoulder: 30 degrees up
		{Value: -0.7854}, // Elbow: -45 degrees (bend inward)
		{Value: 0.2618},  // Wrist pitch: 15 degrees
		{Value: 0.0},     // Wrist roll: no rotation
	}

	err = leaderArm.MoveToJointPositions(ctx, reachPositions, map[string]interface{}{
		"speed":        300, // Slower speed for precision
		"acceleration": 20,  // Gentle acceleration
	})
	if err != nil {
		logger.Errorf("Failed to perform reaching motion: %v", err)
	} else {
		logger.Info("Successfully performed reaching motion")
	}

	time.Sleep(4 * time.Second)

	// Test Movement 4: Move wrist roll
	logger.Info("Test 4: Testing wrist roll...")
	wristRollPositions := []referenceframe.Input{
		{Value: 0.0},     // Base
		{Value: 0.5236},  // Shoulder: keep at 30 degrees
		{Value: -0.7854}, // Elbow: keep at -45 degrees
		{Value: 0.2618},  // Wrist pitch: keep at 15 degrees
		{Value: 1.5708},  // Wrist roll: 90 degrees
	}

	err = leaderArm.MoveToJointPositions(ctx, wristRollPositions, nil)
	if err != nil {
		logger.Errorf("Failed to move wrist roll: %v", err)
	} else {
		logger.Info("Successfully moved wrist roll to 90 degrees")
	}

	time.Sleep(3 * time.Second)

	// Test Movement 5: Return to home position
	logger.Info("Test 5: Returning to home position...")
	err = leaderArm.MoveToJointPositions(ctx, homePositions, map[string]interface{}{
		"speed":        400,
		"acceleration": 25,
	})
	if err != nil {
		logger.Errorf("Failed to return to home: %v", err)
	} else {
		logger.Info("Successfully returned to home position")
	}

	time.Sleep(3 * time.Second)

	// Final position check (with timeout handling)
	logger.Info("Attempting to read final joint positions...")
	ctx_timeout, cancel := context.WithTimeout(ctx, 2*time.Second)
	defer cancel()

	finalPositions, err := leaderArm.JointPositions(ctx_timeout, nil)
	if err != nil {
		logger.Warnf("Could not read final joint positions (this is normal with servo communication issues): %v", err)
	} else {
		logger.Infof("Final joint positions: %+v", finalPositions)
	}

	// Optional: If you want to test with a follower arm as well
	if false { // Set to true when you want to test follower (requires separate controller implementation)
		followerCfg := &soarm.SoArm101Config{
			Port:                "/dev/tty.usbmodem5A4B0465041", // Second SoArm
			Baudrate:            1000000,
			Timeout:             5 * time.Second,
			DefaultSpeed:        1000,
			DefaultAcceleration: 50,
			ServoIDs:            []int{1, 2, 3, 4, 5},
			Mode:                "follower",
			LeaderArm:           "soarm-leader", // Reference to leader arm
			MirrorMode:          false,          // Set to true for mirroring
			ScaleFactor:         1.0,
			SyncRate:            20,
		}

		followerArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-follower"), followerCfg, logger)
		if err != nil {
			logger.Errorf("Failed to create follower arm: %v", err)
		} else {
			defer followerArm.Close(ctx)
			logger.Info("SO-101 Follower arm initialized successfully")
		}
	}

	// Keep the program running for testing
	logger.Info("Movement tests completed! Arms running... Press Ctrl+C to exit")
	time.Sleep(30 * time.Second) // Extended time to observe the arm

	return nil
}

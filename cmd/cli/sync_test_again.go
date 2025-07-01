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
	ctx := context.Background()
	logger := logging.NewLogger("sync-test")
	deps := resource.Dependencies{}

	// Configuration for SO-101 Leader arm (same as working simple test)
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem5A4B0464471",
		Baudrate:            1000000, // Same as working simple test
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

	// Test movement that should match the working simple test
	logger.Info("Testing movement equivalent to working simple test...")

	// The simple test moved servo 1 to position 35000
	// Convert that back to radians: (35000 - 32768) * 360 / 65535 * PI/180
	// This should be approximately 0.12 radians
	testPositions := []referenceframe.Input{
		{Value: 0.12}, // Base: equivalent to position 35000
		{Value: 0.0},  // Shoulder
		{Value: 0.0},  // Elbow
		{Value: 0.0},  // Wrist pitch
		{Value: 0.0},  // Wrist roll
	}

	logger.Info("Moving to test position (should match simple_test movement)...")
	err = leaderArm.MoveToJointPositions(ctx, testPositions, map[string]interface{}{
		"speed":        500,
		"acceleration": 25,
	})
	if err != nil {
		logger.Errorf("Movement failed: %v", err)
	} else {
		logger.Info("Movement command sent successfully")
	}

	logger.Info("Waiting 3 seconds to observe movement...")
	time.Sleep(3 * time.Second)

	// Move back to center
	logger.Info("Moving back to center...")
	centerPositions := []referenceframe.Input{
		{Value: 0.0}, // Base: center
		{Value: 0.0}, // Shoulder
		{Value: 0.0}, // Elbow
		{Value: 0.0}, // Wrist pitch
		{Value: 0.0}, // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, centerPositions, nil)
	if err != nil {
		logger.Errorf("Return movement failed: %v", err)
	} else {
		logger.Info("Return movement command sent")
	}

	logger.Info("Test complete. Did you see movement matching the simple_test?")
	time.Sleep(2 * time.Second)
}

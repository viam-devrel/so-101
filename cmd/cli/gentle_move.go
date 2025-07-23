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
	logger := logging.NewLogger("soarm-gentle")
	deps := resource.Dependencies{}

	// Configuration for SO-101 arm - VERY SLOW settings
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem5A4B0465041",
		Baudrate:            1000000,
		Timeout:             10 * time.Second, // Longer timeout
		DefaultSpeed:        50,               // MUCH slower
		DefaultAcceleration: 5,                // MUCH gentler
		ServoIDs:            []int{1, 2, 3, 4, 5},
		Mode:                "leader",
		ScaleFactor:         1.0,
		SyncRate:            20,
	}

	leaderArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-leader"), cfg, logger)
	if err != nil {
		panic(err)
	}
	defer leaderArm.Close(ctx)

	logger.Info("Gentle Movement Test - ULTRA SLOW MODE")
	logger.Info("=====================================")

	// Step 1: Try to go to zero position VERY slowly
	logger.Info("Step 1: Moving to zero position - VERY SLOWLY...")
	zeroPosition := []referenceframe.Input{
		{Value: 0.0}, // Base
		{Value: 0.0}, // Shoulder
		{Value: 0.0}, // Elbow
		{Value: 0.0}, // Wrist pitch
		{Value: 0.0}, // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, zeroPosition, map[string]interface{}{
		"speed":        30, // Extremely slow
		"acceleration": 3,  // Extremely gentle
	})
	if err != nil {
		logger.Errorf("Movement failed: %v", err)
	} else {
		logger.Info("Movement command sent - should be very slow and smooth")
	}

	logger.Info("Waiting 10 seconds for movement to complete...")
	time.Sleep(10 * time.Second)

	// Step 2: Try a small movement
	logger.Info("Step 2: Small test movement...")
	smallMove := []referenceframe.Input{
		{Value: 0.1745}, // Base: 10 degrees
		{Value: 0.0},    // Shoulder
		{Value: 0.0},    // Elbow
		{Value: 0.0},    // Wrist pitch
		{Value: 0.0},    // Wrist roll
	}

	err = leaderArm.MoveToJointPositions(ctx, smallMove, map[string]interface{}{
		"speed":        20, // Even slower
		"acceleration": 2,  // Even gentler
	})
	if err != nil {
		logger.Errorf("Small movement failed: %v", err)
	} else {
		logger.Info("Small movement sent - should be barely perceptible")
	}

	logger.Info("Waiting 8 seconds...")
	time.Sleep(8 * time.Second)

	// Step 3: Back to zero
	logger.Info("Step 3: Returning to zero...")
	err = leaderArm.MoveToJointPositions(ctx, zeroPosition, map[string]interface{}{
		"speed":        25,
		"acceleration": 3,
	})
	if err != nil {
		logger.Errorf("Return movement failed: %v", err)
	}

	time.Sleep(8 * time.Second)

	logger.Info("Gentle movement test complete!")
	logger.Info("If movements were still too fast or jerky, there may be a calibration issue.")
}

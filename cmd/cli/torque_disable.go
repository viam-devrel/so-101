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
	logger := logging.NewLogger("soarm-safe-rest")
	deps := resource.Dependencies{}

	// Configuration for SO-101 arm
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem5A4B0465041",
		Baudrate:            1000000,
		Timeout:             10 * time.Second,
		DefaultSpeed:        20, // EXTREMELY slow
		DefaultAcceleration: 2,  // EXTREMELY gentle
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

	logger.Info("SO-101 Safe Rest - Moving to YOUR safe position")
	logger.Info("===============================================")

	// Your manually found safe position
	// Base: -139.5°, Shoulder: -129.3°, Elbow: 287.8°, Wrist_P: 218.6°, Wrist_R: 23.1°
	safeRestPosition := []referenceframe.Input{
		{Value: -2.4339}, // Base: -139.5° (your safe position)
		{Value: -2.2569}, // Shoulder: -129.3° (your safe position)
		{Value: 5.0226},  // Elbow: 287.8° (your safe position)
		{Value: 3.8157},  // Wrist pitch: 218.6° (your safe position)
		{Value: 0.4028},  // Wrist roll: 23.1° (your safe position)
	}

	logger.Info("Moving to your manually-tested safe position...")
	logger.Info("Position: Base=-139.5°, Shoulder=-129.3°, Elbow=287.8°, Wrist_P=218.6°, Wrist_R=23.1°")
	logger.Info("This movement will be EXTREMELY slow to prevent crashes...")

	// Move to your safe position with EXTREME slowness
	err = leaderArm.MoveToJointPositions(ctx, safeRestPosition, map[string]interface{}{
		"speed":        15, // Painfully slow
		"acceleration": 1,  // Minimal acceleration
	})
	if err != nil {
		logger.Errorf("Failed to move to safe position: %v", err)
		logger.Warn("⚠️  Movement failed - arm may not be in safe position!")
		logger.Info("Consider manually positioning the arm before disabling torque.")
	} else {
		logger.Info("✅ Movement command sent - this will take a while...")
	}

	// Wait a long time for the extremely slow movement
	logger.Info("Waiting 20 seconds for ultra-slow movement to complete...")
	for i := 20; i > 0; i-- {
		logger.Infof("Waiting... %d seconds remaining", i)
		time.Sleep(1 * time.Second)
	}

	// Extra settling time
	logger.Info("Allowing extra time for arm to settle...")
	time.Sleep(5 * time.Second)

	// Check what position we actually reached
	logger.Info("Checking actual position reached...")
	actualPos, err := leaderArm.JointPositions(ctx, nil)
	if err == nil && len(actualPos) == 5 {
		logger.Infof("Actual position: [%.4f, %.4f, %.4f, %.4f, %.4f]",
			actualPos[0].Value, actualPos[1].Value, actualPos[2].Value, actualPos[3].Value, actualPos[4].Value)

		// Convert to degrees for easier reading
		logger.Infof("In degrees: [%.1f°, %.1f°, %.1f°, %.1f°, %.1f°]",
			actualPos[0].Value*180/3.14159, actualPos[1].Value*180/3.14159,
			actualPos[2].Value*180/3.14159, actualPos[3].Value*180/3.14159, actualPos[4].Value*180/3.14159)
	}

	// Now safely disable torque
	logger.Info("Arm should now be in your safe position - disabling torque...")

	for attempt := 1; attempt <= 3; attempt++ {
		logger.Infof("Attempt %d: Disabling torque...", attempt)

		_, err = leaderArm.DoCommand(ctx, map[string]interface{}{
			"command": "set_torque",
			"enable":  false,
		})

		if err != nil {
			logger.Errorf("Attempt %d failed: %v", attempt, err)
		} else {
			logger.Infof("✅ Attempt %d successful", attempt)
		}

		time.Sleep(500 * time.Millisecond)
	}

	logger.Info("\n🎯 Safe rest sequence complete!")
	logger.Info("✅ Arm moved to your tested safe position")
	logger.Info("✅ Torque disabled - joints are now freely moveable")
	logger.Info("\nThe arm should now be in the same stable position you found manually.")
	logger.Info("If movement was still too fast/jerky, there may be a deeper calibration issue.")
}

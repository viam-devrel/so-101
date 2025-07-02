package main

import (
	"context"
	"fmt"
	"time"

	soarm "so_arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

func main() {
	ctx := context.Background()
	logger := logging.NewLogger("soarm-reader")
	deps := resource.Dependencies{}

	// Configuration for SO-101 arm
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem58CD1767051", // Leader arm port
		Baudrate:            1000000,
		Timeout:             5 * time.Second,
		DefaultSpeed:        100, // Very slow for safety
		DefaultAcceleration: 5,   // Very gentle
		ServoIDs:            []int{1, 2, 3, 4, 5},
		Mode:                "follower",
		ScaleFactor:         1.0,
		SyncRate:            20,
	}

	// Create SO-101 arm
	leaderArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-leader"), cfg, logger)
	if err != nil {
		panic(err)
	}
	defer leaderArm.Close(ctx)

	logger.Info("SO-101 Position Reader Started")
	logger.Info("=====================================")
	logger.Info("")
	logger.Info("INSTRUCTIONS:")
	logger.Info("1. Manually move the arm to a position you like")
	logger.Info("2. The current position will be displayed every 2 seconds")
	logger.Info("3. When you find a good position, copy the values")
	logger.Info("4. Press Ctrl+C to exit")
	logger.Info("")
	logger.Info("Position format: [Base, Shoulder, Elbow, Wrist_Pitch, Wrist_Roll]")
	logger.Info("Values are in radians (-3.14 to +3.14)")
	logger.Info("")

	// First, disable torque so you can move the arm manually
	logger.Info("Disabling torque so you can move the arm by hand...")
	_, err = leaderArm.DoCommand(ctx, map[string]interface{}{
		"command": "set_torque",
		"enable":  false,
	})
	if err != nil {
		logger.Errorf("Failed to disable torque: %v", err)
		logger.Info("You may need to move the arm gently to overcome stiffness")
	} else {
		logger.Info("✅ Torque disabled - you can now position the arm by hand")
	}

	logger.Info("")
	logger.Info("Starting position readings...")
	logger.Info("=====================================")

	// Continuously read and display positions
	for {
		// Try to read current position
		positions, err := leaderArm.JointPositions(ctx, nil)
		if err != nil {
			fmt.Printf("⚠️  Position read failed: %v\n", err)
		} else {
			// Convert to a more readable format
			fmt.Printf("Current Position: [")
			for i, pos := range positions {
				if i > 0 {
					fmt.Printf(", ")
				}
				fmt.Printf("%.4f", pos.Value)
			}
			fmt.Printf("]\n")

			// Also show in degrees for easier understanding
			fmt.Printf("In degrees:       [")
			for i, pos := range positions {
				if i > 0 {
					fmt.Printf(", ")
				}
				degrees := pos.Value * 180.0 / 3.14159
				fmt.Printf("%6.1f°", degrees)
			}
			fmt.Printf("]\n")

			// Show joint names for reference
			fmt.Printf("Joint names:      [Base,  Shoulder, Elbow, Wrist_P, Wrist_R]\n")
			fmt.Printf("-----------------------------------------------------------\n")
		}

		// Wait before next reading
		time.Sleep(2 * time.Second)
	}
}

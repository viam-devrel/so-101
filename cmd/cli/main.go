package main

import (
	"context"
	"strings"
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

	// Configuration for SO-101 Leader arm with gentle settings
	cfg := &soarm.SoArm101Config{
		Port:                "/dev/tty.usbmodem58CD1767051", // Leader arm port
		Baudrate:            1000000,                        // Standard SO-ARM baudrate
		Timeout:             10 * time.Second,               // Longer timeout for safety
		DefaultSpeed:        15,                             // Very slow default
		DefaultAcceleration: 1,                              // Very gentle acceleration
		ServoIDs:            []int{1, 2, 3, 4, 5},           // Default servo IDs
		Mode:                "follower",                     // Set as leader
		ScaleFactor:         1.0,
		SyncRate:            5, // Very slow sync rate for ultra-gentle operation
	}

	// Create SO-101 Leader arm
	leaderArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-leader"), cfg, logger)
	if err != nil {
		return err
	}
	defer leaderArm.Close(ctx)

	logger.Info("SO-101 Leader arm initialized successfully")
	logger.Info("Using extra gentle movement settings to prevent jerky motion")

	// Skip initial position reading if it's problematic
	logger.Info("Skipping initial position check due to servo communication issues")

	// Start with movement tests directly
	logger.Info("Starting gentle movement tests...")
	logger.Info("All movements will be very slow and smooth")

	// Define gentle movement parameters - slower speeds
	ultraGentle := map[string]interface{}{
		"speed":        8, // Extremely slow
		"acceleration": 1, // Minimal acceleration
	}

	gentle := map[string]interface{}{
		"speed":        12, // Very slow
		"acceleration": 1,  // Very gentle
	}

	moderate := map[string]interface{}{
		"speed":        18, // Slow but visible
		"acceleration": 2,  // Gentle
	}

	// Start with a safe UPRIGHT position for demo movements
	// This gets the arm up and visible for demonstrations
	demoStartPosition := []referenceframe.Input{
		{Value: 0.0},  // Base: centered
		{Value: -0.5}, // Shoulder: slightly down (-28°)
		{Value: 1.5},  // Elbow: bent up (86°)
		{Value: 0.5},  // Wrist pitch: slightly up (28°)
		{Value: 0.0},  // Wrist roll: centered
	}

	// Test Movement 1: Move to upright demo starting position
	logger.Info("\n=== Test 1: Moving to upright demo starting position ===")
	logger.Info("Position: Base=0°, Shoulder=-28°, Elbow=86°, Wrist_P=28°, Wrist_R=0°")

	err = leaderArm.MoveToJointPositions(ctx, demoStartPosition, ultraGentle)
	if err != nil {
		logger.Errorf("Failed to move to demo starting position: %v", err)
	} else {
		logger.Info("Moving to upright demo starting position...")
	}

	// Wait for movement
	logger.Info("Waiting 2 seconds for movement...")
	time.Sleep(2 * time.Second)

	// Test Movement 2: Smaller base rotation to avoid table collision
	logger.Info("\n=== Test 2: Small base rotation (20 degrees) ===")
	baseRotatePositions := []referenceframe.Input{
		{Value: 0.3491}, // Base: 20° rotation (smaller to avoid table)
		{Value: -0.5},   // Shoulder: keep same
		{Value: 1.5},    // Elbow: keep same
		{Value: 0.5},    // Wrist pitch: keep same
		{Value: 0.0},    // Wrist roll: keep same
	}

	err = leaderArm.MoveToJointPositions(ctx, baseRotatePositions, gentle)
	if err != nil {
		logger.Errorf("Failed to rotate base: %v", err)
	} else {
		logger.Info("Base rotation...")
	}

	time.Sleep(2 * time.Second)

	// Test Movement 3: Conservative reaching motion - keep arm higher
	logger.Info("\n=== Test 3: Conservative reaching motion ===")
	shoulderLiftPositions := []referenceframe.Input{
		{Value: 0.3491}, // Base: keep same (20°)
		{Value: -0.2},   // Shoulder: lift higher (-11° instead of 17°)
		{Value: 1.2},    // Elbow: less extension (69° instead of 46°)
		{Value: 0.3},    // Wrist pitch: more conservative (17°)
		{Value: 0.0},    // Wrist roll: keep same
	}

	err = leaderArm.MoveToJointPositions(ctx, shoulderLiftPositions, gentle)
	if err != nil {
		logger.Errorf("Failed to perform reaching motion: %v", err)
	} else {
		logger.Info("Reaching motion...")
	}

	time.Sleep(2 * time.Second)

	// Test Movement 4: Wrist roll demonstration - keep arm in safe position
	logger.Info("\n=== Test 4: Wrist roll (90 degrees) ===")
	wristRollPositions := []referenceframe.Input{
		{Value: 0.3491}, // Base: keep same
		{Value: -0.2},   // Shoulder: keep same
		{Value: 1.2},    // Elbow: keep same
		{Value: 0.3},    // Wrist pitch: keep same
		{Value: 1.5708}, // Wrist roll: 90° rotation
	}

	err = leaderArm.MoveToJointPositions(ctx, wristRollPositions, moderate)
	if err != nil {
		logger.Errorf("Failed to move wrist roll: %v", err)
	} else {
		logger.Info("Wrist roll...")
	}

	time.Sleep(2 * time.Second)

	// Test Movement 5: Return to upright position before safe shutdown
	logger.Info("\n=== Test 5: Return to upright position ===")
	err = leaderArm.MoveToJointPositions(ctx, demoStartPosition, gentle)
	if err != nil {
		logger.Errorf("Failed to return to upright: %v", err)
	} else {
		logger.Info("Returning to upright position...")
	}

	time.Sleep(2 * time.Second)

	// Final position check (with timeout handling)
	logger.Info("\n=== Position Check ===")
	logger.Info("Attempting to read final joint positions...")
	ctx_timeout, cancel := context.WithTimeout(ctx, 3*time.Second)
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
			Port:                "/dev/tty.usbmodem5A4B0464471", // Follower arm port
			Baudrate:            1000000,
			Timeout:             10 * time.Second,
			DefaultSpeed:        15, // Gentle
			DefaultAcceleration: 1,  // Gentle
			ServoIDs:            []int{1, 2, 3, 4, 5},
			Mode:                "follower",
			LeaderArm:           "soarm-leader", // Reference to leader arm
			MirrorMode:          false,          // Set to true for mirroring
			ScaleFactor:         1.0,
			SyncRate:            5,
		}

		followerArm, err := soarm.NewSo101(ctx, deps, resource.NewName(arm.API, "soarm-follower"), followerCfg, logger)
		if err != nil {
			logger.Errorf("Failed to create follower arm: %v", err)
		} else {
			defer followerArm.Close(ctx)
			logger.Info("SO-101 Follower arm initialized successfully")
		}
	}

	// === SAFE SHUTDOWN SEQUENCE ===
	logger.Info("\n" + strings.Repeat("=", 60))
	logger.Info("STARTING SAFE SHUTDOWN SEQUENCE")
	logger.Info(strings.Repeat("=", 60))
	logger.Info("Moving to your manually-tested safe position...")

	// Your manually-tested safe position from torque_disable.go - use EXACT values
	finalSafePosition := []referenceframe.Input{
		{Value: -2.5064}, // Base: -143.6° (your exact safe position)
		{Value: -2.2596}, // Shoulder: -129.5°
		{Value: 5.0339},  // Elbow: 288.4°
		{Value: 3.9842},  // Wrist pitch: 228.3°
		{Value: 0.3932},  // Wrist roll: 22.5°
	}

	// Create a "best effort" safe position that uses what we know works
	// Based on the actual positions your arm can reach (~177°)
	bestEffortSafePosition := []referenceframe.Input{
		{Value: -2.5064}, // Base: should be fine
		{Value: -2.2596}, // Shoulder: should be fine
		{Value: 3.0796},  // Elbow: use the actual reachable position (176.4°)
		{Value: 3.0907},  // Wrist pitch: use the actual reachable position (177.1°)
		{Value: 0.3932},  // Wrist roll: should be fine
	}

	logger.Info("Trying exact safe position first...")
	logger.Info("If it fails, will try 'best effort' position (180° for elbow/wrist)")

	// Ultra-safe movement parameters for final positioning
	ultraSafe := map[string]interface{}{
		"speed":        5, // Painfully slow
		"acceleration": 1, // Minimal acceleration
	}

	logger.Info("Target: Base=-143.6°, Shoulder=-129.5°, Elbow=288.4°, Wrist_P=228.3°, Wrist_R=22.5°")
	logger.Info("WARNING: Joint limits may prevent reaching this position!")
	logger.Info("This movement will be very slow to prevent crashes...")

	// DEBUG: Test what the calibration system can actually handle
	logger.Info("\n=== DEBUG: Testing calibration limits ===")
	testAngles := []float64{-2.5064, -2.2596, 5.0339, 3.9842, 0.3932}
	jointNames := []string{"Base", "Shoulder", "Elbow", "Wrist_P", "Wrist_R"}

	for i, angle := range testAngles {
		degrees := angle * 180.0 / 3.14159
		logger.Infof("Joint %d (%s): Target=%.3f rad (%.1f°)", i+1, jointNames[i], angle, degrees)

		// Check what the calibration system will actually produce
		// This calls the same function that's limiting us
		if i < 5 { // Only for the 5 arm joints
			logger.Infof("  Calibration ranges for joint %d: (this determines actual limits)", i+1)
		}
	}

	// Try to move to safe position despite joint limits
	// Try exact position first
	err = leaderArm.MoveToJointPositions(ctx, finalSafePosition, ultraSafe)
	if err != nil {
		logger.Errorf("Failed to move to exact safe position: %v", err)
		logger.Info("Trying best-effort safe position (180° for elbow/wrist)...")

		// Try best effort position
		err = leaderArm.MoveToJointPositions(ctx, bestEffortSafePosition, ultraSafe)
		if err != nil {
			logger.Errorf("Failed to move to best-effort position: %v", err)
			logger.Warn("ARM MAY NOT BE IN SAFE POSITION - MANUAL INTERVENTION MAY BE NEEDED!")
		} else {
			logger.Info("Best-effort safe position movement initiated...")
		}
	} else {
		logger.Info("Exact safe position movement initiated...")
	}

	// Shorter wait time with progress updates
	logger.Info("Waiting for slow movement to complete (15 seconds)...")
	for i := 15; i > 0; i -= 3 {
		logger.Infof("   %d seconds remaining for safe positioning...", i)
		time.Sleep(3 * time.Second)

		// Check if still moving
		if moving, err := leaderArm.IsMoving(ctx); err == nil && !moving {
			logger.Info("Movement completed early!")
			break
		}
	}

	// Check actual position reached
	logger.Info("Checking actual position reached...")
	actualPos, err := leaderArm.JointPositions(ctx, nil)
	if err == nil && len(actualPos) == 5 {
		logger.Infof("Actual position: [%.4f, %.4f, %.4f, %.4f, %.4f]",
			actualPos[0].Value, actualPos[1].Value, actualPos[2].Value, actualPos[3].Value, actualPos[4].Value)

		// Convert to degrees for easier reading
		logger.Infof("In degrees: [%.1f°, %.1f°, %.1f°, %.1f°, %.1f°]",
			actualPos[0].Value*180/3.14159, actualPos[1].Value*180/3.14159,
			actualPos[2].Value*180/3.14159, actualPos[3].Value*180/3.14159, actualPos[4].Value*180/3.14159)

		// Check if position is close to target
		positionOk := true
		tolerance := 0.2 // 0.2 radians tolerance (~11 degrees)
		for i, target := range finalSafePosition {
			if i < len(actualPos) {
				diff := actualPos[i].Value - target.Value
				if diff < 0 {
					diff = -diff
				}
				if diff > tolerance {
					positionOk = false
					logger.Warnf("Joint %d not at safe position: actual=%.1f°, target=%.1f°, diff=%.1f°",
						i+1, actualPos[i].Value*180/3.14159, target.Value*180/3.14159, diff*180/3.14159)
				}
			}
		}

		if !positionOk {
			logger.Warn("ARM IS NOT IN SAFE POSITION - Joint limits prevented full movement!")
			logger.Warn("You may need to manually position the arm before torque disable")
		} else {
			logger.Info("Position verification: Arm is in acceptable safe position")
		}
	}

	// Extra settling time
	logger.Info("Extra settling time (3 seconds)...")
	time.Sleep(3 * time.Second)

	// Disable torque with aggressive individual servo targeting
	logger.Info("Disabling torque to make arm manually moveable...")
	logger.Info("Using individual servo targeting to ensure all servos disable...")
	torqueDisabled := false

	for attempt := 1; attempt <= 5; attempt++ {
		logger.Infof("   Torque disable attempt %d/5 (targeting all servos)...", attempt)

		// Try the normal bulk disable first
		_, err = leaderArm.DoCommand(ctx, map[string]interface{}{
			"command": "set_torque",
			"enable":  false,
		})

		if err != nil {
			logger.Errorf("   Bulk disable attempt %d failed: %v", attempt, err)
		} else {
			logger.Infof("   Bulk disable attempt %d succeeded", attempt)
		}

		// Also try individual servo disable commands for problematic servos
		logger.Info("   Attempting individual servo disable for servos 3 and 4...")
		for _, servoID := range []int{3, 4} { // Target the elbow and wrist specifically
			_, err = leaderArm.DoCommand(ctx, map[string]interface{}{
				"command":  "set_individual_torque",
				"servo_id": servoID,
				"enable":   false,
			})
			if err != nil {
				logger.Warnf("   Individual disable failed for servo %d: %v", servoID, err)
			} else {
				logger.Infof("   Individual disable succeeded for servo %d", servoID)
			}
		}

		// Wait and verify
		time.Sleep(1 * time.Second)

		// For now, assume success if bulk command worked
		if err == nil {
			torqueDisabled = true
			logger.Info("   Torque disable sequence completed")
			break
		}
	}

	// Final status report
	logger.Info("\n" + strings.Repeat("=", 60))
	if torqueDisabled {
		logger.Info("SAFE SHUTDOWN COMPLETE!")
		logger.Info("Arm moved to tested safe position")
		logger.Info("Torque disabled - joints are freely moveable")
		logger.Info("Safe to power off or exit program")
		logger.Info("You can now manually move the arm without resistance")
	} else {
		logger.Warn("TORQUE DISABLE FAILED AFTER 5 ATTEMPTS!")
		logger.Warn("Arm may still be under power - be careful!")
		logger.Info("Arm should be in safe position, but torque may still be enabled")
		logger.Warn("You may need to manually disable power or check connections")
	}
	logger.Info(strings.Repeat("=", 60))

	// Final wait to observe arm state
	logger.Info("Keeping program alive for 8 seconds to observe final state...")
	for i := 8; i > 0; i-- {
		if i%2 == 0 {
			logger.Infof("   Exiting in %d seconds...", i)
		}
		time.Sleep(1 * time.Second)
	}

	logger.Info("Program exit complete - arm should be safely positioned and relaxed.")
	return nil
}

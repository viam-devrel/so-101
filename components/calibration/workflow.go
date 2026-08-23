package calibration

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	"so_arm/internal/controller"
	"so_arm/internal/servo"
)

// startCalibration begins the calibration workflow
func (cs *so101CalibrationSensor) startCalibration(ctx context.Context) (map[string]any, error) {
	if cs.state != StateIdle && cs.state != StateCompleted && cs.state != StateError {
		return map[string]any{"success": false},
			fmt.Errorf("calibration already in progress (state: %s)", cs.state.String())
	}

	cs.logger.Info("Starting SO-101 calibration workflow")

	// Disable torque to allow manual movement
	if err := cs.controller.SetTorqueEnable(ctx, false); err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to disable torque: %v", err))
		return map[string]any{"success": false}, err
	}

	// Reset joint data
	for _, joint := range cs.joints {
		joint.HomingOffset = 0
		joint.RangeMin = 0
		joint.RangeMax = 4095
		joint.RecordedMin = math.MaxInt32
		joint.RecordedMax = math.MinInt32
		joint.IsCompleted = false
	}

	cs.setState(StateStarted,
		"Calibration started. Manually move the robot to the middle of its range of motion, then use 'set_homing' command.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// Add new method to reset calibration registers to factory defaults
// resetCalibrationRegisters resets servo calibration registers to factory defaults
func (cs *so101CalibrationSensor) resetCalibrationRegisters(ctx context.Context, servoID int) error {
	// Reset homing offset to 0
	homingData := []byte{0x00, 0x00}
	if err := cs.controller.WriteServoRegister(ctx, servoID, "position_offset", homingData); err != nil {
		return fmt.Errorf("failed to reset homing offset: %w", err)
	}

	// Reset min position limit to 0
	minData := []byte{0x00, 0x00}
	if err := cs.controller.WriteServoRegister(ctx, servoID, "min_angle_limit", minData); err != nil {
		return fmt.Errorf("failed to reset min position limit: %w", err)
	}

	// Reset max position limit to 4095 (0x0FFF)
	maxData := []byte{0xFF, 0x0F}
	if err := cs.controller.WriteServoRegister(ctx, servoID, "max_angle_limit", maxData); err != nil {
		return fmt.Errorf("failed to reset max position limit: %w", err)
	}

	return nil
}

// setHomingPosition sets the homing offsets based on current positions
func (cs *so101CalibrationSensor) setHomingPosition(ctx context.Context) (map[string]any, error) {
	if cs.state != StateStarted {
		return map[string]any{"success": false},
			fmt.Errorf("must start calibration first (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Setting homing positions...")

	// First, reset all calibration registers to factory defaults
	cs.logger.Info("Resetting calibration registers to factory defaults...")
	for _, servoID := range cs.cfg.ServoIDs {
		if err := cs.resetCalibrationRegisters(ctx, servoID); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to reset calibration registers for servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}
		cs.logger.Debugf("Reset calibration registers for servo %d", servoID)
	}

	// Brief delay to ensure register writes are complete before reading positions
	time.Sleep(100 * time.Millisecond)

	rawPositions, err := cs.controller.SyncReadPositions(ctx, cs.cfg.ServoIDs)
	if err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to read servo positions: %v", err))
		return map[string]any{"success": false}, err
	}

	// Calculate homing offsets to center the range
	homingOffsets := make(map[string]any)
	for _, servoID := range cs.cfg.ServoIDs {
		currentRawPos := int(rawPositions[servoID])

		// Calculate offset to make current position the center (2047.5 for 12-bit encoder)
		targetCenter := 2047
		homingOffset := currentRawPos - targetCenter

		homingOffsets[strconv.Itoa(servoID)] = homingOffset
		cs.joints[servoID].HomingOffset = homingOffset
		cs.joints[servoID].CurrentPos = currentRawPos

		cs.logger.Infof("Servo %d (%s): raw_position=%d, homing_offset=%d",
			servoID, cs.joints[servoID].Name, currentRawPos, homingOffset)
	}

	// Write homing offsets to servo registers
	cs.logger.Info("Writing homing offsets to servo registers...")
	for _, servoID := range cs.cfg.ServoIDs {
		homingOffset := homingOffsets[strconv.Itoa(servoID)]
		if err := cs.writeHomingOffset(ctx, servoID, homingOffset.(int)); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write homing offset to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}
		cs.logger.Debugf("Successfully wrote homing offset %d to servo %d", homingOffset, servoID)
	}

	cs.setState(StateHomingPosition,
		"Homing positions set. Now use 'start_range_recording' command, then move all joints through their entire ranges of motion.")

	return map[string]any{
		"success":        true,
		"state":          cs.state.String(),
		"homing_offsets": homingOffsets,
		"message":        cs.lastInstruction,
	}, nil
}

// startRangeRecording begins recording min/max positions
func (cs *so101CalibrationSensor) startRangeRecording(_ context.Context) (map[string]any, error) {
	if cs.state != StateHomingPosition {
		return map[string]any{"success": false},
			fmt.Errorf("must set homing position first (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Starting range of motion recording...")

	// A previous goroutine must be gone before starting another: this is the only place
	// that could leave two of them writing cs.joints, and joining here is impossible
	// because the caller holds cs.mu, which the goroutine needs every tick.
	if cs.recordingDone != nil {
		select {
		case <-cs.recordingDone:
		default:
			return map[string]any{"success": false},
				fmt.Errorf("a previous recording is still stopping; retry in a moment")
		}
	}

	// Create a dedicated context for recording that won't be cancelled when DoCommand returns
	cs.recordingCtx, cs.recordingCancel = context.WithCancel(context.Background())
	cs.recordingDone = make(chan struct{})
	cs.recordingActive = true
	cs.recordingStarted = time.Now()
	cs.positionHistory = []map[int]int{}

	cs.setState(StateRangeRecording,
		"Recording range of motion. Move all joints through their full ranges. Use 'stop_range_recording' when complete.")

	// Start background recording goroutine with dedicated context
	go cs.recordPositions(cs.recordingCtx, cs.recordingDone)

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// recordPositions continuously records servo positions in the background
// done is passed in rather than read off the struct: a later startRangeRecording replaces
// the field, and this goroutine must close the channel its own caller is waiting on.
func (cs *so101CalibrationSensor) recordPositions(recordingCtx context.Context, done chan struct{}) {
	defer close(done)

	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()

	cs.logger.Debug("Position recording goroutine started")

	for {
		select {
		case <-recordingCtx.Done():
			cs.logger.Debug("Position recording goroutine stopped - context cancelled")
			return
		case <-ticker.C:
			cs.mu.RLock()
			if !cs.recordingActive || cs.state != StateRangeRecording {
				cs.mu.RUnlock()
				cs.logger.Debug("Position recording goroutine stopped - recording not active")
				return
			}
			cs.mu.RUnlock()

			// Read current positions for all configured servos
			rawPositions, err := cs.controller.SyncReadPositions(recordingCtx, cs.cfg.ServoIDs)
			if err != nil {
				cs.logger.Errorf("Failed to read positions during recording: %v", err)
				continue
			}

			cs.mu.Lock()
			if cs.recordingActive {
				// Update min/max from raw positions
				for servoID, rawPos := range rawPositions {
					joint := cs.joints[servoID]
					joint.CurrentPos = rawPos

					if rawPos < joint.RecordedMin {
						joint.RecordedMin = rawPos
					}
					if rawPos > joint.RecordedMax {
						joint.RecordedMax = rawPos
					}
				}

				cs.positionHistory = append(cs.positionHistory, rawPositions)

				// Limit history to last 1000 samples to prevent memory issues
				if len(cs.positionHistory) > 1000 {
					cs.positionHistory = cs.positionHistory[len(cs.positionHistory)-1000:]
				}
			}
			cs.mu.Unlock()
		}
	}
}

// stopRangeRecording completes the range recording process
func (cs *so101CalibrationSensor) stopRangeRecording(_ context.Context) (map[string]any, error) {
	if cs.state != StateRangeRecording {
		return map[string]any{"success": false},
			fmt.Errorf("range recording not active (current state: %s)", cs.state.String())
	}

	// Stop the recording goroutine. Deliberately no join: the port stays open, so there is
	// no use-after-close to guard, and joining here would need DoCommand's whole lock scope
	// restructured. cs.recordingDone is left in place so a later Close still observes it.
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}

	cs.recordingActive = false
	recordingDuration := time.Since(cs.recordingStarted)

	cs.logger.Infof("Range recording stopped after %.1f seconds, %d samples collected",
		recordingDuration.Seconds(), len(cs.positionHistory))

	// Validate recorded ranges
	rangeData := make(map[string]any)
	allValid := true

	for servoID, joint := range cs.joints {
		if joint.RecordedMin >= joint.RecordedMax {
			cs.logger.Errorf("Invalid range for servo %d (%s): min=%d, max=%d",
				servoID, joint.Name, joint.RecordedMin, joint.RecordedMax)
			allValid = false
			continue
		}

		joint.RangeMin = joint.RecordedMin
		joint.RangeMax = joint.RecordedMax
		joint.IsCompleted = true

		rangeData[joint.Name] = map[string]any{
			"min":   joint.RangeMin,
			"max":   joint.RangeMax,
			"range": joint.RangeMax - joint.RangeMin,
		}

		cs.logger.Infof("Servo %d (%s): range [%d, %d] (span: %d)",
			servoID, joint.Name, joint.RangeMin, joint.RangeMax, joint.RangeMax-joint.RangeMin)
	}

	if !allValid {
		cs.setState(StateError, "Invalid ranges detected. Some joints may not have been moved through their full range.")
		return map[string]any{"success": false}, fmt.Errorf("invalid ranges detected")
	}

	cs.setState(StateCompleted,
		"Range recording completed. Use 'save_calibration' to write calibration to servos and save to file.")

	return map[string]any{
		"success":            true,
		"state":              cs.state.String(),
		"recording_duration": recordingDuration.Seconds(),
		"samples_collected":  len(cs.positionHistory),
		"ranges":             rangeData,
		"message":            cs.lastInstruction,
	}, nil
}

// saveCalibration writes calibration to servos and saves to file
func (cs *so101CalibrationSensor) saveCalibration(ctx context.Context) (map[string]any, error) {
	if cs.state != StateCompleted {
		return map[string]any{"success": false},
			fmt.Errorf("calibration not completed (current state: %s)", cs.state.String())
	}

	cs.logger.Info("Saving calibration to servos and file...")

	// Create calibration structure
	fullCalibration := controller.SO101FullCalibration{}

	for servoID, joint := range cs.joints {
		motorCal := &servo.MotorCalibration{
			ID:           servoID,
			DriveMode:    0, // Normal direction
			HomingOffset: joint.HomingOffset,
			RangeMin:     joint.RangeMin,
			RangeMax:     joint.RangeMax,
			NormMode:     servo.NormModeDegrees, // Default to degrees
		}

		// Special case for gripper - use percentage mode
		if servoID == 6 {
			motorCal.NormMode = servo.NormModeRange100
		}

		// Assign to appropriate field in full calibration
		switch servoID {
		case 1:
			fullCalibration.ShoulderPan = motorCal
		case 2:
			fullCalibration.ShoulderLift = motorCal
		case 3:
			fullCalibration.ElbowFlex = motorCal
		case 4:
			fullCalibration.WristFlex = motorCal
		case 5:
			fullCalibration.WristRoll = motorCal
		case 6:
			fullCalibration.Gripper = motorCal
		}
	}

	// Save calibration to file
	if err := controller.SaveFullCalibrationToFile(cs.cfg.CalibrationFile, fullCalibration); err != nil {
		cs.setState(StateError, fmt.Sprintf("Failed to save calibration file: %v", err))
		return map[string]any{"success": false}, err
	}

	// Apply calibration to servos (write to registers)
	cs.logger.Info("Writing calibration data to servo registers...")
	for servoID, joint := range cs.joints {
		cs.logger.Infof("Writing to servo %d (%s): min_limit=%d, max_limit=%d",
			servoID, joint.Name, joint.RangeMin, joint.RangeMax)

		// Write min position limit
		if err := cs.writeMinPositionLimit(ctx, servoID, joint.RangeMin); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write min position limit to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}

		// Write max position limit
		if err := cs.writeMaxPositionLimit(ctx, servoID, joint.RangeMax); err != nil {
			cs.setState(StateError, fmt.Sprintf("Failed to write max position limit to servo %d: %v", servoID, err))
			return map[string]any{"success": false}, err
		}

		cs.logger.Debugf("Successfully wrote position limits to servo %d", servoID)
	}

	cs.setState(StateIdle, "Calibration completed and saved successfully. Ready for new calibration.")

	return map[string]any{
		"success":           true,
		"state":             cs.state.String(),
		"calibration_file":  cs.cfg.CalibrationFile,
		"joints_calibrated": len(cs.joints),
		"message":           cs.lastInstruction,
	}, nil
}

// abortCalibration cancels the current calibration process
func (cs *so101CalibrationSensor) abortCalibration(_ context.Context) (map[string]any, error) {
	cs.logger.Info("Aborting calibration...")

	// Stop any active recording
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}
	cs.recordingActive = false

	cs.setState(StateIdle, "Calibration aborted. Ready to start new calibration.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// resetCalibration resets the sensor to initial state
func (cs *so101CalibrationSensor) resetCalibration(_ context.Context) (map[string]any, error) {
	cs.logger.Info("Resetting calibration sensor...")

	// Stop any active recording
	if cs.recordingCancel != nil {
		cs.recordingCancel()
		cs.recordingCancel = nil
	}
	cs.recordingActive = false
	cs.errorMsg = ""
	cs.positionHistory = []map[int]int{}

	// Reset all joint data
	for _, joint := range cs.joints {
		joint.HomingOffset = 0
		joint.RangeMin = 0
		joint.RangeMax = 4095
		joint.RecordedMin = math.MaxInt32
		joint.RecordedMax = math.MinInt32
		joint.IsCompleted = false
	}

	cs.setState(StateIdle, "Calibration sensor reset. Ready to start calibration.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

// getCurrentPositions returns current servo positions
func (cs *so101CalibrationSensor) getCurrentPositions(ctx context.Context) (map[string]any, error) {
	positions, err := cs.controller.GetJointPositionsForServos(ctx, cs.cfg.ServoIDs)
	if err != nil {
		return nil, fmt.Errorf("failed to read positions: %w", err)
	}

	positionData := make(map[string]any)
	for i, servoID := range cs.cfg.ServoIDs {
		rawPos := int(positions[i] * 4095 / (2 * math.Pi))

		joint := cs.joints[servoID]
		joint.CurrentPos = rawPos

		positionData[joint.Name] = map[string]any{
			"servo_id":     servoID,
			"raw_position": rawPos,
			"radians":      positions[i],
			"degrees":      positions[i] * 180 / math.Pi,
		}
	}

	return map[string]any{
		"success":   true,
		"positions": positionData,
	}, nil
}

// writeHomingOffset writes the homing offset to a servo's register
func (cs *so101CalibrationSensor) writeHomingOffset(ctx context.Context, servoID, homingOffset int) error {
	return cs.controller.WriteServoRegister(ctx, servoID, "torque_enable", []byte{(byte(128))})
}

// writeMinPositionLimit writes the minimum position limit to a servo's register
func (cs *so101CalibrationSensor) writeMinPositionLimit(ctx context.Context, servoID, minLimit int) error {
	data := []byte{
		byte(minLimit & 0xFF),
		byte((minLimit >> 8) & 0xFF),
	}

	return cs.controller.WriteServoRegister(ctx, servoID, "min_angle_limit", data)
}

// writeMaxPositionLimit writes the maximum position limit to a servo's register
func (cs *so101CalibrationSensor) writeMaxPositionLimit(ctx context.Context, servoID, maxLimit int) error {
	data := []byte{
		byte(maxLimit & 0xFF),
		byte((maxLimit >> 8) & 0xFF),
	}

	return cs.controller.WriteServoRegister(ctx, servoID, "max_angle_limit", data)
}

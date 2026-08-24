package calibration

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	feetech "github.com/hipsterbrown/feetech-servo/feetech"

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
		"Calibration started. Torque is off -- move every joint to the middle of its range of motion, then set the homing position.")

	return map[string]any{
		"success": true,
		"state":   cs.state.String(),
		"message": cs.lastInstruction,
	}, nil
}

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
		// currentRawPos == 4095 (a joint parked at the top of the encoder) yields 2048, which
		// collides with position_offset's sign bit and would decode back as 0. Trim that one
		// tick instead of failing the whole workflow; encodePositionOffset still rejects
		// anything wider, so a genuinely impossible offset is still an error.
		if clamped := clampHomingOffset(homingOffset); clamped != homingOffset {
			cs.logger.Warnf("Servo %d: homing offset %d is not encodable in position_offset; using %d",
				servoID, homingOffset, clamped)
			homingOffset = clamped
		}

		homingOffsets[strconv.Itoa(servoID)] = homingOffset
		cs.joints[servoID].HomingOffset = homingOffset
		cs.joints[servoID].CurrentPos = currentRawPos

		cs.logger.Infof("Servo %d (%s): raw_position=%d, homing_offset=%d",
			servoID, cs.joints[servoID].Name, currentRawPos, homingOffset)
	}

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
		"Homing positions set. Start range recording, then move every joint through its entire range of motion.")

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
		"Recording range of motion. Move every joint through its full range, then stop recording.")

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
		"Range recording complete. Save the calibration to write it to the servos and to file.")

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

	// INVARIANT: a save failure below keeps the state at StateCompleted, not StateError, so
	// save_calibration stays retryable (see sensor.go's Readings). Safe only because this
	// function is idempotent from cs.joints -- it never consults what a prior attempt wrote,
	// so a retry just redoes everything.
	//
	// The file is written BEFORE the servo registers, deliberately: it is the only durable
	// record of the recording session, so persisting it first means a mid-loop bus failure
	// cannot cost the recording if the module restarts before the retry. Nothing requires the
	// two to agree -- the file is the module's normalization source and the registers are only
	// the fallback read when no file exists (controller.LoadCalibration).
	if err := controller.SaveFullCalibrationToFile(cs.cfg.CalibrationFile, fullCalibration); err != nil {
		cs.setState(StateCompleted, fmt.Sprintf(
			"Failed to save calibration file: %v. Recorded ranges are unaffected -- retry save_calibration.", err))
		return map[string]any{"success": false}, err
	}

	cs.logger.Info("Writing calibration data to servo registers...")
	for servoID, joint := range cs.joints {
		cs.logger.Infof("Writing to servo %d (%s): min_limit=%d, max_limit=%d",
			servoID, joint.Name, joint.RangeMin, joint.RangeMax)

		// Write min position limit
		if err := cs.writeMinPositionLimit(ctx, servoID, joint.RangeMin); err != nil {
			cs.setState(StateCompleted, fmt.Sprintf(
				"Failed to write min position limit to servo %d: %v. The calibration file is already saved -- retry save_calibration to finish the servo registers.",
				servoID, err))
			return map[string]any{"success": false}, err
		}

		// Write max position limit
		if err := cs.writeMaxPositionLimit(ctx, servoID, joint.RangeMax); err != nil {
			cs.setState(StateCompleted, fmt.Sprintf(
				"Failed to write max position limit to servo %d: %v. The calibration file is already saved -- retry save_calibration to finish the servo registers.",
				servoID, err))
			return map[string]any{"success": false}, err
		}

		cs.logger.Debugf("Successfully wrote position limits to servo %d", servoID)
	}

	// Re-enable torque only now that every write above has landed, and only after seeding each
	// servo's Goal_Position with where it physically is. Torque enable is a bare SyncWrite to
	// RegTorqueEnable (ServoGroup.EnableAll), so the firmware resumes driving toward whatever
	// goal it still holds -- stale after a hand-moved calibration, and reinterpreted through the
	// position_offset set_homing wrote, so the arm can snap across its whole calibrated travel.
	// If the seed fails, torque stays OFF rather than shipping that jerk.
	//
	// A failure here does NOT revert to StateCompleted: the calibration is already durably
	// saved (file + registers), so there is nothing left to retry -- reverting would resurrect
	// the save-retry deadlock this state machine was fixed to avoid. Instead it is reported via
	// torque_enabled / torque_enable_error so the caller can warn the user the arm is limp.
	torqueEnabled := true
	var torqueErr string
	if err := cs.seedGoalPositions(ctx); err != nil {
		torqueEnabled = false
		torqueErr = err.Error()
		cs.logger.Errorf("Leaving torque disabled: %v", err)
	} else if err := cs.controller.SetTorqueEnable(ctx, true); err != nil {
		torqueEnabled = false
		torqueErr = err.Error()
		cs.logger.Errorf("Failed to re-enable torque after save: %v", err)
	}

	message := "Calibration completed and saved successfully. Torque re-enabled -- ready for new calibration."
	if !torqueEnabled {
		// Deliberately not "retry save_calibration": the state below is StateIdle, whose only
		// available command is "start", so a retry would be rejected. The calibration itself is
		// saved; only holding torque is missing.
		message = fmt.Sprintf(
			"Calibration saved successfully, but torque was left disabled: %s. The arm is limp -- support it before letting go, then power-cycle the servos or restart the machine to restore holding torque.",
			torqueErr)
	}
	cs.setState(StateIdle, message)

	result := map[string]any{
		"success":           true,
		"state":             cs.state.String(),
		"calibration_file":  cs.cfg.CalibrationFile,
		"joints_calibrated": len(cs.joints),
		"torque_enabled":    torqueEnabled,
		"message":           cs.lastInstruction,
	}
	if !torqueEnabled {
		result["torque_enable_error"] = torqueErr
	}
	return result, nil
}

// abortCalibration cancels the current calibration process
// Torque intentionally stays disabled here. It could be re-enabled safely -- seedGoalPositions
// exists for exactly that -- but abort is reached with the servos' registers half-written
// (resetCalibrationRegisters may have wiped the position limits) and with the user's hands on
// the arm, so an abort deliberately performs no motion-capable operation. resetCalibration
// leaves torque alone for the same reason.
func (cs *so101CalibrationSensor) abortCalibration(_ context.Context) (map[string]any, error) {
	// StateCompleted holds a finished recording session waiting on save_calibration --
	// minutes of hand-moving the arm. Aborting there would silently discard it, so reject
	// rather than let an unadvertised path do what the UI already refuses to expose.
	if cs.state == StateCompleted {
		return map[string]any{"success": false},
			fmt.Errorf("cannot abort: a completed recording is waiting to be saved (current state: %s); use save_calibration or reset instead", cs.state.String())
	}

	// StateIdle has nothing to abort. A no-op success (rather than an error) matches the
	// "abort is always safe" spirit, but the message must not claim it did something.
	if cs.state == StateIdle {
		return map[string]any{
			"success": true,
			"state":   cs.state.String(),
			"message": "Already idle; nothing to abort.",
		}, nil
	}

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

	// Stop any active recording. Cancel only -- never join: the caller (DoCommand) holds
	// cs.mu, which recordPositions needs every tick, so waiting here would deadlock. Safe
	// because the port stays open; only Close, which drops the lock first, must join.
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

// writeHomingOffset writes the homing offset to a servo's position_offset register,
// sign-magnitude encoded (see feetech.RegPositionOffset.SignBit). Previously wrote the
// constant 128 to torque_enable, ignoring homingOffset entirely -- see the SO-101
// remediation design spec's PR2 for why that was wrong.
func (cs *so101CalibrationSensor) writeHomingOffset(ctx context.Context, servoID, homingOffset int) error {
	data, err := encodePositionOffset(homingOffset)
	if err != nil {
		return err
	}
	return cs.controller.WriteServoRegister(ctx, servoID, "position_offset", data)
}

// clampHomingOffset trims an offset to the largest magnitude position_offset can represent
// without colliding with its sign bit.
func clampHomingOffset(offset int) int {
	maxMagnitude := (1 << feetech.RegPositionOffset.SignBit) - 1
	if offset > maxMagnitude {
		return maxMagnitude
	}
	if offset < -maxMagnitude {
		return -maxMagnitude
	}
	return offset
}

// encodePositionOffset sign-magnitude-encodes value at feetech.RegPositionOffset's sign bit
// into little-endian protocol bytes, mirroring the feetech package's unexported
// encodeSignMagnitude (used for RegGoalPosition/RegPresentPosition et al).
func encodePositionOffset(value int) ([]byte, error) {
	signBit := feetech.RegPositionOffset.SignBit
	maxMagnitude := (1 << signBit) - 1
	if value < -maxMagnitude || value > maxMagnitude {
		return nil, fmt.Errorf("homing offset %d out of range [%d, %d]", value, -maxMagnitude, maxMagnitude)
	}
	raw := uint16(value)
	if value < 0 {
		raw = uint16(-value) | (1 << uint(signBit))
	}
	return []byte{byte(raw & 0xFF), byte((raw >> 8) & 0xFF)}, nil
}

// seedGoalPositions writes each configured servo's live position into its Goal_Position, so a
// following torque enable holds the arm where it is instead of chasing a stale goal. Read and
// write share the post-set_homing frame (the firmware applies position_offset to both), so
// this is a fixed point.
//
// Scoped to cs.cfg.ServoIDs like every other loop here; SetTorqueEnable is group-wide, so a
// servo left out of servo_ids is still enabled without a fresh goal.
func (cs *so101CalibrationSensor) seedGoalPositions(ctx context.Context) error {
	positions, err := cs.controller.SyncReadPositions(ctx, cs.cfg.ServoIDs)
	if err != nil {
		return fmt.Errorf("failed to read positions before enabling torque: %w", err)
	}
	for _, servoID := range cs.cfg.ServoIDs {
		raw, ok := positions[servoID]
		if !ok {
			return fmt.Errorf("servo %d reported no position before enabling torque", servoID)
		}
		if err := cs.writeGoalPosition(ctx, servoID, raw); err != nil {
			return fmt.Errorf("failed to hold servo %d at its current position: %w", servoID, err)
		}
	}
	return nil
}

// writeGoalPosition writes a raw tick value to a servo's goal_position register. Values
// outside the encoder range are rejected rather than encoded: goal_position is sign-magnitude
// at bit 15, so a garbled read must never reach the firmware as a negative multi-turn target.
func (cs *so101CalibrationSensor) writeGoalPosition(ctx context.Context, servoID, raw int) error {
	if raw < 0 || raw > feetech.ModelSTS3215.MaxPosition {
		return fmt.Errorf("position %d out of range [0, %d]", raw, feetech.ModelSTS3215.MaxPosition)
	}
	data := []byte{
		byte(raw & 0xFF),
		byte((raw >> 8) & 0xFF),
	}
	return cs.controller.WriteServoRegister(ctx, servoID, "goal_position", data)
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

package arm

import (
	"context"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/controller"
	"so_arm/internal/geometry"
	"so_arm/internal/servo"
	"so_arm/internal/servocmd"
)

func (s *so101) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

// Get3DModels serves the SO-101 link meshes for the 3D scene viewer, plus a colored XYZ
// coordinate-frame marker at the end-effector when the visualize_ee_frame attribute is
// enabled.
func (s *so101) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return geometry.ArmMeshes(s.cfg.VisualizeEEFrame), nil
}

// Status returns the current status of the resource as a map of key-value pairs.
func (s *so101) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if s.manual != nil && s.manual.running() {
		// GetStatus serializes this via structpb.NewStruct, which only accepts
		// structpb-native types — a typed []manualJointStatus slice fails. Emit the
		// joints as []interface{} of map[string]interface{} so the wire format holds.
		js := s.manual.statusJoints()
		joints := make([]interface{}, 0, len(js))
		for _, j := range js {
			joints = append(joints, map[string]interface{}{
				"servo":      j.Servo,
				"actual_deg": j.ActualDeg,
				"goal_deg":   j.GoalDeg,
				"dev_deg":    j.DevDeg,
				"load":       j.Load,
			})
		}
		return map[string]interface{}{
			"mode":        "manual",
			"manual_mode": map[string]interface{}{"joints": joints},
		}, nil
	}
	return map[string]interface{}{"mode": "auto"}, nil
}

// regDecodeLE decodes a little-endian unsigned register value from its raw bytes
// (STS3215 registers are little-endian; low byte first).
func regDecodeLE(data []byte) int {
	v := 0
	for i, b := range data {
		v |= int(b) << (8 * i)
	}
	return v
}

// setTorqueLimitDiag is a diagnostic that writes the torque_limit register directly on every
// arm servo, in isolation from manual mode, so you can confirm the register actually governs
// holding torque on this firmware (e.g. value 10 should make the arm droop; 1000 restores full
// torque). Requires a "value" (0-1000). Run with manual mode OFF; it does not auto-restore —
// set it back to 1000 (or reinitialize) when done. Refuses while manual mode is active so it
// can't fight the control loop.
func (s *so101) setTorqueLimitDiag(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.RLock()
	active := s.manual != nil && s.manual.running()
	s.mu.RUnlock()
	if active {
		return nil, fmt.Errorf("exit manual mode before set_torque_limit (it would fight the control loop)")
	}
	v, ok := cmd["value"].(float64)
	if !ok {
		return nil, fmt.Errorf("set_torque_limit requires a 'value' number (0-1000)")
	}
	iv := int(v)
	if iv < 0 || iv > 1000 {
		return nil, fmt.Errorf("value must be in [0,1000], got %d", iv)
	}
	data := []byte{byte(iv & 0xFF), byte((iv >> 8) & 0xFF)}
	results := make([]interface{}, 0, len(s.armServoIDs))
	for _, id := range s.armServoIDs {
		err := s.controller.WriteServoRegister(ctx, id, "torque_limit", data)
		row := map[string]interface{}{"servo": id, "ok": err == nil}
		if err != nil {
			row["error"] = err.Error()
		}
		// Read back to confirm the write stuck.
		if rb, rerr := s.controller.ReadServoRegister(ctx, id, "torque_limit"); rerr == nil {
			row["readback"] = regDecodeLE(rb)
		}
		results = append(results, row)
	}
	return map[string]interface{}{
		"set_torque_limit": iv,
		"servos":           results,
		"note":             "diagnostic: restore full torque with value 1000 (or reinitialize) when done",
	}, nil
}

// enterManualLocked starts a manual-mode session. Caller holds s.mu.
func (s *so101) enterManualLocked(overrides map[string]interface{}) map[string]interface{} {
	// If already active, tear down first so the new parameters take effect (live re-tuning).
	// exitManualLocked restores the prior compliance registers before the fresh session
	// re-applies with the new values.
	if s.manual != nil && s.manual.running() {
		s.exitManualLocked("re-enter with new parameters")
	}
	var mm *ManualModeConfig
	if s.cfg != nil {
		mm = s.cfg.ManualMode
	}
	params, pGain, torqueLimit := resolveManualParams(mm, overrides)
	io := newControllerManualIO(s.controller, s.armServoIDs, s.calculateJointLimits())
	s.manual = newManualSession(s.cancelCtx, io, params, pGain, torqueLimit, s.logger)
	s.manual.start()
	return map[string]interface{}{"mode": "manual", "servos": s.armServoIDs}
}

// exitManualLocked tears down manual mode if active and holds the current pose.
// Caller holds s.mu. Safe to call when not active.
func (s *so101) exitManualLocked(reason string) {
	if s.manual == nil || !s.manual.running() {
		return
	}
	s.logger.Infof("manual mode exiting: %s", reason)
	s.manual.stop()
	s.manual = nil
}

func (s *so101) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// The servo_* family lets a gripper on this bus drive its own servo without holding a
	// serial connection. Routed before the arm's own commands; servocmd.HandleServoCommand
	// refuses any servo this arm drives itself, so the two cannot collide.
	if command, ok := cmd["command"].(string); ok && servocmd.IsServoCommand(command) {
		return servocmd.HandleServoCommand(ctx, cmd, s.controller, s.armServoIDs)
	}

	// Handle custom commands specific to SO-101
	switch cmd["command"] {
	case "set_torque":
		enable, ok := cmd["enable"].(bool)
		if !ok {
			return nil, fmt.Errorf("set_torque command requires 'enable' boolean parameter")
		}
		err := s.controller.SetTorqueEnable(ctx, enable)
		return map[string]interface{}{"success": err == nil}, err

	case "ping":
		err := s.controller.Ping(ctx)
		return map[string]interface{}{"success": err == nil}, err

	case "controller_status":
		refCount, hasController, configSummary := controller.GetControllerStatus()
		return map[string]interface{}{
			"ref_count":      refCount,
			"has_controller": hasController,
			"config":         configSummary,
			"arm_servo_ids":  s.armServoIDs,
		}, nil

	case "diagnose":
		err := s.diagnoseConnection()
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
		}, nil

	case "verify_config":
		err := s.verifyServoConfig()
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
		}, nil

	case "reinitialize":
		retries := 3 // default
		if r, ok := cmd["retries"].(float64); ok {
			retries = int(r)
		}
		err := s.initializeServosWithRetry(retries)
		return map[string]interface{}{
			"success": err == nil,
			"error":   fmt.Sprintf("%v", err),
			"retries": retries,
		}, nil

	case "test_servo_communication":
		positions, err := s.controller.GetJointPositionsForServos(ctx, s.armServoIDs)
		result := map[string]interface{}{
			"success":       err == nil,
			"arm_servo_ids": s.armServoIDs,
		}
		if err != nil {
			result["error"] = fmt.Sprintf("%v", err)
		} else {
			result["positions"] = positions
		}
		return result, nil

	case "reload_calibration":
		if s.cfg.CalibrationFile == "" {
			return map[string]interface{}{
				"success": false,
				"error":   "No calibration file configured",
			}, nil
		}

		// Load the new calibration
		newCalibration, err := controller.LoadFullCalibrationFromFile(s.cfg.CalibrationFile, s.logger)
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"error":   fmt.Sprintf("Failed to load calibration: %v", err),
			}, nil
		}

		// Update the controller with the new calibration
		if err := s.controller.SetCalibration(newCalibration); err != nil {
			return map[string]interface{}{
				"success": false,
				"error":   fmt.Sprintf("Failed to update calibration: %v", err),
			}, nil
		}

		s.logger.Debugf("Successfully reloaded calibration from %s", s.cfg.CalibrationFile)
		return map[string]interface{}{
			"success":          true,
			"calibration_file": s.cfg.CalibrationFile,
			"message":          "Calibration reloaded successfully",
		}, nil

	case "get_calibration":
		calibration := s.controller.GetCalibration()
		return map[string]interface{}{
			"success":     true,
			"calibration": calibration,
		}, nil

	case "enter_manual_mode":
		s.mu.Lock()
		defer s.mu.Unlock()
		return s.enterManualLocked(cmd), nil

	case "exit_manual_mode":
		s.mu.Lock()
		defer s.mu.Unlock()
		s.exitManualLocked("exit_manual_mode command")
		return map[string]interface{}{"mode": "auto"}, nil

	case "set_torque_limit":
		return s.setTorqueLimitDiag(ctx, cmd)

	default:
		// Check for speed and acceleration setting
		result := make(map[string]interface{})
		changed := false

		if speedVal, ok := cmd["set_speed"]; ok {
			if speed, ok := speedVal.(float64); ok {
				if speed < 3 || speed > 180 {
					return nil, fmt.Errorf("speed must be between 3 and 180 degrees/second, got %.1f", speed)
				}
				s.mu.Lock()
				s.defaultSpeed = float32(speed)
				s.mu.Unlock()
				result["speed_set"] = speed
				changed = true
			} else {
				return nil, fmt.Errorf("set_speed requires a number value")
			}
		}

		if accVal, ok := cmd["set_acceleration"]; ok {
			if acc, ok := accVal.(float64); ok {
				if acc < servo.MinAccelDegsPerSecSq || acc > servo.MaxAccelDegsPerSecSq {
					return nil, fmt.Errorf(
						"acceleration must be between %.0f and %.0f degrees/second^2, got %.1f",
						servo.MinAccelDegsPerSecSq, servo.MaxAccelDegsPerSecSq, acc)
				}
				s.mu.Lock()
				s.defaultAcc = float32(acc)
				s.mu.Unlock()
				result["acceleration_set"] = acc
				changed = true
			} else {
				return nil, fmt.Errorf("set_acceleration requires a number value")
			}
		}

		if getParams, ok := cmd["get_motion_params"]; ok && getParams.(bool) {
			s.mu.RLock()
			speedDegsPerSec := float64(s.defaultSpeed)
			accDegsPerSec := float64(s.defaultAcc)
			s.mu.RUnlock()

			result["current_speed_degs_per_sec"] = speedDegsPerSec
			result["current_acceleration_degs_per_sec_per_sec"] = accDegsPerSec
			changed = true
		}

		if changed {
			return result, nil
		}

		return nil, fmt.Errorf("unknown command: %v", cmd)
	}
}

func (s *so101) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

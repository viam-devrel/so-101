package so_arm

import (
	"math"

	"go.viam.com/rdk/logging"
)

// Servos with a homing offset but untouched position limits read as calibrated (the factory
// 0/4095 passes ReadCalibrationFromServos's min<max check). For the gripper that scales
// 0-100% across a whole encoder revolution, driving the jaw into its stops. Rationale and
// open questions: docs/superpowers/specs/2026-08-18-gripper-travel-guard-design.md.
const (
	gripperEncoderTicks = 4095

	// The gripper is always servo 6: SO101FullCalibration.Gripper maps to that ID.
	gripperServoID = 6

	// A half-turn homing offset puts the jaw's closed stop at the encoder centre. 0% is grab.
	gripperClosedStopTick = 2048

	// Open travel from the closed stop, measured across kits and held under the observed
	// ~1450 so a kit with less travel under-opens rather than stalling. Deliberately not
	// derived from gripperTravelTicks(): the model does not match the hardware, so a
	// kinematics edit must not move a hardware limit.
	gripperSafeTravelTicks = 1200

	// Jaw travel from a recorded calibration (2030..3481). The safe window stays under it.
	gripperObservedTravelTicks = 1451

	gripperSafeRangeMin = gripperClosedStopTick
	gripperSafeRangeMax = gripperClosedStopTick + gripperSafeTravelTicks

	// How much wider than the model a recorded range may be before we stop believing it.
	// Real calibrations overshoot slightly; 0/4095 and the 500/3500 default do not.
	gripperMaxPlausibleTravelFactor = 1.25
)

// gripperTravelTicks is the jaw's travel in encoder ticks, derived from the URDF joint limits
// so it tracks the kinematics.
func gripperTravelTicks() int {
	return int(math.Round((gripperJointMax - gripperJointMin) / (2 * math.Pi) * gripperEncoderTicks))
}

// gripperRangeIsPlausible judges a recorded range by width, so it catches any too-wide range
// rather than only the factory 0/4095.
func gripperRangeIsPlausible(rangeMin, rangeMax int) bool {
	width := rangeMax - rangeMin
	if width <= 0 {
		return false
	}
	return float64(width) <= float64(gripperTravelTicks())*gripperMaxPlausibleTravelFactor
}

// guardGripperTravel narrows the gripper's range when it is too wide to be real, and reports
// whether it did. Arm servos are left alone: degrees mode takes only a center point from the
// range, so a wrong one costs them a few degrees rather than a scaled command.
func guardGripperTravel(cal SO101FullCalibration, logger logging.Logger) (SO101FullCalibration, bool) {
	if cal.Gripper == nil || gripperRangeIsPlausible(cal.Gripper.RangeMin, cal.Gripper.RangeMax) {
		return cal, false
	}

	// Copy: DefaultSO101FullCalibration is a package-level var of pointers.
	guarded := *cal.Gripper
	oldMin, oldMax := guarded.RangeMin, guarded.RangeMax
	guarded.RangeMin, guarded.RangeMax = gripperSafeRangeMin, gripperSafeRangeMax
	cal.Gripper = &guarded

	if logger != nil {
		logger.Warnf(
			"gripper servo %d reports a %d-tick range (%d-%d), wider than the jaw can move (~%d ticks): "+
				"it has a homing offset but no real position limits. Using %d-%d instead, centered on "+
				"mid-travel. The jaw will not open fully; run the calibration workflow to record its range.",
			guarded.ID, oldMax-oldMin, oldMin, oldMax, gripperTravelTicks(), gripperSafeRangeMin, gripperSafeRangeMax)
	}
	return cal, true
}

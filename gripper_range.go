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

	// Where a half-turn homing offset puts mid-travel.
	gripperEncoderCenter = 2048

	// Fraction of modeled travel the substituted window spans. Under 1 so a bad guess means
	// the jaw won't fully open rather than stalling.
	gripperSafeTravelFraction = 0.8

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

// safeGripperRange is the window used when the recorded one isn't credible: centered where the
// homing offset puts mid-travel, spanning a fraction of modeled travel.
func safeGripperRange() (rangeMin, rangeMax int) {
	half := int(math.Round(float64(gripperTravelTicks()) * gripperSafeTravelFraction / 2))
	return gripperEncoderCenter - half, gripperEncoderCenter + half
}

// guardGripperTravel narrows the gripper's range when it is too wide to be real, and reports
// whether it did. Arm servos are left alone: degrees mode takes only a center point from the
// range, so a wrong one costs them a few degrees rather than a scaled command.
func guardGripperTravel(cal SO101FullCalibration, logger logging.Logger) (SO101FullCalibration, bool) {
	if cal.Gripper == nil || gripperRangeIsPlausible(cal.Gripper.RangeMin, cal.Gripper.RangeMax) {
		return cal, false
	}

	safeMin, safeMax := safeGripperRange()
	// Copy: DefaultSO101FullCalibration is a package-level var of pointers.
	guarded := *cal.Gripper
	oldMin, oldMax := guarded.RangeMin, guarded.RangeMax
	guarded.RangeMin, guarded.RangeMax = safeMin, safeMax
	cal.Gripper = &guarded

	if logger != nil {
		logger.Warnf(
			"gripper servo %d reports a %d-tick range (%d-%d), wider than the jaw can move (~%d ticks): "+
				"it has a homing offset but no real position limits. Using %d-%d instead, centered on "+
				"mid-travel. The jaw will not open fully; run the calibration workflow to record its range.",
			guarded.ID, oldMax-oldMin, oldMin, oldMax, gripperTravelTicks(), safeMin, safeMax)
	}
	return cal, true
}

package servo

import (
	"math"

	"go.viam.com/rdk/utils"
)

// Servo / motion tuning constants for hardware speed control.
const (
	// stepsPerRevolution is the STS3215 encoder resolution (full turn).
	stepsPerRevolution = 4096.0
	// StepsPerDegree converts an angle in degrees to servo steps (~11.378).
	StepsPerDegree = stepsPerRevolution / 360.0

	// Goal-velocity (steps/sec) clamp. 180 deg/s ~= 2048 steps/sec, so 3000 is a safe
	// ceiling above the configured range; 0 would mean "max speed" to the servo, so the
	// floor is 1.
	minSpeedSteps = 1
	maxSpeedSteps = 3000

	// Configured-speed bounds in deg/s (mirror the config validation), used to clamp a
	// MoveOptions-derived speed so a tiny or huge MaxVelRads can't produce an unsafe command.
	minSpeedDegsPerSec = 3.0
	maxSpeedDegsPerSec = 180.0

	// Wait-for-stop safety-timeout shaping.
	moveTimeoutFactor = 2.0
	minMoveTimeoutMs  = 1000
	// MaxMoveTimeoutMs is the safety-timeout ceiling for waiting on a move to complete.
	MaxMoveTimeoutMs = 15000
)

// DegPerSecToStepsPerSec converts an angular speed in degrees/second to the servo
// goal-velocity unit (steps/second), clamped to a safe range.
//
// NOTE: the steps/sec scale assumes the feetech goal-velocity register is in steps/sec
// (the package's documented contract). If bench testing shows the commanded speed differs
// materially from the measured speed, adjust StepsPerDegree by the measured factor.
func DegPerSecToStepsPerSec(degPerSec float64) int {
	steps := int(math.Round(degPerSec * StepsPerDegree))
	if steps < minSpeedSteps {
		steps = minSpeedSteps
	}
	if steps > maxSpeedSteps {
		steps = maxSpeedSteps
	}
	return steps
}

// ResolveSpeedDegsPerSec picks the effective move speed in deg/s. A per-move
// MoveOptions.MaxVelRads (radians/second) wins when positive; otherwise the configured
// default is used. The result is clamped to the configured deg/s bounds.
func ResolveSpeedDegsPerSec(maxVelRads, defaultSpeedDegsPerSec float64) float64 {
	if maxVelRads <= 0 {
		return defaultSpeedDegsPerSec
	}
	degs := utils.RadToDeg(maxVelRads)
	if degs < minSpeedDegsPerSec {
		degs = minSpeedDegsPerSec
	}
	if degs > maxSpeedDegsPerSec {
		degs = maxSpeedDegsPerSec
	}
	return degs
}

// MoveTimeoutMs returns a safety timeout (ms) for waiting on a move to complete, based on
// the worst-case joint travel and the commanded profile, clamped to a sane window.
//
// It models the ramp rather than assuming d/v. Acceleration is now enforced on hardware, and
// in the acceleration-limited regime the true duration 2*sqrt(d/a) ALWAYS exceeds 2*d/v --
// which is exactly the margin moveTimeoutFactor used to provide. Worked example inside the
// validated config range: at 50 deg/s and the 50 deg/s^2 minimum, a 30 degree move really
// takes 1.55s against a d/v-derived timeout of 1.2s, so WaitForServosToStop would warn and
// return while the arm was still moving.
func MoveTimeoutMs(maxTravelDeg, speedDegsPerSec, accelDegsPerSecSq float64) int {
	if speedDegsPerSec <= 0 || accelDegsPerSecSq <= 0 {
		return MaxMoveTimeoutMs
	}
	// Triangular when the move is too short to reach cruising speed, trapezoidal otherwise.
	seconds := maxTravelDeg/speedDegsPerSec + speedDegsPerSec/accelDegsPerSecSq
	if maxTravelDeg < speedDegsPerSec*speedDegsPerSec/accelDegsPerSecSq {
		seconds = 2 * math.Sqrt(maxTravelDeg/accelDegsPerSecSq)
	}
	ms := int(math.Round(seconds * 1000.0 * moveTimeoutFactor))
	if ms < minMoveTimeoutMs {
		ms = minMoveTimeoutMs
	}
	if ms > MaxMoveTimeoutMs {
		ms = MaxMoveTimeoutMs
	}
	return ms
}

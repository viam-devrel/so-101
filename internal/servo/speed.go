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
	// NOTE: minSpeedDegsPerSec is BELOW MinExecutableSpeedDegsPerSec -- see docs/arm.md,
	// "Motion and speed". Raising it would be a breaking config change.
	minSpeedDegsPerSec = 3.0
	maxSpeedDegsPerSec = 180.0

	// Wait-for-stop safety-timeout shaping.
	moveTimeoutFactor = 2.0
	minMoveTimeoutMs  = 1000
	// minDwellTimeoutMs is the floor for one intermediate waypoint's dwell, deliberately far
	// below minMoveTimeoutMs. A densified path issues hundreds of waypoints, so a 1s floor on
	// each would turn a tolerance the hardware cannot actually reach -- a joint's
	// steady-state error under load, say -- into a playback lasting minutes.
	minDwellTimeoutMs = 50
	// MaxMoveTimeoutMs is the safety-timeout ceiling for waiting on a move to complete.
	MaxMoveTimeoutMs = 15000
)

// MinExecutableSpeedDegsPerSec is the slowest per-joint speed the STS3215 honours, and the
// floor CoordinatedProfiles applies to every joint it scales down. Below it the register
// fails in OPPOSITE directions depending on load, so no single correction recovers it.
//
// A tuning value, not a hardware constant -- measured on one arm at p_gain 32. See
// docs/arm.md, "Motion and speed", for the measurements and their limits.
const MinExecutableSpeedDegsPerSec = 12.0

// Waypoint-lookahead bounds, in degrees: how far ahead of the arm the commanded goal is
// allowed to run. The dwell holds an intermediate waypoint until every joint is this close
// to it, and only then commands the next one.
const (
	// DefaultLookaheadDeg is the lookahead's SATISFIABILITY floor, not a good default on its
	// own: a lookahead below the droop the arm parks with is never reached, and every
	// waypoint burns its full dwell timeout. 3.0 clears the p_gain 32 droop band with ~30%
	// margin; it deliberately does not clear p_gain 16, which the dwell's stall escape
	// backstops instead. NOT sufficient alone -- see LookaheadDegFor for the other bound.
	// Measurements: docs/arm.md, "Waypoint streams".
	DefaultLookaheadDeg = 3.0
	MinLookaheadDeg     = 0.1
	MaxLookaheadDeg     = 45.0

	// lookaheadRampMargin keeps the next goal arriving slightly BEFORE the servo reaches its
	// deceleration point, rather than exactly at it.
	lookaheadRampMargin = 1.2
)

// LookaheadDegFor returns the waypoint lookahead a motion profile needs, from the two
// independent lower bounds it has to clear: the servo's deceleration distance v^2/(2a) --
// release the next goal after the servo is inside that ramp and the stream dips in velocity
// at every waypoint -- and DefaultLookaheadDeg, the droop floor, which governs at low speed.
//
// See docs/arm.md, "Waypoint streams", for the hardware measurements behind both.
func LookaheadDegFor(speedDegsPerSec, accelDegsPerSecSq float64) float64 {
	// The Acc register floors at ~43 deg/s^2, so a lower request is silently raised by the
	// hardware and the ramp is shorter than the configured value implies.
	a := math.Max(accelDegsPerSecSq, AccFloorDegsPerSecSq)
	if speedDegsPerSec <= 0 {
		return DefaultLookaheadDeg
	}
	ramp := lookaheadRampMargin * speedDegsPerSec * speedDegsPerSec / (2 * a)
	return math.Min(MaxLookaheadDeg, math.Max(DefaultLookaheadDeg, ramp))
}

// DegPerSecToStepsPerSec converts an angular speed in degrees/second to the servo
// goal-velocity unit (steps/second), clamped to a safe range.
//
// NOTE: the steps/sec scale assumes the feetech goal-velocity register is in steps/sec
// (the package's documented contract). If bench testing shows the commanded speed differs
// materially from the measured speed, adjust StepsPerDegree by the measured factor.
func DegPerSecToStepsPerSec(degPerSec float64) int {
	return min(max(int(math.Round(degPerSec*StepsPerDegree)), minSpeedSteps), maxSpeedSteps)
}

// ResolveSpeedDegsPerSec picks the effective move speed in deg/s. A per-move
// MoveOptions.MaxVelRads (radians/second) wins when positive; otherwise the configured
// default is used. The result is clamped to the configured deg/s bounds.
func ResolveSpeedDegsPerSec(maxVelRads, defaultSpeedDegsPerSec float64) float64 {
	if maxVelRads <= 0 {
		return defaultSpeedDegsPerSec
	}
	return min(max(utils.RadToDeg(maxVelRads), minSpeedDegsPerSec), maxSpeedDegsPerSec)
}

// ResolveAccelDegsPerSecSq mirrors ResolveSpeedDegsPerSec for acceleration. The symmetry is
// load-bearing: LookaheadDegFor needs the acceleration ACTUALLY in force, or a caller that
// lowers it gets a longer real ramp and a lookahead still sized for the shorter one.
func ResolveAccelDegsPerSecSq(maxAccRads, defaultAccelDegsPerSecSq float64) float64 {
	if maxAccRads <= 0 {
		return defaultAccelDegsPerSecSq
	}
	return min(max(utils.RadToDeg(maxAccRads), MinAccelDegsPerSecSq), MaxAccelDegsPerSecSq)
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
	return rampTimeoutMs(maxTravelDeg, speedDegsPerSec, accelDegsPerSecSq, minMoveTimeoutMs)
}

// DwellTimeoutMs bounds how long ONE intermediate waypoint may be held before the stream
// gives up on it and commands the next. Same ramp model as MoveTimeoutMs, applied to that
// segment's own travel rather than the whole move's, but floored at minDwellTimeoutMs.
func DwellTimeoutMs(maxTravelDeg, speedDegsPerSec, accelDegsPerSecSq float64) int {
	return rampTimeoutMs(maxTravelDeg, speedDegsPerSec, accelDegsPerSecSq, minDwellTimeoutMs)
}

func rampTimeoutMs(maxTravelDeg, speedDegsPerSec, accelDegsPerSecSq float64, floorMs int) int {
	if speedDegsPerSec <= 0 || accelDegsPerSecSq <= 0 {
		return MaxMoveTimeoutMs
	}
	// Triangular when the move is too short to reach cruising speed, trapezoidal otherwise.
	seconds := maxTravelDeg/speedDegsPerSec + speedDegsPerSec/accelDegsPerSecSq
	if maxTravelDeg < speedDegsPerSec*speedDegsPerSec/accelDegsPerSecSq {
		seconds = 2 * math.Sqrt(maxTravelDeg/accelDegsPerSecSq)
	}
	ms := int(math.Round(seconds * 1000.0 * moveTimeoutFactor))
	if ms < floorMs {
		ms = floorMs
	}
	if ms > MaxMoveTimeoutMs {
		ms = MaxMoveTimeoutMs
	}
	return ms
}

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
	//
	// NOTE minSpeedDegsPerSec is BELOW MinExecutableSpeedDegsPerSec: the configured range
	// promises speeds this hardware does not honour. Measured, not inferred -- see that
	// constant. Left alone here because raising it is a breaking config change.
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

// MinExecutableSpeedDegsPerSec is the slowest per-joint speed the STS3215 actually honours,
// and the floor CoordinatedProfiles applies to every joint it scales down.
//
// Below it the register is not merely imprecise, it fails in OPPOSITE directions depending
// on load, so neither the commanded speed nor any single correction is recoverable. Measured
// on an SO-101 (p_gain 32), 3 reps per point, 25 degree sweeps:
//
//	commanded   shoulder (heavy)   elbow (light)   wrist (unloaded)
//	 4 deg/s     0.80  (0.20x)      1.57 (0.39x)    7.60 (1.90x)
//	 8 deg/s     3.13  (0.39x)      8.82 (1.10x)    8.78 (1.10x)
//	10 deg/s     8.18  (bimodal, 4.22-10.19 across reps)
//	10.5 deg/s  10.52  (1.00x)     10.50 (1.00x)   10.44 (0.99x)
//
// The light joints floor hard at ~8.8 deg/s -- command less and you still get 8.8. The
// gravity-loaded shoulder does the reverse and under-executes to a fifth of its command.
// All three track within 1% from 10.5 up.
//
// 12 carries margin over the worst joint's measured 10.5 and clear of the bimodal 10.0. It
// is a tuning value, NOT a hardware constant: it comes from one arm, three joints, one pose
// each, on 10-25 degree sweeps. A payload or a different pose moves the loaded joint's
// threshold, and short accel-limited segments were not measured.
const MinExecutableSpeedDegsPerSec = 12.0

// Waypoint-lookahead bounds, in degrees: how far ahead of the arm the commanded goal is
// allowed to run. The dwell holds an intermediate waypoint until every joint is this close
// to it, and only then commands the next one.
const (
	// DefaultLookaheadDeg is the lookahead's SATISFIABILITY floor, not a good default on its
	// own. A servo settles where its position error times its P gain balances the load, so a
	// lookahead below that droop is never reached and every waypoint burns its full timeout.
	// Measured droop on a gravity-loaded SO-101 joint: 4.8 deg at p_gain 16, 2.2 at 32,
	// 1.5 at 48. 2.0 clears the p_gain 32 case, which is the Feetech default.
	//
	// It is NOT sufficient by itself -- see LookaheadDegFor for the other lower bound.
	DefaultLookaheadDeg = 2.0
	MinLookaheadDeg     = 0.1
	MaxLookaheadDeg     = 45.0

	// lookaheadRampMargin keeps the next goal arriving slightly BEFORE the servo reaches its
	// deceleration point, rather than exactly at it.
	lookaheadRampMargin = 1.2
)

// LookaheadDegFor returns the waypoint lookahead a motion profile needs, from the two
// independent lower bounds it has to clear.
//
// The first is the servo's DECELERATION DISTANCE, v^2/(2a). A servo slows as it approaches
// its goal, so unless the next goal is written before that ramp begins, a waypoint stream
// decelerates and re-accelerates at every waypoint. On a 10 Hz recording that is ten
// velocity dips a second, and it reads as jerk. Measured on a recorded wave replayed at
// 58.8 deg/s: the ramp is 3.45 deg, the old fixed 2.0 released the next goal only after the
// servo was already inside it, and raising the lookahead to 4 removed the jerk.
//
// The second is DefaultLookaheadDeg, the droop the arm parks with -- a lookahead under it is
// never satisfied at all. That one dominates at low speed: a nod at 25.2 deg/s has a ramp of
// only 0.64 deg, so it keeps the 2.0 floor.
//
// So the fixed 2.0 this replaces was right only below ~45 deg/s at the default acceleration,
// which is why a slow recording felt fine and a fast one did not.
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

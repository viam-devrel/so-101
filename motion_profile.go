package so_arm

import "math"

// Acceleration-register calibration, measured on hardware 2026-08-19 (see
// docs/superpowers/results/2026-08-19-bench-run.md).
//
// The register is linear from Acc 1 to about 50 and saturates above it. A single global
// slope and offset are used rather than a per-joint table: only three of five joints were
// measured, and the 89-128 steps/s^2 per unit spread implies roughly 10% arrival-timing
// error between joints -- small against a current spread of 50-100% of move duration.
const (
	accStepsPerUnit = 108.0 // mean measured slope, steps/s^2 per Acc unit
	accOffsetSteps  = 386.0 // mean measured offset, steps/s^2

	// minAccUnits is 1, NOT 0. Acc 0 means an UNLIMITED ramp -- the jerkiest possible
	// command, the exact opposite of what a caller asking for gentle acceleration wants.
	minAccUnits = 1

	// maxAccUnits is the measured knee. Above it the register has no further effect, so
	// scaling silently stops working.
	maxAccUnits = 50
)

// degPerSecSqToAccUnits converts an angular acceleration into the servo's Acc register
// value, clamped to the range where the register actually does something.
func degPerSecSqToAccUnits(degsPerSecSq float64) int {
	steps := degsPerSecSq * stepsPerDegree
	units := int(math.Round((steps - accOffsetSteps) / accStepsPerUnit))
	if units < minAccUnits {
		units = minAccUnits
	}
	if units > maxAccUnits {
		units = maxAccUnits
	}
	return units
}

// defaultAccelDegsPerSecSq is the shipped default, deliberately chosen so it maps to an Acc
// value BELOW maxAccUnits. 600 deg/s^2 maps to 59.6 and would clamp -- clamping the
// reference joint (k=1) while lower-k joints keep their exact scaled value breaks the
// coordination this whole mechanism exists to provide.
const defaultAccelDegsPerSecSq = 500.0

// jointProfile is one servo's commanded motion profile, in register units.
type jointProfile struct {
	speedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	accUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// coordinatedProfiles computes a profile per joint so that every joint finishes its travel
// at the same moment.
//
// Each joint is scaled by its share of the longest travel, k = d/d_max, applied to BOTH
// speed and acceleration. Scaling both makes each joint's profile a time-scaled copy of the
// same shape, which yields equal durations whether the motion is acceleration-limited
// (t = 2*sqrt(d/a); k cancels inside the radical) or speed-limited
// (t = d/v + v/a; k cancels term by term). Scaling speed alone would coordinate only the
// speed-limited case -- and at the defaults the crossover is 5 degrees, so most
// motion-planner waypoints fall in the acceleration-limited regime.
//
// travelsDeg may hold negative values; only magnitude matters. caps may be nil.
// Returns nil when no joint moves, in which case the caller should command nothing.
func coordinatedProfiles(travelsDeg []float64, refSpeedDegsPerSec, refAccelDegsPerSecSq float64, caps []jointLimits) []jointProfile {
	maxTravel := 0.0
	for _, d := range travelsDeg {
		if a := math.Abs(d); a > maxTravel {
			maxTravel = a
		}
	}
	if maxTravel == 0 {
		return nil
	}

	speed, accel := reduceReference(travelsDeg, maxTravel, refSpeedDegsPerSec, refAccelDegsPerSecSq, caps)

	out := make([]jointProfile, len(travelsDeg))
	for i, d := range travelsDeg {
		k := math.Abs(d) / maxTravel
		out[i] = jointProfile{
			// degPerSecToStepsPerSec, NOT resolveSpeedDegsPerSec: the latter floors at
			// 3 deg/s, which would destroy scaling for any joint below k ~ 0.06.
			speedSteps: degPerSecToStepsPerSec(speed * k),
			accUnits:   degPerSecSqToAccUnits(accel * k),
		}
	}
	return out
}

// reduceReference is implemented in Task 4.
func reduceReference(travelsDeg []float64, maxTravel, speed, accel float64, caps []jointLimits) (float64, float64) {
	return speed, accel
}

// jointLimits is defined properly in Task 4.
type jointLimits struct{}

package so_arm

import (
	"fmt"
	"math"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/utils"
)

// Acceleration-register calibration, measured on hardware 2026-08-19 (see
// docs/superpowers/results/2026-08-19-bench-run.md).
//
// The register is linear from Acc 1 to the low 50s and saturates above that. A single global
// slope and offset are used rather than a per-joint table: only three of five joints were
// measured, and the 89-128 steps/s^2 per unit spread implies roughly 10% arrival-timing
// error between joints -- small against a current spread of 50-100% of move duration.
const (
	accStepsPerUnit = 108.0 // mean measured slope, steps/s^2 per Acc unit
	accOffsetSteps  = 386.0 // mean measured offset, steps/s^2

	// minAccUnits is 1, NOT 0. Acc 0 means an UNLIMITED ramp -- the jerkiest possible
	// command, the exact opposite of what a caller asking for gentle acceleration wants.
	minAccUnits = 1

	// maxAccUnits is a conservative round-down below the LOWEST measured knee. The bench
	// measured knees of 52, 55 and 63 across three joints; 50 sits under all of them. Above
	// the knee the register has no further effect, so scaling silently stops working.
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

// Configured-acceleration bounds, in deg/s^2. Both ends are set by what the register can
// actually express: Acc 1 delivers about 43 deg/s^2, so the old minimum of 10 was
// unreachable by more than 4x; Acc 50 delivers about 508, so values above that do nothing.
const (
	minAccelDegsPerSecSq = 50.0
	maxAccelDegsPerSecSq = 500.0
)

// jointProfile is one servo's commanded motion profile, in register units.
type jointProfile struct {
	speedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	accUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// jointTravelsDeg returns each joint's travel magnitude in degrees, and the largest of them.
//
// Extracted from moveJoints so it can be tested: moveJoints itself holds a concrete
// controller and cannot be unit-tested without hardware, which would otherwise leave the
// arm's only piece of pure wiring logic verified solely by a bench run.
//
// from and to must be the same length; the caller checks that.
func jointTravelsDeg(from, to []float64) (travels []float64, maxTravel float64) {
	travels = make([]float64, len(to))
	for i := range to {
		d := math.Abs(to[i]-from[i]) * 180.0 / math.Pi
		travels[i] = d
		if d > maxTravel {
			maxTravel = d
		}
	}
	return travels, maxTravel
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

// jointLimits is an optional per-joint cap derived from arm.MoveOptions. A zero field means
// "no cap" -- never "stop", which is why every check below tests for > 0.
type jointLimits struct {
	maxSpeedDegsPerSec   float64
	maxAccelDegsPerSecSq float64
}

// reduceReference lowers the shared speed and acceleration reference until no per-joint cap
// would be exceeded.
//
// Caps are constraints, not targets. Clamping one joint to its cap would leave it out of
// step with the others and destroy the coordination; instead the most-constrained joint
// sets the pace for everyone. Since joint i is commanded v_max*k_i, requiring
// v_max <= cap_i/k_i for every moving joint guarantees every joint honors its own cap.
//
// Stationary joints are excluded. Note this is defensive rather than load-bearing: Go yields
// +Inf for c/0 with c > 0, and +Inf < speed is already false, so a stationary joint could
// never bind even without the guard. It stays because the intent should be explicit -- but do
// not assume removing it would fault.
func reduceReference(travelsDeg []float64, maxTravel, speed, accel float64, caps []jointLimits) (float64, float64) {
	for i, d := range travelsDeg {
		if i >= len(caps) {
			break
		}
		k := math.Abs(d) / maxTravel
		if k == 0 {
			continue
		}
		if c := caps[i].maxSpeedDegsPerSec; c > 0 && c/k < speed {
			speed = c / k
		}
		if c := caps[i].maxAccelDegsPerSecSq; c > 0 && c/k < accel {
			accel = c / k
		}
	}
	return speed, accel
}

// jointLimitsFromMoveOptions converts arm.MoveOptions into per-joint caps.
//
// Per arm.proto the scalar limit is IGNORED when the corresponding per-joint slice is set,
// rather than merged with it. RDK's own conversion explicitly leaves that to the
// implementer (components/arm/pb_helpers.go), so this driver enforces it.
//
// Returns an error for a per-joint slice whose length does not match the arm's DoF: RDK
// sizes the slice from whatever the client sent without checking, and silently ignoring or
// truncating it would produce motion that violates a cap the caller believes is in force.
func jointLimitsFromMoveOptions(opts *arm.MoveOptions, dof int) ([]jointLimits, error) {
	if opts == nil {
		return nil, nil
	}
	limits := make([]jointLimits, dof)

	if n := len(opts.MaxVelRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"max_vel_degs_per_sec_joints has %d entries but the arm has %d joints", n, dof)
		}
		for i, v := range opts.MaxVelRadsJoints {
			limits[i].maxSpeedDegsPerSec = utils.RadToDeg(v)
		}
	} else if opts.MaxVelRads > 0 {
		d := utils.RadToDeg(opts.MaxVelRads)
		for i := range limits {
			limits[i].maxSpeedDegsPerSec = d
		}
	}

	if n := len(opts.MaxAccRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"max_acc_degs_per_sec2_joints has %d entries but the arm has %d joints", n, dof)
		}
		for i, a := range opts.MaxAccRadsJoints {
			limits[i].maxAccelDegsPerSecSq = utils.RadToDeg(a)
		}
	} else if opts.MaxAccRads > 0 {
		d := utils.RadToDeg(opts.MaxAccRads)
		for i := range limits {
			limits[i].maxAccelDegsPerSecSq = d
		}
	}

	return limits, nil
}

// uniformSpeedUnderCaps returns the fastest uniform speed that violates no per-joint cap.
//
// Used only on the read-failure fallback path, where there is no travel reference and so no
// way to scale per joint. Taking the minimum is conservative -- every joint ends at or below
// its own cap -- and is far better than discarding caps the caller believes are in force.
func uniformSpeedUnderCaps(speed float64, caps []jointLimits) float64 {
	for _, c := range caps {
		if c.maxSpeedDegsPerSec > 0 && c.maxSpeedDegsPerSec < speed {
			speed = c.maxSpeedDegsPerSec
		}
	}
	return speed
}

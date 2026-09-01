package servo

import (
	"fmt"
	"math"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/utils"
)

// Acceleration-register calibration, measured on an SO-101 (STS3215, firmware 3.10) by
// sweeping the Acc register 0-255 at fixed travel and recovering the velocity profile from
// sampled position and velocity.
//
// The register is linear from Acc 1 to the low 50s and saturates above that. A single global
// slope and offset are used rather than a per-joint table: measured slopes were 127.6, 89.3
// and 108.2 steps/s^2 per unit on joints 1, 2 and 3 (joints 4 and 5 were not measured), and
// that spread implies roughly 10% arrival-timing error between joints -- small against the
// 50-100% of move duration that uncoordinated motion produced.
const (
	accStepsPerUnit = 108.0 // mean measured slope, steps/s^2 per Acc unit
	accOffsetSteps  = 386.0 // mean measured offset, steps/s^2

	// MinAccUnits is 1, NOT 0. Acc 0 means an UNLIMITED ramp -- the jerkiest possible
	// command, the exact opposite of what a caller asking for gentle acceleration wants.
	MinAccUnits = 1

	// maxAccUnits is a conservative round-down below the LOWEST measured knee. Measured
	// knees were 52, 55 and 63 across three joints; 50 sits under all of them. Above the
	// knee the register has no further effect, so scaling silently stops working.
	maxAccUnits = 50
)

// DegPerSecSqToAccUnits converts an angular acceleration into the servo's Acc register
// value, clamped to the range where the register actually does something.
func DegPerSecSqToAccUnits(degsPerSecSq float64) int {
	steps := degsPerSecSq * StepsPerDegree
	units := int(math.Round((steps - accOffsetSteps) / accStepsPerUnit))
	if units < MinAccUnits {
		units = MinAccUnits
	}
	if units > maxAccUnits {
		units = maxAccUnits
	}
	return units
}

// DefaultAccelDegsPerSecSq is the shipped default, deliberately chosen so it maps to an Acc
// value BELOW maxAccUnits. 600 deg/s^2 maps to 59.6 and would clamp -- clamping the
// reference joint (k=1) while lower-k joints keep their exact scaled value breaks the
// coordination this whole mechanism exists to provide.
const DefaultAccelDegsPerSecSq = 500.0

// AccFloorDegsPerSecSq is what Acc 1 actually delivers. Any lower request is silently raised
// to this by the register floor, so timing calculations must not assume the lower value.
const AccFloorDegsPerSecSq = 43.0

// Configured-acceleration bounds, in deg/s^2. Both ends are set by what the register can
// actually express: Acc 1 delivers about 43 deg/s^2, so the old minimum of 10 was
// unreachable by more than 4x; Acc 50 delivers about 508, so values above that do nothing.
const (
	MinAccelDegsPerSecSq = 50.0
	MaxAccelDegsPerSecSq = 500.0
)

// JointProfile is one servo's commanded motion profile, in register units.
type JointProfile struct {
	SpeedSteps int // goal velocity, steps/sec. Never 0: that means MAX SPEED.
	AccUnits   int // acceleration register. Never 0: that means UNLIMITED.
}

// JointTravelsDeg returns each joint's travel magnitude in degrees, and the largest of them.
//
// Extracted from moveJoints so it can be tested: moveJoints itself holds a concrete
// controller and cannot be unit-tested without hardware, which would otherwise leave the
// arm's only piece of pure wiring logic verified solely by a bench run.
//
// from and to must be the same length; the caller checks that.
func JointTravelsDeg(from, to []float64) (travels []float64, maxTravel float64) {
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

// CoordinatedProfiles computes a profile per joint so that every joint finishes its travel
// at the same moment.
//
// Each joint scales by its share of the longest travel, k = d/d_max, applied to BOTH speed
// and acceleration. That makes each profile a time-scaled copy of one shape, so durations
// match whether the motion is acceleration-limited (t = 2*sqrt(d/a); k cancels inside the
// radical) or speed-limited (t = d/v + v/a; k cancels term by term). Scaling speed alone
// coordinates only the speed-limited case, and at the defaults the crossover is 5 degrees --
// so most motion-planner waypoints fall in the acceleration-limited regime.
//
// travelsDeg may be negative (only magnitude matters); caps may be nil. Returns nil when no
// joint moves, in which case the caller should command nothing.
func CoordinatedProfiles(travelsDeg []float64, refSpeedDegsPerSec, refAccelDegsPerSecSq float64, caps []JointLimits) []JointProfile {
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

	out := make([]JointProfile, len(travelsDeg))
	for i, d := range travelsDeg {
		k := math.Abs(d) / maxTravel

		// Scaling below MinExecutableSpeedDegsPerSec buys nothing: the servo stops honouring
		// the register there, in opposite directions depending on load. So the floor costs
		// only the coordination this scaling could not deliver anyway, and a low-k joint
		// arrives early instead of executing an unpredictable speed.
		//
		// A STATIONARY joint is left at the old 1-step floor rather than raised to this one:
		// its goal is its own position, so the value is inert, and writing a real speed to a
		// joint that is not moving would be misleading on the wire.
		v := speed * k
		if k > 0 {
			v = math.Max(v, MinExecutableSpeedDegsPerSec)
		}

		// A per-joint cap still wins over the floor. It is a constraint the caller may be
		// relying on for safety, so a cap slower than the floor is UNDER-executed rather
		// than silently exceeded -- reduceReference already honoured it, and raising the
		// speed back above it here would quietly break that guarantee.
		if i < len(caps) && caps[i].MaxSpeedDegsPerSec > 0 && v > caps[i].MaxSpeedDegsPerSec {
			v = caps[i].MaxSpeedDegsPerSec
		}

		out[i] = JointProfile{
			// DegPerSecToStepsPerSec, NOT ResolveSpeedDegsPerSec: the latter floors at
			// 3 deg/s, which is both the wrong value and the wrong place for a floor.
			SpeedSteps: DegPerSecToStepsPerSec(v),
			AccUnits:   DegPerSecSqToAccUnits(accel * k),
		}
	}
	return out
}

// JointLimits is an optional per-joint cap derived from arm.MoveOptions. A zero field means
// "no cap" -- never "stop", which is why every check below tests for > 0.
type JointLimits struct {
	MaxSpeedDegsPerSec   float64
	MaxAccelDegsPerSecSq float64
}

// reduceReference lowers the shared speed and acceleration reference until no per-joint cap
// would be exceeded.
//
// Caps are constraints, not targets. Clamping one joint to its cap would leave it out of step
// with the others and destroy the coordination; instead the most-constrained joint sets the
// pace. Since joint i is commanded v_max*k_i, requiring v_max <= cap_i/k_i for every moving
// joint guarantees every joint honors its own cap.
//
// The stationary-joint exclusion is defensive, not load-bearing: Go yields +Inf for c/0 with
// c > 0, and +Inf < speed is already false, so a stationary joint could never bind without it.
// It stays to make the intent explicit -- but do not assume removing it would fault.
func reduceReference(travelsDeg []float64, maxTravel, speed, accel float64, caps []JointLimits) (float64, float64) {
	for i, d := range travelsDeg {
		if i >= len(caps) {
			break
		}
		k := math.Abs(d) / maxTravel
		if k == 0 {
			continue
		}
		if c := caps[i].MaxSpeedDegsPerSec; c > 0 && c/k < speed {
			speed = c / k
		}
		if c := caps[i].MaxAccelDegsPerSecSq; c > 0 && c/k < accel {
			accel = c / k
		}
	}
	return speed, accel
}

// JointLimitsFromMoveOptions converts arm.MoveOptions into per-joint caps.
//
// Per arm.proto the scalar limit is IGNORED when the corresponding per-joint slice is set,
// rather than merged with it. RDK's own conversion explicitly leaves that to the
// implementer (components/arm/pb_helpers.go), so this driver enforces it.
//
// Returns an error for a per-joint slice whose length does not match the arm's DoF: RDK
// sizes the slice from whatever the client sent without checking, and silently ignoring or
// truncating it would produce motion that violates a cap the caller believes is in force.
func JointLimitsFromMoveOptions(opts *arm.MoveOptions, dof int) ([]JointLimits, error) {
	if opts == nil {
		return nil, nil
	}
	limits := make([]JointLimits, dof)

	if n := len(opts.MaxVelRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"max_vel_degs_per_sec_joints has %d entries but the arm has %d joints", n, dof)
		}
		for i, v := range opts.MaxVelRadsJoints {
			limits[i].MaxSpeedDegsPerSec = utils.RadToDeg(v)
		}
	} else if opts.MaxVelRads > 0 {
		d := utils.RadToDeg(opts.MaxVelRads)
		for i := range limits {
			limits[i].MaxSpeedDegsPerSec = d
		}
	}

	if n := len(opts.MaxAccRadsJoints); n > 0 {
		if n != dof {
			return nil, fmt.Errorf(
				"max_acc_degs_per_sec2_joints has %d entries but the arm has %d joints", n, dof)
		}
		for i, a := range opts.MaxAccRadsJoints {
			limits[i].MaxAccelDegsPerSecSq = utils.RadToDeg(a)
		}
	} else if opts.MaxAccRads > 0 {
		d := utils.RadToDeg(opts.MaxAccRads)
		for i := range limits {
			limits[i].MaxAccelDegsPerSecSq = d
		}
	}

	return limits, nil
}

// UniformSpeedUnderCaps returns the fastest uniform speed that violates no per-joint cap.
//
// Used only on the read-failure fallback path, where there is no travel reference and so no
// way to scale per joint. Taking the minimum is conservative -- every joint ends at or below
// its own cap -- and is far better than discarding caps the caller believes are in force.
func UniformSpeedUnderCaps(speed float64, caps []JointLimits) float64 {
	for _, c := range caps {
		if c.MaxSpeedDegsPerSec > 0 && c.MaxSpeedDegsPerSec < speed {
			speed = c.MaxSpeedDegsPerSec
		}
	}
	return speed
}

package so_arm

import (
	"math"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
)

const (
	// defaultOrientationToleranceDeg is the approach-axis cone half-angle used when
	// orientation_tolerance_deg is zero or unset.
	defaultOrientationToleranceDeg = 30.0

	// defaultPositionToleranceMM is the per-axis positional leeway used when
	// position_tolerance_mm is zero or unset. It must stay comfortably above the IK
	// solver's convergence: a cloud the solver realistically cannot land inside is worse
	// than no cloud at all, because planning reverts to strict 6-DOF scoring.
	defaultPositionToleranceMM = 1.0

	// warnPositionToleranceMM is the point above which a cloud is loose enough that the
	// planner may report success visibly far from the goal.
	warnPositionToleranceMM = 10.0

	// poseCloudSaturationCos is the cosine at which the cone accepts every orientation.
	// PoseInCloud adds a 0.001 epsilon to the OZ leeway and the maximum possible
	// deviation is 2.0, so saturation begins where 1-cos(a)+0.001 >= 2. Expressed as a
	// cosine rather than the rounded 177.44 degrees: the true threshold is 177.4374, and
	// a literal degree comparison would miss the [177.4374, 177.44) band.
	poseCloudSaturationCos = -0.999
)

// goalCloudConfig is the resolved tolerance pair, letting both arm models share one code
// path despite their separate config structs. Defaults are already applied.
type goalCloudConfig struct {
	OrientationToleranceDeg float64
	PositionToleranceMM     float64
}

// resolveGoalCloudConfig applies defaults to zero or unset values and emits the two
// construction warnings. It is the single owner of both, so each constructor is one call
// and neither arm model re-implements the thresholds. logger may be nil (unit tests).
func resolveGoalCloudConfig(tolDeg, posTolMM float64, logger logging.Logger) goalCloudConfig {
	cfg := goalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: posTolMM}
	if cfg.OrientationToleranceDeg == 0 {
		cfg.OrientationToleranceDeg = defaultOrientationToleranceDeg
	}
	if cfg.PositionToleranceMM == 0 {
		cfg.PositionToleranceMM = defaultPositionToleranceMM
	}

	if logger == nil {
		return cfg
	}
	if cfg.PositionToleranceMM > warnPositionToleranceMM {
		logger.Warnf("position_tolerance_mm=%g is large; the planner may report success up to %.2fmm "+
			"from the goal (the cloud is a per-axis box, so the worst-case corner is sqrt(3) times this)",
			cfg.PositionToleranceMM, math.Sqrt(3)*cfg.PositionToleranceMM)
	}
	// Cite 177.4374 (the true acos(-0.999) threshold), NOT the rounded 177.44 used in
	// user-facing docs: this guard fires across [177.4374, 177.44), so quoting 177.44 here
	// would contradict the action taken. %g likewise avoids printing 177.4374 as "177.4".
	if math.Cos(utils.DegToRad(cfg.OrientationToleranceDeg)) <= poseCloudSaturationCos {
		logger.Warnf("orientation_tolerance_deg=%g saturates the approach-axis cone (>= 177.4374); "+
			"every orientation will be accepted, which is equivalent to ignoring orientation",
			cfg.OrientationToleranceDeg)
	}
	return cfg
}

// coneToPoseCloud converts an approach-axis cone half-angle into a PoseCloud.
//
// Every field here is load-bearing and was established empirically; see
// docs/superpowers/specs/2026-07-15-pose-cloud-orientation-design.md before changing any
// of them:
//
//   - X/Y/Z must be > 0. PoseInCloud adds a 0.001 epsilon to each leeway, so a zero
//     leeway demands a match within 1 micron -- which no IK solution will realistically
//     achieve, making the cloud useless in practice even though it does technically
//     accept an exact match.
//   - OX/OY are deliberately 1 (unconstrained). OZ alone defines the cone.
//   - Theta must be exactly 180. For a pure tilt, the relative Theta reports the tilt's
//     AZIMUTH (about X -> 90, about Y -> 0), not the roll -- and it does so independent of
//     the tilt's magnitude. The reported Theta sweeps the full [-180, 180] as azimuth goes
//     around, so ONLY a leeway of 180 admits every azimuth; that is, only 180 yields an
//     isotropic cone. Smaller values do not reject tilts outright -- they silently carve out
//     a wedge of tilt DIRECTIONS (Theta=90 still accepts a tilt about X, but rejects azimuths
//     past 180). TestConeBoundsTiltIsotropically pins this: at azimuth 270 the reported Theta
//     is exactly -180.
//     The cost is that roll cannot be constrained at all, which is why this is
//     approach-axis planning with free roll.
func coneToPoseCloud(cfg goalCloudConfig) *referenceframe.PoseCloud {
	return &referenceframe.PoseCloud{
		X:     cfg.PositionToleranceMM,
		Y:     cfg.PositionToleranceMM,
		Z:     cfg.PositionToleranceMM,
		OX:    1,
		OY:    1,
		OZ:    1 - math.Cos(utils.DegToRad(cfg.OrientationToleranceDeg)),
		Theta: 180,
	}
}

package so_arm

import (
	"math"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/utils"
)

const (
	// defaultOrientationToleranceDeg is the approach-axis cone half-angle used when
	// orientation_tolerance_deg is zero or unset.
	defaultOrientationToleranceDeg = 30.0

	// defaultPositionToleranceMM is the per-axis positional leeway used when
	// position_tolerance_mm is zero or unset. It must stay comfortably above the IK
	// solver's convergence: a cloud the solver can never land inside is worse than no
	// cloud at all, because planning reverts to strict 6-DOF scoring and always fails.
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

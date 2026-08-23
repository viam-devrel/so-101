// Package planning builds approach-axis orientation goal clouds so motion requests
// constrain the tool's approach axis while leaving roll free.
package planning

import (
	"fmt"
	"math"
	"sort"
	"strings"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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
	// cosine rather than a degree literal: the exact threshold is acos(-0.999) =
	// 177.4374412669, so any rounded degree constant is wrong in one direction or the
	// other -- 177.4374 sits below it (misses [177.4374412669, 177.44)), and 177.44 sits
	// above it. Comparing cosines avoids the choice entirely.
	poseCloudSaturationCos = -0.999
)

// GoalCloudConfig is the resolved tolerance pair, letting both arm models share one code
// path despite their separate config structs. Defaults are already applied.
type GoalCloudConfig struct {
	OrientationToleranceDeg float64
	PositionToleranceMM     float64
}

// ResolveGoalCloudConfig applies defaults to zero or unset values and emits the two
// construction warnings. It is the single owner of both, so each constructor is one call
// and neither arm model re-implements the thresholds. logger may be nil (unit tests).
func ResolveGoalCloudConfig(tolDeg, posTolMM float64, logger logging.Logger) GoalCloudConfig {
	cfg := GoalCloudConfig{OrientationToleranceDeg: tolDeg, PositionToleranceMM: posTolMM}
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
	// Cite 177.4374412669 (the exact acos(-0.999) threshold), NOT the rounded 177.44 used
	// in user-facing docs: this guard fires across [177.4374412669, 177.44), so quoting
	// 177.44 here would contradict the action taken. Do not round the cited value down
	// either -- 177.4374 sits below the threshold, where this guard does not fire. %g
	// likewise avoids printing it as "177.4".
	if math.Cos(utils.DegToRad(cfg.OrientationToleranceDeg)) <= poseCloudSaturationCos {
		logger.Warnf("orientation_tolerance_deg=%g saturates the approach-axis cone (>= 177.4374412669); "+
			"every orientation will be accepted, which is equivalent to ignoring orientation",
			cfg.OrientationToleranceDeg)
	}
	return cfg
}

// ValidateGoalCloudTolerances is shared by both arm models' Validate methods. Note NaN
// must be rejected explicitly: NaN comparisons are always false, so a NaN would slip
// through the range checks.
func ValidateGoalCloudTolerances(tolDeg, posTolMM float64) error {
	if math.IsNaN(tolDeg) || tolDeg < 0 || tolDeg > 180 {
		return fmt.Errorf("orientation_tolerance_deg must be in [0, 180], got %v", tolDeg)
	}
	if math.IsNaN(posTolMM) || posTolMM < 0 {
		return fmt.Errorf("position_tolerance_mm must not be negative, got %v", posTolMM)
	}
	return nil
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
func coneToPoseCloud(cfg GoalCloudConfig) *referenceframe.PoseCloud {
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

// poseCloudSetters maps the accepted pose_cloud keys to their fields. The names match
// referenceframe.PoseCloud's own JSON tags exactly, and are all lowercase. Note the Viam
// docs render these as OX/OY and protobuf JSON spells them o_x/oX; neither is accepted,
// which is why unknown keys are an error rather than ignored. This is also the complete
// field set: "reference_frame" is not a PoseCloud field in rdk v1.0.0.
var poseCloudSetters = map[string]func(*referenceframe.PoseCloud, float64){
	"x":     func(p *referenceframe.PoseCloud, v float64) { p.X = v },
	"y":     func(p *referenceframe.PoseCloud, v float64) { p.Y = v },
	"z":     func(p *referenceframe.PoseCloud, v float64) { p.Z = v },
	"ox":    func(p *referenceframe.PoseCloud, v float64) { p.OX = v },
	"oy":    func(p *referenceframe.PoseCloud, v float64) { p.OY = v },
	"oz":    func(p *referenceframe.PoseCloud, v float64) { p.OZ = v },
	"theta": func(p *referenceframe.PoseCloud, v float64) { p.Theta = v },
}

func poseCloudKeyList() string {
	keys := make([]string, 0, len(poseCloudSetters))
	for k := range poseCloudSetters {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return strings.Join(keys, ", ")
}

// parsePoseCloud converts a caller-supplied extra["pose_cloud"] into a PoseCloud.
// Omitted fields stay zero, which -- given the additive 0.001 epsilon -- means
// near-zero leeway. That is faithful passthrough and the point of an escape hatch, but it
// is sharp: a cloud whose leeways are all zero demands a match within 1 micron / 0.001
// degrees, which no IK solution will realistically achieve.
func parsePoseCloud(raw interface{}) (*referenceframe.PoseCloud, error) {
	m, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("pose_cloud must be an object, got %T", raw)
	}
	pc := &referenceframe.PoseCloud{}
	for k, rv := range m {
		set, known := poseCloudSetters[k]
		if !known {
			return nil, fmt.Errorf("pose_cloud: unknown key %q; valid keys are %s (all lowercase; "+
				"the Viam docs' OX/OY casing and protobuf's o_x are not accepted)", k, poseCloudKeyList())
		}
		v, ok := rv.(float64)
		if !ok {
			return nil, fmt.Errorf("pose_cloud: %q must be a number, got %T", k, rv)
		}
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf("pose_cloud: %q must be finite, got %v", k, v)
		}
		if v < 0 {
			return nil, fmt.Errorf("pose_cloud: %q must not be negative, got %v", k, v)
		}
		set(pc, v)
	}
	return pc, nil
}

const (
	extraKeyPoseCloud      = "pose_cloud"
	extraKeyGoalMetricType = "goal_metric_type"
)

// GoalPath reports which precedence row BuildMoveDestination took, so callers can wrap
// failures appropriately without re-inspecting extra (which would duplicate the
// precedence logic across both arm models and let it drift).
// The type is exported so callers can name what BuildMoveDestination hands them; its
// values are not, so no caller can name a path other than the one it hands back.
type GoalPath int

const (
	// pathInvalid is the zero value, returned alongside any error. It exists so that an
	// error return is never mistakable for pathCone: WrapMoveErr would otherwise dress a
	// pose_cloud parse error up as a cone-planning failure and tell the caller to widen
	// tolerances they never set.
	pathInvalid    GoalPath = iota
	pathCone                // cone from config
	pathRawCloud            // caller-supplied pose_cloud
	pathMetricType          // caller-supplied goal_metric_type; no cloud sent
)

// BuildMoveDestination applies the precedence table (see the plan/spec).
//
// originFrame is the FULLY-FORMED frame name (e.g. "myarm_origin"); this function does
// not append "_origin". It is passed to NewPoseInFrame unmodified.
//
// On success it returns a destination, a NEW extras map (the caller's map is never
// mutated), and the path taken. On error it returns (nil, nil, pathInvalid, err).
func BuildMoveDestination(originFrame string, pose spatialmath.Pose, cfg GoalCloudConfig,
	extra map[string]interface{},
) (*referenceframe.PoseInFrame, map[string]interface{}, GoalPath, error) {
	rawCloud, hasCloud := extra[extraKeyPoseCloud]
	_, hasMetric := extra[extraKeyGoalMetricType]

	if hasCloud && hasMetric {
		return nil, nil, pathInvalid, fmt.Errorf(
			"extra cannot contain both %q and %q (got %s=%v): %s=\"position_only\" sets "+
				"orientScale=0, so the cone's orientation leeways stop mattering. Pass one or the other",
			extraKeyPoseCloud, extraKeyGoalMetricType, extraKeyGoalMetricType,
			extra[extraKeyGoalMetricType], extraKeyGoalMetricType)
	}

	planExtra := make(map[string]interface{}, len(extra))
	for k, v := range extra {
		if k == extraKeyPoseCloud {
			continue // consumed here; not a planner key
		}
		planExtra[k] = v
	}

	switch {
	case hasMetric:
		// The caller's metric wins and no cloud is sent: position_only sets orientScale=0,
		// so a cloud's orientation leeways would stop mattering.
		return referenceframe.NewPoseInFrame(originFrame, pose), planExtra, pathMetricType, nil
	case hasCloud:
		pc, err := parsePoseCloud(rawCloud)
		if err != nil {
			return nil, nil, pathInvalid, err
		}
		return referenceframe.NewPoseInFrameWithGoalCloud(originFrame, pose, pc), planExtra, pathRawCloud, nil
	default:
		return referenceframe.NewPoseInFrameWithGoalCloud(originFrame, pose, coneToPoseCloud(cfg)),
			planExtra, pathCone, nil
	}
}

// WrapMoveErr adds actionable guidance to a planning failure. The cone-specific wording
// applies only on the cone path: on the other paths it would misdirect, telling a caller
// who just passed position_only to pass position_only, or blaming a config tolerance that
// had no bearing on a raw-cloud failure.
//
// The cone message names BOTH tolerances. Either can cause the failure: too tight a cone
// leaves no reachable orientation, and too small a position tolerance means the solver can
// never land inside the cloud (which reverts planning to strict 6-DOF scoring and fails
// every move). Naming only the orientation knob would point at the wrong one half the time.
func WrapMoveErr(err error, path GoalPath, cfg GoalCloudConfig) error {
	if err == nil {
		return nil
	}
	switch path {
	case pathCone:
		return fmt.Errorf("move to position failed with orientation_tolerance_deg=%g, "+
			"position_tolerance_mm=%g (approach-axis cone): %w; widen either tolerance "+
			"(too small a position_tolerance_mm means the solver can never land inside the "+
			"goal cloud), pass extra {\"goal_metric_type\": \"position_only\"} to ignore "+
			"orientation, or check that viam-server is >= 0.127.0 (older servers silently "+
			"ignore goal clouds)",
			cfg.OrientationToleranceDeg, cfg.PositionToleranceMM, err)
	case pathRawCloud:
		return fmt.Errorf("move to position failed with the caller-supplied pose_cloud: %w; "+
			"widen the cloud, or check that viam-server is >= 0.127.0 (older servers silently "+
			"ignore goal clouds)", err)
	default:
		return err
	}
}

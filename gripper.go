package so_arm

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	SO101GripperModel = resource.NewModel("devrel", "so101", "gripper")
)

// Gripper variants selectable via the gripper_type config attribute.
const (
	leaderGripper   = "leader"
	followerGripper = "follower"
)

// Mesh detail levels selectable via the mesh_detail config attribute. The gripper
// mesh is also the motion-planning collision geometry, so "low" (heavily decimated)
// is the responsive default; "high" is full resolution, for visual comparison.
const (
	highDetail = "high"
	lowDetail  = "low"
)

type SO101GripperConfig struct {
	Port     string `json:"port,omitempty"`
	Baudrate int    `json:"baudrate,omitempty"`

	// Default to 6
	ServoID int `json:"servo_id,omitempty"`

	Timeout time.Duration `json:"timeout,omitempty"`

	// Shared with arm
	CalibrationFile string `json:"calibration_file,omitempty"`

	// GripperType selects which meshes Geometries() serves: "follower" (moving
	// jaw, the default) or "leader" (thumb-loop handle + trigger).
	GripperType string `json:"gripper_type,omitempty"`

	// MeshDetail selects the gripper mesh resolution Geometries() serves:
	// "low" (decimated, the default -- keeps motion planning fast) or "high"
	// (full resolution).
	MeshDetail string `json:"mesh_detail,omitempty"`
}

// Validate ensures all parts of the config are valid
func (cfg *SO101GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Port == "" {
		return nil, nil, fmt.Errorf("must specify port for serial communication")
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	if cfg.ServoID < 1 || cfg.ServoID > 6 {
		return nil, nil, fmt.Errorf("servo_id must be between 1 and 6, got %d", cfg.ServoID)
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000
	}

	if cfg.GripperType != "" && cfg.GripperType != leaderGripper && cfg.GripperType != followerGripper {
		return nil, nil, fmt.Errorf("gripper_type must be %q or %q, got %q",
			leaderGripper, followerGripper, cfg.GripperType)
	}

	if cfg.MeshDetail != "" && cfg.MeshDetail != highDetail && cfg.MeshDetail != lowDetail {
		return nil, nil, fmt.Errorf("mesh_detail must be %q or %q, got %q",
			highDetail, lowDetail, cfg.MeshDetail)
	}

	return nil, nil, nil
}

type so101Gripper struct {
	resource.AlwaysRebuild

	name        resource.Name
	logger      logging.Logger
	controller  *SafeSoArmController
	gripperType string
	meshDetail  string
	servoID     int
	model       referenceframe.Model

	mu       sync.Mutex
	isMoving atomic.Bool

	// Gripper positions in percentage, 0-100%
	openPosition   float64
	closedPosition float64

	speed        float32
	acceleration float32
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		SO101GripperModel,
		resource.Registration[gripper.Gripper, *SO101GripperConfig]{
			Constructor: newSO101Gripper,
		},
	)
}

func newSO101Gripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*SO101GripperConfig](conf)
	if err != nil {
		return nil, err
	}

	if cfg.ServoID == 0 {
		cfg.ServoID = 6
	}

	if cfg.Baudrate == 0 {
		cfg.Baudrate = 1000000
	}

	controllerConfig := &SoArm101Config{
		Port:            cfg.Port,
		Baudrate:        cfg.Baudrate,
		ServoIDs:        []int{1, 2, 3, 4, 5, 6},
		Timeout:         cfg.Timeout,
		CalibrationFile: cfg.CalibrationFile,
		Logger:          logger,
	}

	controllerConfig.Validate(cfg.CalibrationFile)

	fullCalibration, fromFile := controllerConfig.LoadCalibration(logger)

	if fullCalibration.Gripper.ID != cfg.ServoID {
		logger.Debugf("Updating gripper calibration servo ID from %d to %d (from config)",
			fullCalibration.Gripper.ID, cfg.ServoID)
		fullCalibration.Gripper.ID = cfg.ServoID
	}

	controller, err := GetSharedControllerWithCalibration(controllerConfig, fullCalibration, fromFile)
	if err != nil {
		return nil, fmt.Errorf("failed to get shared controller for gripper: %w", err)
	}

	gripperType := cfg.GripperType
	if gripperType == "" {
		gripperType = followerGripper
	}

	meshDetail := cfg.MeshDetail
	if meshDetail == "" {
		meshDetail = lowDetail
	}

	model, err := buildGripperModel(gripperType, meshDetail, conf.ResourceName().ShortName())
	if err != nil {
		ReleaseSharedController()
		return nil, fmt.Errorf("failed to build gripper kinematic model: %w", err)
	}

	g := &so101Gripper{
		name:           conf.ResourceName(),
		logger:         logger,
		controller:     controller,
		gripperType:    gripperType,
		meshDetail:     meshDetail,
		servoID:        cfg.ServoID,
		model:          model,
		speed:          30,
		acceleration:   50,
		openPosition:   95.0,
		closedPosition: 0.0,
	}

	logger.Debugf("SO-101 gripper initialized with servo ID %d, open=%.1f%%, closed=%.1f%%",
		cfg.ServoID, g.openPosition, g.closedPosition)

	return g, nil
}

func (g *so101Gripper) Name() resource.Name {
	return g.name
}

func (g *so101Gripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Opening gripper")

	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.openPositionRadians()}, 0, 0); err != nil {
		return fmt.Errorf("failed to open gripper: %w", err)
	}

	time.Sleep(500 * time.Millisecond)

	g.logger.Debug("Gripper opened")
	return nil
}

func (g *so101Gripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	g.logger.Debug("Attempting to grab with gripper")

	if err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{g.closedPositionRadians()}, 0, 0); err != nil {
		return false, fmt.Errorf("failed to close gripper: %w", err)
	}

	time.Sleep(500 * time.Millisecond)

	currentPositions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
	if err != nil {
		g.logger.Warnf("Failed to read gripper position after grab: %v", err)
		return true, nil
	}

	if len(currentPositions) == 0 {
		g.logger.Warn("No position data received from gripper")
		return false, nil
	}

	currentPercent := g.radiansToPercent(currentPositions[0])

	positionDifference := currentPercent - g.closedPosition
	threshold := 15.0

	grabbed := positionDifference > threshold

	if grabbed {
		g.logger.Debugf("Gripper successfully grabbed an object (position difference: %.1f%%)", positionDifference)
	} else {
		g.logger.Debug("Gripper closed but may not have grabbed anything")
	}

	return grabbed, nil
}

func (g *so101Gripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(false)
	return g.controller.Stop(ctx)
}

func (g *so101Gripper) IsMoving(ctx context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

// Gripper geometry from the SO-101 URDF (TheRobotStudio/SO-ARM100), in millimeters.
// Poses are relative to the gripper component's frame (the arm's tool frame).
var (
	// gripperBodyPose places the static body mesh.
	gripperBodyPose = spatialmath.NewPose(
		r3.Vector{Y: -0.22, Z: 0.95}, &spatialmath.EulerAngles{Roll: math.Pi})
	// gripperJointPose is the gripper joint origin; the moving part rotates about its Z axis.
	gripperJointPose = spatialmath.NewPose(
		r3.Vector{X: 20.2, Y: 18.8, Z: -23.4}, &spatialmath.EulerAngles{Roll: math.Pi / 2})
	// gripperMovingVisualPose offsets the moving-part mesh within the joint's link.
	gripperMovingVisualPose = spatialmath.NewPoseFromPoint(r3.Vector{Z: 18.9})
	// toolFromGripperLink maps the URDF gripper_link frame (which the poses above are
	// authored in) into the Viam "tool" frame the gripper component attaches at.
	// so101.json's "tool" link rotates ~180 deg from gripper_link, so without this
	// correction the gripper renders facing backwards.
	toolFromGripperLink = spatialmath.PoseInverse(
		spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Pitch: math.Pi, Yaw: 0.0486795}))
	// gripperTCPPose is the follower gripper's tool center point: the grasp point between
	// the tips of the two jaws, expressed in the same frame as the poses above (the arm's
	// "tool" frame). Measured off the follower meshes as buildGripperMeshes poses them, with
	// the jaws closed (see TestGripperTCPLiesBetweenTheJawTips, which re-derives these bounds
	// from the meshes so a mesh regeneration cannot silently strand the TCP inside a finger):
	//
	//	static finger contact face  X = 7.67   -- the jaws meet at X = 6.8
	//	moving jaw contact face     X = 5.92
	//	contact pads span           Z = 94.4 .. 104.3, center 99.9
	//
	// It is a pure translation: the TCP keeps the tool frame's axes, so +Z remains the
	// approach axis that the arm's goal-cloud orientation planning is built around.
	gripperTCPPose = spatialmath.NewPoseFromPoint(r3.Vector{X: 6.835, Y: 0, Z: 99.9})
)

// Gripper joint limits from the SO-101 URDF, in radians.
const (
	gripperJointMin = -0.174533
	gripperJointMax = 1.74533
)

// buildGripperMeshes returns the gripper's static body mesh plus the moving part
// (jaw for the follower, trigger for the leader) posed at jawAngle radians about
// the gripper joint, at the requested mesh detail level ("high" or "low"). It is
// split out from Geometries so it can be tested without a hardware controller.
func buildGripperMeshes(gripperType, meshDetail string, jawAngle float64) ([]spatialmath.Geometry, error) {
	// Mesh names for this gripper variant: the static body part(s) plus the moving
	// part. The follower's body is one piece; the leader's is the wrist-roll part
	// (which connects to the arm) plus the handle attached to it.
	staticNames := []string{"follower_body"}
	movingName := "follower_jaw"
	if gripperType == leaderGripper {
		staticNames = []string{"leader_wrist_roll", "leader_body"}
		movingName = "leader_trigger"
	}

	bodyPose := spatialmath.Compose(toolFromGripperLink, gripperBodyPose)
	geoms := make([]spatialmath.Geometry, 0, len(staticNames)+1)
	for i, name := range staticNames {
		ply, err := gripperMeshPLY(meshDetail, name)
		if err != nil {
			return nil, err
		}
		m, err := spatialmath.NewMeshFromProto(bodyPose,
			&commonpb.Mesh{ContentType: "ply", Mesh: ply}, fmt.Sprintf("gripper_body_%d", i))
		if err != nil {
			return nil, err
		}
		geoms = append(geoms, m)
	}

	// Moving part: joint origin, then a rotation about the joint Z axis, then the
	// moving-part mesh offset -- the whole chain corrected into the tool frame.
	movingPose := spatialmath.Compose(toolFromGripperLink, spatialmath.Compose(
		spatialmath.Compose(gripperJointPose,
			spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Yaw: jawAngle})),
		gripperMovingVisualPose,
	))
	movingPLY, err := gripperMeshPLY(meshDetail, movingName)
	if err != nil {
		return nil, err
	}
	moving, err := spatialmath.NewMeshFromProto(movingPose,
		&commonpb.Mesh{ContentType: "ply", Mesh: movingPLY}, "gripper_moving")
	if err != nil {
		return nil, err
	}

	return append(geoms, moving), nil
}

// gripperTCPFrame is the leaf link of the gripper's kinematic model -- the frame the whole
// model resolves to, and therefore the pose viam-server reports for the gripper component.
const gripperTCPFrame = "tcp"

// buildGripperModel builds the gripper's kinematic model: a zero-DoF chain of static links
// carrying the gripper meshes, ending at gripperTCPPose. Two things hang off this:
//
//   - The model's leaf is what the frame system calls "the gripper", so GetPose and any motion
//     request targeting the gripper resolve to the TCP between the jaw tips instead of the arm's
//     wrist, where the meshes are anchored.
//   - The frame system takes an arm/gantry/gripper part's collision geometry from its kinematic
//     model and never calls Geometries() for those subtypes, so attaching the meshes here is what
//     makes the gripper a real obstacle to motion planning. On viam-server >= 1.0.0 a gripper
//     whose Kinematics() errors is dropped from the frame system outright, so returning a model
//     is also what keeps the gripper on the robot at all.
//
// The model is static, so the meshes are frozen at the closed jaw pose (gripperJointMin). A DoF
// here would instead become a variable the motion planner is free to drive, and freezing the jaw
// open would inflate the swept volume enough to block otherwise-valid plans. Geometries() still
// serves the live, articulating jaw for the 3D viewer.
//
// The result is round-tripped through SVA JSON rather than assembled in memory: a component ships
// its kinematics to viam-server as ModelConfig().OriginalFile.Bytes, and a model without those
// bytes transmits as UNSPECIFIED (see TestGripperModelSurvivesSerialization). SVA JSON carries the
// meshes across the boundary in GeometryConfig.MeshData.
func buildGripperModel(gripperType, meshDetail, name string) (referenceframe.Model, error) {
	meshes, err := buildGripperMeshes(gripperType, meshDetail, gripperJointMin)
	if err != nil {
		return nil, err
	}

	cfg := &referenceframe.ModelConfigJSON{Name: name, KinParamType: "SVA"}
	// Each mesh gets its own link because a link carries at most one geometry. The links are
	// chained (SVA models must have exactly one leaf) but all have identity transforms, so the
	// full mesh pose lives in the geometry offset and every link sits at the gripper's mount.
	parent := referenceframe.World
	for _, mesh := range meshes {
		geomCfg, err := spatialmath.NewGeometryConfig(mesh)
		if err != nil {
			return nil, fmt.Errorf("converting gripper mesh %q to a geometry config: %w", mesh.Label(), err)
		}
		cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
			ID:       mesh.Label(),
			Parent:   parent,
			Geometry: geomCfg,
		})
		parent = mesh.Label()
	}

	// The leader gripper is a hand-held trigger, not a pair of jaws, so it has no grasp point:
	// its frame stays at the mount.
	tcp := gripperTCPPose
	if gripperType == leaderGripper {
		tcp = spatialmath.NewZeroPose()
	}
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID:          gripperTCPFrame,
		Parent:      parent,
		Translation: tcp.Point(),
	})

	jsonBytes, err := json.Marshal(cfg)
	if err != nil {
		return nil, fmt.Errorf("serializing the %s gripper model: %w", gripperType, err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
}

// jawAngle maps the gripper's current open percentage onto the URDF gripper-joint
// range. If the live position cannot be read it assumes the gripper is closed.
func (g *so101Gripper) jawAngle(ctx context.Context) float64 {
	positions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
	if err != nil || len(positions) == 0 {
		return gripperJointMin
	}
	pct := math.Max(0, math.Min(1, g.radiansToPercent(positions[0])/100.0))
	return gripperJointMin + pct*(gripperJointMax-gripperJointMin)
}

// Geometries serves the gripper as meshes: a static body and a moving part posed by
// the live gripper opening, so the jaw/trigger articulates in the 3D viewer.
func (g *so101Gripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return buildGripperMeshes(g.gripperType, g.meshDetail, g.jawAngle(ctx))
}

// Status returns the current status of the resource as a map of key-value pairs.
func (g *so101Gripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

// gripperSetPositionPercent resolves a set_position target percentage from either
// "percentage" (the key documented in the README and used by the simulated gripper
// and the teleop service) or "position_percentage" (the key get_position returns,
// kept so a single-key get/set round-trip — e.g. arm-recorder — still works).
// "percentage" takes precedence when both are supplied.
func gripperSetPositionPercent(cmd map[string]interface{}) (float64, bool) {
	if pct, ok := cmd["percentage"].(float64); ok {
		return pct, true
	}
	if pct, ok := cmd["position_percentage"].(float64); ok {
		return pct, true
	}
	return 0, false
}

func (g *so101Gripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["get"] == true {
		positions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
		if err != nil {
			return nil, err
		}
		if len(positions) == 0 {
			return nil, fmt.Errorf("no position data available")
		}

		percentPos := g.radiansToPercent(positions[0])
		return map[string]interface{}{"position": percentPos}, nil
	}

	if percentPos, ok := cmd["set"].(float64); ok {
		if percentPos < 0 {
			percentPos = 0
		}
		if percentPos > 100 {
			percentPos = 100
		}

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		targetRadians := g.percentToRadians(percentPos)
		err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{targetRadians}, 0, 0)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": percentPos}, nil
	}

	switch cmd["command"] {
	case "get_position":
		positions, err := g.controller.GetJointPositionsForServos(ctx, []int{g.servoID})
		if err != nil {
			return nil, err
		}
		if len(positions) == 0 {
			return nil, fmt.Errorf("no position data available")
		}

		percentPos := g.radiansToPercent(positions[0])

		return map[string]interface{}{
			"position_radians":    positions[0],
			"position_percentage": percentPos,
			"open_position":       g.openPosition,
			"closed_position":     g.closedPosition,
		}, nil

	case "set_position":
		var targetPercent float64

		if percentPos, ok := gripperSetPositionPercent(cmd); ok {
			targetPercent = percentPos
		} else if servoPos, ok := cmd["servo_position"].(float64); ok {
			cal := g.controller.getCalibrationForServo(g.servoID)
			if cal != nil {
				normalizedPos := (servoPos - float64(cal.RangeMin)) / float64(cal.RangeMax-cal.RangeMin)
				targetPercent = normalizedPos * 100.0
			} else {
				targetPercent = (servoPos / 4095.0) * 100.0
			}
		} else {
			return nil, fmt.Errorf("set_position command requires 'percentage', 'position_percentage', or 'servo_position' parameter")
		}

		if targetPercent < 0 {
			targetPercent = 0
		}
		if targetPercent > 100 {
			targetPercent = 100
		}

		g.mu.Lock()
		defer g.mu.Unlock()

		g.isMoving.Store(true)
		defer g.isMoving.Store(false)

		targetRadians := g.percentToRadians(targetPercent)
		err := g.controller.MoveServosToPositions(ctx, []int{g.servoID}, []float64{targetRadians}, 0, 0)
		return map[string]interface{}{"success": err == nil}, err

	case "controller_status":
		refCount, hasController, configSummary := GetControllerStatus()
		return map[string]interface{}{
			"ref_count":      refCount,
			"has_controller": hasController,
			"config":         configSummary,
			"servo_id":       g.servoID,
		}, nil

	case "calibrate_positions":
		if openPos, ok := cmd["open_position"].(float64); ok {
			if openPos >= 0 && openPos <= 100 {
				g.openPosition = openPos
			}
		}
		if closedPos, ok := cmd["closed_position"].(float64); ok {
			if closedPos >= 0 && closedPos <= 100 {
				g.closedPosition = closedPos
			}
		}

		g.logger.Debugf("Gripper positions calibrated: open=%.1f%%, closed=%.1f%%", g.openPosition, g.closedPosition)

		return map[string]interface{}{
			"success":         true,
			"open_position":   g.openPosition,
			"closed_position": g.closedPosition,
		}, nil

	case "set_motion_params":
		if speed, ok := cmd["speed"].(float64); ok {
			if speed > 0 && speed <= 180 {
				g.speed = float32(speed)
			}
		}
		if acc, ok := cmd["acceleration"].(float64); ok {
			if acc > 0 && acc <= 500 {
				g.acceleration = float32(acc)
			}
		}

		return map[string]interface{}{
			"success":      true,
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	case "get_motion_params":
		return map[string]interface{}{
			"speed":        g.speed,
			"acceleration": g.acceleration,
		}, nil

	default:
		return nil, fmt.Errorf("unknown command: %v", cmd["command"])
	}
}

func (g *so101Gripper) Close(ctx context.Context) error {
	ReleaseSharedController()
	return nil
}

func (g *so101Gripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *so101Gripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

// Kinematics returns the gripper's static model, whose leaf frame is the TCP between the jaw
// tips. See buildGripperModel for why the gripper needs one at all.
func (g *so101Gripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

func (g *so101Gripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, nil
}

func (g *so101Gripper) openPositionRadians() float64 {
	return g.percentToRadians(g.openPosition)
}

func (g *so101Gripper) closedPositionRadians() float64 {
	return g.percentToRadians(g.closedPosition)
}

func (g *so101Gripper) percentToRadians(percent float64) float64 {
	// Since the gripper calibration uses NormModeRange100 (0-100%),
	// we can directly use the percentage value and let feetech-servo handle conversion
	// But we need to convert to the expected format for the controller

	// For now, we'll do a simple mapping based on the calibration range
	cal := g.controller.getCalibrationForServo(g.servoID)
	if cal == nil {
		// Fallback to default behavior
		return (percent - 50.0) / 50.0 * math.Pi
	}

	// Convert percentage to normalized position within the calibrated range
	normalizedPos := percent / 100.0

	// Convert to radians (assuming ±π range)
	radians := (normalizedPos*2.0 - 1.0) * math.Pi // Convert 0-1 to -π to +π

	// Apply drive mode if needed
	if cal.DriveMode != 0 {
		radians = -radians
	}

	return radians
}

func (g *so101Gripper) radiansToPercent(radians float64) float64 {
	cal := g.controller.getCalibrationForServo(g.servoID)
	if cal == nil {
		// Fallback to default behavior
		return (radians/math.Pi)*50.0 + 50.0
	}

	// Apply drive mode if needed
	adjustedRadians := radians
	if cal.DriveMode != 0 {
		adjustedRadians = -radians
	}

	// Convert radians to normalized position (-1 to 1)
	normalizedPos := adjustedRadians / math.Pi

	// Convert to percentage (0-100)
	percent := (normalizedPos + 1.0) / 2.0 * 100.0

	// Clamp to valid range
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}

	return percent
}

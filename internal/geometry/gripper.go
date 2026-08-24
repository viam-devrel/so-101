package geometry

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Gripper variants selectable via the gripper_type config attribute.
const (
	LeaderGripper   = "leader"
	FollowerGripper = "follower"
)

// Mesh detail levels selectable via the mesh_detail config attribute. The gripper
// mesh is also the motion-planning collision geometry, so "low" (heavily decimated)
// is the responsive default; "high" is full resolution, for visual comparison.
const (
	HighDetail = "high"
	LowDetail  = "low"
)

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
	// GripperTCPPose is the follower gripper's tool center point: the grasp point between
	// the tips of the two jaws, expressed in the same frame as the poses above (the arm's
	// "tool" frame). Measured off the follower meshes as BuildGripperMeshes poses them, jaws
	// closed. TestGripperTCPLiesBetweenTheJawTips re-derives these bounds from the meshes, so a
	// mesh regeneration cannot silently strand the TCP inside a finger:
	//
	//	static finger contact face  X = 7.67   -- the jaws meet at X = 6.8
	//	moving jaw contact face     X = 5.92
	//	contact pads span           Z = 94.4 .. 104.3, center 99.9
	//
	// A pure translation: the TCP keeps the tool frame's axes, so +Z remains the approach axis
	// the arm's goal-cloud orientation planning is built around.
	GripperTCPPose = spatialmath.NewPoseFromPoint(r3.Vector{X: 6.835, Y: 0, Z: 99.9})
)

// Gripper joint limits from the SO-101 URDF, in radians.
const (
	GripperJointMin = -0.174533
	GripperJointMax = 1.74533
)

// JawRadiansFromPct maps a gripper opening percentage (0 closed, 100 open) onto the jaw
// joint angle in radians, clamping out-of-range input. The single definition of the jaw
// mapping; both directions live here.
func JawRadiansFromPct(pct float64) float64 {
	p := math.Max(0, math.Min(100, pct)) / 100.0
	return GripperJointMin + p*(GripperJointMax-GripperJointMin)
}

// JawPctFromRadians is the inverse of JawRadiansFromPct. It saturates an out-of-range
// angle to 0/100 rather than rejecting it, so a caller needing limit enforcement must
// validate before converting.
func JawPctFromRadians(rad float64) float64 {
	pct := 100.0 * (rad - GripperJointMin) / (GripperJointMax - GripperJointMin)
	return math.Max(0, math.Min(100, pct))
}

// gripperStaticMeshNames returns the static body mesh names for a gripper variant. The
// follower's body is one piece; the leader's is the wrist-roll part (which connects to the
// arm) plus the handle attached to it.
func gripperStaticMeshNames(gripperType string) []string {
	if gripperType == LeaderGripper {
		return []string{"leader_wrist_roll", "leader_body"}
	}
	return []string{"follower_body"}
}

// gripperMovingMeshName returns the moving part's mesh name: the jaw for the follower, the
// trigger for the leader.
func gripperMovingMeshName(gripperType string) string {
	if gripperType == LeaderGripper {
		return "leader_trigger"
	}
	return "follower_jaw"
}

// BuildGripperMeshes returns the gripper's static body mesh plus the moving part
// (jaw for the follower, trigger for the leader) posed at jawAngle radians about
// the gripper joint, at the requested mesh detail level ("high" or "low"). It is
// split out from Geometries so it can be tested without a hardware controller.
func BuildGripperMeshes(gripperType, meshDetail string, jawAngle float64) ([]spatialmath.Geometry, error) {
	staticNames := gripperStaticMeshNames(gripperType)
	movingName := gripperMovingMeshName(gripperType)

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

// BuildGripperModel builds the gripper's kinematic model: a zero-DoF chain of static links
// carrying the gripper meshes, ending at GripperTCPPose.
//
//   - The model's leaf is what the frame system calls "the gripper", so GetPose and any motion
//     request targeting the gripper resolve to the TCP between the jaw tips, not the arm's
//     wrist where the meshes are anchored.
//   - The frame system reads an arm/gantry/gripper part's collision geometry from its kinematic
//     model and never calls Geometries() for those subtypes, so the meshes become planning
//     obstacles only by riding on this model. On viam-server >= 1.0.0 a gripper whose
//     Kinematics() errors is dropped from the frame system outright.
//
// Static on purpose, with the meshes frozen at the closed jaw pose (GripperJointMin): a DoF
// would become a variable the motion planner is free to drive, and freezing the jaw open would
// inflate the swept volume enough to block otherwise-valid plans. Geometries() still serves the
// live, articulating jaw for the 3D viewer. jawDoF selects the articulated topology; false is
// today's static model.
//
// Round-tripped through SVA JSON rather than assembled in memory: a component ships its
// kinematics to viam-server as ModelConfig().OriginalFile.Bytes, and a model without those bytes
// transmits as UNSPECIFIED (see TestGripperModelSurvivesSerialization). SVA JSON carries the
// meshes across the boundary in GeometryConfig.MeshData.
func BuildGripperModel(gripperType, meshDetail, name string, jawDoF bool) (referenceframe.Model, error) {
	meshes, err := BuildGripperMeshes(gripperType, meshDetail, GripperJointMin)
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
	tcp := GripperTCPPose
	if gripperType == LeaderGripper {
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

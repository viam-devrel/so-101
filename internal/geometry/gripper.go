package geometry

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
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

// buildGripperBodyMeshes returns the static body meshes, posed in the gripper component's
// (tool) frame.
func buildGripperBodyMeshes(gripperType, meshDetail string) ([]spatialmath.Geometry, error) {
	bodyPose := spatialmath.Compose(toolFromGripperLink, gripperBodyPose)
	names := gripperStaticMeshNames(gripperType)
	geoms := make([]spatialmath.Geometry, 0, len(names))
	for i, name := range names {
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
	return geoms, nil
}

// buildGripperMovingMesh returns the moving part's mesh at the given pose. Callers compute that
// pose by different routes (by hand here, from the jaw joint in appendJawBranch), so both must
// agree on where the mesh actually ends up.
func buildGripperMovingMesh(gripperType, meshDetail string, pose spatialmath.Pose) (spatialmath.Geometry, error) {
	ply, err := gripperMeshPLY(meshDetail, gripperMovingMeshName(gripperType))
	if err != nil {
		return nil, err
	}
	return spatialmath.NewMeshFromProto(pose,
		&commonpb.Mesh{ContentType: "ply", Mesh: ply}, "gripper_moving")
}

// movingMeshPose is the moving part's pose in the tool frame at jawAngle radians: joint origin,
// rotation about the joint's Z, then the mesh offset. The single definition both BuildGripperMeshes
// and the frozen (jawDoF: false) branch of BuildGripperModel use.
func movingMeshPose(jawAngle float64) spatialmath.Pose {
	return spatialmath.Compose(toolFromGripperLink, spatialmath.Compose(
		spatialmath.Compose(gripperJointPose,
			spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Yaw: jawAngle})),
		gripperMovingVisualPose))
}

// BuildGripperMeshes returns the gripper's static body mesh plus the moving part
// (jaw for the follower, trigger for the leader) posed at jawAngle radians about
// the gripper joint, at the requested mesh detail level ("high" or "low"). It is
// split out from Geometries so it can be tested without a hardware controller.
func BuildGripperMeshes(gripperType, meshDetail string, jawAngle float64) ([]spatialmath.Geometry, error) {
	geoms, err := buildGripperBodyMeshes(gripperType, meshDetail)
	if err != nil {
		return nil, err
	}

	moving, err := buildGripperMovingMesh(gripperType, meshDetail, movingMeshPose(jawAngle))
	if err != nil {
		return nil, err
	}

	return append(geoms, moving), nil
}

// gripperTCPFrame is the leaf link of the gripper's kinematic model -- the frame the whole
// model resolves to, and therefore the pose viam-server reports for the gripper component.
const gripperTCPFrame = "tcp"

// Frame names for the articulated (jawDoF: true) jaw branch. The mesh link itself is named
// from moving.Label() in appendJawBranch, not a const here, so it matches the 0-DoF path's
// mesh.Label()-derived name by construction rather than by two literals staying in sync.
const (
	gripperJawMountFrame = "jaw_mount"
	gripperJawJointFrame = "jaw_joint"
)

// appendJawBranch hangs the revolute jaw branch off parent:
//
//	parent -> jaw_mount (static) -> jaw_joint (revolute, +Z) -> gripper_moving (mesh)
//
// jaw_mount carries the joint origin already corrected into the tool frame, so the joint's
// rotation axis is plain local +Z -- matching the EulerAngles{Yaw: theta} that
// BuildGripperMeshes applies at the same point in its chain. gripper_moving's mesh lives in
// the GEOMETRY offset, which SVA measures from the link's PARENT frame (the joint's output),
// so the mesh swings with the joint.
func appendJawBranch(cfg *referenceframe.ModelConfigJSON, gripperType, meshDetail, parent string) error {
	mountPose := spatialmath.Compose(toolFromGripperLink, gripperJointPose)
	mountOrient, err := spatialmath.NewOrientationConfig(mountPose.Orientation())
	if err != nil {
		return fmt.Errorf("encoding the jaw mount orientation: %w", err)
	}
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID:          gripperJawMountFrame,
		Parent:      parent,
		Translation: mountPose.Point(),
		Orientation: mountOrient,
	})

	moving, err := buildGripperMovingMesh(gripperType, meshDetail, gripperMovingVisualPose)
	if err != nil {
		return err
	}
	movingCfg, err := spatialmath.NewGeometryConfig(moving)
	if err != nil {
		return fmt.Errorf("converting the jaw mesh to a geometry config: %w", err)
	}

	// JointConfig Min/Max are DEGREES; frame_json.go converts them back to radians.
	cfg.Joints = append(cfg.Joints, referenceframe.JointConfig{
		ID:     gripperJawJointFrame,
		Type:   "revolute",
		Parent: gripperJawMountFrame,
		Axis:   spatialmath.AxisConfig{X: 0, Y: 0, Z: 1},
		Min:    utils.RadToDeg(GripperJointMin),
		Max:    utils.RadToDeg(GripperJointMax),
	})
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID:       moving.Label(),
		Parent:   gripperJawJointFrame,
		Geometry: movingCfg,
	})
	return nil
}

// BuildGripperModel builds the gripper's kinematic model, ending at GripperTCPPose.
//
//   - The model's leaf is what the frame system calls "the gripper", so GetPose and any motion
//     request targeting the gripper resolve to the TCP between the jaw tips, not the arm's
//     wrist where the meshes are anchored.
//   - The frame system reads an arm/gantry/gripper part's collision geometry from its kinematic
//     model and never calls Geometries() for those subtypes, so the meshes become planning
//     obstacles only by riding on this model. On viam-server >= 1.0.0 a gripper whose
//     Kinematics() errors is dropped from the frame system outright.
//
// jawDoF selects the topology:
//
//   - false (the default): a zero-DoF chain of static links, meshes frozen at the closed jaw
//     pose (GripperJointMin). A DoF here would become a variable the motion planner is free to
//     drive, and freezing the jaw open would inflate the swept volume enough to block otherwise-
//     valid plans. tcp's parent is the moving mesh's link.
//   - true: a branching 1-DoF model -- the jaw mesh hangs off a revolute joint, but the TCP leaf
//     is reached by a separate static branch, so the model's Transform (the TCP) never moves as
//     the jaw articulates. This is the topology the jaw-DoF experiment measures. tcp's parent is
//     the last static body mesh's link instead, since the moving mesh now hangs off the jaw
//     branch; poses are identical either way (every body-mesh link has an identity transform),
//     but the frame-system node tree the 3D viewer draws differs in shape between the two modes.
//
// Geometries() always serves the live, articulating jaw to the 3D viewer regardless of jawDoF.
//
// Round-tripped through SVA JSON rather than assembled in memory: a component ships its
// kinematics to viam-server as ModelConfig().OriginalFile.Bytes, and a model without those bytes
// transmits as UNSPECIFIED (see TestGripperModelSurvivesSerialization). SVA JSON carries the
// meshes across the boundary in GeometryConfig.MeshData.
func BuildGripperModel(gripperType, meshDetail, name string, jawDoF bool) (referenceframe.Model, error) {
	cfg := &referenceframe.ModelConfigJSON{Name: name, KinParamType: "SVA"}

	bodies, err := buildGripperBodyMeshes(gripperType, meshDetail)
	if err != nil {
		return nil, err
	}
	if !jawDoF {
		// Frozen closed, in the chain, exactly as before.
		frozen, err := buildGripperMovingMesh(gripperType, meshDetail, movingMeshPose(GripperJointMin))
		if err != nil {
			return nil, err
		}
		bodies = append(bodies, frozen)
	}

	// Each mesh gets its own link because a link carries at most one geometry. All transforms
	// are identity, so the full mesh pose lives in the geometry offset. bodyLeaf is the fork
	// point: both the tcp link below and appendJawBranch's jaw_mount hang off it directly, so
	// jawDoF only ever adds a branch here -- it never reroutes the static chain.
	bodyLeaf := referenceframe.World
	for _, mesh := range bodies {
		geomCfg, err := spatialmath.NewGeometryConfig(mesh)
		if err != nil {
			return nil, fmt.Errorf("converting gripper mesh %q to a geometry config: %w", mesh.Label(), err)
		}
		cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
			ID: mesh.Label(), Parent: bodyLeaf, Geometry: geomCfg,
		})
		bodyLeaf = mesh.Label()
	}

	// The leader gripper is a hand-held trigger, not a pair of jaws, so it has no grasp point:
	// its frame stays at the mount.
	tcp := GripperTCPPose
	if gripperType == LeaderGripper {
		tcp = spatialmath.NewZeroPose()
	}
	cfg.Links = append(cfg.Links, referenceframe.LinkConfig{
		ID: gripperTCPFrame, Parent: bodyLeaf, Translation: tcp.Point(),
	})

	if jawDoF {
		// Two leaves (tcp and the jaw mesh) are legal only with output_frames, which v1.0.0
		// accepts for exactly one entry.
		if err := appendJawBranch(cfg, gripperType, meshDetail, bodyLeaf); err != nil {
			return nil, err
		}
		cfg.OutputFrames = []string{gripperTCPFrame}
	}

	jsonBytes, err := json.Marshal(cfg)
	if err != nil {
		return nil, fmt.Errorf("serializing the %s gripper model: %w", gripperType, err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
}

package so_arm

import (
	_ "embed"

	commonpb "go.viam.com/api/common/v1"
)

// SO-101 link meshes. These are decimated, Draco-compressed glTF binaries derived
// from the meshes originally authored for the rdk fake-arm SO-101 model. so101ArmModels
// keys them by the active kinematic source's link frame names (so101.json or
// arm/so101.urdf) so the 3D scene viewer can attach each mesh to the correct moving link.

//go:embed meshes/so101/base.glb
var so101BaseGLB []byte

//go:embed meshes/so101/shoulder.glb
var so101ShoulderGLB []byte

//go:embed meshes/so101/upper_arm.glb
var so101UpperArmGLB []byte

//go:embed meshes/so101/lower_arm.glb
var so101LowerArmGLB []byte

//go:embed meshes/so101/wrist.glb
var so101WristGLB []byte

// so101EEFrameGLB is a colored XYZ coordinate-frame marker (see meshes/gen_ee_frame.py).
// Served at the "tool" frame when an arm's visualize_ee_frame attribute is enabled.
//
//go:embed meshes/so101/ee_frame.glb
var so101EEFrameGLB []byte

// SO-101 gripper meshes (ASCII PLY, see meshes/gen_gripper_meshes.py). The gripper
// component serves these via Geometries() as spatialmath.Mesh geometries: a static
// body and a moving part (jaw for the follower, trigger for the leader).

//go:embed meshes/gripper/follower_body.ply
var so101FollowerBodyPLY []byte

//go:embed meshes/gripper/follower_jaw.ply
var so101FollowerJawPLY []byte

//go:embed meshes/gripper/leader_wrist_roll.ply
var so101LeaderWristRollPLY []byte

//go:embed meshes/gripper/leader_body.ply
var so101LeaderBodyPLY []byte

//go:embed meshes/gripper/leader_trigger.ply
var so101LeaderTriggerPLY []byte

const gltfBinary = "model/gltf-binary"

// so101EEFrameMeshKey is the frame the EE-frame marker mounts on. It works in both kinematic
// modes: makeSO101ModelURDF (arm.go) grafts so101.json's exact "tool" LinkConfig onto the
// wrist-roll joint's output of the URDF model too, so the URDF-sourced model has a "tool" leaf
// coincident with so101.json's (see TestURDFvsJSONFrameAlignment) and visualize_ee_frame gives
// it the same placeholder geometry under use_urdf as it does on the JSON path.
const so101EEFrameMeshKey = "tool"

// so101ArmMeshParts maps each embedded link GLB to the frame name it is keyed by under
// each kinematic source. The JSON (so101.json) and URDF (arm/so101.urdf) link frames
// coincide geometrically, so the same GLB serves both — only the key differs.
var so101ArmMeshParts = []struct {
	glb                []byte
	jsonName, urdfName string
}{
	{so101BaseGLB, "base", "base_link"},
	{so101ShoulderGLB, "shoulder", "shoulder_link"},
	{so101UpperArmGLB, "upper_arm", "upper_arm_link"},
	{so101LowerArmGLB, "lower_arm", "lower_arm_link"},
	{so101WristGLB, "wrist", "wrist_link"},
}

// so101EEFrameMesh returns the colored end-effector coordinate-frame marker mesh.
func so101EEFrameMesh() *commonpb.Mesh {
	return &commonpb.Mesh{ContentType: gltfBinary, Mesh: so101EEFrameGLB}
}

// so101ArmModels returns the SO-101 link meshes for the 3D scene viewer, keyed by the
// active kinematic source's frame names (URDF link names when useURDF, so101.json names
// otherwise). Adds the colored EE coordinate-frame marker at the "tool" frame when
// visualizeEEFrame is set. Shared by the hardware and simulated arm so the two
// Get3DModels implementations never drift.
func so101ArmModels(visualizeEEFrame, useURDF bool) map[string]*commonpb.Mesh {
	models := make(map[string]*commonpb.Mesh, len(so101ArmMeshParts)+1)
	for _, p := range so101ArmMeshParts {
		name := p.jsonName
		if useURDF {
			name = p.urdfName
		}
		models[name] = &commonpb.Mesh{ContentType: gltfBinary, Mesh: p.glb}
	}
	if visualizeEEFrame {
		models[so101EEFrameMeshKey] = so101EEFrameMesh()
	}
	return models
}

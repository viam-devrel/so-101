package geometry

import (
	"embed"

	commonpb "go.viam.com/api/common/v1"
)

// SO-101 link meshes. These are decimated, Draco-compressed glTF binaries derived
// from the meshes originally authored for the rdk fake-arm SO-101 model. armMeshParts
// keys them by the active kinematic source's link frame names (so101.json or
// assets/urdf/so101.urdf) so the 3D scene viewer can attach each mesh to the correct moving link.

//go:embed meshes/so101/base.glb
var baseGLB []byte

//go:embed meshes/so101/shoulder.glb
var shoulderGLB []byte

//go:embed meshes/so101/upper_arm.glb
var upperArmGLB []byte

//go:embed meshes/so101/lower_arm.glb
var lowerArmGLB []byte

//go:embed meshes/so101/wrist.glb
var wristGLB []byte

// eeFrameGLB is a colored XYZ coordinate-frame marker (see tools/gen_ee_frame.py).
// Served at the "tool" frame when an arm's visualize_ee_frame attribute is enabled.
//
//go:embed meshes/so101/ee_frame.glb
var eeFrameGLB []byte

// SO-101 gripper meshes (ASCII PLY, see tools/gen_gripper_meshes.py). The gripper
// component serves these via Geometries() as spatialmath.Mesh geometries: a static
// body and a moving part (jaw for the follower, trigger for the leader). Each mesh
// is embedded at two detail levels -- meshes/gripper/{high,low}/ -- chosen by the
// gripper's mesh_detail attribute.
//
//go:embed meshes/gripper
var gripperMeshFS embed.FS

// gripperMeshPLY returns the embedded PLY bytes for a gripper mesh ("follower_body",
// "leader_trigger", ...) at the given detail level ("high" or "low").
func gripperMeshPLY(detail, name string) ([]byte, error) {
	return gripperMeshFS.ReadFile("meshes/gripper/" + detail + "/" + name + ".ply")
}

// gltfBinary is the MIME content type used for the embedded arm-link GLB meshes.
const gltfBinary = "model/gltf-binary"

// eeFrameMeshKey is the frame the EE-frame marker mounts on. It works in both kinematic
// modes: assets/urdf/so101.urdf ends at a "tool" leaf at so101.json's exact TCP (see
// TestURDFvsJSONFrameAlignment), so the URDF-sourced model has a "tool" frame coincident with
// so101.json's, and visualize_ee_frame gives it the same placeholder geometry either way.
const eeFrameMeshKey = "tool"

// armMeshParts maps each embedded link GLB to the frame it mounts on. assets/urdf/so101.urdf uses
// these same so101.json link names, so one key set serves both kinematic sources -- the same GLB
// attaches to the same frame regardless of use_urdf.
var armMeshParts = []struct {
	glb  []byte
	name string
}{
	{baseGLB, "base"},
	{shoulderGLB, "shoulder"},
	{upperArmGLB, "upper_arm"},
	{lowerArmGLB, "lower_arm"},
	{wristGLB, "wrist"},
}

// eeFrameMesh returns the colored end-effector coordinate-frame marker mesh.
func eeFrameMesh() *commonpb.Mesh {
	return &commonpb.Mesh{ContentType: gltfBinary, Mesh: eeFrameGLB}
}

// ArmMeshes returns the SO-101 link meshes for the 3D scene viewer, keyed by the arm's
// frame names (identical in JSON and URDF modes). Adds the colored EE coordinate-frame marker
// at the "tool" frame when visualizeEEFrame is set. Shared by the hardware and simulated arm so
// the two Get3DModels implementations never drift.
func ArmMeshes(visualizeEEFrame bool) map[string]*commonpb.Mesh {
	models := make(map[string]*commonpb.Mesh, len(armMeshParts)+1)
	for _, p := range armMeshParts {
		models[p.name] = &commonpb.Mesh{ContentType: gltfBinary, Mesh: p.glb}
	}
	if visualizeEEFrame {
		models[eeFrameMeshKey] = eeFrameMesh()
	}
	return models
}

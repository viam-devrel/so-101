package so_arm

import (
	"embed"

	commonpb "go.viam.com/api/common/v1"
)

// SO-101 link meshes. These are decimated, Draco-compressed glTF binaries derived
// from the meshes originally authored for the rdk fake-arm SO-101 model. so101ArmModels
// keys them by the active kinematic source's link frame names (so101.json or
// assets/urdf/so101.urdf) so the 3D scene viewer can attach each mesh to the correct moving link.

//go:embed internal/geometry/meshes/so101/base.glb
var so101BaseGLB []byte

//go:embed internal/geometry/meshes/so101/shoulder.glb
var so101ShoulderGLB []byte

//go:embed internal/geometry/meshes/so101/upper_arm.glb
var so101UpperArmGLB []byte

//go:embed internal/geometry/meshes/so101/lower_arm.glb
var so101LowerArmGLB []byte

//go:embed internal/geometry/meshes/so101/wrist.glb
var so101WristGLB []byte

// so101EEFrameGLB is a colored XYZ coordinate-frame marker (see tools/gen_ee_frame.py).
// Served at the "tool" frame when an arm's visualize_ee_frame attribute is enabled.
//
//go:embed internal/geometry/meshes/so101/ee_frame.glb
var so101EEFrameGLB []byte

// SO-101 gripper meshes (ASCII PLY, see tools/gen_gripper_meshes.py). The gripper
// component serves these via Geometries() as spatialmath.Mesh geometries: a static
// body and a moving part (jaw for the follower, trigger for the leader). Each mesh
// is embedded at two detail levels -- internal/geometry/meshes/gripper/{high,low}/ -- chosen by the
// gripper's mesh_detail attribute.
//
//go:embed internal/geometry/meshes/gripper
var gripperMeshFS embed.FS

// gripperMeshPLY returns the embedded PLY bytes for a gripper mesh ("follower_body",
// "leader_trigger", ...) at the given detail level ("high" or "low").
func gripperMeshPLY(detail, name string) ([]byte, error) {
	return gripperMeshFS.ReadFile("internal/geometry/meshes/gripper/" + detail + "/" + name + ".ply")
}

const gltfBinary = "model/gltf-binary"

// so101EEFrameMeshKey is the frame the EE-frame marker mounts on. It works in both kinematic
// modes: assets/urdf/so101.urdf ends at a "tool" leaf at so101.json's exact TCP (see
// TestURDFvsJSONFrameAlignment), so the URDF-sourced model has a "tool" frame coincident with
// so101.json's, and visualize_ee_frame gives it the same placeholder geometry either way.
const so101EEFrameMeshKey = "tool"

// so101ArmMeshParts maps each embedded link GLB to the frame it mounts on. assets/urdf/so101.urdf uses
// these same so101.json link names, so one key set serves both kinematic sources — the same GLB
// attaches to the same frame regardless of use_urdf.
var so101ArmMeshParts = []struct {
	glb  []byte
	name string
}{
	{so101BaseGLB, "base"},
	{so101ShoulderGLB, "shoulder"},
	{so101UpperArmGLB, "upper_arm"},
	{so101LowerArmGLB, "lower_arm"},
	{so101WristGLB, "wrist"},
}

// so101EEFrameMesh returns the colored end-effector coordinate-frame marker mesh.
func so101EEFrameMesh() *commonpb.Mesh {
	return &commonpb.Mesh{ContentType: gltfBinary, Mesh: so101EEFrameGLB}
}

// so101ArmModels returns the SO-101 link meshes for the 3D scene viewer, keyed by the arm's
// frame names (identical in JSON and URDF modes). Adds the colored EE coordinate-frame marker
// at the "tool" frame when visualizeEEFrame is set. Shared by the hardware and simulated arm so
// the two Get3DModels implementations never drift.
func so101ArmModels(visualizeEEFrame bool) map[string]*commonpb.Mesh {
	models := make(map[string]*commonpb.Mesh, len(so101ArmMeshParts)+1)
	for _, p := range so101ArmMeshParts {
		models[p.name] = &commonpb.Mesh{ContentType: gltfBinary, Mesh: p.glb}
	}
	if visualizeEEFrame {
		models[so101EEFrameMeshKey] = so101EEFrameMesh()
	}
	return models
}

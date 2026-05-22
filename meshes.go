package so_arm

import (
	"embed"

	commonpb "go.viam.com/api/common/v1"
)

// SO-101 link meshes. These are decimated, Draco-compressed glTF binaries derived
// from the meshes originally authored for the rdk fake-arm SO-101 model. The keys
// returned by so101Meshes match the link ids in so101.json so the 3D scene viewer
// can attach each mesh to the correct moving link.

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

const gltfBinary = "model/gltf-binary"

// so101EEFrameMeshKey is the so101.json frame the EE-frame marker mounts on.
const so101EEFrameMeshKey = "tool"

// so101Meshes returns the 3D mesh for each SO-101 link, keyed by the link id used
// in so101.json. so101.json's "tool" link has no geometry and therefore no mesh.
func so101Meshes() map[string]*commonpb.Mesh {
	return map[string]*commonpb.Mesh{
		"base":      {ContentType: gltfBinary, Mesh: so101BaseGLB},
		"shoulder":  {ContentType: gltfBinary, Mesh: so101ShoulderGLB},
		"upper_arm": {ContentType: gltfBinary, Mesh: so101UpperArmGLB},
		"lower_arm": {ContentType: gltfBinary, Mesh: so101LowerArmGLB},
		"wrist":     {ContentType: gltfBinary, Mesh: so101WristGLB},
	}
}

// so101EEFrameMesh returns the colored end-effector coordinate-frame marker mesh.
func so101EEFrameMesh() *commonpb.Mesh {
	return &commonpb.Mesh{ContentType: gltfBinary, Mesh: so101EEFrameGLB}
}

package so_arm

import (
	_ "embed"

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

// so101Meshes returns the 3D mesh for each SO-101 link, keyed by the link id used
// in so101.json. so101.json's "tool" link has no geometry and therefore no mesh.
func so101Meshes() map[string]*commonpb.Mesh {
	const gltfBinary = "model/gltf-binary"
	return map[string]*commonpb.Mesh{
		"base":      {ContentType: gltfBinary, Mesh: so101BaseGLB},
		"shoulder":  {ContentType: gltfBinary, Mesh: so101ShoulderGLB},
		"upper_arm": {ContentType: gltfBinary, Mesh: so101UpperArmGLB},
		"lower_arm": {ContentType: gltfBinary, Mesh: so101LowerArmGLB},
		"wrist":     {ContentType: gltfBinary, Mesh: so101WristGLB},
	}
}

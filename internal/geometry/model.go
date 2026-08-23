// Package geometry holds the SO-101 kinematic model builders and mesh assets shared by the
// hardware and simulated arm/gripper components: the embedded so101.json kinematics, the
// bundled assets/urdf/so101.urdf loader, the arm and gripper collision/visual meshes, and the
// gripper's kinematic model. It has no dependency on the root module package.
package geometry

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/pkg/errors"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// so101.json's SVA kinematics, embedded so the arm model builds with no filesystem
// access. Derived from TheRobotStudio/SO-ARM100's URDF; its "tool" link is the
// end-effector (TCP) frame.
//
//go:embed so101.json
var modelJSON []byte

// ComputeOOBPosition takes a frame and a slice of Inputs and returns the cartesian position of
// the frame after transforming it by the given inputs even if the inputs given would violate the
// Limits of the frame. This is performed statelessly without changing any data.
//
// This replaces referenceframe.ComputeOOBPosition, which was removed upstream. Callers rely on
// being able to compute a pose for out-of-bounds joint positions (e.g. reporting EndPosition
// while a joint is slightly past its calibrated limit). rdk v0.123's Frame.Transform early-returns
// at the first out-of-bounds joint, yielding a pose truncated at that joint (and everything
// downstream), so inputs are clamped into the joint limits first to always compose the full chain.
func ComputeOOBPosition(frame referenceframe.Frame, inputs []referenceframe.Input) (spatialmath.Pose, error) {
	if inputs == nil {
		return nil, errors.New("cannot compute position for nil joints")
	}
	if frame == nil {
		return nil, errors.New("cannot compute position for nil frame")
	}

	limits := frame.DoF()
	safe := make([]referenceframe.Input, len(inputs))
	for i, v := range inputs {
		if i < len(limits) {
			v = math.Max(limits[i].Min, math.Min(limits[i].Max, v))
		}
		safe[i] = v
	}
	return frame.Transform(safe)
}

// ArmModelJSON builds the SO-101 kinematic model from the embedded so101.json.
func ArmModelJSON(resourceName string) (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     modelJSON,
			Extension: "json",
		},
	}
	err := json.Unmarshal(modelJSON, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	return m.ParseConfig(resourceName)
}

// eeMarkerGeometry returns the small placeholder box given to the "tool" link so the 3D
// viewer draws that frame (a Get3DModels mesh keyed to a geometry-less frame is dropped).
// The colored EE-frame mesh from Get3DModels replaces it visually. Shared by the JSON and
// URDF model builders so the two visualize_ee_frame paths use the identical placeholder.
func eeMarkerGeometry() *spatialmath.GeometryConfig {
	return &spatialmath.GeometryConfig{Type: spatialmath.BoxType, X: 10, Y: 10, Z: 10}
}

// armModelJSONWithEEMarker builds the SO-101 kinematics with a small placeholder
// geometry on the otherwise-empty "tool" link. so101.json's "tool" link has no geometry,
// so the 3D viewer never draws it -- and a Get3DModels mesh keyed to a frame the viewer
// does not draw is dropped. Giving "tool" a geometry makes the viewer draw the link, so
// the colored EE-frame mesh served by Get3DModels renders there. so101.json is untouched;
// this is used only when an arm's visualize_ee_frame attribute is enabled.
func armModelJSONWithEEMarker(resourceName string) (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     modelJSON,
			Extension: "json",
		},
	}
	if err := json.Unmarshal(modelJSON, m); err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}
	for i := range m.Links {
		if m.Links[i].ID == eeFrameMeshKey {
			m.Links[i].Geometry = eeMarkerGeometry()
		}
	}
	return m.ParseConfig(resourceName)
}

// ArmModel builds the SO-101 kinematic model from either the embedded so101.json
// (default) or the bundled assets/urdf/so101.urdf (when useURDF). Shared by the hardware and
// simulated arms. On the JSON path, when visualizeEE is set the "tool" link gets a placeholder
// geometry so the viewer draws the EE marker (see armModelJSONWithEEMarker). VisualizeEEFrame's
// placeholder handling under URDF mode is deferred to a later task (frame-alignment), so it is
// not applied here.
func ArmModel(useURDF bool, ratios []float64, visualizeEE bool, name string) (referenceframe.Model, error) {
	if !useURDF {
		if visualizeEE {
			return armModelJSONWithEEMarker(name)
		}
		return ArmModelJSON(name)
	}
	return armModelURDF(ratios, visualizeEE, name)
}

// so101.urdf carries one merged collision mesh per arm link (base/shoulder/upper_arm/lower_arm/
// wrist, 5 total, in document order). The RDK URDF parser assigns mesh_decimation_ratios per mesh
// in that order, so the default fills one ratio per mesh. 0.9 is a light default (keep ~90% of
// triangles) that trims the dense merged meshes without the sliver artifacts more aggressive
// ratios produced (see tools/gen_collision_meshes.py).
const (
	numCollisionMeshes    = 5
	defaultMeshDecimation = 0.9
)

// armModelURDF builds the SO-101 model from assets/urdf/so101.urdf. The URDF is authored so its
// raw form is already a drop-in for so101.json: so101.json's frame names (base, shoulder,
// upper_arm, lower_arm, wrist; joints 1-5) and its "tool" TCP (joint-5 output + 180deg flip),
// with only the collision geometry differing (per-link meshes vs so101.json's primitives). That
// correctness must live in the file itself, not an in-memory patch: a component ships its
// kinematics to viam-server as ModelConfig().OriginalFile.Bytes -- this raw URDF -- so in-memory
// edits are lost across the module gRPC boundary (see the assets/urdf/so101.urdf header and
// TestURDFModelSurvivesSerialization). Returning the parsed URDF lets it transmit AS URDF, with
// meshes sent separately: a much smaller payload than embedding them in SVA JSON.
//
// visualize_ee_frame is the one exception. It needs a placeholder geometry on the "tool" link so
// the 3D viewer draws that frame and Get3DModels' EE-marker mesh renders there. Baking that box
// into the URDF would add a spurious collision volume for everyone, so it is added in memory and
// the model re-serialized to SVA JSON, which (unlike the URDF path) carries in-memory geometry
// and the embedded meshes across the boundary. That payload is larger, hence marker-only.
func armModelURDF(ratios []float64, visualizeEE bool, name string) (referenceframe.Model, error) {
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		return nil, errors.New("use_urdf is set but VIAM_MODULE_ROOT is empty")
	}
	path := filepath.Join(root, "assets", "urdf", "so101.urdf")
	// The RDK URDF parser assigns mesh_decimation_ratios per collision mesh in document order
	// (base/shoulder/upper_arm/lower_arm/wrist), so an empty or short slice would leave trailing
	// meshes undecimated. When the user supplies none, default every mesh to defaultMeshDecimation.
	if len(ratios) == 0 {
		ratios = make([]float64, numCollisionMeshes)
		for i := range ratios {
			ratios[i] = defaultMeshDecimation
		}
	}
	model, err := referenceframe.ParseModelXMLFile(path, name, ratios)
	if err != nil {
		return nil, fmt.Errorf("parsing URDF %q: %w", path, err)
	}
	if !visualizeEE {
		return model, nil
	}

	// visualize_ee_frame: give "tool" a placeholder box and rebuild from SVA JSON so the
	// geometry survives the gRPC boundary (a raw-URDF transmission would drop it).
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("parsing URDF %q: expected *referenceframe.SimpleModel, got %T", path, model)
	}
	mc := sm.ModelConfig()
	for i := range mc.Links {
		if mc.Links[i].ID == eeFrameMeshKey {
			mc.Links[i].Geometry = eeMarkerGeometry()
		}
	}
	mc.OriginalFile = nil
	jsonBytes, err := json.Marshal(mc)
	if err != nil {
		return nil, fmt.Errorf("serializing URDF model %q for visualize_ee_frame: %w", path, err)
	}
	return referenceframe.UnmarshalModelJSON(jsonBytes, name)
}

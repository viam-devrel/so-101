package geometry

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/stretchr/testify/require"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"so_arm/internal/testfake"
)

// aabbExtents returns the axis-aligned bounding-box side lengths (world frame) of a geometry's
// point set, sorted ascending so the comparison is orientation-agnostic.
func aabbExtents(g spatialmath.Geometry) r3.Vector {
	pts := g.ToPoints(0)
	if len(pts) == 0 {
		return r3.Vector{}
	}
	lo := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vector{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, p := range pts {
		lo.X, lo.Y, lo.Z = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z)
		hi.X, hi.Y, hi.Z = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z)
	}
	d := []float64{hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z}
	// simple 3-element sort ascending
	if d[0] > d[1] {
		d[0], d[1] = d[1], d[0]
	}
	if d[1] > d[2] {
		d[1], d[2] = d[2], d[1]
	}
	if d[0] > d[1] {
		d[0], d[1] = d[1], d[0]
	}
	return r3.Vector{X: d[0], Y: d[1], Z: d[2]}
}

// TestURDFCollisionCoversWholeLink guards against the "only Collision[0] survives" defect: rdk's
// URDF parser keeps a single <collision> element per link (model_urdf.go), so a link authored as a
// multi-part assembly (as the upstream SO-ARM100 meshes are) would otherwise contribute only one
// sub-part's mesh -- a fragment offset within the link. assets/urdf/so101.urdf therefore ships ONE merged
// collision mesh per link (tools/gen_collision_meshes.py). This test asserts each URDF-mode link's
// collision geometry spans essentially the same extent as the JSON-mode primitive for that link
// (the JSON boxes were fit to the same full-link bounds), which a lone sub-part mesh cannot do.
func TestURDFCollisionCoversWholeLink(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())

	jsonModel, err := ArmModelJSON("so101")
	require.NoError(t, err)
	urdfModel, err := ArmModel(true, nil, false, "so101")
	require.NoError(t, err)

	zero := make([]referenceframe.Input, 5)
	jsonGeoms, err := jsonModel.Geometries(zero)
	require.NoError(t, err)
	urdfGeoms, err := urdfModel.Geometries(zero)
	require.NoError(t, err)

	byLabelExtent := func(gg *referenceframe.GeometriesInFrame) map[string]r3.Vector {
		out := map[string]r3.Vector{}
		for _, g := range gg.Geometries() {
			out[g.Label()] = aabbExtents(g)
		}
		return out
	}
	jExt := byLabelExtent(jsonGeoms)
	uExt := byLabelExtent(urdfGeoms)

	// JSON label <-> URDF label per link. The URDF model renames its frames to so101.json's
	// names (assets/urdf/so101.urdf uses so101.json's link names), so the geometry labels are identical between modes.
	pairs := [][2]string{
		{"so101:base", "so101:base"},
		{"so101:shoulder", "so101:shoulder"},
		{"so101:upper_arm", "so101:upper_arm"},
		{"so101:lower_arm", "so101:lower_arm"},
		{"so101:wrist", "so101:wrist"},
	}

	for _, pr := range pairs {
		j, ok := jExt[pr[0]]
		require.Truef(t, ok, "missing JSON geometry %q; have %v", pr[0], keysOf(jExt))
		u, ok := uExt[pr[1]]
		require.Truef(t, ok, "missing URDF geometry %q; have %v", pr[1], keysOf(uExt))

		// Compare the largest (dominant) extent of each link's collision AABB. A lone sub-part
		// mesh (the pre-merge bug) covers only a fraction of the link, so its dominant extent is
		// far short of the JSON primitive's. Require the URDF mesh to reach >=70% of the JSON
		// extent (merged meshes land within a few mm, i.e. ~100%).
		jMax, uMax := j.Z, u.Z
		ratio := uMax / jMax
		t.Logf("%s: JSON maxExtent=%.1fmm  URDF maxExtent=%.1fmm  ratio=%.2f", pr[0], jMax, uMax, ratio)
		require.Greaterf(t, ratio, 0.70,
			"%s URDF collision geometry only spans %.1fmm vs JSON %.1fmm (ratio %.2f) -- "+
				"looks like a single sub-part mesh survived instead of the merged full-link mesh",
			pr[1], uMax, jMax, ratio)
	}
}

func keysOf(m map[string]r3.Vector) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}

// TestURDFGeometryPoseMatchesJSON guards GLB visual-mesh placement. The 3D scene viewer places
// each Get3DModels GLB at the link's *geometry* pose (not the bare frame origin), and the same
// GLB serves both kinematic sources (the geometry package's armMeshParts). so101.json's per-link geometry is a
// primitive box with a translation offset, so the URDF collision mesh must share that exact
// geometry pose or the GLBs scatter by the box offsets (tens of mm/link) under use_urdf. The
// merged meshes are generated relative to the box pose and assets/urdf/so101.urdf carries the matching
// <collision><origin> (see tools/gen_collision_meshes.py, URDF_TO_JSON_LINK). This asserts the two
// sources agree per link; the collision *shape* is unchanged (TestURDFCollisionCoversWholeLink),
// only its reported pose is aligned to so101.json's.
func TestURDFGeometryPoseMatchesJSON(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", testfake.RepoRoot())
	jsonModel, err := ArmModelJSON("so101")
	require.NoError(t, err)
	urdfModel, err := ArmModel(true, nil, false, "so101")
	require.NoError(t, err)

	zero := make([]referenceframe.Input, 5)
	geomPoses := func(m referenceframe.Model) map[string]spatialmath.Pose {
		gif, err := m.Geometries(zero)
		require.NoError(t, err)
		out := map[string]spatialmath.Pose{}
		for _, g := range gif.Geometries() {
			out[g.Label()] = g.Pose()
		}
		return out
	}
	jp := geomPoses(jsonModel)
	up := geomPoses(urdfModel)
	for _, label := range []string{"so101:base", "so101:shoulder", "so101:upper_arm", "so101:lower_arm", "so101:wrist"} {
		j, ok := jp[label]
		require.Truef(t, ok, "missing JSON geometry %q", label)
		u, ok := up[label]
		require.Truef(t, ok, "missing URDF geometry %q", label)
		posD := j.Point().Sub(u.Point()).Norm()
		require.Lessf(t, posD, 1.0,
			"%s geometry pose differs by %.2fmm between JSON and URDF -- the shared Get3DModels GLB "+
				"would render offset by that much under use_urdf (viewer places it at the geometry pose)",
			label, posD)
	}
}

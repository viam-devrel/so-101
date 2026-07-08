#!/usr/bin/env python3
"""Pre-decimate the SO-101 arm collision meshes in arm/meshes/ in place.

Why: arm/so101.urdf's collision meshes (vendored from TheRobotStudio/SO-ARM100,
Apache-2.0) run up to ~54k triangles each (~294k total). rdk's runtime mesh
decimation (ParseModelXMLFile with non-zero mesh_decimation_ratios) hangs on
meshes this dense, so runtime decimation is kept OFF (ratios = 0) and these
meshes are decimated offline instead. They're used only as collision geometry
for motion planning, so a few thousand triangles per link is plenty.

Each *.stl in arm/meshes/ is quadric-decimated to a target of ~2000 triangles
(already-small meshes are left alone), overwritten in place as binary STL, in
the same units (meters) -- no rescaling.

Requires trimesh + fast-simplification for simplify_quadric_decimation().

Run from the repo root:  python3 arm/gen_decimated_meshes.py
"""
import glob
import os

import trimesh

TARGET_TRIANGLES = 2000


def main():
    mesh_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
    paths = sorted(glob.glob(os.path.join(mesh_dir, "*.stl")))
    if not paths:
        raise SystemExit(f"no .stl files found in {mesh_dir}")

    total_before = 0
    total_after = 0
    for path in paths:
        name = os.path.basename(path)
        mesh = trimesh.load(path, process=False)
        before = len(mesh.faces)

        target = min(TARGET_TRIANGLES, before)
        if target < before:
            mesh = mesh.simplify_quadric_decimation(face_count=target)
            mesh.remove_unreferenced_vertices()
            mesh.fix_normals()
        after = len(mesh.faces)

        if after == 0 or not mesh.vertices.size:
            raise SystemExit(f"{name}: decimation collapsed the mesh (0 triangles) -- aborting")

        mesh.export(path, file_type="stl")  # binary STL by default, units unchanged (meters)

        total_before += before
        total_after += after
        print(f"{name:42s} {before:6d} -> {after:6d} tris")

    print(f"{'TOTAL':42s} {total_before:6d} -> {total_after:6d} tris")


if __name__ == "__main__":
    main()

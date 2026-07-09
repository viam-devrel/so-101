#!/usr/bin/env python3
"""Generate the SO-101 arm's per-link collision meshes in arm/meshes/.

Why one merged mesh per link:
  rdk's URDF parser keeps only the FIRST <collision> element per link
  (referenceframe/model_urdf.go: it falls back to Collision[0] once the
  capsule-pattern check fails). The upstream SO-ARM100 meshes describe each
  link as an assembly of several parts (e.g. base_link = motor holder + base +
  servo + mounting plate), each its own <collision> with its own <origin>. Fed
  to rdk as-is, every link would contribute only ONE sub-part -- a fragment
  offset within the link -- so collision geometry is both incomplete and
  visibly misaligned in the 3D viewer.

  This script fixes that offline: for each of the 5 arm links it loads every
  collision sub-part, bakes that sub-part's <origin> into the mesh vertices,
  concatenates them into a single mesh expressed in the link frame, quadric-
  decimates the result to ~TARGET_TRIANGLES, and writes one
  arm/meshes/<link>_collision.stl. arm/so101.urdf then references exactly one
  such mesh per link with an identity <origin>, so rdk's Collision[0] IS the
  full link. (rdk's runtime mesh decimation hangs on dense meshes, so
  decimation is done here, offline, and use_urdf ships these light meshes with
  mesh_decimation_ratios empty.)

Source of truth: TheRobotStudio/SO-ARM100, Apache-2.0 (see SO-ARM100-LICENSE),
pinned to UPSTREAM_COMMIT below. By default the sub-part meshes and their
per-link collision layout are downloaded from that commit. For offline / CI
runs, pass --assets-dir DIR (a directory of the upstream *.stl files) and
--layout-urdf FILE (a URDF whose 5 arm links carry the upstream <collision>
elements -- arm/so101.urdf's own history, or the upstream file, both work).

Requires trimesh + fast-simplification (for simplify_quadric_decimation) and,
for the default download path, network access.

Run from the repo root:
    python3 arm/gen_collision_meshes.py                     # download from upstream
    python3 arm/gen_collision_meshes.py --assets-dir /path  --layout-urdf /path.urdf
"""
import argparse
import os
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

UPSTREAM_REPO = "TheRobotStudio/SO-ARM100"
UPSTREAM_COMMIT = "fda892cba81032c46c40976a48c9ceadbf40a9ca"
UPSTREAM_DIR = "Simulation/SO101"  # so101_new_calib.urdf + assets/*.stl live here
RAW = f"https://raw.githubusercontent.com/{UPSTREAM_REPO}/{UPSTREAM_COMMIT}/{UPSTREAM_DIR}"

# The 5 arm links (servos 1-5), tip-excluded gripper. Order is cosmetic.
ARM_LINKS = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link"]

TARGET_TRIANGLES = 800


def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis (extrinsic XYZ) roll-pitch-yaw -> 3x3 rotation R = Rz @ Ry @ Rx."""
    cx, cy, cz = np.cos([roll, pitch, yaw])
    sx, sy, sz = np.sin([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def fetch(url):
    with urllib.request.urlopen(url) as resp:  # noqa: S310 (pinned raw.githubusercontent URL)
        return resp.read()


def load_layout(layout_urdf):
    """Return {link_name: [(mesh_basename, xyz, rpy), ...]} for the arm links."""
    if layout_urdf:
        root = ET.parse(layout_urdf).getroot()
    else:
        root = ET.fromstring(fetch(f"{RAW}/so101_new_calib.urdf"))
    layout = {}
    for link in root.findall("link"):
        name = link.get("name")
        if name not in ARM_LINKS:
            continue
        parts = []
        for col in link.findall("collision"):
            origin = col.find("origin")
            xyz = [float(v) for v in origin.get("xyz").split()]
            rpy = [float(v) for v in origin.get("rpy").split()]
            fname = os.path.basename(col.find("geometry/mesh").get("filename"))
            parts.append((fname, xyz, rpy))
        layout[name] = parts
    missing = [link_name for link_name in ARM_LINKS if link_name not in layout]
    if missing:
        raise SystemExit(f"layout source is missing arm links: {missing}")
    return layout


def load_submesh(basename, assets_dir):
    if assets_dir:
        return trimesh.load(os.path.join(assets_dir, basename), process=False)
    data = fetch(f"{RAW}/assets/{basename}")
    return trimesh.load(trimesh.util.wrap_as_stream(data), file_type="stl", process=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--assets-dir", help="local dir of upstream *.stl sub-part meshes (else download)")
    ap.add_argument("--layout-urdf", help="URDF carrying the arm links' upstream <collision> layout (else download)")
    ap.add_argument("--target-triangles", type=int, default=TARGET_TRIANGLES)
    args = ap.parse_args()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
    os.makedirs(out_dir, exist_ok=True)

    layout = load_layout(args.layout_urdf)
    cache = {}

    def submesh(basename):
        if basename not in cache:
            cache[basename] = load_submesh(basename, args.assets_dir)
        return cache[basename].copy()

    total_out = 0
    for link in ARM_LINKS:
        parts = []
        for basename, xyz, rpy in layout[link]:
            mesh = submesh(basename)
            transform = np.eye(4)
            transform[:3, :3] = rpy_to_matrix(*rpy)
            transform[:3, 3] = xyz
            mesh.apply_transform(transform)
            parts.append(mesh)

        merged = trimesh.util.concatenate(parts)
        before = len(merged.faces)
        target = min(args.target_triangles, before)
        if target < before:
            merged = merged.simplify_quadric_decimation(face_count=target)
            merged.remove_unreferenced_vertices()
        after = len(merged.faces)
        if after == 0 or not merged.vertices.size:
            raise SystemExit(f"{link}: merge/decimation collapsed the mesh -- aborting")

        path = os.path.join(out_dir, f"{link}_collision.stl")
        merged.export(path, file_type="stl")  # binary STL, meters (unchanged)
        total_out += after
        print(f"{link:16s} {len(parts)} parts, {before:6d} -> {after:5d} tris  {os.path.getsize(path) // 1024}KB")

    print(f"{'TOTAL':16s} {'':8s}          {total_out:5d} tris")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Generate the SO-101 arm's per-link collision meshes in assets/urdf/meshes/.

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
  concatenates them, and shifts the result to be relative to so101.json's
  per-link box pose (see URDF_TO_JSON_LINK). assets/urdf/so101.urdf then references
  exactly one such mesh per link, with a <collision><origin> equal to that box
  offset, so rdk's Collision[0] IS the full link AND its geometry pose matches
  so101.json's (which the 3D viewer needs to place the shared GLBs correctly).

Mesh sources (both from TheRobotStudio/SO-ARM100, Apache-2.0, pinned to
UPSTREAM_COMMIT):
  * Printable plastic parts -> STL/SO101/Individual/*.stl. These are the
    optimized/decimated print files (~3x lighter than the simulation assets)
    and, once scaled from millimeters to meters, sit in the exact same local
    frame as the simulation meshes (verified corner-for-corner), so the URDF
    collision origins apply unchanged. Merged as-is -- no decimation -- so the
    geometry stays clean (decimating the *merged* non-manifold mesh created
    sliver artifacts, hence per-source-part handling instead).
  * Servo bodies (sts3215_03a*, not a printed part, so absent from Individual)
    -> Simulation/SO101/assets/*.stl. The servo is a single clean manifold
    reused at every joint and dominates the triangle budget, so it is quadric-
    decimated to SERVO_TARGET_TRIANGLES *before* merging (a single manifold
    decimates cleanly, unlike the merged mesh).

  Net: ~88k triangles / ~4.2 MB total, full clean coverage. Because the meshes
  ship pre-optimized, use_urdf runs with mesh_decimation_ratios empty (rdk's
  runtime mesh decimation hangs on dense meshes anyway).

By default all inputs are downloaded from UPSTREAM_COMMIT. For offline / CI
runs, pass --individual-dir (STL/SO101/Individual), --assets-dir
(Simulation/SO101/assets), and --layout-urdf (a URDF whose 5 arm links carry
the upstream <collision> elements -- the upstream so101_new_calib.urdf, or
assets/urdf/so101.urdf from before it was rewritten to single-collision).

Requires trimesh + fast-simplification (for simplify_quadric_decimation) and,
for the default download path, network access.

Run from the repo root:
    python3 tools/gen_collision_meshes.py
    python3 tools/gen_collision_meshes.py --individual-dir DIR --assets-dir DIR --layout-urdf FILE
"""
import argparse
import json
import os
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

UPSTREAM_REPO = "TheRobotStudio/SO-ARM100"
UPSTREAM_COMMIT = "fda892cba81032c46c40976a48c9ceadbf40a9ca"
RAW = f"https://raw.githubusercontent.com/{UPSTREAM_REPO}/{UPSTREAM_COMMIT}"
INDIVIDUAL_PATH = "STL/SO101/Individual"       # printable parts, millimeters
ASSETS_PATH = "Simulation/SO101/assets"        # simulation meshes (servo), meters
LAYOUT_URDF_PATH = "Simulation/SO101/so101_new_calib.urdf"

# The 5 arm links (servos 1-5), gripper excluded. Order is cosmetic.
ARM_LINKS = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link"]

# URDF sub-part filename (simulation-assets basename, as referenced by the layout URDF's
# <collision> elements) -> optimized Individual print-file name. Sub-parts absent from this map
# (the sts3215 servo bodies) are pulled from the simulation assets and decimated instead.
INDIVIDUAL_PARTS = {
    "base_motor_holder_so101_v1.stl": "Base_motor_holder_SO101.stl",
    "base_so101_v2.stl": "Base_SO101.stl",
    "waveshare_mounting_plate_so101_v2.stl": "WaveShare_Mounting_Plate_SO101.stl",
    "motor_holder_so101_base_v1.stl": "Motor_holder_SO101_Base.stl",
    "rotation_pitch_so101_v1.stl": "Rotation_Pitch_SO101.stl",
    "upper_arm_so101_v1.stl": "Upper_arm_SO101.stl",
    "under_arm_so101_v1.stl": "Under_arm_SO101.stl",
    "motor_holder_so101_wrist_v1.stl": "Motor_holder_SO101_Wrist.stl",
    "wrist_roll_pitch_so101_v2.stl": "Wrist_Roll_Pitch_SO101.stl",
}

# Individual STLs are authored in millimeters; the simulation meshes / URDF are in meters.
MM_TO_M = 0.001
SERVO_TARGET_TRIANGLES = 3000

# Upstream URDF link name -> so101.json link name. The generated meshes are expressed relative
# to so101.json's per-link geometry (box) pose, not the bare link frame, so the 3D scene viewer --
# which places each Get3DModels GLB at the link's *geometry* pose -- lands the GLB in the same
# spot under use_urdf as under so101.json. Each merged mesh is therefore shifted by minus that
# box's translation offset, and assets/urdf/so101.urdf gives the matching <collision><origin>. Without
# this the GLBs scatter by the box offsets (tens of mm/link) in URDF mode.
URDF_TO_JSON_LINK = {
    "base_link": "base",
    "shoulder_link": "shoulder",
    "upper_arm_link": "upper_arm",
    "lower_arm_link": "lower_arm",
    "wrist_link": "wrist",
}


def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis (extrinsic XYZ) roll-pitch-yaw -> 3x3 rotation R = Rz @ Ry @ Rx."""
    cx, cy, cz = np.cos([roll, pitch, yaw])
    sx, sy, sz = np.sin([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def fetch_stl(url):
    with urllib.request.urlopen(url) as resp:  # noqa: S310 (pinned raw.githubusercontent URL)
        data = resp.read()
    return trimesh.load(trimesh.util.wrap_as_stream(data), file_type="stl", process=False)


def load_layout(layout_urdf):
    """Return {link_name: [(subpart_basename, xyz, rpy), ...]} for the arm links."""
    if layout_urdf:
        root = ET.parse(layout_urdf).getroot()
    else:
        with urllib.request.urlopen(f"{RAW}/{LAYOUT_URDF_PATH}") as resp:  # noqa: S310
            root = ET.fromstring(resp.read())
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


def load_box_offsets(so101_json_path):
    """Return {json_link_name: np.array([x, y, z]) in meters} of each link's primitive box
    translation offset in so101.json. so101.json is in mm; the meshes/URDF are in meters."""
    with open(so101_json_path) as f:
        model = json.load(f)
    offsets = {}
    for link in model["links"]:
        geom = link.get("geometry")
        if geom is None:
            continue
        t = geom.get("translation", {})
        offsets[link["id"]] = np.array([t.get("x", 0.0), t.get("y", 0.0), t.get("z", 0.0)]) * MM_TO_M
    return offsets


def load_subpart(basename, args):
    """Load a collision sub-part in meters. Printable parts come from the optimized Individual
    set (scaled mm->m); servo bodies come from the simulation assets and are decimated."""
    if basename in INDIVIDUAL_PARTS:
        indiv_name = INDIVIDUAL_PARTS[basename]
        if args.individual_dir:
            mesh = trimesh.load(os.path.join(args.individual_dir, indiv_name), process=False)
        else:
            mesh = fetch_stl(f"{RAW}/{INDIVIDUAL_PATH}/{indiv_name}")
        mesh.apply_scale(MM_TO_M)
        return mesh
    # Servo body (meters already) -> decimate the single manifold before merging.
    if args.assets_dir:
        mesh = trimesh.load(os.path.join(args.assets_dir, basename), process=False)
    else:
        mesh = fetch_stl(f"{RAW}/{ASSETS_PATH}/{basename}")
    if args.servo_triangles and len(mesh.faces) > args.servo_triangles:
        mesh = mesh.simplify_quadric_decimation(face_count=args.servo_triangles)
        mesh.remove_unreferenced_vertices()
    return mesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--individual-dir", help="local dir of STL/SO101/Individual print files (else download)")
    ap.add_argument("--assets-dir", help="local dir of Simulation/SO101/assets meshes for the servo (else download)")
    ap.add_argument("--layout-urdf", help="URDF carrying the arm links' upstream <collision> layout (else download)")
    ap.add_argument("--servo-triangles", type=int, default=SERVO_TARGET_TRIANGLES,
                    help="decimate each servo sub-part to this many triangles before merge (0 = keep full)")
    ap.add_argument("--so101-json", help="path to so101.json (default: ../internal/geometry/so101.json from tools/)")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.join(here, os.pardir)
    out_dir = os.path.join(repo_root, "assets", "urdf", "meshes")
    os.makedirs(out_dir, exist_ok=True)

    so101_json = args.so101_json or os.path.join(repo_root, "internal", "geometry", "so101.json")
    box_offsets = load_box_offsets(so101_json)

    layout = load_layout(args.layout_urdf)
    cache = {}

    def subpart(basename):
        if basename not in cache:
            cache[basename] = load_subpart(basename, args)
        return cache[basename].copy()

    total_tris = 0
    for link in ARM_LINKS:
        parts = []
        for basename, xyz, rpy in layout[link]:
            mesh = subpart(basename)
            transform = np.eye(4)
            transform[:3, :3] = rpy_to_matrix(*rpy)
            transform[:3, 3] = xyz
            mesh.apply_transform(transform)
            parts.append(mesh)

        merged = trimesh.util.concatenate(parts)
        if not merged.faces.size or not merged.vertices.size:
            raise SystemExit(f"{link}: merge produced an empty mesh -- aborting")

        # Express the mesh relative to so101.json's box pose (shift by -offset) so its geometry
        # pose matches the JSON model's; assets/urdf/so101.urdf's <collision><origin> must be +offset to
        # put the collision shape back at the true link location. See URDF_TO_JSON_LINK.
        offset = box_offsets[URDF_TO_JSON_LINK[link]]
        merged.apply_translation(-offset)

        path = os.path.join(out_dir, f"{link}_collision.stl")
        merged.export(path, file_type="stl")  # binary STL, meters
        total_tris += len(merged.faces)
        off_mm = offset * 1000
        print(f"{link:16s} {len(parts)} parts -> {len(merged.faces):6d} tris  {os.path.getsize(path) // 1024}KB  "
              f"origin=({off_mm[0]:.3f},{off_mm[1]:.3f},{off_mm[2]:.3f})mm")

    print(f"{'TOTAL':16s}   {total_tris:6d} tris")
    print("Set each arm link's <collision>/<visual> <origin xyz> in assets/urdf/so101.urdf to the "
          "printed origin (converted to meters).")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Generate the SO-101 gripper meshes in meshes/gripper/ as ASCII PLY.

Source: TheRobotStudio/SO-ARM100 (Apache-2.0). The follower meshes come from the
SO-101 simulation assets (already in the URDF assembly frame); the leader meshes
come from the printable STLs. Each binary STL is welded + grid-decimated and
written as ASCII PLY -- rdk's spatialmath.NewMeshFromProto accepts ASCII PLY only,
and the gripper component serves these via Geometries() as spatialmath.Mesh.

Each mesh is written at two detail levels -- meshes/gripper/high/ and
meshes/gripper/low/ -- chosen at runtime by the gripper's mesh_detail attribute.
The gripper mesh doubles as the motion-planning collision geometry, so "low" is
the responsive default and "high" is for visual comparison.

PLY vertices are kept in METERS (rdk's PLY reader scales x1000 to millimeters).

Run from the repo root:  python3 meshes/gen_gripper_meshes.py
"""
import os
import struct
import urllib.request

import numpy as np

BASE = "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main"

# name -> (source URL, source-unit-to-meters scale). The follower simulation
# assets are authored in meters; the leader printable STLs are in millimeters.
MESHES = {
    "follower_body": (f"{BASE}/Simulation/SO101/assets/wrist_roll_follower_so101_v1.stl", 1.0),
    "follower_jaw": (f"{BASE}/Simulation/SO101/assets/moving_jaw_so101_v1.stl", 1.0),
    "leader_wrist_roll": (f"{BASE}/STL/SO101/Individual/Wrist_Roll_SO101.stl", 0.001),
    "leader_body": (f"{BASE}/STL/SO101/Individual/Handle_SO101.stl", 0.001),
    "leader_trigger": (f"{BASE}/STL/SO101/Individual/Trigger_SO101.stl", 0.001),
}

# detail level -> weld grid cell size in meters. A coarser grid welds more
# vertices together, yielding far fewer faces (and far cheaper collision checks).
GRIDS = {"high": 0.0015, "low": 0.01}


def load_binary_stl(data):
    """Return an (n, 3, 3) array of triangle vertices from binary STL bytes."""
    (n,) = struct.unpack("<I", data[80:84])
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(data, dtype=rec, count=n, offset=84)
    return tris["v"].astype(np.float64)


def weld(verts, grid):
    """Weld vertices onto a grid: returns (unique_vertices, faces) with degenerate
    triangles dropped. Each unique vertex is the mean of the originals in its cell."""
    flat = verts.reshape(-1, 3)
    keys = np.round(flat / grid).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    nu = int(inv.max()) + 1
    sums = np.zeros((nu, 3))
    np.add.at(sums, inv, flat)
    counts = np.bincount(inv, minlength=nu)
    reps = sums / counts[:, None]
    faces = inv.reshape(-1, 3)
    good = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return reps, faces[good]


def write_ascii_ply(path, verts, faces):
    out = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(verts)}",
        "property float x",
        "property float y",
        "property float z",
        f"element face {len(faces)}",
        "property list uchar int vertex_indices",
        "end_header",
    ]
    out += [f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in verts]
    out += [f"3 {f[0]} {f[1]} {f[2]}" for f in faces]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")


def main():
    base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gripper")
    for name, (url, scale) in MESHES.items():
        with urllib.request.urlopen(url) as resp:
            stl = resp.read()
        verts = load_binary_stl(stl) * scale
        for detail, grid in GRIDS.items():
            out_dir = os.path.join(base_dir, detail)
            os.makedirs(out_dir, exist_ok=True)
            reps, faces = weld(verts, grid)
            path = os.path.join(out_dir, f"{name}.ply")
            write_ascii_ply(path, reps, faces)
            print(f"{detail:4s} {name:18s} {len(faces):6d} faces, "
                  f"{os.path.getsize(path) / 1024:7.1f} KB")


if __name__ == "__main__":
    main()

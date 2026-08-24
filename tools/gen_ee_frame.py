#!/usr/bin/env python3
"""Generate internal/geometry/meshes/so101/ee_frame.glb -- a colored XYZ
coordinate-frame marker for the SO-101 end-effector.

The marker is a grey origin cube plus three axis bars. Axis colors follow the
viam-labs/viam-viz-helpers-go CoordinateFrame defaults (X=red, Y=green, Z=blue).
The arm's Get3DModels serves this mesh at the "tool" frame when the
visualize_ee_frame attribute is enabled.

Run from the repo root:  python3 tools/gen_ee_frame.py
"""
import json
import os
import struct

# Dimensions are in METERS: the SO-101 link GLBs are authored in meters and the
# viewer scales meshes to the millimeter kinematics, so a mesh authored in mm
# renders 1000x too large.
AXIS_LEN = 0.0175  # 17.5 mm, length of each axis bar
AXIS_THK = 0.0015  # 1.5 mm, thickness of each axis bar
ORIGIN = 0.0035    # 3.5 mm, origin cube edge

# Axis colors as sRGB 0-255, from viam-viz-helpers-go CoordinateFrame defaults.
AXES = [
    ("origin", (120, 120, 120)),
    ("x", (230, 25, 75)),
    ("y", (60, 180, 75)),
    ("z", (0, 130, 200)),
]


def srgb_to_linear(c):
    """Convert one sRGB channel (0-255) to linear 0-1 for glTF color factors."""
    c /= 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def box_verts(xmin, ymin, zmin, xmax, ymax, zmax):
    return [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]


# 12 triangles for an 8-vertex box (materials are double-sided, so winding is moot).
BOX_IDX = [
    0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
    0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6,
    0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
]

t = AXIS_THK / 2.0
o = ORIGIN / 2.0
BOXES = {
    "origin": box_verts(-o, -o, -o, o, o, o),
    "x": box_verts(0, -t, -t, AXIS_LEN, t, t),
    "y": box_verts(-t, 0, -t, t, AXIS_LEN, t),
    "z": box_verts(-t, -t, 0, t, t, AXIS_LEN),
}

buf = bytearray()
buffer_views = []
accessors = []
materials = []
primitives = []

for i, (name, color) in enumerate(AXES):
    verts = BOXES[name]

    pos_off = len(buf)
    for v in verts:
        buf += struct.pack("<3f", *v)
    pos_len = len(buf) - pos_off

    idx_off = len(buf)
    for ix in BOX_IDX:
        buf += struct.pack("<H", ix)
    idx_len = len(buf) - idx_off
    while len(buf) % 4:
        buf += b"\x00"

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]

    bv_pos = len(buffer_views)
    buffer_views.append({"buffer": 0, "byteOffset": pos_off, "byteLength": pos_len, "target": 34962})
    bv_idx = len(buffer_views)
    buffer_views.append({"buffer": 0, "byteOffset": idx_off, "byteLength": idx_len, "target": 34963})

    acc_pos = len(accessors)
    accessors.append({
        "bufferView": bv_pos, "componentType": 5126, "count": len(verts), "type": "VEC3",
        "min": [min(xs), min(ys), min(zs)], "max": [max(xs), max(ys), max(zs)],
    })
    acc_idx = len(accessors)
    accessors.append({"bufferView": bv_idx, "componentType": 5123, "count": len(BOX_IDX), "type": "SCALAR"})

    rgb = [srgb_to_linear(c) for c in color]
    materials.append({
        "name": name,
        "pbrMetallicRoughness": {"baseColorFactor": rgb + [1.0], "metallicFactor": 0.0, "roughnessFactor": 1.0},
        "emissiveFactor": rgb,
        "doubleSided": True,
    })
    primitives.append({"attributes": {"POSITION": acc_pos}, "indices": acc_idx, "material": i})

gltf = {
    "asset": {"version": "2.0", "generator": "so-101 gen_ee_frame.py"},
    "scene": 0,
    "scenes": [{"nodes": [0]}],
    "nodes": [{"mesh": 0, "name": "ee_frame"}],
    "meshes": [{"name": "ee_frame", "primitives": primitives}],
    "materials": materials,
    "accessors": accessors,
    "bufferViews": buffer_views,
    "buffers": [{"byteLength": len(buf)}],
}

json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
while len(json_bytes) % 4:
    json_bytes += b" "
while len(buf) % 4:
    buf += b"\x00"

glb = bytearray()
glb += struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(json_bytes) + 8 + len(buf))
glb += struct.pack("<II", len(json_bytes), 0x4E4F534A) + json_bytes
glb += struct.pack("<II", len(buf), 0x004E4942) + bytes(buf)

out = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    "internal", "geometry", "meshes", "so101", "ee_frame.glb",
)
with open(out, "wb") as f:
    f.write(glb)
print(f"wrote {out}: {len(glb)} bytes")

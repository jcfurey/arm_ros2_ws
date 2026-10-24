#!/usr/bin/env python3
"""Decimate UFACTORY's STL meshes for simulation rendering and collision.

The xArm6, gripper, and camera-stand visuals total ~130k triangles, more than
the pixels the arm covers in a 640x360 camera image. Under Docker's software
OpenGL every camera pass transforms all of them, so each color and depth
image costs milliseconds of geometry work before shading a pixel; MoveIt's
collision checks and the physics engine use the same meshes. This writes
reduced copies under src/arm_description/meshes/sim/ with the same relative
paths, each the smallest that stays within --tolerance-mm of the original
surface, and reports each mesh's deviation (measured both ways between
dense surface samples and the other surface).

Run on any machine with numpy, scipy, and fast-simplification (pip install
scipy fast-simplification); the outputs are committed, so builds need none.
"""
import argparse
from pathlib import Path
import struct

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
SOURCE = WORKSPACE / "src/external/xarm_ros2/xarm_description/meshes"
TARGET = WORKSPACE / "src/arm_description/meshes/sim"
MESHES = [f"xarm6/visual/{name}.stl" for name in ["link_base", "link1", "link2", "link3", "link4", "link5", "link6"]] + [
    "end_tool/collision/end_tool.stl",
    *[f"gripper/xarm/{side}_{part}.stl" for side in ["left", "right"] for part in ["outer_knuckle", "finger", "inner_knuckle"]],
    "gripper/xarm/base_link.stl",
    "camera/realsense/visual/d435_with_cam_stand.stl",
    "camera/realsense/collision/d435_with_cam_stand.stl",
]
# The fingers and knuckles sit 10-20 cm in front of the wrist camera, where
# 1 mm spans ~3 pixels; they get half the tolerance.
CLOSE_TO_CAMERA = "gripper/"


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path} is not a binary STL")
    records = np.frombuffer(data, dtype=np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
                                                  ("attribute", "<u2")]), count=count, offset=84)
    return records["vertices"].reshape(-1, 3).astype(np.float64)


def weld(corners, tolerance=1e-7):
    """Shared vertices and triangle indices from an STL triangle soup."""
    keys = np.round(corners / tolerance).astype(np.int64)
    points, inverse = np.unique(keys, axis=0, return_inverse=True)
    vertices = np.zeros((len(points), 3))
    np.add.at(vertices, inverse.ravel(), corners)
    vertices /= np.bincount(inverse.ravel())[:, None]
    faces = inverse.reshape(-1, 3)
    return vertices, faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]


def write_stl(path, vertices, faces):
    triangles = vertices[faces].astype(np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    records = np.zeros(len(faces), dtype=np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
                                                   ("attribute", "<u2")]))
    records["normal"], records["vertices"] = normals, triangles
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"arm_description simplified UFACTORY mesh".ljust(80, b"\0")
                     + struct.pack("<I", len(faces)) + records.tobytes())


def point_triangle_distance(points, triangles):
    """Distances from points (N, 3) to triangles (N, K, 3, 3), minimised over K.

    Closest points follow Ericson's Voronoi-region tests (Real-Time Collision
    Detection, 5.1.5).
    """
    a, b, c = (triangles[..., i, :] for i in range(3))
    p = points[:, None, :]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = p - b
    d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = p - c
    d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    with np.errstate(divide="ignore", invalid="ignore"):
        denominator = 1.0 / (va + vb + vc)
        closest = a + ab * (vb * denominator)[..., None] + ac * (vc * denominator)[..., None]
        edge_ab = a + ab * (d1 / (d1 - d3))[..., None]
        edge_ac = a + ac * (d2 / (d2 - d6))[..., None]
        edge_bc = b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[..., None]
    regions = [
        ((d1 <= 0) & (d2 <= 0), a), ((d3 >= 0) & (d4 <= d3), b), ((d6 >= 0) & (d5 <= d6), c),
        ((vc <= 0) & (d1 >= 0) & (d3 <= 0), edge_ab), ((vb <= 0) & (d2 >= 0) & (d6 <= 0), edge_ac),
        ((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0), edge_bc),
    ]
    for mask, point in reversed(regions):
        closest = np.where(mask[..., None], np.broadcast_to(point, closest.shape), closest)
    return np.sqrt(((p - closest) ** 2).sum(-1)).min(-1)


def surface_samples(vertices, faces, count, rng):
    triangles = vertices[faces]
    areas = 0.5 * np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    chosen = rng.choice(len(faces), size=count, p=areas / areas.sum())
    u, v = rng.random(count), rng.random(count)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = triangles[chosen]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])


def deviation(a_vertices, a_faces, b_vertices, b_faces, rng, count=6000, candidates=16):
    """Symmetric max point-to-surface distance between two meshes (metres), from surface samples.

    Exact for the samples: the nearest-centroid triangles give each sample an
    upper bound, and every triangle whose bounding box lies closer than that
    bound is then tested, since CAD meshes have long triangles whose centroids
    sit far from their nearest point.
    """
    from scipy.spatial import cKDTree

    worst = 0.0
    for (sv, sf), (tv, tf) in [((a_vertices, a_faces), (b_vertices, b_faces)),
                               ((b_vertices, b_faces), (a_vertices, a_faces))]:
        triangles = tv[tf]
        low, high = triangles.min(axis=1), triangles.max(axis=1)
        samples = surface_samples(sv, sf, count, rng)
        _, nearest = cKDTree(triangles.mean(axis=1)).query(samples, k=min(candidates, len(tf)))
        bounds = point_triangle_distance(samples, triangles[nearest.reshape(len(samples), -1)])
        for chunk in np.array_split(np.arange(len(samples)), max(1, len(samples) * len(tf) // 4_000_000)):
            points = samples[chunk]
            gap = np.maximum(0.0, np.maximum(low[None] - points[:, None], points[:, None] - high[None]))
            box_distance = np.sqrt((gap ** 2).sum(-1))
            for row, index in enumerate(chunk):
                closer = np.nonzero(box_distance[row] < bounds[index])[0]
                distance = bounds[index]
                if len(closer):
                    distance = min(distance, float(point_triangle_distance(points[row:row + 1],
                                                                           triangles[closer][None])[0]))
                worst = max(worst, distance)
    return worst


def simplify(vertices, faces, tolerance, rng, fractions=(0.1, 0.15, 0.25, 0.4, 0.6, 0.8)):
    """The smallest of `fractions` of the triangles whose surface stays within `tolerance`.

    Returns the original when none does: some UFACTORY meshes are many open
    shells (the gripper base has 43, down to 5 mm screws), and quadric
    decimation moves their boundaries further than the tolerance at any useful
    reduction. agg=3 trades speed for shape (fast-simplification's default 7
    deviated 3.5 mm where 3 kept 1 mm on the camera stand).
    """
    import fast_simplification

    for fraction in fractions:
        reduced_vertices, reduced_faces = fast_simplification.simplify(
            vertices.astype(np.float32), faces.astype(np.int64), 1.0 - fraction, agg=3)
        reduced_vertices = reduced_vertices.astype(np.float64)
        error = deviation(vertices, faces, reduced_vertices, reduced_faces, rng)
        if error <= tolerance:
            return reduced_vertices, reduced_faces, error
    return vertices, faces, 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tolerance-mm", type=float, default=1.0,
                        help="Largest allowed distance between a reduced surface and the original "
                             "(half this for the gripper, which the wrist camera sees close up).")
    parser.add_argument("--only", nargs="*", default=[], help="Regenerate only meshes whose path contains these.")
    args = parser.parse_args()
    rng = np.random.default_rng(0)
    total_before = total_after = 0
    for name in MESHES:
        if args.only and not any(part in name for part in args.only):
            continue
        tolerance = args.tolerance_mm / 1000.0 / (2 if name.startswith(CLOSE_TO_CAMERA) else 1)
        vertices, faces = weld(read_stl(SOURCE / name))
        reduced_vertices, reduced_faces, error = (vertices, faces, 0.0) if len(faces) < 1000 else simplify(
            vertices, faces, tolerance, rng)
        write_stl(TARGET / name, reduced_vertices, reduced_faces)
        total_before, total_after = total_before + len(faces), total_after + len(reduced_faces)
        print(f"{name:50s} {len(faces):6d} -> {len(reduced_faces):6d} triangles, "
              f"max deviation {error * 1000:.2f} mm", flush=True)
    print(f"{'total':50s} {total_before:6d} -> {total_after:6d} triangles")


if __name__ == "__main__":
    main()

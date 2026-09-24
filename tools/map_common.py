#!/usr/bin/env python3
"""Shared geometry helpers for the converted point-cloud map.

The map that ships with this workspace started life as a .pcd scan that
`pcd_to_gazebo.py` turned into a coloured voxel-surface OBJ. Two properties of
that mesh decide everything else in this folder, so they are measured here
once and reused:

  * the floor is TILTED by ~1.2 deg relative to the scan axes. A Gazebo ground
    plane can only be level, so the mesh is rotated to cancel the tilt before
    it is used; otherwise the robots drive up a ramp that does not exist and
    the wheels spend the whole run fighting the floor mesh.

  * the mesh is a *surface* mesh with no notion of "floor" vs "wall". For
    driving it is better to keep only the vertical structure as collision and
    let the world's flat ground plane be the floor; see prepare_map_mesh.py.

Everything works on plain numpy arrays and is deliberately independent of ROS
so it can be run from a bare python3.
"""

import numpy as np


def load_obj(path):
    """Read an OBJ into (vertices, faces) as float64/int64 arrays."""
    verts, faces = [], []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) for t in line.split()[1:]]
                if len(idx) == 3:
                    faces.append(idx)
                elif len(idx) > 3:  # fan-triangulate quads/ngons
                    for k in range(1, len(idx) - 1):
                        faces.append([idx[0], idx[k], idx[k + 1]])
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64) - 1


def face_geometry(V, F):
    """Per-face area, unit normal and centroid."""
    A = V[F]
    cross = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0])
    norm = np.linalg.norm(cross, axis=1)
    area = 0.5 * norm
    nz = cross[:, 2] / (norm + 1e-12)  # +1 = facing up
    centroid = A.mean(axis=1)
    return area, nz, centroid


def fit_floor_plane(V, F, min_area=1e-4):
    """Area-weighted least-squares plane z = a*x + b*y + c through the floor.

    The floor is found without knowing where it is: take the upward-facing
    faces, histogram their z, and keep the widest band. Furniture tops are
    both smaller in total area and spread over many different heights, so the
    floor mode wins comfortably.

    The histogram is weighted by FACE AREA, not by face count. A triangle mesh
    does not have uniform triangle size: on this scan the floor is coarsely
    tessellated while walls and furniture carry many more (smaller) triangles,
    so an unweighted histogram picks whichever surface happens to be diced
    finely - on geomap_final_noroof that put the walking surface 0.80 m BELOW
    z=0 and left the robots floating above the visible floor.
    """
    area, nz, centroid = face_geometry(V, F)
    up = (nz > 0.85) & (area > min_area)
    z = centroid[up, 2]
    hist, edges = np.histogram(z, bins=40, weights=area[up])
    peak = int(np.argmax(hist))
    centre = 0.5 * (edges[peak] + edges[peak + 1])
    band = up & (centroid[:, 2] > centre - 0.35) & (centroid[:, 2] < centre + 0.35)

    X = np.stack([centroid[band, 0], centroid[band, 1], np.ones(int(band.sum()))], axis=1)
    w = area[band]
    lhs = X.T @ (w[:, None] * X)
    rhs = X.T @ (w * centroid[band, 2])
    coef = np.linalg.solve(lhs, rhs)
    res = centroid[band, 2] - X @ coef
    stats = {
        "faces": int(band.sum()),
        "area": float(w.sum()),
        "tilt_deg": float(np.degrees(np.arctan(np.hypot(coef[0], coef[1])))),
        "residual_std": float(res.std()),
        "residual_p5": float(np.percentile(res, 5)),
        "residual_p95": float(np.percentile(res, 95)),
    }
    return tuple(float(v) for v in coef), stats


def level_transform(V, F):
    """Rotation + translation that puts the fitted floor in the z=0 plane.

    For small angles an SDF pose (roll, pitch, 0) maps a point to
    z' = z - pitch*x + roll*y. Cancelling the floor's slope a*x + b*y + c
    therefore needs pitch = a, roll = -b, and then a translation of -c.
    """
    (a, b, c), stats = fit_floor_plane(V, F)
    roll, pitch = -b, a
    # Second-order effect of the rotation on the plane's height at the origin,
    # evaluated on the real vertices so the floor really lands on z=0.
    R = _rpy_matrix(roll, pitch, 0.0)
    area, nz, centroid = face_geometry(V, F)
    up = (nz > 0.85) & (area > 1e-4)
    z = centroid[up, 2]
    hist, edges = np.histogram(z, bins=40, weights=area[up])   # area-weighted (see above)
    peak = int(np.argmax(hist))
    centre = 0.5 * (edges[peak] + edges[peak + 1])
    band = up & (centroid[:, 2] > centre - 0.35) & (centroid[:, 2] < centre + 0.35)
    z_after = (centroid[band] @ R.T)[:, 2]
    dz = -np.median(z_after)
    stats["z_shift"] = float(dz)
    stats["roll_deg"] = float(np.degrees(roll))
    stats["pitch_deg"] = float(np.degrees(pitch))
    return (roll, pitch, dz), stats


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def apply_level(V, transform):
    roll, pitch, dz = transform
    return V @ _rpy_matrix(roll, pitch, 0.0).T + np.array([0.0, 0.0, dz])


def write_obj(path, V, F, header=None, mtl=None, uv=None):
    """Write a triangle OBJ. Faces are 1-based; vertices are reused as-is."""
    with open(path, "w") as f:
        if header:
            f.write(f"# {header}\n")
        if mtl:
            f.write(f"mtllib {mtl}\nusemtl {mtl.rsplit('.', 1)[0]}\n")
        for x, y, z in V:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for tri in F:
            f.write("f %d %d %d\n" % (tri[0] + 1, tri[1] + 1, tri[2] + 1))


def rasterise_footprint(V, F, res, xmin, ymin, shape, thicken=0.7):
    """Top-down projection of triangles onto a grid of `res` metres per cell.

    A plain inside-the-triangle fill is not enough for a voxel-surface mesh:
    a wall is a *vertical* rectangle whose XY projection is a line of zero
    area, so every wall would be projected onto nothing and the resulting map
    would have the robots driving through the building. Triangles whose
    projection is degenerate (or thinner than a cell) are therefore
    rasterised as a capsule of radius `thicken` cells around their longest
    projected edge.

    Returns a boolean array (True = the mesh covers this cell).
    """
    H, W = shape
    tri = V[F]
    px = (tri[:, :, 0] - xmin) / res
    py = (ymin + H * res - tri[:, :, 1]) / res  # row 0 is the top of the grid
    grid = np.zeros((H, W), dtype=bool)

    signed = ((px[:, 1] - px[:, 0]) * (py[:, 2] - py[:, 0])
              - (px[:, 2] - px[:, 0]) * (py[:, 1] - py[:, 0]))
    sign = np.where(signed >= 0, 1.0, -1.0)

    for k in range(len(tri)):
        cmin = max(0, int(np.floor(px[k].min() - thicken)))
        cmax = min(W - 1, int(np.ceil(px[k].max() + thicken)))
        rmin = max(0, int(np.floor(py[k].min() - thicken)))
        rmax = min(H - 1, int(np.ceil(py[k].max() + thicken)))
        if cmax < cmin or rmax < rmin:
            continue
        cc, rr = np.meshgrid(np.arange(cmin, cmax + 1), np.arange(rmin, rmax + 1))
        if abs(signed[k]) >= 1.0:
            inside = np.ones(cc.shape, dtype=bool)
            for i in range(3):
                j = (i + 1) % 3
                ex, ey = px[k, j] - px[k, i], py[k, j] - py[k, i]
                inside &= -sign[k] * ((cc - px[k, i]) * ey
                                      - (rr - py[k, i]) * ex) >= -0.02
        else:
            # Degenerate projection: keep the longest edge as a thick segment.
            best, seg = -1.0, None
            for i in range(3):
                j = (i + 1) % 3
                d = (px[k, j] - px[k, i]) ** 2 + (py[k, j] - py[k, i]) ** 2
                if d > best:
                    best, seg = d, (px[k, i], py[k, i], px[k, j], py[k, j])
            x1, y1, x2, y2 = seg
            dx, dy = x2 - x1, y2 - y1
            length2 = dx * dx + dy * dy
            t = np.clip(((cc - x1) * dx + (rr - y1) * dy) / max(length2, 1e-9), 0, 1)
            dist2 = (cc - (x1 + t * dx)) ** 2 + (rr - (y1 + t * dy)) ** 2
            inside = dist2 <= thicken ** 2
        grid[rmin:rmax + 1, cmin:cmax + 1] |= inside
    return grid

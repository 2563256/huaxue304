#!/usr/bin/env python3
"""Analyse a converted point-cloud mesh: floor height, free space, passage widths.

Reads an OBJ (the mesh produced by pcd_to_gazebo.py) and answers the questions
that matter when sizing robots for it:
  * where is the walkable floor (z) and how thick is the clutter above it,
  * which cells are blocked for a robot of height h,
  * how wide are the free corridors (max inscribed disc radius).

Writes a PNG preview with the occupancy grid so the numbers can be eyeballed.
"""
import argparse
import sys

import numpy as np
from PIL import Image, ImageDraw


def load_obj(path):
    verts = []
    faces = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) for t in line.split()[1:]]
                if len(idx) == 3:
                    faces.append(idx)
                elif len(idx) > 3:  # fan-triangulate
                    for k in range(1, len(idx) - 1):
                        faces.append([idx[0], idx[k], idx[k + 1]])
    V = np.asarray(verts, dtype=np.float64)
    F = np.asarray(faces, dtype=np.int64) - 1  # OBJ is 1-based
    return V, F


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--robot-height", type=float, default=1.2)
    ap.add_argument("--floor-clearance", type=float, default=0.10)
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    V, F = load_obj(a.obj)
    print(f"vertices={len(V)} faces={len(F)}")
    print(f"x [{V[:,0].min():.3f}, {V[:,0].max():.3f}] span {np.ptp(V[:,0]):.3f}")
    print(f"y [{V[:,1].min():.3f}, {V[:,1].max():.3f}] span {np.ptp(V[:,1]):.3f}")
    print(f"z [{V[:,2].min():.3f}, {V[:,2].max():.3f}] span {np.ptp(V[:,2]):.3f}")

    hist, edges = np.histogram(V[:, 2], bins=60)
    print("\nz histogram (vertices per 5 cm-ish bin):")
    for h, e in zip(hist, edges):
        if h:
            print(f"  z={e:+.2f} {h:7d} {'#' * min(60, h // 400)}")

    tri_z = V[F][:, :, 2]
    zmin, zmax = tri_z.min(axis=1), tri_z.max(axis=1)

    floor = np.percentile(V[:, 2], 2)
    print(f"\nfloor estimate (2nd pct of vertices) = {floor:.3f}")

    band_lo = floor + a.floor_clearance
    band_hi = floor + a.robot_height
    # A triangle blocks the robot if it reaches down into the robot's height band.
    blocks = (zmax > band_lo) & (zmin < band_hi)
    print(f"blocking triangles (z overlapping [{band_lo:.2f}, {band_hi:.2f}]): "
          f"{int(blocks.sum())}/{len(F)}")

    x0, x1 = V[:, 0].min() - 0.5, V[:, 0].max() + 0.5
    y0, y1 = V[:, 1].min() - 0.5, V[:, 1].max() + 0.5
    W = int(np.ceil((x1 - x0) / a.res))
    H = int(np.ceil((y1 - y0) / a.res))
    print(f"grid {W}x{H} @ {a.res} m  origin ({x0:.2f},{y0:.2f})")

    grid = np.zeros((H, W), dtype=bool)
    tri = V[F[blocks]]
    # Rasterise the blocking triangles: for each one, scan its bounding box and
    # test the pixels against the three edges (top-down projection).
    xs = tri[:, :, 0]
    ys = tri[:, :, 1]
    for k in range(len(tri)):
        px = (xs[k] - x0) / a.res
        py = (y1 - ys[k]) / a.res
        cmin = max(0, int(np.floor(px.min())))
        cmax = min(W - 1, int(np.ceil(px.max())))
        rmin = max(0, int(np.floor(py.min())))
        rmax = min(H - 1, int(np.ceil(py.max())))
        if cmax < cmin or rmax < rmin:
            continue
        cc, rr = np.meshgrid(np.arange(cmin, cmax + 1), np.arange(rmin, rmax + 1))
        inside = np.ones(cc.shape, dtype=bool)
        for i in range(3):
            j = (i + 1) % 3
            ex, ey = px[j] - px[i], py[j] - py[i]
            inside &= ((cc - px[i]) * ey - (rr - py[i]) * ex) >= -0.02
        grid[rmin:rmax + 1, cmin:cmax + 1] |= inside

    free = ~grid
    print(f"free cells {int(free.sum())} ({100*free.sum()/free.size:.1f}%), "
          f"area {free.sum()*a.res*a.res:.1f} m^2")

    # Connected free space (4-neighbour flood from the largest component) and
    # the distance transform that gives passage widths.
    from scipy import ndimage  # noqa: PLC0415
    lab, n = ndimage.label(free)
    if n:
        sizes = ndimage.sum(free, lab, range(1, n + 1))
        order = np.argsort(sizes)[::-1]
        print(f"free components: {n}")
        for i in order[:6]:
            cid = i + 1
            mask = lab == cid
            print(f"  comp {cid}: {sizes[i]*a.res*a.res:8.1f} m^2")
        main_id = order[0] + 1
        main = lab == main_id
    else:
        main = free
    dist = ndimage.distance_transform_edt(main) * a.res
    print(f"max inscribed disc radius in main free area: {dist.max():.2f} m "
          f"(= {2*dist.max():.2f} m wide)")

    # Report clearance statistics of the reachable area: how much of the free
    # space is at least X wide.
    for width in (0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5, 3.0):
        frac = (dist >= width / 2).sum() / max(1, main.sum())
        print(f"  area with >= {width:.1f} m clearance: {100*frac:5.1f}%")

    # Largest open locations, useful as spawn/zone candidates.
    from scipy.ndimage import maximum_filter
    mx = maximum_filter(dist, size=int(2.0 / a.res))
    peaks = np.argwhere((dist == mx) & (dist > 0.8))
    picked = []
    for r, c in peaks:
        wx, wy = x0 + (c + 0.5) * a.res, y1 - (r + 0.5) * a.res
        if all((wx - px) ** 2 + (wy - py) ** 2 > 4.0 for px, py, _ in picked):
            picked.append((wx, wy, dist[r, c]))
    picked.sort(key=lambda t: -t[2])
    print("\nlargest open spots (x, y, clearance radius):")
    for px, py, d in picked[:15]:
        print(f"  ({px:7.2f}, {py:7.2f})  r={d:.2f}")

    if a.png:
        im = Image.new("L", (W, H), 255)
        im.paste(Image.fromarray(np.where(grid, 0, 255).astype(np.uint8)), (0, 0))
        d = ImageDraw.Draw(im)
        for px, py, r in picked[:15]:
            c = (px - x0) / a.res
            rr = (y1 - py) / a.res
            d.ellipse([c - r / a.res, rr - r / a.res, c + r / a.res, rr + r / a.res],
                      outline=128)
        im.save(a.png)
        print(f"\nwrote {a.png}")


if __name__ == "__main__":
    sys.exit(main())

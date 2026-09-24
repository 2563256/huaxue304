#!/usr/bin/env python3
"""Rasterise the levelled scan mesh into a Nav2 occupancy map, and report on it.

POLARITY follows map_server's `negate: 0` convention, exactly like the
warehouse map this replaces: white (255) = drivable floor, black (0) =
obstacle. Getting this backwards makes every cell occupied, the costmap
inflates the whole world and Nav2 answers `START_OCCUPIED` to every goal - see
the note at the top of hetero_fleet_bringup/scripts/make_map.py.

What the map says:
  occupied = walls/furniture (the collision mesh, projected top-down)
             | everything the scan never saw
  free     = scanned floor that is not covered by structure

Everything outside the scanned footprint is marked occupied on purpose. The
scan is an island inside the world's 300 x 300 m ground plane, and a costmap
that reads "free" out there would happily plan a route off the edge of the
building.

With --report it prints how much of the free space is at least N metres wide
and ranks the roomiest spots - that is where the zone markers and the robot
spawn poses in robot_profiles.yaml come from.
"""

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image

import map_common as mc


def build_map(visual_obj, collision_obj, res=0.05, margin=0.6, z_limit=None,
              cover_close=0.5, structure_close=0.1, smooth=1,
              structure_dilate=0, min_width=0.60, verbose=True):
    """Return (occupied_grid, meta) for a scan mesh pair.

    The collision mesh is expected to contain *only* what the robots cannot
    drive through: prepare_map_mesh.py already dropped the floor and every
    piece of clutter that is entirely below head height. So by default every
    face of it is projected onto the grid (z_limit=None).

    z_limit exists for experiments: if you pass a height, only triangles
    reaching below it are treated as obstacles. It is NOT what produces the
    ring corridor - that is decided when the collision mesh is built, so that
    the map and the physics keep describing the same room.

    cover_close closes the gaps in the scanned floor before they are treated
    as "unscanned = occupied". The scan simply never saw some floor cells
    (occlusion, dark corners); without this the aisle is cut into a dozen
    rooms by holes that are not physically there.

    smooth is a majority-vote (median) filter of that many cells on the final
    occupancy mask. Projecting a triangle mesh onto a 5 cm grid leaves the
    wall/free boundary as a staircase with single-cell teeth, and those teeth
    are what the costmap inflates and what make_collision_boxes.py turns into
    colliders - so the wall the robots see is jagged. One pass (smooth=1) is a
    3x3 vote: it removes 1-cell spikes and fills 1-cell notches, i.e. it
    smooths the outline, and moves the boundary by at most one cell (5 cm).
    Keep the value low: a large kernel starts eating the narrow passages.

    structure_dilate grows every wall by that many cells after rasterisation.
    It is 0 by default because it is pure lost drivable area: `structure_close`
    already makes thin walls solid, and the extra ring only made the map
    stricter than the room. Measured against the simulated lidar, the map was
    predicting obstacles a median 0.35 m closer than the scan actually saw
    them - i.e. the walls were drawn fatter than they are. Removing one cell of
    dilation gives ~13% more usable floor (largest connected region at the
    fleet's clearance requirement: 50.1 -> 56.4 m^2 on this map).
    """
    from scipy import ndimage

    Vv, Fv = mc.load_obj(visual_obj)
    Vc, Fc = mc.load_obj(collision_obj)

    xmin = min(Vv[:, 0].min(), Vc[:, 0].min()) - margin
    xmax = max(Vv[:, 0].max(), Vc[:, 0].max()) + margin
    ymin = min(Vv[:, 1].min(), Vc[:, 1].min()) - margin
    ymax = max(Vv[:, 1].max(), Vc[:, 1].max()) + margin
    W = int(math.ceil((xmax - xmin) / res))
    H = int(math.ceil((ymax - ymin) / res))

    k = max(1, int(round(cover_close / res)) | 1)
    cover = mc.rasterise_footprint(Vv, Fv, res, xmin, ymin, (H, W))
    cover = ndimage.binary_fill_holes(
        ndimage.binary_closing(cover, structure=np.ones((k, k))))

    if z_limit is None:
        faces = Fc
    else:
        zmin = Vc[Fc][:, :, 2].min(axis=1)
        faces = Fc[zmin < z_limit]
    structure = mc.rasterise_footprint(Vc, faces, res, xmin, ymin, (H, W))
    # A wall one voxel thick projects to two parallel lines 10 cm apart; close
    # the gap so the wall is solid instead of a 2-cell free slot.
    ks = max(1, int(round(structure_close / res)) | 1)
    structure = ndimage.binary_closing(structure, structure=np.ones((ks, ks)))
    if structure_dilate > 0:
        structure = ndimage.binary_dilation(
            structure, iterations=int(structure_dilate))

    occupied = structure | ~cover
    if min_width and min_width > 0:
        # Erase drivable space that is narrower than min_width. A robot can
        # physically squeeze into a 0.4 m slot but cannot turn or back out of
        # one, and the planner happily routes through it as a shortcut - that
        # is the "robot stuck on the edge" failure. Filling such slots here
        # fills them in the physics too, because make_collision_boxes.py builds
        # the colliders from this very grid.
        r = max(1, int(round(0.5 * min_width / res)))
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        free = ndimage.binary_opening(~occupied, structure=disk)
        occupied = ~free
    if smooth and smooth > 0:
        k = 2 * int(smooth) + 1
        occupied = ndimage.median_filter(
            occupied.astype(np.uint8), size=k, mode="nearest").astype(bool)
    meta = {
        "res": res,
        "xmin": xmin,
        "ymin": ymin,
        "W": W,
        "H": H,
        "z_limit": z_limit,
        "covered_m2": float(cover.sum()) * res * res,
        "structure_m2": float(structure.sum()) * res * res,
        "free_m2": float((~occupied).sum()) * res * res,
        "bbox_m2": W * H * res * res,
    }
    if verbose:
        print(f"grid {W}x{H} @ {res} m, origin ({xmin:.2f}, {ymin:.2f}), "
              f"covers {xmax-xmin:.1f} x {ymax-ymin:.1f} m")
        print(f"  obstacles: {len(faces)} collision triangles "
              f"(structure covers {meta['structure_m2']:.1f} m^2 of the grid)")
        print(f"  scanned floor {meta['covered_m2']:.1f} m^2 of {meta['bbox_m2']:.1f} m^2, "
              f"drivable {meta['free_m2']:.1f} m^2")
    return occupied, meta


def report(occupied, meta, min_spot_radius=0.6, separation=3.0):
    from scipy import ndimage

    res = meta["res"]
    free = ~occupied
    lab, n = ndimage.label(free)
    if n == 0:
        print("no free space at all - check the polarity!")
        return None
    sizes = ndimage.sum(free, lab, range(1, n + 1))
    order = np.argsort(sizes)[::-1]
    print(f"free components: {n}; largest {sizes[order[0]]*res*res:.1f} m^2 "
          f"({100*sizes[order[0]]/free.sum():.0f}% of all free space)")
    for i in order[:4]:
        r, c = np.argwhere(lab == i + 1)[0]
        print(f"   comp {i+1}: {sizes[i]*res*res:7.1f} m^2 at "
              f"({meta['xmin']+c*res:.1f}, {meta['ymin'] + (meta['H']-r)*res:.1f})")

    main = lab == (order[0] + 1)
    dist = ndimage.distance_transform_edt(main) * res
    print("clearance available over the largest free component:")
    for width in (0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0):
        frac = (dist >= width / 2).sum() / main.sum()
        print(f"   >= {width:.1f} m wide: {100*frac:5.1f}%  "
              f"({(dist >= width/2).sum()*res*res:6.1f} m^2)")

    peaks = np.argwhere(dist > min_spot_radius)
    order = np.argsort(dist[peaks[:, 0], peaks[:, 1]])[::-1]
    picked = []
    for idx in order:
        r, c = peaks[idx]
        x = meta["xmin"] + (c + 0.5) * res
        y = meta["ymin"] + (meta["H"] - 0.5 - r) * res
        if all((x - ox) ** 2 + (y - oy) ** 2 > separation ** 2 for ox, oy, _ in picked):
            picked.append((float(x), float(y), float(dist[r, c])))
    print(f"\nroomiest spots (clearance radius >= {min_spot_radius} m, "
          f">= {separation} m apart):")
    for x, y, d in picked:
        print(f"   ({x:7.2f}, {y:7.2f})  r={d:.2f} m  -> fits a "
              f"{2*d:.2f} m wide robot")
    return picked


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("visual_obj")
    ap.add_argument("collision_obj")
    ap.add_argument("out_basename", nargs="?")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.6)
    ap.add_argument("--z-limit", type=float, default=None,
                    help="optional: only triangles dipping below this height "
                         "count as obstacles (default: all of them)")
    ap.add_argument("--cover-close", type=float, default=0.5)
    ap.add_argument("--structure-close", type=float, default=0.1)
    ap.add_argument("--smooth", type=int, default=1,
                    help="majority-vote passes over the occupancy mask; smooths "
                         "the staircase edge the mesh projection leaves behind "
                         "(0 disables, 1 = 3x3 vote, 2 = 5x5, ...)")
    ap.add_argument("--min-width", type=float, default=0.60,
                    help="drivable space narrower than this is filled in, so "
                         "the robots can never be sent into a slot they cannot "
                         "manoeuvre out of (0 disables)")
    ap.add_argument("--structure-dilate", type=int, default=0,
                    help="grow walls by this many cells after rasterisation "
                         "(0 = off; see build_map's docstring)")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    occupied, meta = build_map(a.visual_obj, a.collision_obj, a.res, a.margin,
                               a.z_limit, a.cover_close, a.structure_close,
                               a.smooth, a.structure_dilate, a.min_width)
    if a.report:
        report(occupied, meta)

    if a.out_basename:
        im = Image.fromarray(np.where(occupied, 0, 255).astype(np.uint8))
        im.save(a.out_basename + ".pgm")
        Path(a.out_basename + ".yaml").write_text(
            f"image: {Path(a.out_basename).name}.pgm\n"
            f"resolution: {a.res}\n"
            f"origin: [{meta['xmin']:.6f}, {meta['ymin']:.6f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        print(f"wrote {a.out_basename}.pgm ({meta['W']}x{meta['H']}) and .yaml")

    if a.png:
        Image.fromarray(np.where(occupied, 40, 255).astype(np.uint8)).save(a.png)
        print(f"wrote {a.png}")


if __name__ == "__main__":
    main()

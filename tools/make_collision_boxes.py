#!/usr/bin/env python3
"""Turn the Nav2 occupancy grid into Gazebo collision geometry made of boxes.

WHY THIS EXISTS. The map model used to collide with a 209k-triangle mesh
(`geomap_colour_noroof_collision.obj`). Two things are wrong with that:

  1. DART and ODE segfault on triangle-mesh collision in this Gazebo install,
     which is why the world was pinned to Bullet.
  2. Bullet-Featherstone cannot produce in-place rotation for a wheeled base
     on this map: the drive wheels counter-rotate at exactly the commanded
     rate while the body yaw stays at zero, so every Nav2 goal that needs a
     turn dies with "Failed to make progress". The same happens to Gazebo's
     own reference diff-drive demo model when Bullet is forced, and the same
     robots rotate correctly under DART - so the engine is the variable.

Replacing the mesh collision with axis-aligned boxes removes both problems:
box-vs-plane and box-vs-cylinder collisions are primitives every engine
handles, DART included.

WHAT THE BOXES ARE. The occupancy grid is already the single description of
"what the robots cannot drive through" (white = free, black = obstacle), and
Nav2 plans on exactly that grid. So the boxes are built from it rather than
from the mesh: physics and costmap then describe the same room, which is the
property MAP_SWAP.md wanted in the first place.

Only obstacles within `--margin` metres of drivable floor are turned into
boxes. Everything further away is unscanned space; boxing all of it would add
thousands of irrelevant colliders, while the shell around the drivable area is
what actually keeps the robots inside the building.

Usage (run after make_nav2_map.py, before make_geomap_world.py):
    python3 tools/make_collision_boxes.py \
        --map src/hetero_fleet_bringup/maps/geomap \
        --model src/hetero_fleet_gz/models/geomap_colour_noroof/model.sdf
"""

import argparse
import re
from pathlib import Path

import numpy as np
from PIL import Image


def read_pgm(path):
    return np.asarray(Image.open(path).convert("L"))


def read_yaml(path):
    """Minimal reader for the four fields map_server needs (avoids a yaml dep)."""
    meta = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    meta["resolution"] = float(meta["resolution"])
    origin = re.findall(r"[-+0-9.eE]+", meta["origin"])
    meta["origin"] = [float(v) for v in origin[:3]]
    return meta


def runs_of(row):
    """Yield (col_start, col_end_exclusive) for each True run in a 1-D mask."""
    padded = np.concatenate(([False], row.astype(bool), [False])).astype(np.int8)
    idx = np.flatnonzero(np.diff(padded))
    return list(zip(idx[0::2], idx[1::2]))


def merge_rectangles(mask):
    """Greedy rectangle decomposition: grow row runs downwards, then fuse
    horizontally adjacent rectangles that share the same row span.

    Keeps the collider count low enough that DART stays real time - the raw
    per-cell decomposition of this map is ~10k boxes.
    """
    height, _ = mask.shape
    rects = []
    active = {}
    for r in range(height):
        spans = runs_of(mask[r])
        still = {}
        for c0, c1 in spans:
            rect = active.get((c0, c1))
            if rect is not None:
                rect[3] = r
            else:
                rect = [c0, c1, r, r]
                rects.append(rect)
            still[(c0, c1)] = rect
        active = still

    # Fuse horizontal neighbours with an identical row span, repeatedly, so a
    # wall that is 3 cells thick becomes 3 long boxes instead of hundreds.
    changed = True
    while changed:
        changed = False
        rects.sort(key=lambda q: (q[2], q[3], q[0]))
        out = []
        for rect in rects:
            if out and out[-1][2:] == rect[2:] and out[-1][1] == rect[0]:
                out[-1][1] = rect[1]
                changed = True
            else:
                out.append(rect)
        rects = out
    return rects


def build_boxes(occupied, res, margin_m, margin_x, margin_y, block=1):
    """Obstacle cells that touch drivable floor, as world-frame boxes.

    `block` coarsens the band before the rectangle decomposition: an N x N
    group of cells becomes one obstacle cell if any of them is an obstacle.
    Coarsening cuts DART's per-step collision cost, but it also *grows* every
    wall by up to (N-1)/2 cells, and that is why the default is 1 (no
    coarsening): with the default 2 the physical walls were up to 0.1 m fatter
    than the occupancy grid, so a path that SMAC had certified as collision
    free (it checks the true footprint against the *grid*) could still be
    physically blocked and the robot would wedge against an invisible wall.
    Physics and costmap have to agree cell for cell, so use 1 and pay the
    extra colliders - see the step-size note in make_geomap_world.py.
    """
    from scipy import ndimage

    free = ~occupied
    cells = max(1, int(round(margin_m / res)))
    band = occupied & ndimage.binary_dilation(free, iterations=cells)
    if block > 1:
        h, w = band.shape
        hh, ww = (h // block) * block, (w // block) * block
        band = band[:hh, :ww].reshape(hh // block, block, ww // block, block).any(axis=(1, 3))
        res = res * block
    rects = merge_rectangles(band)

    # NOTE: the row/column indices below are into the (possibly coarsened)
    # band, so the grid height must come from `band` - using the full-resolution
    # `occupied.shape` together with the coarsened `res` shifts every box by
    # (block-1) * rows * res in y, which puts the colliders outside the map.
    height, width = band.shape
    boxes = []
    for c0, c1, r0, r1 in rects:
        # image row 0 is the top of the map, i.e. the largest y
        x0 = margin_x + c0 * res
        x1 = margin_x + c1 * res
        y1 = margin_y + (height - r0) * res
        y0 = margin_y + (height - r1 - 1) * res
        boxes.append((x0, y0, x1, y1))
    return boxes


def write_model_sdf(path, model_name, boxes, height, base_z):
    lines = [
        '<?xml version="1.0" ?>',
        "<!-- generated by prepare_map_mesh.py (visual) + make_collision_boxes.py "
        "(collision) -->",
        "<!-- do not edit by hand: rerun tools/make_collision_boxes.py -->",
        '<sdf version="1.10">',
        f'  <model name="{model_name}">',
        "    <static>true</static>",
        "    <pose>0 0 0 0 0 0</pose>",
        '    <link name="map">',
        '      <visual name="visual">',
        "        <geometry>",
        "          <mesh>",
        f"            <uri>model://{model_name}/meshes/{model_name}.obj</uri>",
        "          </mesh>",
        "        </geometry>",
        "      </visual>",
    ]
    zc = base_z + height / 2.0
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        sx, sy = x1 - x0, y1 - y0
        lines += [
            f'      <collision name="wall_{i:04d}">',
            f"        <pose>{0.5*(x0+x1):.3f} {0.5*(y0+y1):.3f} {zc:.3f} 0 0 0</pose>",
            "        <geometry>",
            f"          <box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box>",
            "        </geometry>",
            "      </collision>",
        ]
    lines += ["    </link>", "  </model>", "</sdf>", ""]
    Path(path).write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="src/hetero_fleet_bringup/maps/geomap",
                    help="basename of the .pgm/.yaml pair (map_server format)")
    ap.add_argument("--model",
                    default="src/hetero_fleet_gz/models/geomap_colour_noroof/model.sdf")
    ap.add_argument("--margin", type=float, default=0.30,
                    help="box obstacles within this distance of drivable floor. "
                         "Only the shell around drivable space matters: the "
                         "robots can never reach the deeper occupied cells, and "
                         "boxing all of them costs thousands of colliders.")
    ap.add_argument("--height", type=float, default=1.5,
                    help="collider height (robot lidars sit at 0.39-0.52 m)")
    ap.add_argument("--block", type=int, default=1,
                    help="coarsen the collider grid by this factor (default 1: "
                         "5 cm boxes, exactly the occupancy grid. Raising it "
                         "makes the physical walls fatter than the map, which "
                         "wedges robots on paths the planner thinks are clear)")
    ap.add_argument("--base-z", type=float, default=-0.05,
                    help="collider bottom, just under the world ground plane")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()

    meta = read_yaml(a.map + ".yaml")
    img = read_pgm(a.map + ".pgm")
    res = meta["resolution"]
    occupied = img < 128           # map_server negate=0: black is an obstacle
    x0, y0 = meta["origin"][0], meta["origin"][1]

    boxes = build_boxes(occupied, res, a.margin, x0, y0, a.block)
    model_path = Path(a.model)
    model_name = model_path.parent.name
    write_model_sdf(model_path, model_name, boxes, a.height, a.base_z)

    area = sum((b[2] - b[0]) * (b[3] - b[1]) for b in boxes)
    print(f"map        : {img.shape[1]}x{img.shape[0]} @ {res} m, "
          f"origin ({x0:.3f}, {y0:.3f}), collider grid {res*a.block:.3f} m")
    print(f"obstacles  : {int(occupied.sum())} cells "
          f"({occupied.sum()*res*res:.1f} m^2), free {int((~occupied).sum())} cells")
    print(f"colliders  : {len(boxes)} boxes, {area:.1f} m^2 footprint, "
          f"height {a.height:.2f} m")
    print(f"wrote      : {model_path}")

    if a.report:
        xs = np.array([b[0] for b in boxes] + [b[2] for b in boxes])
        print(f"  x range  : {xs.min():+.2f} .. {xs.max():+.2f}")
        for name, (px, py) in {
            "spawn light_1": (-7.66, -0.20),
            "spawn heavy_1": (-0.01, 0.30),
            "spawn omni_1": (12.44, -0.25),
        }.items():
            col = int((px - x0) / res)
            row = int(occupied.shape[0] - (py - y0) / res)
            print(f"  {name:14s} on free cell: {not occupied[row, col]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

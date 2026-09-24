#!/usr/bin/env python3
"""Pick spawn poses and pick/drop/charger zones that a robot can actually use.

A scanned building is not a warehouse: the drivable space of this map is one
long open aisle plus a lot of pockets between furniture. Choosing zone
coordinates by eye is how you get the classic failures - "goal is occupied",
"no valid path found", a robot teleporting through a wall - so this script
does it from the occupancy grid instead:

  1. build the same map Nav2 will load (make_nav2_map.build_map),
  2. keep the connected region that contains a given seed (the map's main
     aisle), eroded by the clearance the robots need,
  3. greedily pick well-separated spots inside it, always preferring the
     roomiest cells,
  4. print the result as the SDF zone markers and the YAML fleet entries,
     together with a reachability check between every pair of spots.

Run it again after changing the robot footprints: the clearance requirement is
derived from robot_profiles.yaml, so the spots move with the robots.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

import make_nav2_map as nav
import map_common as mc


def cell_of(x, y, meta):
    """World (x, y) -> (row, col). Row 0 is the TOP of the grid, i.e. max y."""
    res, H = meta["res"], meta["H"]
    return int((meta["ymin"] + H * res - y) / res), int((x - meta["xmin"]) / res)


def xy_of(r, c, meta):
    """(row, col) -> the world coordinates of that cell's centre."""
    res, H = meta["res"], meta["H"]
    return meta["xmin"] + (c + 0.5) * res, meta["ymin"] + (H - r - 0.5) * res


def clearance_field(mapdir, res, z_limit, cover_close, structure_close,
                    smooth=1, structure_dilate=0, min_width=0.60):
    """Clearance (metres) to the nearest obstacle, on the Nav2 map's own grid.

    Delegates to make_nav2_map.build_map, so the spots this script picks are
    chosen on exactly the grid Nav2 will load - not on an approximation of it.
    """
    occupied, meta = nav.build_map(mapdir["visual"], mapdir["collision"], res,
                                   z_limit=z_limit, cover_close=cover_close,
                                   structure_close=structure_close,
                                   smooth=smooth,
                                   structure_dilate=structure_dilate,
                                   min_width=min_width,
                                   verbose=False)
    dist = ndimage.distance_transform_edt(~occupied) * res
    return dist, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("profiles", help="robot_profiles.yaml (for the footprints)")
    ap.add_argument("--seed", nargs=2, type=float, default=[-0.01, 0.30],
                    help="a point known to be in the main drivable area")
    ap.add_argument("--model-name", default="geomap_final_noroof",
                    help="model directory under src/hetero_fleet_gz/models/")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--z-limit", type=float, default=None,
                    help="optional obstacle-height filter; default uses the "
                         "collision mesh as built by prepare_map_mesh.py")
    ap.add_argument("--cover-close", type=float, default=0.5)
    ap.add_argument("--structure-close", type=float, default=0.1)
    ap.add_argument("--smooth", type=int, default=1,
                    help="must match the --smooth used by make_nav2_map.py so "
                         "the spots are chosen on the very grid Nav2 loads")
    ap.add_argument("--structure-dilate", type=int, default=0,
                    help="must match make_nav2_map.py's --structure-dilate")
    ap.add_argument("--min-width", type=float, default=0.60,
                    help="must match make_nav2_map.py's --min-width")
    ap.add_argument("--spacing", type=float, default=1.6,
                    help="minimum distance between chosen spots")
    ap.add_argument("--spot-margin", type=float, default=0.15,
                    help="extra clearance a chosen spot must have on top of "
                         "what the robots need, so a robot standing on it is "
                         "not already scraping the inflation ring")
    ap.add_argument("--spawns", type=int, default=None,
                    help="how many spawns to pick (default: one per robot in "
                         "the fleet list of robot_profiles.yaml)")
    ap.add_argument("--zones", type=int, default=9)
    a = ap.parse_args()

    tree = Path(__file__).resolve().parents[1]
    model_dir = tree / "src/hetero_fleet_gz/models" / a.model_name / "meshes"
    mapdir = {
        "visual": model_dir / f"{a.model_name}.obj",
        "collision": model_dir / f"{a.model_name}_collision.obj",
    }
    for p in mapdir.values():
        if not p.exists():
            raise SystemExit(f"missing {p}; run prepare_map_mesh.py first")

    with open(a.profiles) as f:
        profiles = yaml.safe_load(f)
    types = profiles["robot_types"]
    if a.spawns is None:
        a.spawns = len(profiles["fleet"])

    # The clearance a robot needs to plan through a cell is its inscribed
    # radius plus the costmap inflation radius it is configured with. Take the
    # worst robot, so every spot works for the whole fleet.
    need = 0.0
    for name, t in types.items():
        length, width = t["footprint"]
        inscribed = min(length, width) / 2.0
        inflation = float(t.get("inflation_radius", 0.22))
        print(f"{name}: footprint {length} x {width}, inscribed {inscribed:.2f}, "
              f"inflation {inflation:.2f} -> needs {inscribed + inflation:.2f} m")
        need = max(need, inscribed + inflation)
    print(f"worst-case clearance requirement: {need:.2f} m\n")

    dist, meta = clearance_field(mapdir, a.res, a.z_limit,
                                 a.cover_close, a.structure_close, a.smooth,
                                 a.structure_dilate, a.min_width)
    x0, y0, W, H = meta["xmin"], meta["ymin"], meta["W"], meta["H"]
    passable = dist >= need
    lab, n = ndimage.label(passable)
    seed_rc = cell_of(a.seed[0], a.seed[1], meta)
    if lab[seed_rc] == 0:
        print(f"seed {a.seed} is not passable at {need:.2f} m clearance; "
              f"the map's open area is elsewhere")
        order = np.argsort(ndimage.sum(passable, lab, range(1, n + 1)))[::-1]
        for i in order[:5]:
            r, c = np.argwhere(lab == i + 1).mean(axis=0).astype(int)
            print(f"   candidate region: ({xy_of(r, c, meta)[0]:.1f}, "
                  f"{xy_of(r, c, meta)[1]:.1f})")
        return 1

    region = lab == lab[seed_rc]
    print(f"main drivable region: {region.sum()*a.res**2:.1f} m^2")
    rows, cols = np.where(region)
    print(f"   x [{x0+cols.min()*a.res:.2f}, {x0+(cols.max()+1)*a.res:.2f}]  "
          f"y [{xy_of(rows.max(), 0, meta)[1]-a.res/2:.2f}, "
          f"{xy_of(rows.min(), 0, meta)[1]+a.res/2:.2f}]")
    print(f"   max clearance inside: {dist[region].max():.2f} m")

    # Pick the spots by farthest-point sampling: start at the roomiest cell and
    # always add whichever candidate is furthest from everything chosen so far.
    # Taking the roomiest cells greedily instead would cluster every marker in
    # the middle of the aisle and leave the two ends unused, which makes the
    # travel-time part of the scheduler's cost meaningless.
    # Only cells with room to spare may host a marker or a spawn: a spot that
    # only just satisfies `need` leaves the robot standing inside its own
    # inflation ring, where SMAC answers "start occupied".
    comfy = region & (dist >= need + a.spot_margin)
    if comfy.sum() < 1:
        print("no cell has the requested spare clearance; lower --spot-margin")
        return 1
    rows, cols = np.where(comfy)
    print(f"cells with >= {need + a.spot_margin:.2f} m clearance: "
          f"{comfy.sum()*a.res**2:.1f} m^2")
    cells = np.stack([rows, cols], axis=1)
    xy = np.stack([np.array([xy_of(r, c, meta)[0] for r, c in cells]),
                   np.array([xy_of(r, c, meta)[1] for r, c in cells])], axis=1)
    # Clearance of each candidate, used to bias the spread sampling below.
    room = dist[rows, cols]
    want = a.spawns + a.zones
    picked_idx = [int(np.argmax(room))]
    d2 = ((xy - xy[picked_idx[0]]) ** 2).sum(axis=1)
    while len(picked_idx) < want:
        # Farthest-point sampling alone parks every marker that is not the
        # first one on the edge of the passable region - it maximises spread
        # and ignores how tight a cell is, so the chosen zones end up with the
        # minimum clearance the filter allows (~0.54 m here) even though the
        # map has several spots with 1 m+. Score candidates by
        # "distance from what is already chosen" x "clearance" instead, so the
        # spots are both spread out and roomy.
        score = np.sqrt(np.maximum(d2, 0.0)) * room
        best = int(np.argmax(np.where(d2 >= a.spacing ** 2, score, -1.0)))
        if d2[best] < a.spacing ** 2:
            print(f"only {len(picked_idx)} spots fit at {a.spacing} m spacing "
                  f"in this region; lower --spacing for more")
            break
        picked_idx.append(best)
        d2 = np.minimum(d2, ((xy - xy[best]) ** 2).sum(axis=1))
    picked = [(float(xy[i, 0]), float(xy[i, 1]), float(dist[rows[i], cols[i]]))
              for i in picked_idx]
    picked.sort(key=lambda t: t[0])

    print(f"\npicked {len(picked)} spots (>= {a.spacing} m apart):")
    for i, (x, y, d) in enumerate(picked):
        print(f"   {i:2d} ({x:6.2f}, {y:6.2f})  clearance {d:.2f} m")

    # Reachability matrix: in one connected region everything is reachable in
    # principle, but a pair closer than the separation is a scheduling hazard,
    # so report the ones the slot logic would have to place side by side.
    print("\nclosest pairs (both robots at the same zone share an approach slot):")
    for i in range(len(picked)):
        for j in range(i + 1, len(picked)):
            d = ((picked[i][0] - picked[j][0]) ** 2
                 + (picked[i][1] - picked[j][1]) ** 2) ** 0.5
            if d < 2.0:
                print(f"   {i}-{j}: {d:.2f} m")

    # Spawns go to the ROOMIEST spots, not to the leftmost ones. A robot that
    # starts in a tight corner spends its first metres scraping a wall, and
    # with the true (larger) footprint it can even start inside its own
    # inflation ring, where the planner answers "Start occupied" and the first
    # task fails. Zones take whatever is left.
    by_clearance = sorted(range(len(picked)), key=lambda i: -picked[i][2])
    spawn_idx = sorted(by_clearance[:a.spawns])
    zone_idx = [i for i in range(len(picked)) if i not in spawn_idx]
    spawns = [picked[i] for i in spawn_idx]
    zones = [picked[i] for i in zone_idx]
    print("\nspawns (roomiest first):")
    for i in spawn_idx:
        print(f"   ({picked[i][0]:6.2f}, {picked[i][1]:6.2f})  "
              f"clearance {picked[i][2]:.2f} m")
    names = ["pickup_A", "pickup_B", "pickup_C",
             "dropoff_A", "dropoff_B", "dropoff_C",
             "charger_A", "charger_B", "charger_C", "charger_D"]
    print("\n# --- robot_profiles.yaml fleet entries ---")
    for (x, y, _), name in zip(spawns, profiles["fleet"]):
        print(f"  # {name['name']}")
    for (x, y, _), entry in zip(spawns, profiles["fleet"]):
        print(f"  - name: {entry['name']}\n    type: {entry['type']}\n"
              f"    spawn: [{x:.2f}, {y:.2f}, 0.0]")
    print("\n# --- world zone markers ---")
    for (x, y, _), name in zip(zones, names):
        print(f'    ("{name}", {x:6.2f}, {y:6.2f}),')
    return 0


if __name__ == "__main__":
    sys.exit(main())

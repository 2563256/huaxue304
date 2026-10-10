#!/usr/bin/env python3
"""Offline sanity check of the scene: map x robots x zones.

Everything the fleet needs is written in three places that can silently
disagree - the occupancy map, the robot profiles and the zone markers in the
world - and when they disagree the symptom is always the same: robots that
never move, or "goal rejected" in the logs twenty minutes into a run.

This script reads all three and checks, per robot type:

  * every spawn and every zone marker sits on a free cell of the map,
  * that cell clears the padded circumscribed footprint plus half a cell diagonal,
  * spawns, zones and every pickup/dropoff pairing share one connected region
    of cells that satisfy that clearance - i.e. Nav2 can plan between them.

Run it after editing robot_profiles.yaml or the zone table:

    python3 tools/verify_scene.py
"""

import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

ZONE_NAMES = ["pickup_A", "pickup_B", "pickup_C",
              "dropoff_A", "dropoff_B", "dropoff_C",
              "charger_A", "charger_C", "charger_D"]


def load_map(yaml_path):
    meta = yaml.safe_load(Path(yaml_path).read_text())
    pgm = Path(yaml_path).with_name(meta["image"])
    img = np.asarray(Image.open(pgm).convert("L"))
    occupied = img < 128  # negate: 0 -> dark = obstacle
    res = float(meta["resolution"])
    ox, oy = meta["origin"][0], meta["origin"][1]
    return occupied, res, ox, oy


def world_to_cell(x, y, res, ox, oy, shape):
    h, w = shape
    c = (x - ox) / res
    r = h - (y - oy) / res  # PGM row 0 is the top = max y
    return int(round(c)), int(round(r))


def parse_zones(world):
    import re
    zones = {}
    text = Path(world).read_text()
    for name, x, y in re.findall(
            r"<model name='zone_(\w+)'>\s*<static>true</static>\s*<pose>([-\d.]+) ([-\d.]+)", text):
        zones[name] = (float(x), float(y))
    return zones


def main():
    tree = Path(__file__).resolve().parents[1]
    yaml_path = tree / "src/hetero_fleet_bringup/maps/geomap.yaml"
    world = tree / "src/hetero_fleet_gz/worlds/geomap_fleet.sdf"
    profiles_path = tree / "src/hetero_fleet_description/config/robot_profiles.yaml"
    for p in (yaml_path, world, profiles_path):
        if not p.exists():
            raise SystemExit(f"missing {p}")

    occupied, res, ox, oy = load_map(yaml_path)
    H, W = occupied.shape
    free = ~occupied
    dist = ndimage.distance_transform_edt(free) * res
    profiles = yaml.safe_load(profiles_path.read_text())
    types = profiles["robot_types"]
    fleet = profiles["fleet"]
    zones = parse_zones(world)

    print(f"map    : {yaml_path.name} {W}x{H} @ {res} m, origin ({ox:.2f}, {oy:.2f}), "
          f"free {free.sum()*res*res:.1f} m^2")
    print(f"world  : {world.name}, {len(zones)} zone markers")
    print(f"fleet  : {[r['name'] for r in fleet]}")

    missing = [z for z in ZONE_NAMES if z not in zones]
    if missing:
        print(f"zones declared in this script but not in the world: {missing}")

    print("\nper-robot clearance check (spawn, then every zone):")
    ok = True
    fields = {}
    for entry in fleet:
        name = entry["name"]
        t = types[entry["type"]]
        length, width = t["footprint"]
        # Inflation is a soft cost field, not extra robot body width. Use a
        # conservative all-orientation circle around the padded rectangle,
        # with half a grid-cell diagonal for raster quantisation.
        padding = float(t.get("footprint_padding", 0.0))
        need = np.hypot(length / 2.0 + padding, width / 2.0 + padding) + res / np.sqrt(2.0)
        passable = dist >= need
        lab, _ = ndimage.label(passable)

        sx, sy, _ = entry["spawn"]
        sc = world_to_cell(sx, sy, res, ox, oy, (H, W))
        print(f"  {name} ({entry['type']}): footprint {length}x{width}, "
              f"needs {need:.2f} m clearance")
        marks = [("spawn", (sx, sy))] + [(z, zones[z]) for z in ZONE_NAMES if z in zones]
        labels = {}
        for label, (x, y) in marks:
            c, r = world_to_cell(x, y, res, ox, oy, (H, W))
            if not (0 <= c < W and 0 <= r < H):
                print(f"     {label:10s} ({x:6.2f},{y:6.2f}) OUTSIDE the map")
                ok = False
                continue
            d = dist[r, c]
            comp = lab[r, c]
            labels[label] = comp
            flag = "ok  " if d >= need and comp else "FAIL"
            if flag == "FAIL":
                ok = False
            print(f"     {flag} {label:10s} ({x:6.2f},{y:6.2f})  clearance {d:.2f} m  "
                  f"region {comp}")
        comps = {c for c in labels.values() if c}
        if len(comps) > 1:
            print(f"     FAIL: {len(comps)} disconnected regions among spawn+zones "
                  f"-> tasks between them are impossible")
            ok = False
        fields[name] = passable

    print("\nzone to zone straight-line distances (m):")
    picks = [z for z in ZONE_NAMES if z.startswith("pickup")]
    drops = [z for z in ZONE_NAMES if z.startswith("dropoff")]
    if picks and drops:
        print("            " + "".join(f"{d:>10s}" for d in drops))
        for p in picks:
            row = "".join(
                f"{np.hypot(zones[p][0]-zones[d][0], zones[p][1]-zones[d][1]):10.2f}"
                for d in drops)
            print(f"   {p:9s}{row}")

    print("\nRESULT:", "scene is consistent" if ok else "PROBLEMS FOUND")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

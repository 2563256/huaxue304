#!/usr/bin/env python3
"""Turn the raw scan mesh into a Gazebo model this workspace can drive on.

Input : the OBJ produced by pcd_to_gazebo.py (raw scan axes, tilted floor)
Output: a model directory with

  meshes/<name>.obj            visual mesh, copied line-for-line except for the
                               vertex positions, so the baked colour atlas and
                               its UVs survive untouched
  meshes/<name>_collision.obj  the same surface with the FLOOR REMOVED, so the
                               robots ride the world's flat ground plane
                               instead of the noisy scanned floor
  model.sdf / model.config     visual only, no pose (the level transform is
                               baked into the vertices)

NOTE ON COLLISION. model.sdf deliberately contains NO collision geometry. The
_collision.obj written here is only an *input* to make_nav2_map.py, which turns
it into the occupancy grid; the physics colliders are then generated from that
grid by tools/make_collision_boxes.py (axis-aligned boxes). Colliding against
the 209k-triangle mesh does not work in this install: DART segfaults on it, and
Bullet-Featherstone is the engine that cannot rotate a wheeled base on this map
(see make_geomap_world.py and make_collision_boxes.py). If you rerun this
script, rerun the two steps after it as well.

Why split visual and collision: the scan's floor is flat only to +-6 cm, and
keeping it as collision makes every wheel follow that noise while the world
ground plane fights it. Dropping the near-horizontal faces at floor level
leaves walls and furniture, which is exactly what the robots must not drive
through; the ground plane then provides a clean, level floor.
"""

import argparse
import shutil
from pathlib import Path

import numpy as np

import map_common as mc


def parse_source(path):
    """Read the OBJ keeping the raw line structure (UVs, materials, faces)."""
    verts = []
    lines = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            lines.append(line.rstrip("\n"))
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
    return np.asarray(verts, dtype=np.float64), lines


def write_visual(dst, lines, level, header):
    """Rewrite only the `v` lines; every other line (vt, f, mtllib) is verbatim.

    Vertex and UV indices must not move, otherwise the baked colour atlas
    decorates the wrong voxels - so the source line order is preserved exactly
    and only the three numbers on each `v` line are recomputed.
    """
    R = mc._rpy_matrix(level[0], level[1], 0.0)
    dz = level[2]
    out = []
    for line in lines:
        if line.startswith("v "):
            x, y, z = (float(t) for t in line.split()[1:4])
            x, y, z = R @ np.array([x, y, z]) + np.array([0.0, 0.0, dz])
            out.append(f"v {x:.4f} {y:.4f} {z:.4f}")
        elif line.startswith("#"):
            continue
        else:
            out.append(line)
    with open(dst, "w") as f:
        f.write(f"# {header}\n")
        f.write("\n".join(out) + "\n")
    return out


def write_collision(dst, V, F, header, keep_above=0.80):
    """Keep only the structure a robot cannot drive under: walls and furniture
    that reach above `keep_above` metres.

    The first version of this filter only removed the floor, and the resulting
    world was unusable: the scan of a lab ring corridor is full of chairs,
    carts, boxes and floor-level clutter, and treating every one of them as an
    obstacle chopped the corridor into ~60 pockets. The room is a ring -
    measured on the raw scan, the free space is one 102 m^2 loop around a
    31 m^2 core - and it only reappears once clutter that is *entirely* below
    head height is ignored.

    The map is built from this same file, so physics and costmap agree by
    construction: what the map calls free floor really is drivable, and low
    clutter stays visible (the visual mesh is untouched) but no longer blocks
    the way.
    """
    area, nz, _ = mc.face_geometry(V, F)
    tri = V[F]
    # Keep geometry that reaches above the threshold: walls, partitions, tall
    # cabinets. KNOWN LIMITATION: this keeps only the part of a wall that is
    # above the threshold, so the physical wall starts at 0.8 m and the robots
    # (0.4 m tall, lidar at 0.34 m) cannot see its lower half even though the
    # map - a top-down projection - still shows it. Doing it by `zmin` instead
    # brings the floor and all the clutter back and re-fragments the map, so
    # the real fix is to extrude the kept faces down to z=0 (see MAP_SWAP.md
    # 6.0); until then the safe direction is "map stricter than physics".
    low = tri[:, :, 2].max(axis=1) <= keep_above
    keep = F[~low]
    removed_area = float(area[low].sum())

    # Deduplicate vertices so the collision file stays a sensible size.
    uniq, inverse = np.unique(keep.reshape(-1), return_inverse=True)
    new_faces = inverse.reshape(-1, 3)
    mc.write_obj(dst, V[uniq], new_faces, header=header)
    return {
        "faces_in": int(len(F)),
        "faces_kept": int(len(new_faces)),
        "faces_dropped_below": int(low.sum()),
        "area_dropped": removed_area,
        "verts_kept": int(len(uniq)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src_obj")
    ap.add_argument("out_dir", help="model directory to create")
    ap.add_argument("--name", default="geomap_colour_noroof")
    ap.add_argument("--keep-above", type=float, default=0.80,
                    help="only geometry reaching above this height (floor "
                         "relative, measured after levelling) stays in the "
                         "collision mesh; everything lower - floor, tape, "
                         "chairs, boxes - is visual-only")
    a = ap.parse_args()

    src = Path(a.src_obj)
    out = Path(a.out_dir)
    (out / "meshes").mkdir(parents=True, exist_ok=True)

    V, lines = parse_source(src)
    _, F = mc.load_obj(src)
    print(f"source: {src}  vertices={len(V)} faces={len(F)}")

    _, stats = mc.fit_floor_plane(V, F)
    print(f"floor fit: tilt={stats['tilt_deg']:.2f} deg  "
          f"residual std={100*stats['residual_std']:.1f} cm  "
          f"p5..p95 = {100*stats['residual_p5']:.1f}..{100*stats['residual_p95']:.1f} cm")

    level, lstats = mc.level_transform(V, F)
    print(f"level transform: roll={lstats['roll_deg']:+.3f} deg  "
          f"pitch={lstats['pitch_deg']:+.3f} deg  dz={lstats['z_shift']:+.4f} m")

    vis = out / "meshes" / f"{a.name}.obj"
    write_visual(vis, lines, level, f"levelled {src.name} ({a.name})")
    print(f"wrote {vis} ({vis.stat().st_size/1e6:.1f} MB)")

    for extra in (src.with_suffix(".mtl"), src.parent / f"{src.stem}_atlas.png"):
        if extra.exists():
            shutil.copy2(extra, out / "meshes" / extra.name)
            print(f"copied {extra.name}")

    levelled = mc.apply_level(V, level)
    col = out / "meshes" / f"{a.name}_collision.obj"
    cstats = write_collision(col, levelled, F, f"collision {a.name}",
                             keep_above=a.keep_above)
    print(f"wrote {col} ({col.stat().st_size/1e6:.1f} MB)")
    print("  collision: " + ", ".join(f"{k}={v}" for k, v in cstats.items()))

    # Floor flatness after levelling, per 5 cm slice: how much of the walking
    # surface is within the slab that the collision filter removes.
    z = levelled[:, 2]
    frac = float((z < a.keep_above).mean())
    print(f"  levelled vertices below {a.keep_above} m (visual only): {100*frac:.1f}%")
    print(f"  levelled z range: {z.min():+.3f} .. {z.max():+.3f}")

    (out / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{a.name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author><name>pcd_to_gazebo.py + prepare_map_mesh.py</name></author>
  <description>Colour point-cloud scan levelled to z=0. Visual only: the
  physics colliders are boxes generated from the Nav2 occupancy grid by
  tools/make_collision_boxes.py, so physics and costmap agree.</description>
</model>
"""
    )
    (out / "model.sdf").write_text(
        f"""<?xml version="1.0" ?>
<!-- generated by prepare_map_mesh.py - visual only.
     The collision geometry is generated from the occupancy grid by
     tools/make_collision_boxes.py; run it after tools/make_nav2_map.py. -->
<sdf version="1.10">
  <model name="{a.name}">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="map">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{a.name}/meshes/{a.name}.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
    )
    print(f"wrote {out/'model.sdf'} and {out/'model.config'}")


if __name__ == "__main__":
    main()

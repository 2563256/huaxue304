#!/usr/bin/env python3
"""Measure a robot's true ground footprint from its (rendered) URDF.

Why this exists: robot_profiles.yaml used to declare footprints that were
15-25% smaller than the bodies they described, so Nav2 planned with an
optimistic footprint and tools/verify_scene.py checked clearance against the
same optimistic number. The numbers in that file now come from this script.

It walks the kinematic tree from the root link, accumulates every joint
transform at its nominal (zero) position, and takes the axis-aligned bounding
box of ALL collision shapes - bumper, corner guards and drive wheels included.
The wheels are usually the widest part, which is exactly what the old numbers
missed.

    xacro src/hetero_fleet_description/urdf/light_diff_amr.xacro \
        robot_name:=chk controllers_file:=generated/controllers/chk.yaml \
        > generated/light.urdf
    python3 tools/measure_footprint.py generated/light.urdf

The printed length x width is what belongs in robot_profiles.yaml `footprint`
and in the costmap polygon of nav2_params_<type>.yaml.

Walks the kinematic tree from the root link, accumulates the fixed/revolute
joint transforms at their nominal (zero) positions, and returns the
axis-aligned bounding box of every collision shape in the root frame.
"""

import math
import subprocess
import sys
import xml.etree.ElementTree as ET


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rpy_of(elem, tag="origin"):
    node = elem.find(tag)
    if node is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in node.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in node.get("rpy", "0 0 0").split()]
    return xyz, rpy


def shape_half_extents(geom):
    box = geom.find("box")
    if box is not None:
        s = [float(v) for v in box.get("size").split()]
        return [s[0] / 2, s[1] / 2, s[2] / 2]
    cyl = geom.find("cylinder")
    if cyl is not None:
        r, l = float(cyl.get("radius")), float(cyl.get("length"))
        return [r, r, l / 2]
    sph = geom.find("sphere")
    if sph is not None:
        r = float(sph.get("radius"))
        return [r, r, r]
    raise ValueError("unsupported geometry")


def main(path):
    root = ET.parse(path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    children = {}
    parent_of = {}
    for j in root.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        children.setdefault(p, []).append((c, j))
        parent_of[c] = p
    root_link = [n for n in links if n not in parent_of][0]

    lo = [1e9] * 3
    hi = [-1e9] * 3
    stack = [(root_link, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])]
    while stack:
        link, R, t = stack.pop()
        for coll in links[link].findall("collision"):
            xyz, rpy = rpy_of(coll)
            Rc = rpy_matrix(*rpy)
            half = shape_half_extents(coll.find("geometry"))
            Rt = mat_mul(R, Rc)
            tt = [t[i] + mat_vec(R, xyz)[i] for i in range(3)]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        v = mat_vec(Rt, [sx * half[0], sy * half[1], sz * half[2]])
                        for i in range(3):
                            lo[i] = min(lo[i], tt[i] + v[i])
                            hi[i] = max(hi[i], tt[i] + v[i])
        for child, joint in children.get(link, []):
            xyz, rpy = rpy_of(joint)
            Rj = rpy_matrix(*rpy)
            stack.append((child, mat_mul(R, Rj), [t[i] + mat_vec(R, xyz)[i] for i in range(3)]))

    print(f"{path.split('/')[-1]:24s} length(x)={hi[0]-lo[0]:.4f}  width(y)={hi[1]-lo[1]:.4f}  "
          f"height(z)={hi[2]-lo[2]:.4f}   x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}]")
    return (hi[0] - lo[0], hi[1] - lo[1])


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)

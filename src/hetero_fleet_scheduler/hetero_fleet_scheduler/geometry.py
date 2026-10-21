"""Static rectangular body checks, independent of ROS and Nav2 tolerances."""
import math


def rectangles_overlap(a, a_size, b, b_size, margin=0.0):
    """Filled oriented rectangles, with a shared separation margin."""
    ca, sa, cb, sb = math.cos(a[2]), math.sin(a[2]), math.cos(b[2]), math.sin(b[2])
    axes_a, axes_b = ((ca, sa), (-sa, ca)), ((cb, sb), (-sb, cb))
    dx, dy = b[0] - a[0], b[1] - a[1]
    for ux, uy in (*axes_a, *axes_b):
        ra = sum(size / 2 * abs(ux * ax + uy * ay) for size, (ax, ay) in zip(a_size, axes_a))
        rb = sum(size / 2 * abs(ux * ax + uy * ay) for size, (ax, ay) in zip(b_size, axes_b))
        if abs(dx * ux + dy * uy) > ra + rb + margin:
            return False
    return True


def rectangle_is_free(occupancy, pose, footprint, padding=0.0):
    """Check all occupied cell areas with rectangle/cell separating axes."""
    x, y, yaw = pose
    length, width = footprint
    if not all(math.isfinite(v) for v in (x, y, yaw, length, width, padding)):
        return False
    if length <= 0 or width <= 0 or padding < 0:
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = length / 2 + padding, width / 2 + padding
    ax, ay = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
    res, ox, oy = occupancy['res'], occupancy['ox'], occupancy['oy']
    c0, c1 = math.floor((x - ax - ox) / res), math.floor((x + ax - ox) / res)
    r0, r1 = math.floor((y - ay - oy) / res), math.floor((y + ay - oy) / res)
    if c0 < 0 or r0 < 0 or c1 >= occupancy['w'] or r1 >= occupancy['h']:
        return False
    projection = res / 2 * (abs(c) + abs(s))
    for col in range(c0, c1 + 1):
        dx = ox + (col + .5) * res - x
        for row in range(r0, r1 + 1):
            value = occupancy['px'][col, occupancy['h'] - 1 - row]
            probability = value / 255 if occupancy.get('negate', 0) else 1 - value / 255
            if probability < occupancy.get('free_thresh', .196):
                continue
            dy = oy + (row + .5) * res - y
            if (abs(dx) <= ax + res / 2 and abs(dy) <= ay + res / 2
                    and abs(c * dx + s * dy) <= hx + projection
                    and abs(-s * dx + c * dy) <= hy + projection):
                return False
    return True

"""Geometry certificate for bounded native Spin recovery; no command output."""
import math
from .geometry import rectangle_is_free, rectangles_overlap


def heading_sweep_clear(occupancy,pose,target,footprint,padding,peers=()):
    if occupancy is None or not all(math.isfinite(v) for v in (*pose,target)):
        return False
    delta=math.atan2(math.sin(target-pose[2]),math.cos(target-pose[2]))
    radius=math.hypot(footprint[0]/2+padding,footprint[1]/2+padding)
    step=.0125
    n=max(1,math.ceil(radius*abs(delta)/step))
    own=[v+2*(padding+step/2) for v in footprint]
    for k in range(n+1):
        p=(pose[0],pose[1],pose[2]+delta*k/n)
        if not rectangle_is_free(occupancy,p,footprint,padding+step/2):return False
        if any(rectangles_overlap(p,own,other,size,.025) for other,size in peers):return False
    return True

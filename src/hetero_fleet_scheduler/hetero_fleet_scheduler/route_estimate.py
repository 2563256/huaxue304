"""Static map-aware travel estimates for scheduling, not executable paths.

A* uses the padded inscribed radius; it does not prove turning feasibility.
State Lattice and the physical reservation gate remain authoritative.
"""
import heapq
import math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
from .joint_schedule import Leg
from .reservations import ChannelReservations, intersects


class RouteEstimator:
    def __init__(self, map_path, profiles, channels, factor=1.5):
        self.factor = factor
        meta = yaml.safe_load(Path(map_path).read_text())
        pixels = np.asarray(Image.open(Path(map_path).parent/meta['image']).convert('L'))
        occupancy = pixels/255 if meta.get('negate',0) else 1-pixels/255
        self.free = np.flipud(occupancy < meta['free_thresh'])
        self.res = float(meta['resolution'])
        self.ox,self.oy = meta['origin'][:2]
        self.clearance = distance_transform_edt(self.free)*self.res
        self.bodies = {r['name']:(*profiles['robot_types'][r['type']]['footprint'],
                                 profiles['robot_types'][r['type']]['footprint_padding']) for r in profiles['fleet']}
        core = ChannelReservations(channels, self.bodies)
        self.regions = core.regions
        self.margin = core.margin
        self.cache = {}
        self.passable = {}
        self.max_cache = 5000

    def cell(self, pose):
        return (math.floor((pose.x-self.ox)/self.res), math.floor((pose.y-self.oy)/self.res))

    def __call__(self, robot, start, goal):
        a,b = self.cell(start),self.cell(goal)
        key = robot.id,a,b
        if key in self.cache:
            return self.cache[key]
        l,w,p = self.bodies[robot.id]
        if robot.id not in self.passable:
            self.passable[robot.id] = self.clearance >= min(l,w)/2+p+self.res/2
        free = self.passable[robot.id]
        h,width = free.shape
        valid = lambda c: 0<=c[0]<width and 0<=c[1]<h and free[c[1],c[0]]
        if not valid(a) or not valid(b):
            return None
        queue=[(math.dist(a,b),0.,a)]; costs={a:0.}; parent={}
        while queue:
            _,cost,c = heapq.heappop(queue)
            if cost != costs[c]:
                continue
            if c == b:
                break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=c[0]+dx,c[1]+dy
                if not valid(nxt) or (dx and dy and (not valid((c[0]+dx,c[1])) or not valid((c[0],c[1]+dy)))):
                    continue
                v=cost+math.hypot(dx,dy)
                if v < costs.get(nxt,math.inf):
                    costs[nxt]=v; parent[nxt]=c
                    heapq.heappush(queue,(v+math.dist(nxt,b),v,nxt))
        else:
            return None
        cells=[b]
        while cells[-1]!=a:
            cells.append(parent[cells[-1]])
        cells.reverse()
        groups=set()
        for i,c in enumerate(cells):
            nxt=cells[min(i+1,len(cells)-1)]
            yaw=math.atan2(nxt[1]-c[1],nxt[0]-c[0]) if nxt!=c else goal.yaw
            pose=(self.ox+(c[0]+.5)*self.res,self.oy+(c[1]+.5)*self.res,yaw)
            for group,regions in self.regions.items():
                if any(intersects(pose,(l,w),p+self.margin,bounds) for bounds in regions):
                    groups.add(group)
        distance=costs[b]*self.res
        result=Leg(max(.1,distance/max(robot.max_speed,.05)*self.factor),distance,frozenset(groups))
        if len(self.cache)>=self.max_cache:
            self.cache.clear()
        self.cache[key]=result
        return result

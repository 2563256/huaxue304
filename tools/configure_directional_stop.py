#!/usr/bin/env python3
"""Keep emergency stops in the commanded direction, allowing retreat."""
from pathlib import Path
import yaml,math
ROOT=Path(__file__).resolve().parents[1]
for kind in ['light','heavy','omni','g1']:
 p=ROOT/f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml';text=p.read_text();cfg=yaml.safe_load(text)
 f=yaml.safe_load(cfg['local_costmap']['local_costmap']['ros__parameters']['footprint']);hx=max(v[0] for v in f);hy=max(v[1] for v in f)
 def points(x0,x1,y0,y1):return str([[round(x1,4),round(y1,4)],[round(x1,4),round(y0,4)],[round(x0,4),round(y0,4)],[round(x0,4),round(y1,4)]])
 def zone(pts,lo,hi,a0=None,a1=None):
  z={'points':pts,'linear_min':lo,'linear_max':hi,'theta_min':-.10,'theta_max':.10}
  if a0 is not None:z.update(direction_start_angle=math.radians(a0),direction_end_angle=math.radians(a1))
  return z
 front=points(0,hx+.12,-hy-.05,hy+.05);rear=points(-hx-.12,0,-hy-.05,hy+.05)
 poly={'type':'velocity_polygon','action_type':'stop','min_points':2,'visualize':True,'polygon_pub_topic':'polygon_stop','holonomic':kind=='omni'}
 if kind=='omni':
  poly['velocity_polygons']=['forward','reverse_left','reverse_right','left','right','turning']
  poly['forward']=zone(front,.02,2.,-45.,45.)
  poly['reverse_left']=zone(rear,.02,2.,135.,180.)
  poly['reverse_right']=zone(rear,.02,2.,-180.,-135.)
  poly['left']=zone(points(-hx-.05,hx+.05,0,hy+.12),.02,2.,45.,135.)
  poly['right']=zone(points(-hx-.05,hx+.05,-hy-.12,0),.02,2.,-135.,-45.)
 else:
  poly['velocity_polygons']=['forward','reverse','turning']
  poly['forward']=zone(front,.02,2.)
  poly['reverse']=zone(rear,-2.,-.02)
 poly['turning']={'points':points(-hx-.03,hx+.03,-hy-.03,hy+.03),'linear_min':0. if kind=='omni' else -2.,'linear_max':2.,'theta_min':-100.,'theta_max':100.}
 cfg['collision_monitor']['ros__parameters']['PolygonStop']=poly
 text=text[:text.index('collision_monitor:\n')]+yaml.safe_dump({'collision_monitor':cfg['collision_monitor']},sort_keys=False)
 p.write_text(text)

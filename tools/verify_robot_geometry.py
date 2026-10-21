#!/usr/bin/env python3
"""Check physical collision extents, costmap bodies and wheel kinematics after scaling."""
from pathlib import Path
import math,xml.etree.ElementTree as ET
import numpy as np,yaml,xacro
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];desc=root/'src/hetero_fleet_description'
profiles=yaml.safe_load((desc/'config/robot_profiles.yaml').read_text())
def transform(el):
 t=np.eye(4)
 if el is not None:
  t[:3,3]=list(map(float,el.get('xyz','0 0 0').split()))
  t[:3,:3]=Rotation.from_euler('xyz',list(map(float,el.get('rpy','0 0 0').split()))).as_matrix()
 return t
for kind in ('light','heavy','omni'):
 typ={'light':'light_diff_amr','heavy':'heavy_diff_amr','omni':'omni_mecanum_amr'}[kind]
 robot=ET.fromstring(xacro.process_file(str(desc/'urdf'/f'{typ}.xacro')).toxml())
 poses={'base_footprint':np.eye(4)};joints=list(robot.findall('joint'))
 while joints:
  for j in joints[:]:
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent in poses:poses[child]=poses[parent]@transform(j.find('origin'));joints.remove(j)
 extents=[]
 for link in robot.findall('link'):
  for col in link.findall('collision'):
   t=poses[link.get('name')]@transform(col.find('origin'));g=list(col.find('geometry'))[0]
   if g.tag=='box':half=np.abs(t[:3,:3])@(np.array(list(map(float,g.get('size').split())))/2)
   elif g.tag=='sphere':half=np.full(3,float(g.get('radius')))
   elif g.tag=='cylinder':
    axis=t[:3,2];half=float(g.get('length'))/2*np.abs(axis)+float(g.get('radius'))*np.sqrt(np.maximum(0,1-axis**2))
   else:raise AssertionError(g.tag)
   extents.extend([t[:3,3]-half,t[:3,3]+half])
 extent=np.max(np.abs(np.array(extents)[:,:2]),axis=0)*2
 body=np.array(profiles['robot_types'][typ]['footprint']);assert np.allclose(extent,body,atol=1e-6),(kind,extent,body)
 cfg=yaml.safe_load((root/'src/hetero_fleet_bringup/config'/f'nav2_params_{kind}.yaml').read_text())
 for space in ('local_costmap','global_costmap'):
  fp=np.array(yaml.safe_load(cfg[space][space]['ros__parameters']['footprint']));assert np.allclose(fp.max(axis=0)*2,body)
 drive=yaml.safe_load((desc/'config'/f'{typ}_controllers.yaml').read_text())
 if kind!='omni':
  d=drive['/<robot_name>/diff_drive_controller']['ros__parameters']
  a=poses['left_wheel_link'][:3,3];b=poses['right_wheel_link'][:3,3]
  assert abs(np.linalg.norm(a-b)-d['wheel_separation'])<1e-8
  wheel=robot.find("link[@name='left_wheel_link']/collision/geometry/cylinder")
  assert abs(float(wheel.get('radius'))-d['wheel_radius'])<1e-8
 else:
  d=drive['/<robot_name>/mecanum_drive_controller']['ros__parameters']['kinematics']
  wheel=robot.find("link[@name='front_left_wheel_link']/collision/geometry/cylinder")
  assert abs(float(wheel.get('radius'))-d['wheels_radius'])<1e-8
  a=poses['front_left_wheel_link'][:3,3]
  assert abs(abs(a[0])+abs(a[1])-d['sum_of_robot_center_projection_on_X_Y_axis'])<1e-8
 print(kind,'PASS','physical collision',extent.tolist(),'matches planner and controller kinematics')

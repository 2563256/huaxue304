#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as E, shutil
import argparse
parser=argparse.ArgumentParser(description='Prepare the 0.90-scale G1 with fixed upper body and 12 active leg joints.')
parser.add_argument('source_dir',type=Path,help='G1 directory containing g1_23dof_mode_10.urdf and meshes/')
source=parser.parse_args().source_dir
root=Path(__file__).resolve().parents[1];dst=root/'src/hetero_fleet_description';s=.9
shutil.copytree(source/'meshes',dst/'meshes/g1',dirs_exist_ok=True)
shutil.copy(source/'g1_23dof_mode_10.urdf',dst/'urdf/g1_original.urdf')
r=E.parse(source/'g1_23dof_mode_10.urdf').getroot();r.set('name','g1_humanoid');r.set('xmlns:xacro','http://www.ros.org/wiki/xacro')
E.SubElement(r,'xacro:arg',name='robot_name',default='g1_1');E.SubElement(r,'xacro:arg',name='controllers_file',default='')
# Keep upper body rigid in the same neutral pose used to train the 12 leg DOF policy.
for j in r.findall('joint'):
 if j.get('type')!='fixed' and not any(j.get('name').startswith(side) for side in ['left_hip','right_hip','left_knee','right_knee','left_ankle','right_ankle']):
  j.set('type','fixed')
  for tag in ['axis','limit','dynamics','hardware']:
   for c in j.findall(tag):j.remove(c)
 for c in j.findall('hardware'):j.remove(c)
 if j.get('type')!='fixed':E.SubElement(j,'dynamics',damping='0.001',friction='0.0')
for o in r.iter('origin'):
 if 'xyz' in o.attrib:o.set('xyz',' '.join(str(float(v)*s) for v in o.get('xyz').split()))
for mesh in r.iter('mesh'):
 mesh.set('filename','package://hetero_fleet_description/meshes/g1/'+Path(mesh.get('filename')).name);mesh.set('scale',f'{s} {s} {s}')
for mass in r.iter('mass'):mass.set('value',str(float(mass.get('value'))*s**3))
for inertia in r.iter('inertia'):
 for k,v in list(inertia.attrib.items()):inertia.set(k,str(float(v)*s**5))
# DART-safe primitives: never use detailed triangle meshes for contact.
for link in r.findall('link'):
 for c in link.findall('collision'):link.remove(c)
 name=link.get('name')
 def box(xyz,size):
  c=E.SubElement(link,'collision');E.SubElement(c,'origin',xyz=' '.join(str(v*s) for v in xyz));g=E.SubElement(c,'geometry');E.SubElement(g,'box',size=' '.join(str(v*s) for v in size))
 if name.endswith('ankle_roll_link'):box([.035,0,-.0225],[.19,.09,.025])
 elif name=='pelvis':box([0,0,-.065],[.22,.24,.15])
 elif name=='torso_link':box([0,0,.17],[.23,.30,.34])
 elif name=='head_link':box([.01,0,.46],[.18,.18,.18])
 elif name.endswith('knee_link'):box([.008,0,-.14],[.085,.085,.24])
 elif name.endswith('hip_yaw_link'):box([-.035,0,-.10],[.09,.09,.17])
 elif name.endswith('elbow_link'):box([.065,0,0],[.16,.065,.065])
 color=link.find('visual/material/color')
 if color is not None:
  gz=E.SubElement(r,'gazebo',reference=name);m=E.SubElement(gz,'material');E.SubElement(m,'ambient').text=color.get('rgba');E.SubElement(m,'diffuse').text=color.get('rgba')
# Root reference at projected pelvis origin, feet rest at about 0.793*s below pelvis.
E.SubElement(r,'link',name='base_link');j=E.SubElement(r,'joint',name='g1_base_joint',type='fixed');E.SubElement(j,'parent',link='base_link');E.SubElement(j,'child',link='pelvis');E.SubElement(j,'origin',xyz=f'0 0 {0.793*s}')
# Mount above the head: low sensors see the moving legs as obstacles.
l=E.SubElement(r,'link',name='lidar_link');v=E.SubElement(l,'visual');g=E.SubElement(v,'geometry');E.SubElement(g,'cylinder',radius='.025',length='.035');j=E.SubElement(r,'joint',name='g1_lidar_joint',type='fixed');E.SubElement(j,'parent',link='pelvis');E.SubElement(j,'child',link='lidar_link');E.SubElement(j,'origin',xyz='0 0 0.56')
gz=E.SubElement(r,'gazebo',reference='lidar_link');sensor=E.SubElement(gz,'sensor',name='$(arg robot_name)_lidar',type='gpu_lidar')
for tag,value in [('always_on','1'),('visualize','false'),('update_rate','15'),('topic','/$(arg robot_name)/scan'),('frame_id','lidar_link')]:E.SubElement(sensor,tag).text=value
li=E.SubElement(sensor,'lidar');h=E.SubElement(E.SubElement(li,'scan'),'horizontal')
for tag,value in [('samples','720'),('resolution','1'),('min_angle','-3.14159'),('max_angle','3.14159')]:E.SubElement(h,tag).text=value
ra=E.SubElement(li,'range')
for tag,value in [('min','.12'),('max','12.0'),('resolution','.01')]:E.SubElement(ra,tag).text=value
for side in ['left','right']:
 gz=E.SubElement(r,'gazebo',reference=side+'_ankle_roll_link')
 for tag,value in [('mu1','1.0'),('mu2','1.0'),('maxContacts','4')]:E.SubElement(gz,tag).text=value
gz=E.SubElement(r,'gazebo');p=E.SubElement(gz,'plugin',filename='hetero_g1_locomotion',name='hetero_fleet::G1Locomotion')
for tag,value in [('robot_name','$(arg robot_name)'),('scale',str(s)),('policy_path','$(find hetero_fleet_gz)/policies/g1/motion.bin')]:E.SubElement(p,tag).text=value
E.indent(r);(dst/'urdf/g1_humanoid.xacro').write_text('<?xml version="1.0"?>\n'+E.tostring(r,encoding='unicode')+'\n')

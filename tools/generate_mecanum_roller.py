#!/usr/bin/python3
"""Generate a smooth barrel roller, with isotropic contact and a free axle."""
from pathlib import Path
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def main():
    radius=.0095;half_length=.0215
    vertices=[]
    for i in range(21):
        theta=math.pi*i/20
        for j in range(24):
            phi=2*math.pi*j/24
            vertices.append(np.array([radius*math.sin(theta)*math.cos(phi),radius*math.sin(theta)*math.sin(phi),half_length*math.cos(theta)]))
    triangles=[]
    for i in range(20):
        for j in range(24):
            a=i*24+j;b=i*24+(j+1)%24;c=(i+1)*24+j;d=(i+1)*24+(j+1)%24
            triangles.extend([(a,c,b),(b,c,d)])
    lines=['solid mecanum_barrel']
    for ids in triangles:
        v=[vertices[k] for k in ids];normal=np.cross(v[1]-v[0],v[2]-v[0]);size=np.linalg.norm(normal)
        if size<1e-12:continue
        normal/=size;lines.append(' facet normal '+' '.join(f'{x:.9g}' for x in normal));lines.append('  outer loop')
        lines.extend('   vertex '+' '.join(f'{x:.9g}' for x in point) for point in v)
        lines.extend(['  endloop',' endfacet'])
    lines.append('endsolid mecanum_barrel')
    (ROOT/'src/hetero_fleet_description/meshes/mecanum_barrel.stl').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()

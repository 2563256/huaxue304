#!/usr/bin/env python3
"""Apply the 2026-10-10 scene edits to a saved baseline, without narrowing aisles.

Run after the raw-scan generation pipeline. A baseline is kept under
 generated/scene_edit_baseline; repeated runs always read that same baseline.
The raw scan in assets/scan_sources is never edited.
"""
from pathlib import Path
import json
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml
from make_collision_boxes import build_boxes, merge_rectangles, write_model_sdf

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / 'src/hetero_fleet_bringup/maps/geomap.pgm'
MODEL = ROOT / 'src/hetero_fleet_gz/models/geomap_final_noroof'
BASE = ROOT / 'generated/scene_edit_baseline'
# Remove protrusions back to the neighbouring wall line, leaving the outer shell.
REGIONS = [
    ('south_middle', 5.30, 6.85, -1.60, -0.50),
    ('southeast', 11.35, 12.10, -1.65, -0.10),
    ('southwest', -5.85, -4.70, -1.65, -0.45),
    ('northwest', -2.10, -0.85, 3.90, 5.15),
]

def trim_visual(source, dest, removed, regions, ox, oy, res, floor_free):
    lines = source.read_text().splitlines()
    verts = np.array([[float(t) for t in ln.split()[1:4]] for ln in lines if ln.startswith('v ')])
    h,w = removed.shape
    out=[]; count=0
    for ln in lines:
        if not ln.startswith('f '):
            out.append(ln); continue
        ids=[int(t.split('/')[0])-1 for t in ln.split()[1:]]
        tri=verts[ids]
        # Replace the scanned floor and low voxel skirts with a single flat
        # visual surface. Keep wall texture indices and above-floor scan details.
        samples=np.vstack((tri,tri.mean(axis=0), (tri+np.roll(tri,1,axis=0))/2))
        c=np.floor((samples[:,0]-ox)/res).astype(int)
        r=h-1-np.floor((samples[:,1]-oy)/res).astype(int)
        valid=(r>=0)&(r<h)&(c>=0)&(c<w)
        if tri[:,2].max() <= .30:
            # Preserve low wall bases outside the walking surface. Only scan
            # terrain/skirts overlapping the new flat floor need removal.
            if floor_free[r[valid],c[valid]].any():
                count+=1
            else:
                out.append(ln)
            continue
        hit=bool(removed[r[valid],c[valid]].any())
        # Clear the whole above-floor object, not only its rasterised upper half.
        for _,x0,x1,y0,y1 in regions:
            hit |= bool(((samples[:,0]>=x0)&(samples[:,0]<=x1)&(samples[:,1]>=y0)&(samples[:,1]<=y1)).any())
        if hit:
            count+=1
        else:
            out.append(ln)
    dest.write_text('# scene edited 2026-10-10; floor and texture atlas retained\n'+'\n'.join(out)+'\n')
    return count

def main():
    BASE.mkdir(parents=True,exist_ok=True)
    for src in [MAP, MODEL/'meshes/geomap_final_noroof.obj']:
        dst=BASE/src.name
        if not dst.exists(): shutil.copy2(src,dst)
    meta=yaml.safe_load(MAP.with_suffix('.yaml').read_text())
    res=float(meta['resolution']);ox,oy,_=meta['origin']
    img=np.array(Image.open(BASE/'geomap.pgm'));old=img<128;h,w=old.shape
    xx=ox+(np.arange(w)+.5)*res;yy=oy+(h-np.arange(h)-.5)*res
    occ=old.copy();areas={}
    for name,x0,x1,y0,y1 in REGIONS:
        mask=(xx[None,:]>=x0)&(xx[None,:]<=x1)&(yy[:,None]>=y0)&(yy[:,None]<=y1)
        areas[name]=int((occ&mask).sum())*res**2;occ[mask]=False
    # Opening removes outward spikes; intersection makes the no-growth
    # invariant explicit, even at the border. Radius 3 = 15 cm at this map scale.
    gy,gx=np.mgrid[-3:4,-3:4];disk=gx*gx+gy*gy<=9
    opened=ndimage.binary_opening(occ,structure=disk,border_value=1)
    smooth_removed=occ&~opened;occ &= opened
    assert not np.any(occ & ~old), 'must never add obstacles to existing floor'
    removed=old&~occ
    Image.fromarray(np.where(occ,0,255).astype('uint8')).save(MAP)
    faces=trim_visual(BASE/'geomap_final_noroof.obj', MODEL/'meshes/geomap_final_noroof.obj', removed, REGIONS, ox,oy,res, ~occ)
    boxes=build_boxes(occ,res,.30,ox,oy,1)
    write_model_sdf(MODEL/'model.sdf',MODEL.name,boxes,1.5,-.05)
    # One continuous visual level for ALL drivable cells, coincident with
    # the physical world plane: MAP_Z (0.10) + local floor (0.017) = 0.117 m.
    # Its material matches the dominant original floor atlas RGB (99,128,169).
    # Adjacent rectangles share one exact height and material; no raised boxes.
    floor_rects=merge_rectangles(~occ)
    floor_verts=[];floor_faces=[]
    for c0,c1,r0,r1 in floor_rects:
        x0=ox+c0*res;x1=ox+c1*res;y1=oy+(h-r0)*res;y0=oy+(h-r1-1)*res
        n=len(floor_verts)
        floor_verts.extend([(x0,y0,.017),(x1,y0,.017),(x1,y1,.017),(x0,y1,.017)])
        floor_faces.extend([(n+1,n+2,n+3),(n+1,n+3,n+4)])
    (MODEL/'meshes/flat_floor.obj').write_text('# flat visual floor, local z=0.017 m\no flat_floor\nmtllib flat_floor.mtl\nusemtl flat_floor\n'+
        '\n'.join('v %.6f %.6f %.6f'%v for v in floor_verts)+'\n'+
        '\n'.join('f %d %d %d'%f for f in floor_faces)+'\n')
    (MODEL/'meshes/flat_floor.mtl').write_text('newmtl flat_floor\nKa 0.388235294 0.501960784 0.662745098\nKd 0.388235294 0.501960784 0.662745098\nKs 0.1 0.1 0.1\nd 1.0\nillum 2\n')
    tree=ET.parse(MODEL/'model.sdf');link=tree.getroot().find('model/link')
    vis=ET.SubElement(link,'visual',name='flat_floor')
    geom=ET.SubElement(vis,'geometry');mesh=ET.SubElement(geom,'mesh')
    ET.SubElement(mesh,'uri').text='model://geomap_final_noroof/meshes/flat_floor.obj'
    mat=ET.SubElement(vis,'material')
    rgb='0.388235294 0.501960784 0.662745098 1'
    ET.SubElement(mat,'ambient').text=rgb
    ET.SubElement(mat,'diffuse').text=rgb
    ET.SubElement(mat,'specular').text='0.1 0.1 0.1 1'
    ET.SubElement(vis,'cast_shadows').text='false'
    ET.indent(tree,space='  ');tree.write(MODEL/'model.sdf',encoding='unicode',xml_declaration=True)
    # This mesh is an offline artifact, not used by DART. Keep it consistent
    # with the new box colliders so it no longer describes removed obstacles.
    vs=[];fs=[]
    for x0,y0,x1,y1 in boxes:
        n=len(vs);vs.extend([(x,y,z) for z in [-.05,1.45] for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]])
        for quad in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            fs.extend([(n+quad[0]+1,n+quad[1]+1,n+quad[2]+1),(n+quad[0]+1,n+quad[2]+1,n+quad[3]+1)])
    (MODEL/'meshes/geomap_final_noroof_collision.obj').write_text('# derived from edited occupancy grid\n'+'\n'.join('v %.4f %.4f %.4f'%v for v in vs)+'\n'+'\n'.join('f %d %d %d'%f for f in fs)+'\n')
    report=dict(regions=REGIONS,cleared_area_m2=areas,smoothing_removed_m2=float(smooth_removed.sum()*res**2),total_added_floor_m2=float(removed.sum()*res**2),added_obstacle_cells=int((occ&~old).sum()),removed_visual_faces=faces,collision_boxes=len(boxes),flat_floor_local_z=.017,flat_floor_world_z=.117,floor_rgb=[99,128,169],floor_rectangles=len(floor_rects))
    (ROOT/'generated/scene_edit_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()

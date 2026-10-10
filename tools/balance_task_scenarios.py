#!/usr/bin/env python3
"""Balance the three vehicle task streams by map path time, not task count.

Includes initial deadhead, every subsequent repositioning, and handling time.
This predicts an unobstructed schedule; traffic/recovery affects actual duty.
"""
from pathlib import Path
import json,math
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage,sparse
from scipy.sparse.csgraph import dijkstra

ROOT=Path(__file__).resolve().parents[1]
PICKS=[(10.60,-.43),(1.70,4.22),(9.10,4.22)]
DROPS=[(8.10,-.43),(5.40,4.22),(11.,4.22)]
CAPS=['light_payload','heavy_payload','omnidirectional']
PAYLOADS=[[8.,12.,16.],[95.,110.,120.],[30.,45.,65.]]


def distance_table(free,res,origin,footprint,padding,points):
    h,w=free.shape
    radius=math.hypot(footprint[0]/2+padding,footprint[1]/2+padding)+res/math.sqrt(2)
    mask=ndimage.distance_transform_edt(free)*res>=radius
    ids=np.full((h,w),-1,dtype=int);ids[mask]=np.arange(mask.sum())
    rr=[];cc=[];cost=[]
    for dr,dc in [(1,0),(0,1),(1,1),(1,-1)]:
        r0=max(0,-dr);r1=min(h,h-dr);c0=max(0,-dc);c1=min(w,w-dc)
        a=ids[r0:r1,c0:c1];b=ids[r0+dr:r1+dr,c0+dc:c1+dc];ok=(a>=0)&(b>=0)
        if dr and dc:
            ok &= (ids[r0:r1,c0+dc:c1+dc]>=0)&(ids[r0+dr:r1+dr,c0:c1]>=0)
        aa=a[ok];bb=b[ok];rr.extend([aa,bb]);cc.extend([bb,aa]);cost.extend([np.full(len(aa),res*math.hypot(dr,dc))]*2)
    graph=sparse.csr_matrix((np.concatenate(cost),(np.concatenate(rr),np.concatenate(cc))),shape=(mask.sum(),mask.sum()))
    ox,oy,_=origin;indices=[]
    for x,y in points:
        col=int((x-ox)/res);row=h-1-int((y-oy)/res);i=ids[row,col]
        assert i>=0,(x,y,radius)
        indices.append(i)
    dist=dijkstra(graph,indices=indices)[:,indices]
    assert np.isfinite(dist).all()
    return dist


def make_scenario(count,profiles,tables,seed):
    rng=np.random.default_rng(seed);robots=profiles['fleet'];types=profiles['robot_types']
    speeds=np.array([types[r['type']]['max_speed'] for r in robots])
    handling=profiles['scheduler']['load_time_sec']+profiles['scheduler']['unload_time_sec']
    best=None
    # Keep 4/3/3 (demo) or 8/6/6 (statistics) tasks, then choose map routes
    # which balance the actual speed differences and initial bay distances.
    allocation=[0,1,2]*(count//3)+[0]*(count%3)
    if count==20:allocation=[0]*8+[1]*6+[2]*6
    for _ in range(7000):
        order=rng.permutation(allocation);pairs=rng.integers(0,9,count)
        if count==20 and len(set(pairs))<9:continue
        elapsed=np.zeros(3);last=[6,6,6]
        for k,(who,pair) in enumerate(zip(order,pairs)):
            pick,drop=divmod(int(pair),3)
            elapsed[who]+=(tables[who][last[who],pick]+tables[who][pick,3+drop])/speeds[who]+handling
            last[who]=3+drop
        score=(elapsed.max()-elapsed.min())/elapsed.mean()
        if best is None or score<best[0]:best=(score,order,pairs,elapsed)
    score,order,pairs,elapsed=best;tasks=[]
    counts=[0,0,0]
    for i,(who,pair) in enumerate(zip(order,pairs),1):
        who=int(who);pick,drop=divmod(int(pair),3)
        tasks.append(dict(id=f'balanced_{count}_{i:02d}',type='transport',pickup=[*PICKS[pick],0.],dropoff=[*DROPS[drop],0.],payload_kg=PAYLOADS[who][counts[who]%3],required_capability=CAPS[who],priority=2,deadline_sec=1800.))
        counts[who]+=1
    report={robots[i]['name']:{'tasks':counts[i],'estimated_busy_sec':round(float(elapsed[i]),2),'relative_to_mean':round(float(elapsed[i]/elapsed.mean()),4)} for i in range(3)}
    return tasks,dict(robots=report,workload_spread_pct=round(score*100,2))


def main():
    profiles=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    meta=yaml.safe_load((ROOT/'src/hetero_fleet_bringup/maps/geomap.yaml').read_text())
    free=np.array(Image.open(ROOT/'src/hetero_fleet_bringup/maps'/meta['image']))>128
    tables=[]
    for robot in profiles['fleet']:
        typ=profiles['robot_types'][robot['type']]
        tables.append(distance_table(free,meta['resolution'],meta['origin'],typ['footprint'],typ['footprint_padding'],PICKS+DROPS+[tuple(robot['spawn'][:2])]))
    report={}
    for count,file in [(10,'demo_10.yaml'),(20,'mixed_20.yaml')]:
        tasks,info=make_scenario(count,profiles,tables,20261010+count)
        dst=ROOT/'src/hetero_fleet_scheduler/config/tasks'/file
        dst.write_text('# Three-robot balanced workload; all tasks feasible, equal priority.\n# Includes grid path travel, initial/subsequent deadhead and 10 s handling.\n# Keep this order (do not --shuffle) to preserve the workload estimate.\n'+yaml.safe_dump({'tasks':tasks},sort_keys=False))
        report[file]=info
    (ROOT/'generated/balanced_workload.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()

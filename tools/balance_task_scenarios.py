#!/usr/bin/env python3
"""Balance the configured vehicle task streams by map path time, not task count.

Includes initial deadhead, every subsequent repositioning, and handling time.
This predicts an unobstructed schedule; traffic/recovery affects actual duty.
"""
from pathlib import Path
import argparse,json,math
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage,sparse
from scipy.sparse.csgraph import dijkstra

ROOT=Path(__file__).resolve().parents[1]
PICKS=[(10.60,-.43),(1.70,4.22),(9.10,4.22),(-2.0,4.22),(5.8,.05)]
DROPS=[(8.10,-.43),(5.40,4.22),(11.,4.22),(7.0,4.22),(0.0,4.1)]
CAPS=['light_payload','heavy_payload','omnidirectional','humanoid_transport']
PAYLOADS=[[8.,12.,16.],[95.,110.,120.],[30.,45.,65.],[1.,2.,3.]]


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


def make_scenario(count,profiles,tables,seed,travel_factors=None):
    rng=np.random.default_rng(seed);robots=profiles['fleet'];types=profiles['robot_types']
    speeds=np.array([types[r['type']]['max_speed'] for r in robots])
    travel_factors=np.ones(len(robots)) if travel_factors is None else np.array(travel_factors)
    speeds /= travel_factors
    handling=profiles['scheduler']['load_time_sec']+profiles['scheduler']['unload_time_sec']
    best=None
    n=len(robots)
    allocation=list(range(n))*(count//n)+list(range(count%n))
    if n==4 and count==10:allocation=[0]*3+[1]*2+[2]*3+[3]*2
    for _ in range(50000):
        order=rng.permutation(allocation);pairs=rng.integers(0,len(PICKS)*len(DROPS),count)
        if len(set(pairs//len(DROPS)))<len(PICKS) or len(set(pairs%len(DROPS)))<len(DROPS):continue
        # Spread pickup demand and give each robot a distinct initial pad.
        picks=pairs//len(DROPS)
        if np.bincount(picks,minlength=len(PICKS)).max()>math.ceil(count/len(PICKS))+1:continue
        first=[];seen=set()
        for who,pick in zip(order,picks):
            if int(who) not in seen:
                seen.add(int(who));first.append(PICKS[int(pick)])
        if any(math.dist(a,b)<2.0 for i,a in enumerate(first) for b in first[i+1:]):continue
        elapsed=np.zeros(n);last=[len(PICKS)+len(DROPS)]*n
        for k,(who,pair) in enumerate(zip(order,pairs)):
            pick,drop=divmod(int(pair),len(DROPS))
            elapsed[who]+=(tables[who][last[who],pick]+tables[who][pick,len(PICKS)+drop])/speeds[who]+handling
            last[who]=len(PICKS)+drop
        score=(elapsed.max()-elapsed.min())/elapsed.mean()
        if best is None or score<best[0]:best=(score,order,pairs,elapsed)
    score,order,pairs,elapsed=best;tasks=[]
    counts=[0]*n
    for i,(who,pair) in enumerate(zip(order,pairs),1):
        who=int(who);pick,drop=divmod(int(pair),len(DROPS))
        tasks.append(dict(id=f'balanced_{count}_{i:02d}',type='transport',pickup=[*PICKS[pick],0.],dropoff=[*DROPS[drop],0.],payload_kg=PAYLOADS[who][counts[who]%3],required_capability=CAPS[who],priority=2,deadline_sec=1800.))
        counts[who]+=1
    report={robots[i]['name']:{'tasks':counts[i],'estimated_busy_sec':round(float(elapsed[i]),2),'relative_to_mean':round(float(elapsed[i]/elapsed.mean()),4)} for i in range(n)}
    return tasks,dict(robots=report,workload_spread_pct=round(score*100,2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uncalibrated',action='store_true',help='Ignore stored measured travel factors')
    parser.add_argument('--metrics',type=Path,help='Completed batch metrics for measured travel/traffic calibration')
    args=parser.parse_args()
    profiles=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    meta=yaml.safe_load((ROOT/'src/hetero_fleet_bringup/maps/geomap.yaml').read_text())
    free=np.array(Image.open(ROOT/'src/hetero_fleet_bringup/maps'/meta['image']))>128
    tables=[]
    for robot in profiles['fleet']:
        typ=profiles['robot_types'][robot['type']]
        tables.append(distance_table(free,meta['resolution'],meta['origin'],typ['footprint'],typ['footprint_padding'],PICKS+DROPS+[tuple(robot['spawn'][:2])]))
    calibration=ROOT/'src/hetero_fleet_scheduler/config/tasks/workload_calibration.yaml'
    factors=None
    if calibration.exists() and not args.uncalibrated:
        saved=yaml.safe_load(calibration.read_text())['travel_factors']
        factors=[saved[r['name']] for r in profiles['fleet']]
    if args.metrics:
        measured=json.loads(args.metrics.read_text())
        totals={r['name']:0. for r in profiles['fleet']}
        last={r['name']:len(PICKS)+len(DROPS) for r in profiles['fleet']}
        tasks=yaml.safe_load((ROOT/'src/hetero_fleet_scheduler/config/tasks/demo_10.yaml').read_text())['tasks']
        definitions={t['id']:t for t in tasks}
        actual={r['name']:0. for r in profiles['fleet']}
        handling=profiles['scheduler']['load_time_sec']+profiles['scheduler']['unload_time_sec']
        for task in sorted(measured['tasks'],key=lambda t:t['assigned_at_sec']):
            name=task['robot_id'];i=next(i for i,r in enumerate(profiles['fleet']) if r['name']==name)
            definition=definitions[task['task_id']]
            pick=PICKS.index(tuple(definition['pickup'][:2]));drop=DROPS.index(tuple(definition['dropoff'][:2]))
            speed=profiles['robot_types'][profiles['fleet'][i]['type']]['max_speed']
            totals[name]+=(tables[i][last[name],pick]+tables[i][pick,len(PICKS)+drop])/speed
            last[name]=len(PICKS)+drop
            actual[name]+=max(0.,task['completed_at_sec']-task['assigned_at_sec']-handling)
        assert all(totals[r['name']]>0 for r in profiles['fleet']), 'Calibration needs completed tasks from every robot'
        factors=[max(1.,actual[r['name']]/totals[r['name']]) for r in profiles['fleet']]
        calibration.write_text(yaml.safe_dump({'travel_factors':dict(zip([r['name'] for r in profiles['fleet']],factors))},sort_keys=False))
    report={}
    for count,file in [(10,'demo_10.yaml'),(20,'mixed_20.yaml')]:
        tasks,info=make_scenario(count,profiles,tables,20261010+count,factors)
        dst=ROOT/'src/hetero_fleet_scheduler/config/tasks'/file
        dst.write_text('# Four-robot balanced workload; all tasks feasible, equal priority.\n# Includes grid travel, deadhead, measured travel/traffic factors and 10 s handling.\n# Keep this order (do not --shuffle) to preserve the workload estimate.\n'+yaml.safe_dump({'tasks':tasks},sort_keys=False))
        if factors is not None:info['measured_travel_factors']=dict(zip([r['name'] for r in profiles['fleet']],factors))
        report[file]=info
    (ROOT/'generated').mkdir(exist_ok=True)
    (ROOT/'generated/balanced_workload.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()

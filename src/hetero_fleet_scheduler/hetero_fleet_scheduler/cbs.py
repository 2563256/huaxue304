"""CBS/EES over finite, swept-lattice route catalogues and integer time ticks.

Low-level search minimizes finish time over route choice and departure events.
Waiting is allowed only at the validated start; a bottleneck is never split by
an invented holding point. Guarantees concern this finite catalogue, not all
continuous/kinodynamic paths or closed-loop execution.
"""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class Candidate:
    duration: int
    occupancy: tuple  # (resource, begin_tick, end_tick), half open
    start_resources: frozenset = frozenset()
    payload: object = None

    def __post_init__(self):
        if not isinstance(self.duration, int) or self.duration < 1:
            raise ValueError('duration must be positive integer ticks')
        if any(not isinstance(a,int) or not isinstance(b,int) or a < 0 or b <= a
               for _,a,b in self.occupancy):
            raise ValueError('invalid occupancy')


@dataclass(frozen=True)
class Plan:
    candidate: Candidate
    departure: int

    @property
    def finish(self):
        return self.departure+self.candidate.duration

    @property
    def reservations(self):
        return tuple((r,a+self.departure,b+self.departure)
                     for r,a,b in self.candidate.occupancy) + tuple(
                         (r,0,self.departure) for r in self.candidate.start_resources
                         if self.departure)


def low_level(candidates, constraints=(), fixed=(), horizon=7200, deadline=math.inf):
    """Exact event search for the best departure for every native route."""
    bans={}
    for r,a,b in (*constraints,*fixed):
        bans.setdefault(r,[]).append((a,b))
    best=None
    for candidate in candidates:
        departure=0
        while departure+candidate.duration <= horizon:
            if time.perf_counter()>=deadline:raise TimeoutError('CBS low-level budget')
            # A delayed departure occupies the actual start body throughout.
            if any(a < departure and b > 0 for r in candidate.start_resources
                   for a,b in bans.get(r,())):break
            jump=departure
            for r,a,b in candidate.occupancy:
                for x,y in bans.get(r,()):
                    if a+departure < y and x < b+departure:
                        jump=max(jump,y-a)
            if jump==departure:
                plan=Plan(candidate,departure)
                if best is None or plan.finish<best.finish:best=plan
                break
            departure=jump
    return best


def conflicts(plans):
    result=[]; names=sorted(plans)
    calendars={n:{} for n in names}
    for n in names:
        for r,a,b in plans[n].reservations:calendars[n].setdefault(r,[]).append((a,b))
    for i,a in enumerate(names):
        for b in names[i+1:]:
            first=None
            for r in calendars[a].keys() & calendars[b].keys():
                for x,y in calendars[a][r]:
                    for u,v in calendars[b][r]:
                        t=max(x,u)
                        if t<min(y,v) and (first is None or t<first[3]):first=(a,b,r,t)
            if first:result.append(first)
    return sorted(result,key=lambda x:(x[3],x[0],x[1],str(x[2])))


def solve(catalogues, mode='eecbs', weight=1.2, fixed=(), max_nodes=256,
          budget=.15, horizon=7200):
    """Constraint tree: CBS cost ordering or bounded EES CLEANUP/OPEN/FOCAL.

    Exact low-level catalogue costs form an admissible node lower bound. EES
    learns per-expansion cost and conflict-count errors. A solution is returned
    only when its cost is <= weight * minimum outstanding admissible bound.
    Budget exhaustion returns no plan (never an unverified incumbent).
    """
    if mode not in ('cbs','eecbs') or not math.isfinite(weight) or weight<1:
        raise ValueError('invalid CBS/EECBS mode or bound')
    if budget<=0 or max_nodes<1:raise ValueError('invalid CBS budget')
    deadline=time.perf_counter()+budget
    stats=dict(mode=mode,expanded=0,generated=0,weight=1. if mode=='cbs' else weight,
               catalogue_sizes={n:len(v) for n,v in catalogues.items()},reason='infeasible')
    bound=stats['weight']; counter=0; seen=set(); cost_error=conflict_error=0.;learned=0
    earliest_cache={}
    def earliest(name,constraints):
        key=(name,constraints)
        if key not in earliest_cache:
            earliest_cache[key]=tuple(p for c in catalogues[name]
                if (p:=low_level((c,),constraints,fixed,horizon,deadline)) is not None)
        return earliest_cache[key]

    def pair_lower_bound(plans,cons):
        # Admissible mandatory-resource reasoning avoids enumerating every
        # one-tick delay of both ends of a long non-passing corridor. For each
        # catalogue pair, relax all but one mandatory visit and compute the
        # least additional wait of its two possible traversal orders. Taking
        # the maximum pair bound never double-counts a robot's extra cost.
        names=sorted(plans);h=0
        for i,a in enumerate(names):
            for b in names[i+1:]:
                value=math.inf
                for pa in earliest(a,cons[a]):
                    aa={}
                    for r,x,y in pa.candidate.occupancy:
                        if not str(r).startswith('cell:'):aa.setdefault(r,(x+pa.departure,y+pa.departure))
                    for pb in earliest(b,cons[b]):
                        bb={}
                        for r,x,y in pb.candidate.occupancy:
                            if not str(r).startswith('cell:'):bb.setdefault(r,(x+pb.departure,y+pb.departure))
                        delay=0
                        for r in aa.keys() & bb.keys():
                            x,y=aa[r];u,v=bb[r]
                            delay=max(delay,min(max(0,y-u),max(0,v-x)))
                        value=min(value,pa.finish+pb.finish+delay)
                if value<math.inf:h=max(h,value-plans[a].finish-plans[b].finish)
        return max(0,h)

    def node(plans,cons):
        nonlocal counter
        counter+=1
        cs=conflicts(plans);cost=sum(p.finish for p in plans.values())
        return dict(plans=plans,cons=cons,cost=cost,h=pair_lower_bound(plans,cons),conflicts=cs,id=counter)
    try:
        plans={n:low_level(cs,fixed=fixed,horizon=horizon,deadline=deadline)
               for n,cs in catalogues.items()}
        if any(p is None for p in plans.values()):return None,stats
        root=node(plans,{n:() for n in plans});open_nodes=[root];stats['generated']=1
        while open_nodes and stats['expanded']<max_nodes:
            if time.perf_counter()>=deadline:raise TimeoutError()
            cleanup=min(open_nodes,key=lambda n:(n['cost']+n['h'],-n['cost'],n['id']));lb=cleanup['cost']+cleanup['h']
            if mode=='cbs':current=cleanup
            else:
                def estimates(n):
                    d=len(n['conflicts'])/max(.05,1-min(.95,conflict_error))
                    return n['cost']+n['h']+max(0.,cost_error)*d,d
                optimistic=min(open_nodes,key=lambda n:(estimates(n)[0],n['id']))
                fmin=estimates(optimistic)[0]
                focal=[n for n in open_nodes if estimates(n)[0]<=bound*fmin]
                focus=min(focal,key=lambda n:(estimates(n)[1],estimates(n)[0],n['id']))
                if focus['cost']<=bound*lb:current=focus
                elif optimistic['cost']<=bound*lb:current=optimistic
                else:current=cleanup
            open_nodes.remove(current);stats['expanded']+=1
            if not current['conflicts']:
                assert current['cost']<=bound*lb+1e-9
                stats.update(reason='solved',cost_ticks=current['cost'],lower_bound_ticks=lb,
                             cost_ratio=current['cost']/lb if lb else 1.,learned_expansions=learned)
                return current['plans'],stats
            a,b,r,t=current['conflicts'][0];children=[]
            for name in (a,b):
                cons=dict(current['cons']);cons[name]=cons[name]+((r,t,t+1),)
                key=tuple((n,tuple(sorted(v,key=repr))) for n,v in sorted(cons.items()))
                if key in seen:continue
                seen.add(key)
                revised=low_level(catalogues[name],cons[name],fixed,horizon,deadline)
                if revised is None:continue
                new=dict(current['plans']);new[name]=revised
                child=node(new,cons);children.append(child);open_nodes.append(child);stats['generated']+=1
            if children:
                best=min(children,key=lambda n:(n['cost'],len(n['conflicts'])))
                learned+=1
                cost_error+=(best['cost']-current['cost']-cost_error)/learned
                conflict_error+=(len(best['conflicts'])-len(current['conflicts'])+1-conflict_error)/learned
        stats['reason']='node_budget' if open_nodes else 'infeasible'
    except TimeoutError:stats['reason']='time_budget'
    return None,stats

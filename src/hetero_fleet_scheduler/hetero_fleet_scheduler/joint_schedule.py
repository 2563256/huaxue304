"""Bounded CP-SAT task/robot sequencing. All times are ROS seconds.

Reservation intervals are conservative task-wide envelopes over estimated route
resources. They are suggestions, never permission to bypass the physical gate.
"""
from dataclasses import dataclass, field
import math
import time
from ortools.sat.python import cp_model


@dataclass(frozen=True)
class Leg:
    seconds: float
    distance: float
    resources: frozenset = frozenset()


@dataclass
class Schedule:
    epoch: float
    status: str
    rows: list = field(default_factory=list)
    unscheduled: list = field(default_factory=list)
    solve_sec: float = 0.
    bound_sec: float = 0.
    reason: str = ''

    def as_dict(self):
        return dict(epoch=self.epoch, status=self.status, rows=self.rows,
                    unscheduled=self.unscheduled, solve_sec=self.solve_sec,
                    bound_sec=self.bound_sec, reason=self.reason)


class JointScheduler:
    def __init__(self, estimate, budget=.25, min_battery=15., service=10.,
                 horizon=1800., max_tasks=20):
        if not all(math.isfinite(v) and v > 0 for v in (budget, horizon)) or max_tasks < 1:
            raise ValueError('Invalid CP-SAT planning limits')
        self.estimate = estimate
        self.budget, self.min_battery, self.service = budget, min_battery, service
        self.horizon, self.max_tasks = horizon, max_tasks

    def solve(self, tasks, robots, now, availability=None, feedback=None, forbidden_pairs=()):
        started = time.monotonic()
        if not math.isfinite(now) or now < 0:
            raise ValueError('Invalid simulation time')
        availability = availability or {}
        feedback = feedback or {}
        forbidden_pairs = frozenset(forbidden_pairs)
        ordered = sorted(tasks, key=lambda t: (-t.priority, t.created_at_sec, t.id))
        tasks = ordered[:self.max_tasks]
        m = cp_model.CpModel()
        scale = 10  # 100 ms planning ticks, rounded up
        tick = lambda v: max(0, math.ceil(v * scale))
        horizon = tick(self.horizon)
        options, loaded, initial, transitions = {}, {}, {}, {}
        # Reachability and capability checks are hard constraints. Unknown
        # spatial routes do not become straight-line fallbacks.
        for r in robots:
            if r.status == 'FAILED' or r.battery_pct <= self.min_battery or r.max_speed <= 0:
                continue
            for t in tasks:
                if (r.id, t.id) in forbidden_pairs:
                    continue
                if not r.can_carry(t.payload_kg) or not r.has_capability(t.required_capability):
                    continue
                b = self.estimate(r, t.pickup, t.dropoff)
                a = self.estimate(r, r.pose, t.pickup)
                if a is None or b is None:
                    continue
                loaded[r.id, t.id], initial[r.id, t.id] = b, a
                options[r.id, t.id] = (r, t)
        eligible = [t for t in tasks if any(t.id == tid for _, tid in options)]
        missing = [t.id for t in ordered if t not in eligible]
        if not eligible:
            return Schedule(now, 'EMPTY', unscheduled=missing, solve_sec=time.monotonic()-started)
        # Estimate every sequence-dependent empty move before building circuits.
        for r in robots:
            jobs = [t for t in eligible if (r.id, t.id) in options]
            for a in jobs:
                for b in jobs:
                    if a.id != b.id:
                        transitions[r.id, a.id, b.id] = self.estimate(r, a.dropoff, b.pickup)
        starts, ends, chosen, intervals, resources, costs = {}, {}, {}, {}, {}, {}
        task_end = {t.id: m.NewIntVar(0, horizon, 'end_'+t.id) for t in eligible}
        scheduled = {t.id: m.NewBoolVar('scheduled_'+t.id) for t in eligible}
        for (rid, tid), (r, t) in options.items():
            key = rid, tid
            present = chosen[key] = m.NewBoolVar('choose_'+rid+'_'+tid)
            s = starts[key] = m.NewIntVar(0, horizon, 'start_'+rid+'_'+tid)
            e = ends[key] = m.NewIntVar(0, horizon, 'finish_'+rid+'_'+tid)
            d = m.NewIntVar(1, horizon, 'duration_'+rid+'_'+tid)
            intervals[key] = m.NewOptionalIntervalVar(s, d, e, present, 'job_'+rid+'_'+tid)
            m.Add(task_end[tid] == e).OnlyEnforceIf(present)
            ready = max(0., availability.get(rid, 0.))
            m.Add(s >= tick(ready)).OnlyEnforceIf(present)
            m.Add(s == 0).OnlyEnforceIf(present.Not())
            m.Add(e == 0).OnlyEnforceIf(present.Not())
            # A robot may arrive from its live/projected pose or another job.
            # Reserve the union of those possible resource approaches for the
            # whole job; conservative but avoids unreserved reposition legs.
            legs = [initial[key], loaded[key]] + [v for (rr, _, to), v in transitions.items()
                                                 if rr == rid and to == tid and v is not None]
            groups = set().union(*(leg.resources for leg in legs))
            for g in groups:
                resources.setdefault(g, []).append(intervals[key])
            # Budget the worst incoming empty move for cumulative energy.
            empty = max([initial[key].distance] + [v.distance for (rr, _, to), v in transitions.items()
                                                  if rr == rid and to == tid and v is not None])
            costs[key] = math.ceil(1000*(r.energy_for(empty, 0.) +
                                      r.energy_for(loaded[key].distance, t.payload_kg)))
            for g in groups:
                m.Add(s >= tick(max(0., feedback.get((rid, g), now)-now))).OnlyEnforceIf(present)
        for t in eligible:
            m.Add(sum(v for (r, tid), v in chosen.items() if tid == t.id) == scheduled[t.id])
            m.Add(task_end[t.id] == 0).OnlyEnforceIf(scheduled[t.id].Not())
        for r in robots:
            jobs = [t for t in eligible if (r.id, t.id) in options]
            if not jobs:
                continue
            m.AddNoOverlap([intervals[r.id, t.id] for t in jobs])
            arcs = []
            unused = m.NewBoolVar('unused_'+r.id)
            m.Add(sum(chosen[r.id, t.id] for t in jobs) == 0).OnlyEnforceIf(unused)
            m.Add(sum(chosen[r.id, t.id] for t in jobs) >= 1).OnlyEnforceIf(unused.Not())
            arcs.append((0, 0, unused))
            for i, t in enumerate(jobs, 1):
                key = r.id, t.id
                arcs.append((i, i, chosen[key].Not()))
                first = m.NewBoolVar(f'first_{r.id}_{i}')
                last = m.NewBoolVar(f'last_{r.id}_{i}')
                arcs += [(0, i, first), (i, 0, last)]
                for a in (first, last):
                    m.AddImplication(a, chosen[key])
                base = loaded[key].seconds + self.service
                m.Add(ends[key] - starts[key] == tick(initial[key].seconds+base)).OnlyEnforceIf(first)
                for j, prev in enumerate(jobs, 1):
                    if i == j:
                        continue
                    leg = transitions[r.id, prev.id, t.id]
                    if leg is None:
                        continue
                    arc = m.NewBoolVar(f'next_{r.id}_{j}_{i}')
                    arcs.append((j, i, arc))
                    m.AddImplication(arc, chosen[key])
                    m.AddImplication(arc, chosen[r.id, prev.id])
                    m.Add(starts[key] >= ends[r.id, prev.id]).OnlyEnforceIf(arc)
                    m.Add(ends[key] - starts[key] == tick(leg.seconds+base)).OnlyEnforceIf(arc)
            m.AddCircuit(arcs)
            if r.battery_capacity_wh > 0:
                usable = max(0., (r.battery_pct-self.min_battery)/100*r.battery_capacity_wh)
                m.Add(sum(costs[r.id,t.id]*chosen[r.id,t.id] for t in jobs) <= math.floor(usable*1000))
        for values in resources.values():
            m.AddNoOverlap(values)
        makespan = m.NewIntVar(0, horizon, 'makespan')
        m.AddMaxEquality(makespan, list(task_end.values()))
        # Maximize jobs fitting the rolling horizon, then minimize makespan,
        # then weighted completion. Omissions stay pending for a later pass.
        weights = {t.id: max(1, min(100, int(t.priority))) for t in eligible}
        coefficient = horizon*sum(weights.values()) + 1
        omission_penalty = coefficient*(horizon+1)
        m.Minimize(omission_penalty*sum(1-v for v in scheduled.values()) +
                   coefficient*makespan + sum(weights[tid]*e for tid,e in task_end.items()))
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = self.budget
        solver.parameters.num_search_workers = 1
        solver.parameters.random_seed = 0
        status = solver.Solve(m)
        result = Schedule(now, solver.StatusName(status), unscheduled=missing,
                          solve_sec=time.monotonic()-started)
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            result.reason = 'No feasible schedule within horizon/battery/search budget; hold and retry.'
            return result
        for key, present in chosen.items():
            if solver.Value(present):
                rid,tid = key
                groups = [g for g,vals in resources.items() if any(v is intervals[key] for v in vals)]
                result.rows.append(dict(robot_id=rid, task_id=tid,
                                        start=now+solver.Value(starts[key])/scale,
                                        end=now+solver.Value(ends[key])/scale,
                                        groups=sorted(groups)))
        result.rows.sort(key=lambda row:(row['start'],row['robot_id'],row['task_id']))
        omitted = [tid for tid,present in scheduled.items() if not solver.Value(present)]
        result.unscheduled.extend(omitted)
        if omitted:
            result.reason = 'Unscheduled jobs remain pending for a later horizon or recharge.'
        if not result.rows:
            result.status = 'EMPTY'
        # Report a makespan bound only when the whole eligible batch fits.
        result.bound_sec = 0. if omitted else max(0., math.floor(solver.BestObjectiveBound()/coefficient)/scale)
        return result

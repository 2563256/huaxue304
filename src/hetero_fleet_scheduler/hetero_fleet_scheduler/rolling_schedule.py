"""RHCR-inspired task-horizon coordinator; active Nav2 goals are frozen.

Unlike canonical RHCR this is a rolling task/resource horizon, not a new
windowed spatial MAPF solver. Commit only the near-term head job per robot.
"""
import math
from .joint_schedule import JointScheduler


class RollingSchedule:
    def __init__(self, solver, period=5., commit=2., drift=15.):
        if not all(math.isfinite(v) and v>0 for v in (period,commit,drift)):
            raise ValueError('Invalid rolling schedule timing')
        self.solver=solver
        self.period,self.commit,self.drift=period,commit,drift
        self.plan=None; self.last=-math.inf; self.key=None
        self.reason='startup'; self.revision=0

    def due(self, now, key):
        return now < self.last or key != self.key or now-self.last>=self.period

    def invalidate(self, reason):
        self.key=None; self.reason=reason

    def update(self, now, key, tasks, robots, availability=None, feedback=None, forbidden_pairs=()):
        self.plan=self.solver.solve(tasks,robots,now,availability,feedback,forbidden_pairs)
        self.last=now; self.key=key; self.revision+=1
        return self.plan

    def heads(self, now):
        if self.plan is None or now<self.plan.epoch or now-self.plan.epoch>self.period+self.commit:
            return []
        first={}
        for row in self.plan.rows:
            first.setdefault(row['robot_id'],row)
        return [r for r in first.values() if r['start']<=now+self.commit]

    def hints(self):
        return dict(revision=self.revision, replan_reason=self.reason,
                    **(self.plan.as_dict() if self.plan else dict(epoch=0.,status='PENDING',rows=[])))

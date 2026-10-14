"""Coordinate approaching robots and stationary off-charger blockers."""
from dataclasses import dataclass, field
import math
import threading


@dataclass
class Encounter:
    winner: str
    loser: str
    origin: tuple
    started: float
    loser_idle: bool = False
    loser_origin: tuple = ()
    held: threading.Event = field(default_factory=threading.Event)
    yield_started: threading.Event = field(default_factory=threading.Event)
    parked: threading.Event = field(default_factory=threading.Event)
    resumed: threading.Event = field(default_factory=threading.Event)
    winner_finished: threading.Event = field(default_factory=threading.Event)
    done: threading.Event = field(default_factory=threading.Event)


class TrafficCoordinator:
    PRIORITY = {'g1_humanoid': 3, 'heavy_diff_amr': 2,
                'light_diff_amr': 1, 'omni_mecanum_amr': 0}

    def __init__(self, stationary_sec=8.0, radius=1.8):
        self.stationary_sec = stationary_sec
        self.radius = radius
        self.motion = {}
        self.travel = {}
        self.encounter = None
        self.cooldown = {}
        self.lock = threading.Lock()

    def observe(self, name, pose, now):
        with self.lock:
            old = self.motion.get(name)
            if old is None or now < old[1] or math.dist(pose[:2], old[0][:2]) > .10:
                if old is not None and now >= old[1]:
                    self.travel[name] = (math.atan2(pose[1]-old[0][1], pose[0]-old[0][0]), now)
                else:
                    self.travel[name] = (pose[2], now)
                self.motion[name] = (pose, now, now)
            else:
                self.motion[name] = (old[0], old[1], now)

    def request(self, name, robots, positions, active, now, idle=(), targets=None,
                corridor_clear=None):
        with self.lock:
            current = self.encounter
            if current is not None:
                return current if name in (current.winner, current.loser) else None
            own = self.motion.get(name)
            if own is None or name not in positions:
                return None
            for other in sorted(set(active) | set(idle)):
                if other == name or other not in robots or other not in positions:
                    continue
                peer = self.motion.get(other)
                key = tuple(sorted((name, other)))
                if peer is None or now-peer[2] > 1.0:
                    continue
                if now < self.cooldown.get(key, -math.inf):
                    continue
                a, b = positions[name], positions[other]
                distance = math.dist(a[:2], b[:2])
                bearing = math.atan2(b[1]-a[1], b[0]-a[0])
                adir, at = self.travel.get(name, (a[2], now))
                bdir, bt = self.travel.get(other, (b[2], now))
                adir = adir if now-at < 2.0 else a[2]
                bdir = bdir if now-bt < 2.0 else b[2]
                facing = math.cos(bearing-adir) > .65 and math.cos(bearing+math.pi-bdir) > .65
                intended = False
                if targets and name in targets and other in targets:
                    ta, tb = targets[name], targets[other]
                    aa = math.atan2(ta[1]-a[1],ta[0]-a[0])
                    ba = math.atan2(tb[1]-b[1],tb[0]-b[0])
                    intended = math.cos(bearing-aa) > .65 and math.cos(bearing+math.pi-ba) > .65
                idle_blocker = other not in active and other in idle
                stopped = now-own[1] >= self.stationary_sec and now-peer[1] >= self.stationary_sec
                approaching = not idle_blocker and (facing or intended) and distance <= 4.0
                idle_blocked = idle_blocker and distance <= self.radius and now-own[1] >= 2.0 and now-peer[1] >= 2.0
                if not (approaching or idle_blocked or (stopped and distance <= self.radius)):
                    continue
                if corridor_clear is not None and not corridor_clear(a,b):
                    continue
                winner = name if idle_blocker else max((name, other), key=lambda r: (self.PRIORITY.get(robots[r].type, 0), r))
                loser = other if winner == name else name
                current = Encounter(winner, loser, positions[winner], now,
                                    loser_idle=idle_blocker, loser_origin=positions[loser])
                self.encounter = current
                return current
            return None

    def finish(self, request, now):
        with self.lock:
            request.parked.set()  # Release a waiting winner even after failure.
            request.held.set()
            request.done.set()
            self.cooldown[tuple(sorted((request.winner, request.loser)))] = now+15.0
            if self.encounter is request:
                self.encounter = None

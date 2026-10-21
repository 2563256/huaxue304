"""Exclusive, physical-occupancy corridor reservations, independent of ROS."""
from dataclasses import dataclass
import math
import threading
import bisect
import time

from .sipp_pbs import Edge, Route, pbs


def intersects(pose, footprint, padding, bounds):
    """Separating axes for an oriented filled body and an axis-aligned region."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = footprint[0] / 2 + padding, footprint[1] / 2 + padding
    cx, cy = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2
    bx, by = (bounds[2] - bounds[0]) / 2, (bounds[3] - bounds[1]) / 2
    dx, dy = cx - x, cy - y
    return (abs(dx) <= bx + abs(c) * hx + abs(s) * hy and
            abs(dy) <= by + abs(s) * hx + abs(c) * hy and
            abs(c * dx + s * dy) <= hx + abs(c) * bx + abs(s) * by and
            abs(-s * dx + c * dy) <= hy + abs(s) * bx + abs(c) * by)


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def command_path(pose, command, duration, radius, step=.025):
    """Exact constant-twist SE(2) prediction; supports reverse and omni strafe."""
    vx, vy, omega = command
    n = max(1, math.ceil((math.hypot(vx, vy) + radius * abs(omega)) * duration / step))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for k in range(n + 1):
        t = duration * k / n
        if abs(omega) < 1e-8:
            dx, dy = vx * t, vy * t
        else:
            dx = (vx * math.sin(omega * t) + vy * (math.cos(omega * t) - 1)) / omega
            dy = (vx * (1 - math.cos(omega * t)) + vy * math.sin(omega * t)) / omega
        yield (pose[0] + c * dx - s * dy, pose[1] + s * dx + c * dy, pose[2] + omega * t)


@dataclass
class Token:
    robot: str
    granted: float
    entered: bool = False


@dataclass
class Request:
    groups: frozenset
    order: int
    requested: float


class ChannelReservations:
    def __init__(self, config, bodies, enabled=True):
        self.enabled = enabled
        self.maneuvers = {}
        for key in ('exit_waypoints', 'retreat_waypoints'):
            points = config.get(key, [])
            if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
                raise ValueError('Channel maneuver poses must be finite [x,y,yaw]')
            self.maneuvers[key] = [list(map(float, p)) for p in points]
        self.mode = config.get('scheduling', 'fifo')
        if self.mode not in ('fifo', 'sipp_pbs'):
            raise ValueError('Scheduling must be fifo or sipp_pbs')
        self.speeds = config.get('robot_motion_limits', {})
        self.pbs_nodes = int(config.get('pbs_max_nodes', 64))
        self.pbs_budget = float(config.get('pbs_budget_sec', .025))
        self.headway = float(config.get('reservation_headway_sec', .5))
        self.aging = float(config.get('max_queue_wait_sec', 60.))
        if (self.pbs_nodes < 1 or not all(math.isfinite(v) and v > 0
                for v in (self.pbs_budget, self.headway, self.aging))):
            raise ValueError('Invalid SIPP/PBS budget or headway')
        for limits in self.speeds.values():
            if len(limits) != 2 or not all(math.isfinite(v) and v > 0 for v in limits):
                raise ValueError('Motion limits must be positive [linear, angular] speeds')
        self.compiled = {}
        self.schedule_key = None
        self.schedule = {'mode': self.mode, 'windows': {}, 'priorities': [], 'stats': {}}
        self.schedule_order = []
        self.bodies = bodies  # name -> (length,width,padding)
        self.exempt = frozenset(config.get('reservation_exempt_robots', []))
        if not self.exempt <= bodies.keys():
            raise ValueError('Unknown reservation-exempt robot')
        self.participants = set(bodies) - self.exempt
        self.request_distance = float(config.get('request_distance_m', 3.5))
        self.stop_distance = float(config.get('stop_distance_m', 2.2))
        self.margin = float(config.get('body_margin_m', .05))
        self.release_margin = float(config.get('release_margin_m', .10))
        self.sample_step = min(.025, self.margin)
        self.pose_timeout = float(config.get('pose_timeout_sec', .5))
        self.command_timeout = float(config.get('command_timeout_sec', .35))
        self.lease = float(config.get('preentry_lease_sec', 20.))
        self.reaction = float(config.get('reaction_time_sec', .6))
        self.decel = float(config.get('prediction_decel_mps2', .4))
        self.angular_decel = float(config.get('prediction_angular_decel_rps2', .6))
        values = [self.request_distance, self.stop_distance, self.margin, self.release_margin,
                  self.pose_timeout, self.command_timeout, self.lease, self.reaction,
                  self.decel, self.angular_decel]
        if not all(math.isfinite(v) and v > 0 for v in values) or self.stop_distance >= self.request_distance:
            raise ValueError('Invalid corridor reservation timing or distances')
        if not bodies or any(len(b) != 3 or not all(math.isfinite(v) for v in b) or
                             b[0] <= 0 or b[1] <= 0 or b[2] < 0 for b in bodies.values()):
            raise ValueError('Actual body dimensions and padding are required')
        self.channels = [dict(c) for c in config['channels']]
        ids = [c['id'] for c in self.channels]
        if not ids or len(set(ids)) != len(ids):
            raise ValueError('Channel ids must be unique and nonempty')
        for channel in self.channels:
            b = channel['bounds']
            if len(b) != 4 or not all(math.isfinite(v) for v in b) or b[0] >= b[2] or b[1] >= b[3]:
                raise ValueError('Channel bounds must be a finite ordered rectangle')
            channel['bounds'] = [float(v) for v in b]
            if channel.get('capacity', 1) != 1:
                raise ValueError('Only exclusive channels (capacity 1) are supported')
            channel.setdefault('group', channel['id'])
        # No robot may hold one independent group while requesting another:
        # merge sections separated by less than body + request/holding space.
        clearance = self.request_distance + 2 * max(
            math.hypot(l / 2 + p + self.margin + self.release_margin,
                       w / 2 + p + self.margin + self.release_margin)
            for l, w, p in bodies.values())
        parent = list(range(len(self.channels)))
        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        for i, a in enumerate(self.channels):
            for j, b in enumerate(self.channels[:i]):
                aa, bb = a['bounds'], b['bounds']
                gap = math.hypot(max(aa[0] - bb[2], bb[0] - aa[2], 0),
                                 max(aa[1] - bb[3], bb[1] - aa[3], 0))
                if a['group'] == b['group'] or gap <= clearance:
                    parent[find(i)] = find(j)
        components = {}
        for i, c in enumerate(self.channels):
            components.setdefault(find(i), set()).add(c['group'])
        self.regions = {}
        for i, c in enumerate(self.channels):
            c['group'] = '+'.join(sorted(components[find(i)]))
            self.regions.setdefault(c['group'], []).append(c['bounds'])
        self.poses, self.paths, self.cursors, self.commands = {}, {}, {}, {}
        self.active = {name: False for name in bodies}
        self.transport_holds = {name: False for name in bodies}
        self.tokens, self.pending = {}, {}
        self.occupants = {group: set() for group in self.regions}
        self.order = 0
        self.last_sim = None
        self.lock = threading.RLock()

    def observe(self, name, pose, wall_now):
        with self.lock:
            if name not in self.bodies or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise ValueError('Unknown robot or invalid pose')
            self.poses[name] = tuple(pose), wall_now

    def set_active(self, name, active):
        with self.lock:
            self.active[name] = bool(active)
            if not active:
                self.paths.pop(name, None); self.cursors.pop(name, None)
                self.pending.pop(name, None)
                self.compiled.pop(name, None)
                self.schedule_key = None

    def set_transport_hold(self, name, active):
        with self.lock:
            if name not in self.bodies:
                raise ValueError('Unknown robot')
            self.transport_holds[name] = bool(active) and name not in self.exempt

    def set_path(self, name, poses):
        with self.lock:
            if name not in self.bodies or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in poses):
                raise ValueError('Unknown robot or invalid planned path')
            self.paths[name] = [tuple(p) for p in poses]
            self.cursors[name] = 0
            self.compiled.pop(name, None)
            self.schedule_key = None

    def set_command(self, name, command, wall_now):
        with self.lock:
            if len(command) != 3 or not all(math.isfinite(v) for v in command):
                raise ValueError('Invalid velocity command')
            self.commands[name] = tuple(command), wall_now

    def _overlap(self, name, pose, group, release=False):
        l, w, p = self.bodies[name]
        padding = p + self.margin + (self.release_margin if release else 0)
        return any(intersects(pose, (l, w), padding, b) for b in self.regions[group])

    def _crossings(self, name, pose):
        path = self.paths.get(name, [])
        if not path:
            return {}
        begin = self.cursors.get(name, 0)
        # Retain progress through reverse segments and repeated path positions.
        cursor = min(range(begin, len(path)), key=lambda i: (
            math.dist(pose[:2], path[i][:2]) + .2 * abs(angle(pose[2] - path[i][2])), i))
        self.cursors[name] = cursor
        l, w, p = self.bodies[name]
        radius = math.hypot(l / 2 + p + self.margin, w / 2 + p + self.margin)
        previous = pose
        travelled, found = 0., {}
        for endpoint in path[cursor:]:
            turn = angle(endpoint[2] - previous[2])
            distance = math.dist(endpoint[:2], previous[:2]) + radius * abs(turn)
            steps = max(1, math.ceil(distance / self.sample_step))
            for k in range(steps + 1):
                t = k / steps
                progress = travelled + distance * t
                if progress > self.request_distance:
                    return found
                sample = (previous[0] + t * (endpoint[0] - previous[0]),
                          previous[1] + t * (endpoint[1] - previous[1]), previous[2] + t * turn)
                for group in self.regions:
                    if group not in found and self._overlap(name, sample, group):
                        found[group] = progress
            travelled += distance
            previous = endpoint
        return found

    def _compile(self, name):
        """Cache complete body sweeps once per plan, not at every control tick."""
        if name in self.compiled:
            return self.compiled[name]
        path = self.paths[name]
        l, w, padding = self.bodies[name]
        radius = math.hypot(l / 2 + padding + self.margin, w / 2 + padding + self.margin)
        linear, angular = self.speeds.get(name, (.4, .8))
        poses, distances, times = [path[0]], [0.], [0.]
        for a, b in zip(path, path[1:]):
            turn = angle(b[2] - a[2])
            translation = math.dist(a[:2], b[:2])
            distance = translation + radius * abs(turn)
            duration = max(translation / linear, abs(turn) / angular, 1e-6)
            count = max(1, math.ceil(distance / self.sample_step))
            start_distance, start_time = distances[-1], times[-1]
            for k in range(1, count + 1):
                fraction = k / count
                poses.append((a[0] + fraction * (b[0] - a[0]),
                              a[1] + fraction * (b[1] - a[1]), a[2] + fraction * turn))
                distances.append(start_distance + distance * fraction)
                times.append(start_time + duration * fraction)
        # Extra sampling margin covers contact between adjacent samples. Exit
        # hysteresis is included when predicting the full occupancy duration.
        contacts = [frozenset(g for g, bounds in self.regions.items() if any(
            intersects(pose, (l, w), padding + self.margin + self.release_margin + self.sample_step, b)
            for b in bounds)) for pose in poses]
        by_group = {g: [i for i, groups in enumerate(contacts) if g in groups] for g in self.regions}
        self.compiled[name] = poses, distances, times, contacts, by_group
        return self.compiled[name]

    def _cached_crossings(self, name, pose):
        poses, distances, _, _, by_group = self._compile(name)
        begin = self.cursors.get(name, 0)
        # Native lattice samples are dense. Track a local forward window; a
        # large deviation uses the complete suffix to reacquire the route.
        stop = min(len(poses), begin + 100)
        metric = lambda i: math.dist(pose[:2], poses[i][:2]) + .2 * abs(angle(pose[2] - poses[i][2]))
        cursor = min(range(begin, stop), key=lambda i: (metric(i), i))
        if metric(cursor) > .5:
            cursor = min(range(begin, len(poses)), key=lambda i: (metric(i), i))
        self.cursors[name] = cursor
        found = {}
        for group, indices in by_group.items():
            index = bisect.bisect_left(indices, cursor)
            if index < len(indices):
                distance = max(0., distances[indices[index]] - distances[cursor])
                if distance <= self.request_distance:
                    found[group] = distance
            if self._overlap(name, pose, group):
                found[group] = 0.
        return found

    def _temporal_route(self, name, now):
        _, _, times, contacts, _ = self._compile(name)
        cursor = self.cursors.get(name, 0)
        edges = []
        # Merge a contiguous protected run into one transition. Waiting is
        # possible only outside the connected bottleneck, never mid-channel.
        start, protected = cursor, bool(contacts[cursor])
        resources = set(contacts[cursor])
        for i in range(cursor + 1, len(contacts) + 1):
            current = bool(contacts[i]) if i < len(contacts) else not protected
            if current != protected:
                duration = times[i - 1] - times[start]
                if i < len(contacts):
                    duration += times[i] - times[i - 1]
                edges.append(Edge(max(duration, .001), frozenset(resources)))
                start, protected, resources = i, current, set()
            if i < len(contacts):
                resources.update(contacts[i])
        if not edges:
            edges = [Edge(.001)]
        request = self.pending.get(name)
        age = max(0., now - request.requested) if request else 0.
        return Route(tuple(edges), weight=1. + age / self.aging)

    def _ordered_requests(self, sim_now):
        fifo = sorted(list(self.pending.items()), key=lambda item: item[1].order)
        if self.mode == 'fifo' or len(fifo) < 2:
            self.schedule = {'mode': self.mode, 'windows': {}, 'priorities': [], 'stats': {}}
            self.schedule_key = None
            return fifo
        key = (tuple((n, r.order, tuple(sorted(r.groups))) for n, r in fifo),
               tuple(sorted((g, t.robot, t.entered) for g, t in self.tokens.items())),
               int(sim_now / 2))
        if key != self.schedule_key:
            routes = {}
            for name, request in fifo:
                routes[name] = self._temporal_route(name, sim_now)
                # Recovery commands outside the planned route still take an
                # exclusive request; use a conservative duration estimate.
                represented = set().union(*(edge.resources for edge in routes[name].edges))
                missing = request.groups - represented
                if missing:
                    routes[name] = Route(routes[name].edges + (Edge(self.lease, missing),),
                                         weight=routes[name].weight)
            # An aged request gets hard priority over newer overlapping ones.
            hard = {(a, b) for a, ra in fifo for b, rb in fifo
                    if ra.order < rb.order and sim_now - ra.requested >= self.aging
                    and ra.groups & rb.groups}
            fixed = {}
            for group, token in self.tokens.items():
                owner = token.robot
                # Unknown departure from a stationary/cancelled incumbent is
                # an infinite obstacle for prediction; physical gating agrees.
                end = math.inf
                if self.active[owner] and self.paths.get(owner):
                    elapsed, end = 0., 0.
                    for edge in self._temporal_route(owner, sim_now).edges:
                        elapsed += edge.duration
                        if group in edge.resources:
                            end = elapsed + self.headway
                    # A goal inside a channel has no departure in this plan.
                    # Include dwell uncertainty; it is NEVER a release lease.
                    if self._overlap(owner, self.paths[owner][-1], group, release=True):
                        end += self.lease
                fixed.setdefault(group, []).append((0., max(end, self.headway)))
            started = time.perf_counter()
            plans, priorities, stats = pbs(routes, hard, self.headway, self.pbs_nodes,
                                            self.pbs_budget, fixed_calendar=fixed)
            stats['elapsed_sec'] = time.perf_counter() - started
            self.schedule_order = [n for n, _ in fifo]
            windows = {}
            if plans is not None:
                # Dispatch by the first planned protected transition. These
                # are advisory slots; a physical incumbent ALWAYS wins.
                entry = lambda n: min((a for _, a, _ in plans[n].reservations), default=math.inf)
                self.schedule_order.sort(key=lambda n: (entry(n), plans[n].finish,
                                                        self.pending[n].order))
                windows = {n: [{'group': g, 'start': sim_now + a, 'end': sim_now + b}
                               for g, a, b in plans[n].reservations] for n in plans}
            self.schedule = {'mode': self.mode, 'windows': windows, 'priorities': priorities,
                             'fallback_fifo': plans is None, 'stats': stats}
            self.schedule_key = key
        rank = {n: i for i, n in enumerate(self.schedule_order)}
        return sorted(fifo, key=lambda item: (rank.get(item[0], len(rank)), item[1].order))

    def _command_groups(self, name, pose, wall_now):
        command, stamp = self.commands.get(name, ((0., 0., 0.), -math.inf))
        if not 0 <= wall_now - stamp <= self.command_timeout:
            return set()
        l, w, p = self.bodies[name]
        radius = math.hypot(l / 2 + p + self.margin, w / 2 + p + self.margin)
        duration = self.reaction + self.pose_timeout + max(
            math.hypot(*command[:2]) / self.decel, abs(command[2]) / self.angular_decel)
        groups = set()
        for sample in command_path(pose, command, duration, radius, self.sample_step):
            for group in self.regions:
                if self._overlap(name, sample, group):
                    groups.add(group)
        return groups

    def step(self, sim_now, wall_now):
        with self.lock:
            if self.last_sim is not None and sim_now < self.last_sim:
                # Never carry a future permission across a simulator reset.
                # Keep physical occupants until fresh post-reset odometry clears them.
                self.tokens = {g: t for g, t in self.tokens.items() if t.entered}
                self.pending.clear(); self.paths.clear(); self.cursors.clear(); self.commands.clear()
                self.poses.clear()
                self.compiled.clear(); self.schedule_key = None
                self.schedule = {'mode': self.mode, 'windows': {}, 'priorities': [], 'stats': {}}
            self.last_sim = sim_now
            fresh = {name for name, (_, stamp) in self.poses.items()
                     if 0 <= wall_now - stamp <= self.pose_timeout}
            for group in self.regions:
                previous = self.occupants[group]
                token = self.tokens.get(group)
                self.occupants[group] = {name for name in previous if name not in fresh}
                for name in fresh - self.exempt:
                    # Hysteresis belongs to an incumbent's exit, not to a
                    # nearby outside robot that has never entered the channel.
                    exiting = name in previous or (token is not None and token.robot == name)
                    if self._overlap(name, self.poses[name][0], group, release=exiting):
                        self.occupants[group].add(name)
                people = self.occupants[group]
                if len(people) == 1:
                    incumbent = next(iter(people))
                    if token is None or token.robot != incumbent:
                        self.tokens[group] = Token(incumbent, sim_now, True)
                    else:
                        token.entered = True
                elif token is not None and token.robot in fresh and token.robot not in people:
                    continuing_transport = token.entered and self.transport_holds[token.robot]
                    if not continuing_transport and (token.entered or not self.active[token.robot] or sim_now - token.granted >= self.lease):
                        self.tokens.pop(group)
            ready = self.participants <= fresh
            wanted, near, motion = {}, {}, {}
            if not self.enabled:
                self.tokens.clear(); self.pending.clear()
            if ready and self.enabled:
                for name in self.bodies:
                    if name in self.exempt:
                        self.pending.pop(name, None)
                        continue
                    if self.active[name] and self.paths.get(name):
                        pose = self.poses[name][0]
                        near[name] = (self._cached_crossings(name, pose) if self.mode == 'sipp_pbs'
                                      else self._crossings(name, pose))
                        motion[name] = self._command_groups(name, pose, wall_now)
                        wanted[name] = set(near[name]) | motion[name] | {
                            g for g, people in self.occupants.items() if name in people}
                        unowned = frozenset(g for g in wanted[name] if g not in self.tokens or
                                            self.tokens[g].robot != name)
                        old = self.pending.get(name)
                        if not unowned:
                            self.pending.pop(name, None)
                        elif old is None or old.groups != unowned:
                            self.order += 1; self.pending[name] = Request(unowned, self.order, sim_now)
                    else:
                        self.pending.pop(name, None)
                for group, token in list(self.tokens.items()):
                    if (not token.entered and token.robot in fresh and not self.occupants[group] and
                            group not in wanted.get(token.robot, set())):
                        self.tokens.pop(group)  # Cancelled or replanned away, still outside.
                for name, request in self._ordered_requests(sim_now):
                    available = all((g not in self.tokens or self.tokens[g].robot == name) and
                                    (not self.occupants[g] or self.occupants[g] == {name}) for g in request.groups)
                    if available:
                        for group in request.groups:
                            self.tokens[group] = Token(name, sim_now, name in self.occupants[group])
                        self.pending.pop(name)
            decisions = {}
            for name in self.bodies:
                reason = ''
                denied = {g for g in wanted.get(name, []) if g not in self.tokens or self.tokens[g].robot != name}
                conflicts = {g for g, people in self.occupants.items() if len(people) > 1 and name in people}
                blocked = bool(conflicts or motion.get(name, set()) & denied or any(
                    d <= self.stop_distance for g, d in near.get(name, {}).items() if g in denied))
                if not ready:
                    reason = 'pose_unavailable'
                elif not self.active[name]:
                    reason = 'inactive'
                elif not self.paths.get(name):
                    reason = 'waiting_for_plan'
                elif conflicts:
                    reason = 'multiple_occupants'
                elif blocked:
                    reason = 'channel_queue'
                allowed = ready and self.active[name] and bool(self.paths.get(name)) and not blocked
                decisions[name] = {'allowed': allowed if self.enabled else True,
                                   'waiting': (blocked or (not ready and self.active[name] and bool(self.paths.get(name)))) if self.enabled else False,
                                   'reason': reason if self.enabled else 'disabled', 'denied_groups': sorted(denied)}
                if name in self.exempt and self.enabled:
                    decisions[name] = {
                        'allowed': name in fresh and self.active[name] and bool(self.paths.get(name)),
                        'waiting': False, 'reason': 'reservation_exempt', 'denied_groups': []}
            snapshot = {'sim_time': sim_now, 'enabled': self.enabled, 'pose_ready': ready, 'channels': self.channels,
                        'maneuvers': self.maneuvers,
                        'clearance_margin': self.margin + self.release_margin,
                        'exempt_robots': sorted(self.exempt),
                        'scheduler': self.schedule,
                        'groups': {g: {'owner': self.tokens[g].robot if g in self.tokens else None,
                                       'transport_hold': self.transport_holds[self.tokens[g].robot] if g in self.tokens else False,
                                       'entered': self.tokens[g].entered if g in self.tokens else False,
                                       'granted_at': self.tokens[g].granted if g in self.tokens else None,
                                       'lease_deadline': (self.tokens[g].granted + self.lease) if g in self.tokens and not self.tokens[g].entered else None,
                                       'queue_wait_sec': {n: max(0., sim_now - r.requested) for n, r in self.pending.items() if g in r.groups},
                                       'occupants': sorted(self.occupants[g]),
                                       'queue': [name for name, request in sorted(self.pending.items(), key=lambda item: (
                                           self.schedule_order.index(item[0]) if self.mode == 'sipp_pbs' and item[0] in self.schedule_order
                                           else len(self.schedule_order), item[1].order)) if g in request.groups]}
                                   for g in self.regions}, 'robots': decisions}
            return decisions, snapshot

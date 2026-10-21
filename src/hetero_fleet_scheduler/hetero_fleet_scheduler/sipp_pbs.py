"""Continuous-time SIPP and bounded PBS on a compressed channel-route graph.

Spatial paths come from State Lattice. This layer searches time and partial
priorities, not spatial detours. Runtime physical tokens remain authoritative.
All intervals are half-open; safety headway is added to occupied edge endings.
"""
from dataclasses import dataclass
import heapq
import math
import time


@dataclass(frozen=True)
class Edge:
    duration: float
    resources: frozenset = frozenset()


@dataclass(frozen=True)
class Route:
    edges: tuple
    nodes: tuple = ()  # resource sets at vertices; empty means safe holding area
    release: float = 0.
    weight: float = 1.

    def __post_init__(self):
        if not self.nodes:
            object.__setattr__(self, 'nodes', tuple(frozenset() for _ in range(len(self.edges) + 1)))
        if (len(self.nodes) != len(self.edges) + 1 or not math.isfinite(self.release) or self.release < 0
                or not math.isfinite(self.weight) or self.weight <= 0
                or any(not math.isfinite(e.duration) or e.duration <= 0 for e in self.edges)):
            raise ValueError('Invalid temporal route')


@dataclass
class Plan:
    arrivals: tuple
    departures: tuple
    reservations: tuple  # (resource, start, end)

    @property
    def finish(self):
        return self.arrivals[-1]


def safe_intervals(resources, calendar):
    blocked = sorted((a, b) for r in resources for a, b in calendar.get(r, ()) if b > a)
    result, end = [], 0.
    for a, b in blocked:
        if a > end:
            result.append((end, a))
        end = max(end, b)
    if end < math.inf:
        result.append((end, math.inf))
    return result


def sipp(route, calendar, headway=.5, deadline=math.inf):
    """A* over (route vertex, maximal safe interval), with event-time jumps.

    Holding must stay in the same safe interval; transitions must have an
    entirely free resource interval, so edge swaps cannot pass undetected.
    """
    intervals = [safe_intervals(rs, calendar) for rs in route.nodes]
    remaining = [0.] * (len(route.edges) + 1)
    for i in range(len(route.edges) - 1, -1, -1):
        remaining[i] = remaining[i + 1] + route.edges[i].duration
    heap, best, parents = [], {}, {}
    for j, (a, b) in enumerate(intervals[0]):
        # A physically present start cannot jump across an unsafe interval.
        if a <= route.release < b:
            state = (0, j); best[state] = route.release
            heapq.heappush(heap, (route.release + remaining[0], route.release, state))
    while heap:
        if time.perf_counter() >= deadline:
            raise TimeoutError('SIPP budget exhausted')
        _, arrival, state = heapq.heappop(heap)
        if arrival != best[state]:
            continue
        i, j = state
        if i == len(route.edges):
            chain = []
            while state in parents:
                prev, dep = parents[state]; chain.append((state, prev, dep)); state = prev
            chain.reverse()
            arrivals, departures, reservations = [route.release], [], []
            for current, previous, dep in chain:
                k = previous[0]; end = best[current]
                departures.append(dep); arrivals.append(end)
                for r in route.nodes[k]:
                    reservations.append((r, arrivals[-2], dep + headway))
                for r in route.edges[k].resources:
                    reservations.append((r, dep, end + headway))
            # The caller models terminal dwell as an explicit edge when needed.
            return Plan(tuple(arrivals), tuple(departures), tuple(reservations))
        edge = route.edges[i]
        _, origin_end = intervals[i][j]
        for next_j, (target_start, target_end) in enumerate(intervals[i + 1]):
            dep = max(arrival, target_start - edge.duration)
            occupied = sorted((a, b) for r in edge.resources for a, b in calendar.get(r, ()))
            for a, b in occupied:
                if dep < b and a < dep + edge.duration + headway:
                    dep = b  # jump to the next edge-safe interval, no time grid
            end = dep + edge.duration
            if dep + (headway if route.nodes[i] else 0.) >= origin_end or not target_start <= end < target_end:
                continue
            target = (i + 1, next_j)
            if end < best.get(target, math.inf):
                best[target] = end; parents[target] = (state, dep)
                heapq.heappush(heap, (end + remaining[i + 1], end, target))
    return None


def conflict(plans):
    first = None
    names = list(plans)
    for i, name in enumerate(names):
        for other in names[:i]:
            for r, a, b in plans[name].reservations:
                for s, c, d in plans[other].reservations:
                    if r == s and a < d and c < b:
                        item = (max(a, c), other, name, r)
                        if first is None or item < first:
                            first = item
    return first


def topology(names, priorities):
    ancestors = {n: set() for n in names}
    order = []
    while len(order) < len(names):
        candidates = [n for n in names if n not in order and
                      all(high in order for high, low in priorities if low == n)]
        if not candidates:
            return None, None
        n = candidates[0]; order.append(n)
        for high, low in priorities:
            if low == n:
                ancestors[n].add(high); ancestors[n].update(ancestors[high])
    return order, ancestors


def pbs(routes, priorities=(), headway=.5, max_nodes=64, budget=.025, fixed_calendar=None):
    """Conflict-driven priority DAG search; keep best feasible incumbent.

    A total-order SIPP solution seeds the incumbent. PBS branches on both
    possible orders of the first conflict, rejects cycles, and replans in
    topological order against all transitive ancestors. Bounded search makes
    no completeness or optimality guarantee.
    """
    deadline = time.perf_counter() + budget
    names = list(routes)
    stats = {'expanded': 0, 'low_level_calls': 0, 'budget_exhausted': False}

    def solve(edges):
        order, ancestors = topology(names, edges)
        if order is None:
            return None
        plans = {}
        for n in order:
            calendar = {r: list(windows) for r, windows in (fixed_calendar or {}).items()}
            for high in ancestors[n]:
                for r, a, b in plans[high].reservations:
                    calendar.setdefault(r, []).append((a, b))
            stats['low_level_calls'] += 1
            plan = sipp(routes[n], calendar, headway, deadline)
            if plan is None:
                return None
            plans[n] = plan
        return plans

    def cost(plans):
        return sum(routes[n].weight * p.finish for n, p in plans.items())

    incumbent, best_edges = None, frozenset(priorities)
    try:
        order, _ = topology(names, priorities)
        if order is None:
            return None, (), stats
        seed = frozenset(priorities) | frozenset(zip(order, order[1:]))
        incumbent = solve(seed); best_edges = seed
        stack, seen = [frozenset(priorities)], set()
        while stack and stats['expanded'] < max_nodes:
            if time.perf_counter() >= deadline:
                stats['budget_exhausted'] = True; break
            edges = stack.pop()
            if edges in seen:
                continue
            seen.add(edges); stats['expanded'] += 1
            plans = solve(edges)
            if plans is None or (incumbent is not None and cost(plans) > cost(incumbent) + 1e-9):
                continue
            hit = conflict(plans)
            if hit is None:
                incumbent, best_edges = plans, edges
                continue
            if incumbent is not None and cost(plans) >= cost(incumbent) - 1e-9:
                continue
            _, a, b, _ = hit
            # Stack visits the inexpensive-job-first branch first; both are searched.
            pairs = [(a, b), (b, a)]
            pairs.sort(key=lambda pair: routes[pair[0]].weight / max(plans[pair[0]].finish, 1e-6))
            for pair in pairs:
                child = edges | {pair}
                if topology(names, child)[0] is not None:
                    stack.append(child)
        if stack:
            stats['budget_exhausted'] = True
    except TimeoutError:
        stats['budget_exhausted'] = True
    return incumbent, tuple(sorted(best_edges)), stats

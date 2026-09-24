"""
The greedy cost function.

This is the whole "algorithm" in stage one, and it is deliberately the most
explainable thing that could work:

    cost(r, t) = w_travel   * travel_time
               + w_energy   * energy
               + w_queue    * wait_before_start
               + w_late     * expected_lateness
               + w_cap      * (capability / payload / battery violation)

The last term is a large constant rather than an infinity so that infeasible
assignments still produce a *number* - that makes the debug output readable
("heavy_1 rejected task_007: payload 200kg > 150kg") instead of just vanishing
from the candidate list.

Every weight is external so the ablation study the design calls for is a config
change, not a code change.
"""

from __future__ import annotations

from dataclasses import dataclass

from .task_model import RobotState, RobotStatus, Task


@dataclass
class CostWeights:
    travel_time: float = 1.0
    energy: float = 0.05
    queue_wait: float = 0.30
    capability_mismatch: float = 1000.0
    lateness: float = 2.0

    @classmethod
    def from_dict(cls, d: dict) -> "CostWeights":
        d = d or {}
        return cls(
            travel_time=float(d.get("travel_time", 1.0)),
            energy=float(d.get("energy", 0.05)),
            queue_wait=float(d.get("queue_wait", 0.30)),
            capability_mismatch=float(d.get("capability_mismatch", 1000.0)),
            lateness=float(d.get("lateness", 2.0)),
        )


@dataclass
class CostBreakdown:
    """A cost, plus why. The `reasons` list is what makes this debuggable."""

    robot_id: str
    task_id: str
    total: float
    travel_time: float = 0.0
    energy: float = 0.0
    queue_wait: float = 0.0
    lateness: float = 0.0
    mismatch: float = 0.0
    feasible: bool = True
    reasons: list = None

    def __post_init__(self):
        if self.reasons is None:
            self.reasons = []

    def explain(self) -> str:
        if not self.feasible:
            return f"{self.robot_id} INFEASIBLE for {self.task_id}: {'; '.join(self.reasons)}"
        return (
            f"{self.robot_id} -> {self.task_id}: total={self.total:.2f} "
            f"[travel={self.travel_time:.2f} energy={self.energy:.2f} "
            f"wait={self.queue_wait:.2f} late={self.lateness:.2f}]"
        )


def evaluate(
    robot: RobotState,
    task: Task,
    now_sec: float,
    weights: CostWeights,
    min_battery_pct: float,
    load_time_sec: float = 0.0,
    unload_time_sec: float = 0.0,
) -> CostBreakdown:
    """Score one (robot, task) pairing.

    Feasibility is checked first and, if violated, contributes the big
    mismatch penalty. Everything else is a genuine optimisation term.
    """
    reasons = []
    mismatch = 0.0

    # --- hard constraints, expressed as a penalty ------------------------
    if not robot.has_capability(task.required_capability):
        mismatch += weights.capability_mismatch
        reasons.append(
            f"lacks capability '{task.required_capability}' "
            f"(has {robot.capabilities})"
        )

    if not robot.can_carry(task.payload_kg):
        mismatch += weights.capability_mismatch
        reasons.append(
            f"payload {task.payload_kg:.1f}kg > capacity "
            f"{robot.payload_capacity_kg:.1f}kg"
        )

    if robot.status == RobotStatus.FAILED:
        mismatch += weights.capability_mismatch
        reasons.append("robot is in FAILED state")

    feasible = mismatch == 0.0

    # --- soft terms ------------------------------------------------------
    # Travel time: distance to pickup now, plus the loaded leg to dropoff.
    # The robot's own speed limit is what makes heavy_1 prefer nearby tasks.
    d_to_pickup = robot.pose.distance_to(task.pickup)
    d_loaded = task.travel_distance
    speed = max(robot.max_speed, 0.05)
    travel_time = (d_to_pickup + d_loaded) / speed

    # Energy for both legs. The empty leg carries no payload.
    energy = robot.energy_for(d_to_pickup, 0.0) + robot.energy_for(
        d_loaded, task.payload_kg
    )

    # How long the robot is unavailable before it can even start this task.
    # An idle robot starts now; a busy one starts when it frees up.
    wait_until_start = max(0.0, robot.busy_until_sec - now_sec)

    # Estimated completion, used for the deadline term.
    service_time = load_time_sec + unload_time_sec
    completion_estimate = now_sec + wait_until_start + travel_time + service_time
    absolute_deadline = task.created_at_sec + task.deadline_sec
    lateness = max(0.0, completion_estimate - absolute_deadline) if task.deadline_sec > 0 else 0.0

    # Priorities matter: a high-priority task absorbs more lateness penalty.
    priority_factor = 1.0 + 0.25 * max(0, task.priority - 1)
    lateness *= priority_factor

    total = (
        weights.travel_time * travel_time
        + weights.energy * energy
        + weights.queue_wait * wait_until_start
        + weights.lateness * lateness
        + mismatch
    )

    if not feasible:
        total = mismatch  # do not let soft terms make an infeasible pick look ok

    return CostBreakdown(
        robot_id=robot.id,
        task_id=task.id,
        total=total,
        travel_time=travel_time,
        energy=energy,
        queue_wait=wait_until_start,
        lateness=lateness,
        mismatch=mismatch,
        feasible=feasible,
        reasons=reasons,
    )


def select_robot(
    robots: list,
    task: Task,
    now_sec: float,
    weights: CostWeights,
    min_battery_pct: float,
    load_time_sec: float = 0.0,
    unload_time_sec: float = 0.0,
    battery_check: bool = True,
):
    """Pick the cheapest feasible robot for `task`.

    Returns (robot, breakdown) or (None, best_infeasible_breakdown) when no
    robot can do it - the second value is for the log line explaining why.
    """
    best = None
    best_cost = None
    best_infeasible = None

    for r in robots:
        b = evaluate(
            r, task, now_sec, weights, min_battery_pct,
            load_time_sec, unload_time_sec,
        )
        if b.feasible and battery_check and r.battery_pct < min_battery_pct:
            b.feasible = False
            b.reasons.append(
                f"battery {r.battery_pct:.1f}% < floor {min_battery_pct:.1f}%"
            )
            b.total = weights.capability_mismatch

        if b.feasible:
            if best_cost is None or b.total < best_cost:
                best, best_cost = r, b.total
        else:
            if best_infeasible is None or b.total < best_infeasible.total:
                best_infeasible = b

    return best, (best_infeasible if best is None else None)

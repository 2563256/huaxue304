"""
Task and robot-state data model shared by the scheduler.

These dataclasses are the in-process representation. The ROS message types in
hetero_fleet_msgs are the wire format; conversion lives in `to_msg`/`from_msg`
so the scheduling logic never has to know about ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class TaskState(str, Enum):
    """The task state machine, exactly as specified in the design."""

    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    TO_PICKUP = "TO_PICKUP"
    LOADING = "LOADING"
    TO_DROPOFF = "TO_DROPOFF"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    REQUEUED = "REQUEUED"


# States from which a task can still be rescheduled.
TERMINAL_STATES = {TaskState.COMPLETED, TaskState.FAILED}


class RobotStatus(str, Enum):
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    TO_PICKUP = "TO_PICKUP"
    LOADING = "LOADING"
    TO_DROPOFF = "TO_DROPOFF"
    CHARGING = "CHARGING"
    FAILED = "FAILED"


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class Task:
    """One transport job."""

    id: str
    type: str = "transport"
    pickup: Pose2D = field(default_factory=Pose2D)
    dropoff: Pose2D = field(default_factory=Pose2D)
    payload_kg: float = 0.0
    required_capability: str = ""
    priority: int = 1
    deadline_sec: float = 0.0

    # runtime bookkeeping
    created_at_sec: float = 0.0
    assigned_robot: str = ""
    state: TaskState = TaskState.PENDING
    started_at_sec: Optional[float] = None
    completed_at_sec: Optional[float] = None
    # Filled in by the manager when a task is dispatched.
    assigned_at_sec: Optional[float] = None
    requeue_count: int = 0

    @property
    def is_open(self) -> bool:
        return self.state not in TERMINAL_STATES

    @property
    def travel_distance(self) -> float:
        """Straight-line pickup -> dropoff distance, metres."""
        return self.pickup.distance_to(self.dropoff)


@dataclass
class RobotState:
    """Live state of one robot, as the scheduler sees it."""

    id: str
    type: str = ""
    pose: Pose2D = field(default_factory=Pose2D)
    status: RobotStatus = RobotStatus.IDLE

    battery_pct: float = 100.0
    battery_capacity_wh: float = 0.0
    payload_capacity_kg: float = 0.0
    current_payload_kg: float = 0.0
    max_speed: float = 1.0

    capabilities: List[str] = field(default_factory=list)
    current_task_id: str = ""

    # Travelled-distance / energy accounting, used for the metrics.
    odom_distance_m: float = 0.0
    energy_consumed_wh: float = 0.0
    tasks_completed: int = 0

    # Energy model coefficients, from robot_profiles.yaml.
    wh_per_meter_empty: float = 1.0
    wh_per_meter_kg: float = 0.01

    # Set when the robot finishes its current task; drives the idle-time
    # estimate in the cost function.
    busy_until_sec: float = 0.0

    def can_carry(self, payload_kg: float) -> bool:
        return payload_kg <= self.payload_capacity_kg

    def has_capability(self, capability: str) -> bool:
        # An empty requirement is satisfiable by anyone.
        if not capability:
            return True
        return capability in self.capabilities

    def energy_for(self, distance_m: float, payload_kg: float) -> float:
        """Wh needed to cover `distance_m` carrying `payload_kg`.

        Deliberately simple and monotonic: a base draw per metre plus a term
        proportional to mass. Good enough to make the scheduler prefer a light
        robot for a light parcel, which is the behaviour we actually want.
        """
        per_m = self.wh_per_meter_empty + self.wh_per_meter_kg * payload_kg
        return per_m * distance_m


@dataclass
class FleetMetrics:
    """Everything the experiment report is built from."""

    tasks_completed: int = 0
    tasks_failed: int = 0
    total_distance_m: float = 0.0
    total_energy_wh: float = 0.0
    reschedule_count: int = 0
    first_task_at_sec: Optional[float] = None
    last_completion_at_sec: Optional[float] = None
    # Per-robot busy time, for utilisation.
    robot_busy_sec: dict = field(default_factory=dict)
    completion_times: List[float] = field(default_factory=list)
    tardy_tasks: int = 0

    def completion_rate(self, submitted: int) -> float:
        if submitted <= 0:
            return 0.0
        return self.tasks_completed / submitted

    def mean_completion_time(self) -> float:
        if not self.completion_times:
            return 0.0
        return sum(self.completion_times) / len(self.completion_times)

    def makespan(self) -> float:
        if self.first_task_at_sec is None or self.last_completion_at_sec is None:
            return 0.0
        return self.last_completion_at_sec - self.first_task_at_sec

    def utilisation(self, elapsed_sec: float, n_robots: int) -> float:
        """Fraction of total available robot-time spent executing tasks."""
        if elapsed_sec <= 0 or n_robots <= 0:
            return 0.0
        busy = sum(self.robot_busy_sec.values())
        return busy / (elapsed_sec * n_robots)

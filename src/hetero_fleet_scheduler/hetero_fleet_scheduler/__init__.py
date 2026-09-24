"""
hetero_fleet_scheduler - task model, greedy allocation and fleet state machine.

Layering, deliberately kept one-directional so the scheduling logic stays
testable without a running ROS graph:

    task_model.py   pure data structures, no ROS
    cost.py         pure cost function, no ROS
    metrics.py      pure accounting, no ROS
    fleet_manager.py  the only module that touches ROS

That means the interesting part (how a task is matched to a robot) can be
exercised in a plain Python process, which is what `test_cost.py` does.
"""

from .cost import CostBreakdown, CostWeights, evaluate, select_robot
from .metrics import MetricsRecorder
from .task_model import (
    FleetMetrics,
    Pose2D,
    RobotState,
    RobotStatus,
    Task,
    TaskState,
    TERMINAL_STATES,
)

__all__ = [
    "CostBreakdown",
    "CostWeights",
    "evaluate",
    "select_robot",
    "MetricsRecorder",
    "FleetMetrics",
    "Pose2D",
    "RobotState",
    "RobotStatus",
    "Task",
    "TaskState",
    "TERMINAL_STATES",
]

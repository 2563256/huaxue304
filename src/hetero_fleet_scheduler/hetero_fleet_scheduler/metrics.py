"""
Metrics collection and reporting for the fleet experiment.

The design calls for a specific set of numbers, so they are recorded
explicitly rather than derived after the fact from bag files:

  task completion rate, mean task completion time, makespan,
  total travelled distance, energy consumed, robot utilisation,
  reschedule count, and how all of that changes with the fleet mix.

Everything accumulates in a FleetMetrics dataclass and can be dumped to JSON
or CSV when the run ends, or on demand from a service call.
"""

from __future__ import annotations

import csv
import json
import os
from dataclasses import asdict
from typing import Dict, List

from .task_model import FleetMetrics, RobotState, Task

# One row per completed task, for the CSV export.
TASK_CSV_FIELDS = [
    "task_id",
    "type",
    "robot_id",
    "priority",
    "payload_kg",
    "required_capability",
    "created_at_sec",
    "assigned_at_sec",
    "completed_at_sec",
    "completion_time_sec",
    "deadline_sec",
    "tardy",
    "requeue_count",
    "distance_m",
    "energy_wh",
]


class MetricsRecorder:
    """Accumulates per-task and per-robot numbers during a run."""

    def __init__(self, robot_ids: List[str]):
        self.m = FleetMetrics()
        self.robot_ids = list(robot_ids)
        for rid in self.robot_ids:
            self.m.robot_busy_sec[rid] = 0.0
        self.rows: List[dict] = []
        self.submitted = 0

    # --- lifecycle --------------------------------------------------------

    def note_submitted(self, now_sec: float) -> None:
        """Record that a task entered the system."""
        self.submitted += 1
        if self.m.first_task_at_sec is None:
            self.m.first_task_at_sec = now_sec

    def note_completed(
        self,
        task: Task,
        robot: RobotState,
        now_sec: float,
        distance_m: float,
        energy_wh: float,
    ) -> None:
        self.m.tasks_completed += 1
        completed_at = now_sec
        self.m.last_completion_at_sec = completed_at

        started = task.assigned_at_sec or task.created_at_sec
        completion_time = completed_at - task.created_at_sec
        self.m.completion_times.append(completion_time)

        # The robot was busy from assignment until completion.
        busy = completed_at - started
        if busy > 0:
            self.m.robot_busy_sec[robot.id] = (
                self.m.robot_busy_sec.get(robot.id, 0.0) + busy
            )

        deadline_at = task.created_at_sec + task.deadline_sec
        tardy = task.deadline_sec > 0 and completed_at > deadline_at
        if tardy:
            self.m.tardy_tasks += 1

        self.rows.append(
            {
                "task_id": task.id,
                "type": task.type,
                "robot_id": robot.id,
                "priority": task.priority,
                "payload_kg": round(task.payload_kg, 3),
                "required_capability": task.required_capability,
                "created_at_sec": round(task.created_at_sec, 3),
                "assigned_at_sec": round(started, 3),
                "completed_at_sec": round(completed_at, 3),
                "completion_time_sec": round(completion_time, 3),
                "deadline_sec": task.deadline_sec,
                "tardy": int(tardy),
                "requeue_count": task.requeue_count,
                "distance_m": round(distance_m, 3),
                "energy_wh": round(energy_wh, 4),
            }
        )

    def note_failed(self, task: Task, reason: str = "") -> None:
        self.m.tasks_failed += 1
        self.rows.append(
            {
                "task_id": task.id,
                "type": task.type,
                "robot_id": task.assigned_robot,
                "priority": task.priority,
                "payload_kg": round(task.payload_kg, 3),
                "required_capability": task.required_capability,
                "created_at_sec": round(task.created_at_sec, 3),
                "assigned_at_sec": round(task.assigned_at_sec or 0.0, 3),
                "completed_at_sec": "",
                "completion_time_sec": "",
                "deadline_sec": task.deadline_sec,
                "tardy": "",
                "requeue_count": task.requeue_count,
                "distance_m": "",
                "energy_wh": "",
                "failure_reason": reason,
            }
        )

    def note_reschedule(self) -> None:
        self.m.reschedule_count += 1

    # --- reporting --------------------------------------------------------

    def snapshot(self, robots: List[RobotState]) -> dict:
        """A plain-dict summary, suitable for logging or a service response."""
        elapsed = 0.0
        if self.m.first_task_at_sec is not None:
            end = self.m.last_completion_at_sec
            if end is None:
                end = self.m.first_task_at_sec
            elapsed = max(0.0, end - self.m.first_task_at_sec)

        return {
            "tasks_submitted": self.submitted,
            "tasks_completed": self.m.tasks_completed,
            "tasks_failed": self.m.tasks_failed,
            "completion_rate": round(self.m.completion_rate(self.submitted), 4),
            "mean_completion_time_sec": round(self.m.mean_completion_time(), 3),
            "makespan_sec": round(self.m.makespan(), 3),
            "total_distance_m": round(sum(r.odom_distance_m for r in robots), 3),
            "total_energy_wh": round(sum(r.energy_consumed_wh for r in robots), 3),
            "utilisation": round(self.m.utilisation(elapsed, len(self.robot_ids)), 4),
            "reschedule_count": self.m.reschedule_count,
            "tardy_tasks": self.m.tardy_tasks,
            "per_robot": {
                r.id: {
                    "tasks_completed": r.tasks_completed,
                    "distance_m": round(r.odom_distance_m, 3),
                    "energy_wh": round(r.energy_consumed_wh, 3),
                    "battery_pct": round(r.battery_pct, 1),
                    "busy_sec": round(self.m.robot_busy_sec.get(r.id, 0.0), 3),
                }
                for r in robots
            },
        }

    def write_json(self, path: str, robots: List[RobotState]) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump(
                {
                    "summary": self.snapshot(robots),
                    "tasks": self.rows,
                },
                f,
                indent=2,
            )

    def write_csv(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        # Union of keys so the failure rows' extra column survives.
        fields = list(TASK_CSV_FIELDS)
        if any("failure_reason" in r for r in self.rows):
            fields.append("failure_reason")
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                w.writerow(r)

    def log_line(self, robots: List[RobotState]) -> str:
        s = self.snapshot(robots)
        return (
            f"completed={s['tasks_completed']}/{s['tasks_submitted']} "
            f"rate={s['completion_rate']:.0%} "
            f"mean_t={s['mean_completion_time_sec']:.1f}s "
            f"makespan={s['makespan_sec']:.1f}s "
            f"dist={s['total_distance_m']:.1f}m "
            f"energy={s['total_energy_wh']:.1f}Wh "
            f"util={s['utilisation']:.0%} "
            f"resched={s['reschedule_count']}"
        )

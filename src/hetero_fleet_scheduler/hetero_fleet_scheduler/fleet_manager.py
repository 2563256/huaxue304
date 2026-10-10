#!/usr/bin/env python3
"""
fleet_manager - the scheduling brain of the heterogeneous fleet.

What it does, in one paragraph: it keeps a live picture of every robot, accepts
tasks as they arrive, and whenever a task arrives or a robot frees up it re-runs
the greedy cost function and hands the winning robot a NavigateToPose goal. The
robot's own Nav2 stack does the actual driving; this node only decides *who*
does *what*, and then drives the task state machine
PENDING -> ASSIGNED -> TO_PICKUP -> LOADING -> TO_DROPOFF -> COMPLETED
with FAILED / REQUEUED branches.

Design notes worth knowing before reading:

* One ROS node, no ROS in the cost path. Everything about "how good is this
  pairing" lives in cost.py and is pure Python, so it can be unit tested and
  swapped for OR-Tools later without touching the ROS plumbing.

* The manager is a NavigateToPose *client*, once per robot namespace. It never
  publishes cmd_vel. That boundary is what lets the same scheduler drive real
  hardware later.

* Re-scheduling is triggered by events, not a timer loop: a new task, or a
  robot transitioning to IDLE. That is the "recompute on arrival or completion"
  behaviour the design asks for.

* Battery, load/unload dwell time and energy are all simulated here rather than
  read from Gazebo, because the milestone is a *scheduling* experiment. The
  hooks (battery drain per task, charging at a charger) are real, the physics
  underneath them is not.
"""

from __future__ import annotations

import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from hetero_fleet_msgs.msg import FleetStatus, RobotState as RobotStateMsg, Task as TaskMsg

from .cost import CostWeights, evaluate, select_robot
from .metrics import MetricsRecorder
from .task_model import (
    Pose2D,
    RobotState,
    RobotStatus,
    Task,
    TaskState,
    TERMINAL_STATES,
)


try:  # optional: only used to keep approach slots on drivable floor
    from PIL import Image
except ImportError:  # pragma: no cover
    Image = None


# Approach slots around a shared pick/drop point, in units of zone_slot_spacing.
# Slot 0 is the nominal point itself, so a robot that is alone still drives
# exactly to the marker; the rest are only used when another vehicle already
# holds the zone. The zones in this world are 0.9 x 0.9 m markers rather than
# parking bays, so without this every robot with a task at "zone A" drives to
# the same coordinate - the second one arrives on top of the first, the
# collision monitor stops it, and both sit there waiting.
SLOT_RING = (
    (0.0, 0.0),
    (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
    (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0),
)


def yaw_to_quaternion(yaw: float):
    """Convert a planar yaw into the quaternion Nav2 wants."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def default_metrics_dir() -> str:
    """Where metrics.json / tasks.csv are written.

    Defaults to <workspace>/metrics - the folder holding src/, build/ and
    install/ - so a run's numbers stay with the experiment instead of landing
    in /tmp, which is wiped on reboot and shared with every other user on the
    machine. A previous run's files are simply overwritten, which is what you
    want when re-running a scenario.

    Override with the HETERO_FLEET_METRICS_DIR environment variable or the
    `metrics_dir` parameter. The workspace root is derived from this package's
    install location (<ws>/install/<pkg>/share/<pkg> -> <ws>); if that cannot
    be resolved (running from source, package not indexed) we fall back to
    ~/hetero_fleet_geomap_ws/metrics.
    """
    override = os.environ.get("HETERO_FLEET_METRICS_DIR", "").strip()
    if override:
        return override
    try:
        share = get_package_share_directory("hetero_fleet_scheduler")
        workspace = os.path.abspath(os.path.join(share, *([".."] * 4)))
        return os.path.join(workspace, "metrics")
    except Exception:  # noqa: BLE001 - package index may be missing
        return os.path.join(
            os.path.expanduser("~"), "hetero_fleet_geomap_ws", "metrics"
        )


def quaternion_to_yaw(q) -> float:
    """Planar yaw from a quaternion; roll/pitch are ignored."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("profiles_file", "")
        self.declare_parameter("tasks_file", "")
        self.declare_parameter("autostart_tasks", False)
        self.declare_parameter("task_interval_sec", 5.0)
        self.declare_parameter("metrics_dir", default_metrics_dir())
        self.declare_parameter("report_period_sec", 15.0)

        # An empty string (scheduler.launch.py's default) or an unset
        # parameter both mean "use the workspace default" rather than
        # "disable metrics", which is what an empty outdir used to mean.
        self.metrics_dir = (
            str(self.get_parameter("metrics_dir").value or "").strip()
            or default_metrics_dir()
        )
        # How often we simulate battery drain while a robot is executing.
        self.declare_parameter("battery_drain_per_meter_pct", 0.25)
        # Spacing between approach slots at a shared zone (metres). Big enough
        # that two vehicles side by side at a zone do not touch. The widest
        # robot on this map is now 0.50 m (heavy, 0.36 scale), so 1.0 m of ring
        # radius leaves clear air between any pair - and, unlike the 1.4 m this
        # used to be, the slots stay inside the ring corridor instead of landing
        # on the wall beside it (a 1.4 m offset from a marker with 0.6 m of
        # clearance is often outside the drivable area).
        self.declare_parameter("zone_slot_spacing", 1.0)
        self.declare_parameter("zone_dispatch_clearance_m", 2.0)
        # Optional map yaml used to check a slot is on drivable floor; empty
        # falls back to maps/geomap.yaml from hetero_fleet_bringup.
        self.declare_parameter("zone_slot_map", "")
        # --- standoff / deadlock handling ---------------------------------
        # Outer watchdog uses ROS time, so slower Gazebo does not consume the
        # navigation window. Nav2 first gets its local 15 s progress check and
        # recovery; the scheduler intervenes after 45 s of genuine stagnation.
        self.declare_parameter("stall_timeout_sec", 45.0)
        self.declare_parameter("stall_move_eps_m", 0.10)
        self.declare_parameter("stall_yaw_weight", 0.30)
        # Net movement for a detour, ignoring small oscillations. Near the
        # goal rely on path / pose / heading progress instead.
        self.declare_parameter("stall_wander_m", 0.35)
        # If the robot is this close to the goal when the stall fires, accept
        # the arrival instead of reversing. Nav2's own xy_goal_tolerance is
        # 0.15 m; 0.30 m is that plus the width of the zone markers this world
        # draws (0.6 m), and it is what stops the classic "robot sits ON the
        # pickup pad for minutes because the navigation stack never declared
        # success" hang.
        self.declare_parameter("stall_arrive_radius_m", 0.30)
        # How far to reverse when breaking a standoff, and how long to give the
        # reverse goal before giving up on it and re-planning from where we are.
        self.declare_parameter("stall_backoff_m", 0.55)
        # Each retry of the same leg reverses further than the last. A robot
        # that has already backed up 0.55 m and re-planned into the same wall
        # of traffic will not be helped by backing up 0.55 m again.
        self.declare_parameter("stall_backoff_growth", 0.6)
        self.declare_parameter("stall_backoff_max_m", 1.30)
        self.declare_parameter("stall_backoff_sec", 15.0)
        # Recoveries per leg before the leg is declared failed. Failing is not
        # free - the task is requeued and the robot re-drives from its charger
        # - so this is deliberately > 1.
        self.declare_parameter("stall_max_recoveries", 3)
        # --- returning to the charger -------------------------------------
        # When a robot runs out of work it drives back to the charger named in
        # its profile entry (and thus starts there too), so a finished fleet
        # parks on its bays instead of standing on the task pads.
        #
        # This is deliberately NOT a per-task trip. A robot that has just
        # finished a job is immediately available for the next one; with a task
        # stream arriving every few seconds, sending it home in between turns
        # every task into a there-and-back deadhead, which on this map is 5-15 m
        # each way. The trip home happens when THIS robot has nothing left to
        # serve (see _work_for) and it has been idle for `dock_when_idle_sec` -
        # or only `clear_task_point_sec` if it is sitting on a pad. A robot
        # still holding a charger bay does nothing, so the fleet simply sits in
        # the bays until work arrives again.
        self.declare_parameter("return_to_charger", True)
        self.declare_parameter("dock_when_idle_sec", 3.0)
        # A robot that has finished a job is usually standing ON the pad it
        # just used, which is the one place it must not loiter: the next task
        # for that pad has to queue behind it, and the whole point of the zone
        # slots is to stop exactly that. So an idle robot sitting on a pickup
        # or dropoff point goes home much sooner than the general threshold.
        self.declare_parameter("clear_task_point_sec", 1.0)
        self.declare_parameter("clear_task_point_radius_m", 1.2)
        # Close enough to count as "already home", metres. Re-driving a 5 cm
        # move that the controller would spend a recovery on is wasted time.
        self.declare_parameter("charger_arrive_radius_m", 0.35)
        # Fixed RViz fleet-status blocks, in the map frame: one robot per
        # corner of this rectangle. maps/geomap.yaml is 520 x 207 cells at
        # 0.05 m (26.0 x 10.3 m) spanning x [-11.3, 14.7], y [-3.2, 7.1], and
        # this rectangle hugs those bounds so that all four blocks land inside
        # the map instead of hanging off its corners (where RViz clips them and
        # they read as detached from the scene). With the default inset they
        # sit 2.1-7.5 m from the nearest zone marker, so a label never covers
        # the pad a robot is working on.
        # Adjust to match your usual RViz view. text_height is in metres.
        self.declare_parameter("status_panel_x_min", -11.0)
        self.declare_parameter("status_panel_x_max", 14.4)
        self.declare_parameter("status_panel_y_min", -3.0)
        self.declare_parameter("status_panel_y_max", 6.9)
        self.declare_parameter("status_panel_z", 0.4)
        self.declare_parameter("status_panel_text_height", 0.6)
        # How far each label is pulled from its corner towards the centre of
        # the panel rectangle, in metres, along the corner->centre diagonal.
        # A label parked on the exact corner sits in the unscanned (black)
        # part of the map, visually detached from the fleet.
        self.declare_parameter("status_panel_inset_m", 2.6)
        # Absolute map coordinates that override the corner for named robots,
        # as "name:x,y" entries. Unlike the rectangle above, both coordinates
        # are absolute, so this is the knob to use when one label lands
        # somewhere unhelpful in your particular view:
        #   -p status_panel_pos_overrides:=['omni_1:6.0,0.5']
        self.declare_parameter("status_panel_pos_overrides", [])

        self.cb_group = ReentrantCallbackGroup()

        # --- profiles -----------------------------------------------------
        profiles = self._load_profiles()
        self.types = profiles["robot_types"]
        self.fleet_cfg = profiles["fleet"]
        sched = profiles.get("scheduler", {})
        self.weights = CostWeights.from_dict(sched.get("weights", {}))
        self.min_battery_pct = float(sched.get("min_battery_pct", 15.0))
        self.load_time = float(sched.get("load_time_sec", 5.0))
        self.unload_time = float(sched.get("unload_time_sec", 5.0))

        # --- fleet state --------------------------------------------------
        self.robots: dict[str, RobotState] = {}
        # Where each robot's odom frame is anchored on the map. The odom relay
        # publishes ground-truth motion relative to the spawn pose, so
        # world = spawn + R(spawn_yaw) * odom.
        self.spawn_pose: dict[str, Pose2D] = {}
        # Each robot's charging bay: where it is created and where it returns
        # to after every task. Falls back to the spawn pose for a profile that
        # predates the `charger` key, so an old YAML still runs.
        self.charger_pose: dict[str, Pose2D] = {}
        # The same ground-truth pose as self.spawn_pose's anchor implies, but
        # refreshed for *every* odom message rather than only while the robot
        # is idle. robot.pose deliberately stays on the leg-by-leg bookkeeping
        # (see _on_odom); this dict is what the stall watchdog reads, because
        # during a leg robot.pose is frozen at the previous goal and cannot
        # tell "driving" from "wedged".
        self.live_pose: dict[str, tuple[float, float, float]] = {}
        # Goal handle of the navigation the robot is currently running, so a
        # recovery can preempt it.
        self.active_goal: dict[str, object] = {}
        # Half the footprint diagonal, i.e. the radius a zone slot must be
        # clear by for this robot to be able to sit in it.
        self.slot_clearance: dict[str, float] = {}
        for entry in self.fleet_cfg:
            t = self.types[entry["type"]]
            sx, sy, syaw = entry["spawn"]
            self.spawn_pose[entry["name"]] = Pose2D(sx, sy, syaw)
            home = entry.get("charger") or entry["spawn"]
            hx, hy = float(home[0]), float(home[1])
            hyaw = float(home[2]) if len(home) > 2 else 0.0
            self.charger_pose[entry["name"]] = Pose2D(hx, hy, hyaw)
            if math.hypot(hx - sx, hy - sy) > 0.05:
                # Odometry is anchored at `spawn`, so the two MUST agree: if
                # they do not, every position the manager reports for this
                # robot is offset by (spawn - charger) and it will "arrive" at
                # its charger somewhere else.
                self.get_logger().error(
                    f"{entry['name']}: spawn ({sx:.2f},{sy:.2f}) and charger "
                    f"({hx:.2f},{hy:.2f}) disagree - the odometry anchor wins, "
                    f"so the robot will not dock where the charger marker is"
                )
            length, width = t["footprint"]
            self.slot_clearance[entry["name"]] = 0.5 * math.hypot(length, width)
            self.robots[entry["name"]] = RobotState(
                id=entry["name"],
                type=entry["type"],
                pose=Pose2D(sx, sy, syaw),
                status=RobotStatus.IDLE,
                battery_capacity_wh=t["battery_capacity_wh"],
                payload_capacity_kg=t["payload_kg"],
                max_speed=t["max_speed"],
                capabilities=list(t["capabilities"]),
                wh_per_meter_empty=t["wh_per_meter_empty"],
                wh_per_meter_kg=t["wh_per_meter_kg"],
            )

        self.tasks: dict[str, Task] = {}
        self.pending: list[str] = []          # task ids awaiting assignment
        self.metrics = MetricsRecorder(list(self.robots.keys()))

        # Zone -> {slot index: robot id}, so two tasks aimed at the same
        # pick/drop marker are served from different approach positions.
        self.zone_claims: dict[tuple, dict[int, str]] = {}
        self._occupancy = self._load_occupancy()

        # --- ROS interfaces ------------------------------------------------
        for name in self.robots:
            self.create_subscription(
                RobotStateMsg,
                f"/{name}/robot_state",
                lambda msg, n=name: self._on_robot_state(n, msg),
                10,
                callback_group=self.cb_group,
            )
            self.create_subscription(
                PoseStamped,
                f"/{name}/gz_pose",
                lambda msg, n=name: self._on_pose(n, msg),
                10,
                callback_group=self.cb_group,
            )
            # gz_pose is the natural source, but the per-model <name>/pose
            # bridge endpoint does not actually emit in this world, so it
            # never fires. The odometry relay does publish, and its values are
            # ground truth too, so use those to keep the manager's idea of
            # where each robot is from drifting away from reality.
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, n=name: self._on_odom(n, msg),
                10,
                callback_group=self.cb_group,
            )

        self.create_subscription(
            TaskMsg, "/task_request", self._on_task_request, 10,
            callback_group=self.cb_group,
        )

        self.status_pub = self.create_publisher(FleetStatus, "/fleet_status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/task_markers", 10)

        # One action client per robot, in that robot's namespace. The action
        # lives under /<name>/navigate_to_pose, so the name is fully qualified
        # rather than remapped.
        self.nav_clients: dict[str, ActionClient] = {}
        self._nav_feedback: dict[str, dict] = {}
        for name in self.robots:
            self.nav_clients[name] = ActionClient(
                self, NavigateToPose, f"/{name}/navigate_to_pose",
                callback_group=self.cb_group,
            )

        # Guards the assignment path: several callbacks can try to schedule at
        # once (task arrival + robot idle), and double-assignment is the classic
        # bug in this kind of node.
        self._lock = threading.Lock()

        # --- stall watchdog tuning -----------------------------------------
        self.stall_timeout_sec = float(
            self.get_parameter("stall_timeout_sec").value
        )
        self.stall_move_eps_m = float(
            self.get_parameter("stall_move_eps_m").value
        )
        self.stall_yaw_weight = float(
            self.get_parameter("stall_yaw_weight").value
        )
        self.stall_wander_m = float(self.get_parameter("stall_wander_m").value)
        self.stall_arrive_radius_m = float(
            self.get_parameter("stall_arrive_radius_m").value
        )
        self.stall_backoff_m = float(self.get_parameter("stall_backoff_m").value)
        self.stall_backoff_growth = float(
            self.get_parameter("stall_backoff_growth").value
        )
        self.stall_backoff_max_m = float(
            self.get_parameter("stall_backoff_max_m").value
        )
        self.stall_backoff_sec = float(
            self.get_parameter("stall_backoff_sec").value
        )
        self.stall_max_recoveries = int(
            self.get_parameter("stall_max_recoveries").value
        )
        # 0 or less disables the watchdog (useful for A/B runs).
        self.stall_enabled = self.stall_timeout_sec > 0.0
        self.return_to_charger = self._as_bool(
            self.get_parameter("return_to_charger").value
        )
        self.dock_when_idle_sec = float(
            self.get_parameter("dock_when_idle_sec").value
        )
        self.clear_task_point_sec = float(
            self.get_parameter("clear_task_point_sec").value
        )
        self.clear_task_point_radius_m = float(
            self.get_parameter("clear_task_point_radius_m").value
        )
        self.charger_arrive_radius_m = float(
            self.get_parameter("charger_arrive_radius_m").value
        )
        # When each robot last became IDLE, or None if it is not idle. Drives
        # the "the fleet has run out of work, go home" decision in _tick.
        self._idle_since: dict[str, float | None] = {
            name: None for name in self.robots
        }

        # --- timers --------------------------------------------------------
        self.create_timer(2.0, self._tick, callback_group=self.cb_group)
        self.create_timer(
            float(self.get_parameter("report_period_sec").value),
            self._report, callback_group=self.cb_group,
        )

        # --- optional task stream ------------------------------------------
        self.scripted: list[Task] = []
        tasks_file = self.get_parameter("tasks_file").value
        if tasks_file:
            self.scripted = self._load_tasks(tasks_file)
            self.get_logger().info(
                f"loaded {len(self.scripted)} tasks from {tasks_file}"
            )
            autostart = self.get_parameter("autostart_tasks").value
            if isinstance(autostart, str):
                autostart = autostart.strip().lower() in ("1", "true", "yes", "on")
            if autostart:
                interval = float(self.get_parameter("task_interval_sec").value)
                self._schedule_scripted(interval)

        self.get_logger().info(
            f"fleet_manager up: {len(self.robots)} robots "
            f"({', '.join(self.robots)}), weights={self.weights}"
        )

    # ------------------------------------------------------------------ setup

    def _load_profiles(self) -> dict:
        path = self.get_parameter("profiles_file").value
        if not path:
            path = os.path.join(
                get_package_share_directory("hetero_fleet_description"),
                "config", "robot_profiles.yaml",
            )
        with open(path) as f:
            return yaml.safe_load(f)

    def _load_tasks(self, path: str) -> list[Task]:
        with open(path) as f:
            raw = yaml.safe_load(f)
        out = []
        for item in raw.get("tasks", []):
            p = item["pickup"]
            d = item["dropoff"]
            out.append(
                Task(
                    id=str(item["id"]),
                    type=str(item.get("type", "transport")),
                    pickup=Pose2D(float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0),
                    dropoff=Pose2D(float(d[0]), float(d[1]), float(d[2]) if len(d) > 2 else 0.0),
                    payload_kg=float(item.get("payload_kg", 0.0)),
                    required_capability=str(item.get("required_capability", "")),
                    priority=int(item.get("priority", 1)),
                    deadline_sec=float(item.get("deadline_sec", 0.0)),
                )
            )
        return out

    # ------------------------------------------------------ zone approaches
    def _load_occupancy(self):
        """Static map as {pixels, geometry}, or None if it cannot be read.

        Only used to keep approach slots on drivable floor - the scheduler has
        no other reason to know the world. Everything degrades gracefully to
        "slots unchecked" if Pillow is missing or the map is elsewhere.
        """
        if Image is None:
            self.get_logger().warn("Pillow not available; zone slots unchecked")
            return None
        path = self.get_parameter("zone_slot_map").value
        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory("hetero_fleet_bringup"),
                    "maps", "geomap.yaml",
                )
            except Exception:  # noqa: BLE001 - package index may be missing
                return None
        try:
            with open(path) as f:
                meta = yaml.safe_load(f)
            image = Image.open(os.path.join(os.path.dirname(path), meta["image"]))
            image.load()
            image = image.convert("L")
            return {
                "px": image.load(),
                "w": image.size[0],
                "h": image.size[1],
                "res": float(meta["resolution"]),
                "ox": float(meta["origin"][0]),
                "oy": float(meta["origin"][1]),
                "negate": int(meta.get("negate", 0)),
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"zone slot map unusable ({exc!r}); unchecked")
            return None

    def _slot_is_free(self, x: float, y: float, clearance: float) -> bool:
        """True when a disc of `clearance` at (x, y) is free floor."""
        occ = self._occupancy
        if occ is None:
            return True
        res = occ["res"]
        for c in range(int((x - clearance - occ["ox"]) / res),
                       int((x + clearance - occ["ox"]) / res) + 1):
            for r in range(int((y - clearance - occ["oy"]) / res),
                           int((y + clearance - occ["oy"]) / res) + 1):
                if not (0 <= c < occ["w"] and 0 <= r < occ["h"]):
                    return False
                value = occ["px"][c, occ["h"] - 1 - r]
                if occ["negate"]:
                    value = 255 - value
                if value < 128:          # black = occupied
                    return False
        return True

    @staticmethod
    def _zone_key(point: Pose2D) -> tuple:
        """Zones are identified on a 0.5 m grid, so near-identical task
        coordinates from different scenario files share one set of slots."""
        return (round(point.x * 2.0) / 2.0, round(point.y * 2.0) / 2.0)

    @staticmethod
    def _as_bool(value) -> bool:
        """Accept both a real bool (YAML/launch) and a string ("false")."""
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _claim_zone_slot(self, robot: RobotState, point: Pose2D) -> Pose2D:
        """Return the position this robot should drive to for `point`.

        Slot 0 is the marker itself, so a robot with the zone to itself is
        unaffected. If somebody else holds it, the next slot that is both
        unclaimed and on clear floor is used instead, which is what keeps two
        vehicles from queueing on one coordinate.
        """
        spacing = float(self.get_parameter("zone_slot_spacing").value)
        clearance = self.slot_clearance.get(robot.id, 0.45)
        key = self._zone_key(point)
        with self._lock:
            taken = self.zone_claims.setdefault(key, {})
            # Drop any slot this robot was holding at the same zone.
            for slot in [s for s, owner in taken.items() if owner == robot.id]:
                del taken[slot]
            for slot, (dx, dy) in enumerate(SLOT_RING):
                if slot in taken:
                    continue
                x = point.x + dx * spacing
                y = point.y + dy * spacing
                if not self._slot_is_free(x, y, clearance):
                    continue
                taken[slot] = robot.id
                if slot == 0:
                    return point
                self.get_logger().info(
                    f"{robot.id}: zone ({point.x:.1f},{point.y:.1f}) already "
                    f"held - approaching slot {slot} at ({x:.1f},{y:.1f})"
                )
                return Pose2D(x, y, point.yaw)
        # Every slot claimed or blocked: use the marker and accept the queue.
        with self._lock:
            self.zone_claims.setdefault(key, {})[0] = robot.id
        self.get_logger().warn(
            f"{robot.id}: no free approach slot at ({point.x:.1f},{point.y:.1f})"
        )
        return point

    def _release_zone(self, robot: RobotState, point: Pose2D) -> None:
        key = self._zone_key(point)
        with self._lock:
            taken = self.zone_claims.get(key)
            if not taken:
                return
            for slot in [s for s, owner in taken.items() if owner == robot.id]:
                del taken[slot]
            if not taken:
                self.zone_claims.pop(key, None)

    def _schedule_scripted(self, interval: float) -> None:
        """Feed the scripted task list in one at a time."""
        def run():
            for t in self.scripted:
                time.sleep(interval)
                self.submit_task(t)
            self.get_logger().info("all scripted tasks submitted")
        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------- callbacks
    # The robot_state / gz_pose subscriptions are kept so the node works in a
    # full deployment, but the manager is the authority on pose/status while
    # the simulation runs, because it is the one driving the state machine.

    def _on_robot_state(self, name: str, msg: RobotStateMsg) -> None:
        r = self.robots.get(name)
        if r is None:
            return
        # Positions reported by a real robot would win here; in simulation the
        # manager's own tracking is ahead of the published snapshot, so we only
        # take battery if it is lower (i.e. trust the pessimistic value).
        if msg.battery_pct < r.battery_pct:
            r.battery_pct = msg.battery_pct

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        r = self.robots.get(name)
        if r is None or r.status in (RobotStatus.IDLE, RobotStatus.FAILED):
            # Idle pose updates are harmless and keep RViz honest.
            if r is not None:
                r.pose.x = msg.pose.position.x
                r.pose.y = msg.pose.position.y

    def _on_odom(self, name: str, msg: Odometry) -> None:
        """Re-anchor the manager's pose from the ground-truth odometry.

        Only applied when the robot is not executing, so it never fights the
        leg-by-leg bookkeeping of a running task. Without this a failed leg
        left the manager permanently convinced the robot was at the goal it
        never reached - /fleet_status advertised a pickup point 11 m from
        where the robot actually sat, and the cost function measured travel
        from the wrong place.

        The freshly computed value is also stored in self.live_pose for *every*
        message, executing or not: the stall watchdog has to see a robot that
        is holding a navigation goal and not moving, and robot.pose - frozen at
        the previous goal for the duration of a leg - cannot show that.
        """
        r = self.robots.get(name)
        spawn = self.spawn_pose.get(name)
        if r is None or spawn is None:
            return
        dx = msg.pose.pose.position.x
        dy = msg.pose.pose.position.y
        c0, s0 = math.cos(spawn.yaw), math.sin(spawn.yaw)
        x = spawn.x + c0 * dx - s0 * dy
        y = spawn.y + s0 * dx + c0 * dy
        yaw = quaternion_to_yaw(msg.pose.pose.orientation) + spawn.yaw
        self.live_pose[name] = (x, y, yaw)
        if r.status not in (RobotStatus.IDLE, RobotStatus.FAILED):
            return
        r.pose.x, r.pose.y, r.pose.yaw = x, y, yaw

    def _live_xy(self, name: str):
        """Best current position of a robot, or None if never seen.

        Prefers the odometry-relayed ground truth; falls back to the manager's
        own bookkeeping so a robot whose odom relay has not published yet is
        still tracked.
        """
        pose = self.live_pose.get(name)
        if pose is not None:
            return pose
        r = self.robots.get(name)
        if r is None:
            return None
        return (r.pose.x, r.pose.y, r.pose.yaw)
    def _on_task_request(self, msg: TaskMsg) -> None:
        task = Task(
            id=msg.id,
            type=msg.type or "transport",
            pickup=Pose2D(msg.pickup.x, msg.pickup.y, msg.pickup.theta),
            dropoff=Pose2D(msg.dropoff.x, msg.dropoff.y, msg.dropoff.theta),
            payload_kg=float(msg.payload_kg),
            required_capability=msg.required_capability,
            priority=int(msg.priority),
            deadline_sec=float(msg.deadline_sec),
            created_at_sec=self._now(),
        )
        self.submit_task(task)

    # -------------------------------------------------------------- the core

    def submit_task(self, task: Task) -> None:
        """Accept a task into the queue and try to place it immediately."""
        with self._lock:
            if task.id in self.tasks:
                self.get_logger().warn(f"duplicate task id {task.id}, ignoring")
                return
            if task.created_at_sec == 0.0:
                task.created_at_sec = self._now()
            self.tasks[task.id] = task
            self.pending.append(task.id)
            self.metrics.note_submitted(task.created_at_sec)

        self.get_logger().info(
            f"task {task.id} submitted: {task.payload_kg:.1f}kg "
            f"cap={task.required_capability or '-'} prio={task.priority} "
            f"pickup=({task.pickup.x:.1f},{task.pickup.y:.1f}) "
            f"dropoff=({task.dropoff.x:.1f},{task.dropoff.y:.1f})"
        )
        self._try_assign_all()

    def _pickup_ready(self, point: Pose2D, ignore_robot: str) -> bool:
        """Keep waiting jobs off an occupied pad and its narrow exit corridor."""
        radius = float(self.get_parameter("zone_dispatch_clearance_m").value)
        for (x, y), claims in getattr(self, "zone_claims", {}).items():
            if any(owner != ignore_robot for owner in claims.values()) and \
                    math.hypot(point.x-x, point.y-y) < radius:
                return False
        for other in self.robots.values():
            if other.id == ignore_robot:
                continue
            if other.status not in (RobotStatus.ASSIGNED, RobotStatus.TO_PICKUP,
                    RobotStatus.LOADING, RobotStatus.TO_DROPOFF, RobotStatus.CHARGING):
                continue
            pose = self._live_xy(other.id)
            if pose is not None and math.hypot(pose[0]-point.x, pose[1]-point.y) < 1.2:
                return False
        return True

    def _try_assign_all(self, count_reschedule: bool = True) -> None:
        """Greedy pass over the pending queue.

        Idle robots are the scarce resource, so we walk the queue ordered by
        priority (then by age) and let the most urgent task pick first. That is
        still greedy - it just makes the greedy order defensible.

        `count_reschedule` is False for the periodic safety-net pass, so the
        reschedule metric reflects genuine re-planning events (task arrival,
        robot completion) rather than 2 Hz timer ticks.
        """
        with self._lock:
            if not self.pending:
                return

            # Only log the "no robot" explanation when something changed,
            # otherwise an unservable task floods the log at tick rate.
            verbose = count_reschedule
            if count_reschedule:
                self.metrics.note_reschedule()

            order = sorted(
                self.pending,
                key=lambda tid: (
                    -self.tasks[tid].priority,
                    self.tasks[tid].created_at_sec,
                ),
            )

            assignments = []
            dispatch_points = []
            clearance = float(self.get_parameter("zone_dispatch_clearance_m").value)
            now = self._now()
            for tid in order:
                task = self.tasks[tid]
                if task.state != TaskState.PENDING:
                    continue

                free = [
                    r for r in self.robots.values()
                    if r.status == RobotStatus.IDLE
                ]
                if not free:
                    break

                robot, why = select_robot(
                    free, task, now, self.weights, self.min_battery_pct,
                    self.load_time, self.unload_time, battery_check=True,
                )
                if robot is None:
                    if why is not None and verbose:
                        self.get_logger().warn(
                            f"no robot for {task.id}: {why.explain()}"
                        )
                    continue

                # Only dispatch into a free working region. Reserving in this
                # pass closes the race before worker threads claim their pads.
                if not self._pickup_ready(task.pickup, robot.id) or any(
                        task.pickup.distance_to(point) < clearance
                        for point in dispatch_points):
                    continue
                dispatch_points.append(task.pickup)

                # Claim the robot inside the lock so the next iteration cannot
                # pick it again.
                robot.status = RobotStatus.ASSIGNED
                robot.current_task_id = task.id
                task.assigned_robot = robot.id
                task.state = TaskState.ASSIGNED
                task.assigned_at_sec = now
                assignments.append((robot, task))

            for tid in [t.id for _, t in assignments]:
                if tid in self.pending:
                    self.pending.remove(tid)

        # Kick off dispatch outside the lock: these calls block on the action
        # server and must not hold up other scheduling.
        for robot, task in assignments:
            cost = evaluate(
                robot, task, self._now(), self.weights, self.min_battery_pct,
                self.load_time, self.unload_time,
            )
            self.get_logger().info(f"ASSIGN {cost.explain()}")
            threading.Thread(
                target=self._execute_task, args=(robot, task), daemon=True
            ).start()

    # ------------------------------------------------------------ execution

    def _execute_task(self, robot: RobotState, task: Task) -> None:
        """Drive one task end to end, simulating load/unload and battery."""
        started = self._now()
        d_total = 0.0
        e_total = 0.0

        try:
            # --- leg 1: go to pickup ------------------------------------
            task.state = TaskState.TO_PICKUP
            robot.status = RobotStatus.TO_PICKUP
            # Where to actually stop: the marker if this robot has the zone to
            # itself, otherwise a free approach slot beside it.
            pickup_goal = self._claim_zone_slot(robot, task.pickup)
            ok, dist, energy = self._navigate(robot, pickup_goal, f"{task.id}:pickup")
            d_total += dist
            e_total += energy
            if not ok:
                self._fail(robot, task, "navigation to pickup failed")
                return

            # --- load ---------------------------------------------------
            task.state = TaskState.LOADING
            robot.status = RobotStatus.LOADING
            self._dwell(self.load_time)
            robot.current_payload_kg = task.payload_kg
            # Loading done, the robot is about to leave: hand the pickup marker
            # to whoever is waiting for it.
            self._release_zone(robot, task.pickup)

            # --- leg 2: go to dropoff -----------------------------------
            task.state = TaskState.TO_DROPOFF
            robot.status = RobotStatus.TO_DROPOFF
            dropoff_goal = self._claim_zone_slot(robot, task.dropoff)
            ok, dist, energy = self._navigate(robot, dropoff_goal, f"{task.id}:dropoff")
            d_total += dist
            e_total += energy
            if not ok:
                self._fail(robot, task, "navigation to dropoff failed")
                return

            # --- unload -------------------------------------------------
            self._dwell(self.unload_time)
            robot.current_payload_kg = 0.0
            self._release_zone(robot, task.dropoff)

            # --- complete -----------------------------------------------
            task.state = TaskState.COMPLETED
            task.completed_at_sec = self._now()
            robot.tasks_completed += 1

            with self._lock:
                # Fold this task's travel and energy into the robot's running
                # totals, then drain the battery accordingly. Without this the
                # fleet-level distance/energy metrics stay at zero.
                robot.odom_distance_m += d_total
                robot.energy_consumed_wh += e_total
                self._apply_battery_drain(robot, d_total)

                self.metrics.note_completed(
                    task, robot, task.completed_at_sec, d_total, e_total
                )
                # Free the robot for the next task straight away. It does NOT
                # drive home here: the trip back to the charger is a single
                # end-of-day move driven from _tick (see dock_when_idle_sec),
                # not something a robot does between two jobs.
                robot.status = RobotStatus.IDLE
                robot.current_task_id = ""
                robot.busy_until_sec = 0.0

            self.get_logger().info(
                f"COMPLETED {task.id} by {robot.id} in "
                f"{task.completed_at_sec - started:.1f}s, "
                f"{d_total:.1f}m, {e_total:.2f}Wh, "
                f"battery {robot.battery_pct:.1f}%"
            )

        except Exception as exc:  # noqa: BLE001 - never kill the worker thread
            self.get_logger().error(f"task {task.id} crashed: {exc!r}")
            self._fail(robot, task, f"exception: {exc!r}")
        finally:
            # A robot that has stopped working must not keep a zone reserved.
            self._release_zone(robot, task.pickup)
            self._release_zone(robot, task.dropoff)
            # Any path that leaves the robot idle should trigger a re-scan for
            # work, including failure and requeue paths.
            if robot.status == RobotStatus.IDLE:
                self._try_assign_all()

    @staticmethod
    def _can_serve(robot: RobotState, task: Task) -> bool:
        """The hard constraints from cost.py: capability and payload.

        Kept in step with cost.evaluate on purpose - if this said "yes" where
        the cost function said "infeasible", the fleet would wait for a task it
        can never be given (and never go home).
        """
        if task.required_capability and task.required_capability not in robot.capabilities:
            return False
        return robot.can_carry(task.payload_kg)

    def _work_remains(self) -> bool:
        """True while the fleet still has something it could be doing.

        Only tasks some robot can actually serve count. demo_10 contains a
        deliberately impossible one (200 kg against a 150 kg rating); it sits
        in the pending queue for ever, and counting it as "work" would mean the
        fleet never goes home.

        A robot that is CHARGING counts as free, not as busy: it is on its way
        to its bay, which is the end state, and if it counted as work the first
        robot to dock would stop the other three from following it.
        """
        with self._lock:
            for r in self.robots.values():
                if r.status not in (RobotStatus.IDLE, RobotStatus.CHARGING):
                    return True
            for tid in list(self.pending):
                task = self.tasks.get(tid)
                if task is None or task.state != TaskState.PENDING:
                    continue
                if any(self._can_serve(r, task) for r in self.robots.values()):
                    return True
        return False

    def _work_for(self, robot: RobotState) -> bool:
        """True if THIS robot has anything it could be doing right now.

        The docking decision used to ask the fleet-wide question, and that
        turned out to be the wrong question: with a couple of queued pallet
        jobs that only the heavy AMR can take, the light and omni robots were
        "not working" but the fleet "had work", so they never went home and
        parked wherever they happened to finish - measured on a live run as
        omni_1 sitting idle in the middle of the east connector at
        (12.14, 3.35) while light_1 and heavy_1 tried to get past it, which is
        exactly how the three of them ended up in a mutual standoff.

        Pending work whose region is occupied is not dispatchable yet: wait
        at the charging bay instead of blocking a completed dropoff.
        A robot with nothing it can serve should get out of the way; whether
        some OTHER robot still has a job is not its problem.
        """
        if robot.status not in (RobotStatus.IDLE, RobotStatus.CHARGING):
            return True
        with self._lock:
            for tid in list(self.pending):
                task = self.tasks.get(tid)
                if task is None or task.state != TaskState.PENDING:
                    continue
                if self._can_serve(robot, task) and self._pickup_ready(task.pickup, robot.id):
                    return True
        return False

    def _start_docking(self, robot: RobotState) -> None:
        """Claim an idle robot for its charger and drive it there off-thread.

        Status goes to CHARGING inside the lock before the thread is started,
        so the 2 Hz assignment pass can never hand the robot a task while it is
        on its way to the bay.
        """
        with self._lock:
            if robot.status != RobotStatus.IDLE:
                return
            self._idle_since[robot.id] = None
            robot.status = RobotStatus.CHARGING
        threading.Thread(
            target=self._drive_home, args=(robot,), daemon=True
        ).start()

    def _drive_home(self, robot: RobotState) -> None:
        """Drive the robot back to its charging bay and leave it IDLE there.

        Status is CHARGING for the whole trip, so the fleet-status panel and
        the metrics both show a robot that is docking rather than one that is
        free but mysteriously not being given work. The trip is charged to the
        robot's distance/energy totals like any other leg: it is real driving,
        and hiding it would make the utilisation figures lie. It is not
        counted as utilisation though - that measures task work, and the whole
        point of docking is that the fleet is not doing task work.

        Every exit path runs through the `finally` and ends with the robot
        IDLE. That is not tidiness: a robot left in CHARGING is a robot the
        scheduler will never hand work to again.
        """
        home = self.charger_pose.get(robot.id)
        ok = True
        drove = False
        dist = 0.0
        energy = 0.0
        try:
            if home is None or self._at_charger(robot, home):
                # Either there is no bay configured, or the robot is already
                # standing on it - re-issuing a centimetre goal is how a robot
                # spends a whole recovery cycle going nowhere.
                return
            if not self.return_to_charger or self._work_for(robot):
                # Work arrived between the tick that decided to dock and this
                # thread starting. Stay put and take it.
                return
            robot.status = RobotStatus.CHARGING
            drove = True
            self.get_logger().info(
                f"{robot.id}: returning to charger ({home.x:.2f},{home.y:.2f})"
            )
            ok, dist, energy = self._navigate(robot, home, f"{robot.id}:charge")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{robot.id}: charger leg crashed: {exc!r}")
            ok = False
        finally:
            with self._lock:
                robot.odom_distance_m += dist
                robot.energy_consumed_wh += energy
                self._apply_battery_drain(robot, dist)
                robot.status = RobotStatus.IDLE

        if not drove:
            return
        if ok:
            self.get_logger().info(
                f"{robot.id}: docked at charger, battery {robot.battery_pct:.1f}%"
            )
        else:
            # Not fatal: the robot is still IDLE and can be given work from
            # wherever it stopped. Saying so keeps the log honest.
            self.get_logger().warn(
                f"{robot.id}: did not reach its charger - staying where it is"
            )

    def _at_charger(self, robot: RobotState, home: Pose2D) -> bool:
        """True when the robot is already standing on its charging bay."""
        xy = self._live_xy(robot.id)
        if xy is None:
            return False
        return math.hypot(xy[0] - home.x, xy[1] - home.y) \
            <= self.charger_arrive_radius_m

    def _make_goal(self, target: Pose2D) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.x
        goal.pose.pose.position.y = target.y
        qx, qy, qz, qw = yaw_to_quaternion(target.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def _send_goal(self, robot: RobotState, target: Pose2D, tag: str):
        """Hand one NavigateToPose goal to Nav2.

        Returns (handle, None) on success and (None, reason) on any failure -
        "unavailable", "timeout" or "rejected". The reasons are kept apart
        because they mean different things to the caller: only "unavailable"
        (no Nav2 at all) justifies the kinematic stand-in, while a rejected or
        timed-out goal has to fail the leg honestly instead of silently
        "delivering" cargo the robot never carried.
        """
        client = self.nav_clients[robot.id]
        if not client.wait_for_server(timeout_sec=10.0):
            return None, "unavailable"

        # A fresh record per action prevents late feedback from a preempted
        # goal being mistaken for progress on its replacement.
        record = {}
        self._nav_feedback[robot.id] = record
        def on_feedback(message):
            record["remaining"] = float(message.feedback.distance_remaining)
            record["stamp"] = self._now()
        send_future = client.send_goal_async(
            self._make_goal(target), feedback_callback=on_feedback)
        deadline = time.time() + 30.0
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            self.get_logger().warn(f"{robot.id}: goal send timed out for {tag}")
            return None, "timeout"

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{robot.id}: goal rejected for {tag}")
            return None, "rejected"
        return handle, None

    def _cancel_navigation(self, handle) -> None:
        """Acknowledge cancellation before issuing a competing navigation goal."""
        try:
            future = handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        except Exception as exc:
            self.get_logger().warn(f"navigation cancellation failed: {exc}")

    def _wait_for_result(self, robot: RobotState, target: Pose2D, result_future):
        """Block until the goal finishes, or until the robot stops converging.

        Returns (goal_status, stalled, distance_to_goal). This is the deadlock
        detector. Nav2 bounds a goal with its own recovery behaviours, but every
        one of those is about the *map* - clear the costmaps, spin, back up a
        little. None of them can help when the thing in the way is a real
        teammate in a 1.2 m corridor, so the robot cheerfully retries the same
        blocked plan until the action server gives up minutes later, with the
        rest of the fleet queued behind it.

        Use simulation time and action feedback distance along the route, so
        a detour that temporarily increases Euclidean distance is not cancelled.
        Net displacement away from the final metre also counts as progress.
        Near the goal retain position / yaw progress to reject oscillations.
        A bounded leg budget prevents endless circling. Nav2 gets its own local
        recovery window before this outer watchdog intervenes.
        """
        anchor = self._live_xy(robot.id)
        best = None
        best_path = None
        last_distance = float("inf")
        if anchor is not None:
            best = self._goal_error(anchor, target)
            last_distance = math.hypot(anchor[0] - target.x, anchor[1] - target.y)
        started = last_progress = self._now()
        # Bound a leg even if a moving robot keeps wandering. This is simulation
        # time, so a slow / paused Gazebo does not consume the navigation budget.
        leg_budget = max(180.0, 60.0 + last_distance / 0.10)

        while not result_future.done():
            time.sleep(0.1)
            if not self.stall_enabled:
                continue
            xy = self._live_xy(robot.id)
            if xy is None:
                continue
            now = self._now()
            if now < started or now < last_progress:
                started = last_progress = now
                best_path = None
            distance = math.hypot(xy[0] - target.x, xy[1] - target.y)
            last_distance = distance
            error = self._goal_error(xy, target)
            feedback = getattr(self, "_nav_feedback", {}).get(robot.id, {})
            remaining = feedback.get("remaining")
            fresh = now - feedback.get("stamp", -float("inf")) < 2.0
            path_progress = False
            if fresh and remaining is not None and math.isfinite(remaining) and remaining > 0.0:
                if best_path is None:
                    best_path = remaining
                elif best_path - remaining > self.stall_move_eps_m:
                    best_path = remaining
                    path_progress = True
                elif remaining - best_path > 0.5:
                    # A replan can lengthen the route. Reset its distance baseline
                    # without granting extra time unless the robot really moves.
                    best_path = remaining
            moved = anchor is not None and math.hypot(
                xy[0] - anchor[0], xy[1] - anchor[1]) > self.stall_wander_m
            if best is None or best - error > self.stall_move_eps_m or path_progress:
                best, anchor, last_progress = error, xy, now
            elif distance > 1.0 and moved:
                # Net displacement, rather than accumulated wheel distance,
                # allows a detour while rejecting small left/right oscillations.
                anchor, last_progress = xy, now
            if now - started > leg_budget or now - last_progress > self.stall_timeout_sec:
                return None, True, distance

        result = result_future.result()
        status = result.status if result is not None else None
        return status, False, last_distance

    def _goal_error(self, xy, target: Pose2D) -> float:
        """Position error plus a weighted heading error, in metres."""
        dyaw = math.atan2(
            math.sin(xy[2] - target.yaw), math.cos(xy[2] - target.yaw)
        )
        return math.hypot(xy[0] - target.x, xy[1] - target.y) \
            + self.stall_yaw_weight * abs(dyaw)

    def _nearest_teammate_m(self, robot_id: str, x: float, y: float) -> float:
        """Distance from (x, y) to the closest other robot, or inf if alone."""
        best = float("inf")
        for other, pose in self.live_pose.items():
            if other == robot_id:
                continue
            d = math.hypot(pose[0] - x, pose[1] - y)
            if d < best:
                best = d
        return best

    def _back_off(self, robot: RobotState, attempt: int = 1) -> float:
        """Reverse a short way to break a standoff; return the metres driven.

        Candidates are behind the robot and slightly off to each side, so the
        reverse stays inside the corridor instead of angling into a wall, and
        each one is checked against the static map first - a "recovery" that
        reverses into furniture just buys a second stall.

        Among the ones that are free, the candidate that ends FARTHEST from the
        nearest teammate wins. This is the whole point on this map: the usual
        jam is three or four robots meeting in one 1 m-wide pocket of the ring
        corridor (dropoff_B and its approach slots are the classic one), and if
        every one of them reverses by the same rule into the same pocket, the
        jam is not broken - it is moved. Reversing towards open space is what
        disperses the cluster.

        `attempt` is the 1-based recovery number for this leg; the retreat
        grows with it, because a robot that has already backed up and re-planned
        into the same traffic will not be helped by backing up the same amount
        again.

        The reverse is given a bounded window. If it does not finish in time we
        return anyway: the caller re-sends the original goal, which preempts
        the unfinished reverse, so the worst case is that we re-plan from a
        different pose - which is the point.
        """
        xy = self._live_xy(robot.id)
        if xy is None:
            return 0.0
        x, y, yaw = xy
        clearance = self.slot_clearance.get(robot.id, 0.45)
        back = min(
            self.stall_backoff_m
            * (1.0 + self.stall_backoff_growth * max(0, attempt - 1)),
            self.stall_backoff_max_m,
        )

        tried = []
        options = []
        for dyaw, frac in ((0.0, 1.0), (0.45, 1.0), (-0.45, 1.0),
                           (0.9, 0.7), (-0.9, 0.7)):
            tx = x - back * frac * math.cos(yaw + dyaw)
            ty = y - back * frac * math.sin(yaw + dyaw)
            if not self._slot_is_free(tx, ty, clearance):
                tried.append(f"({tx:.2f},{ty:.2f}) blocked")
                continue
            # Prefer open space; break ties towards the straight reverse so a
            # robot with nobody near it still behaves predictably.
            room = self._nearest_teammate_m(robot.id, tx, ty)
            options.append((room - 0.05 * abs(dyaw), tx, ty))
        if not options:
            self.get_logger().warn(
                f"{robot.id}: nowhere to reverse into ({'; '.join(tried)}) - "
                f"re-planning from the current pose"
            )
            return 0.0
        options.sort(key=lambda o: -o[0])
        _, tx, ty = options[0]

        handle, err = self._send_goal(robot, Pose2D(tx, ty, yaw), f"{robot.id}:backoff")
        if err is not None:
            self.get_logger().warn(
                f"{robot.id}: could not send the reverse goal ({err})"
            )
            return 0.0
        future = handle.get_result_async()
        deadline = self._now() + self.stall_backoff_sec
        while not future.done() and self._now() < deadline:
            time.sleep(0.1)
        if not future.done():
            self._cancel_navigation(handle)
        final = self._live_xy(robot.id) or (x, y, yaw)
        moved = math.hypot(final[0] - x, final[1] - y)
        self.get_logger().info(
            f"{robot.id}: reversing {moved:.2f} m to ({tx:.2f},{ty:.2f}) "
            f"to break the standoff"
        )
        return moved

    def _navigate(self, robot: RobotState, target: Pose2D, tag: str):
        """Drive to `target`, breaking standoffs along the way.

        Returns (success, distance_travelled_m, energy_wh).

        A leg is retried up to stall_max_recoveries times: each time the robot
        stops making progress the current goal is preempted, the robot reverses
        a short distance, and the same goal is re-sent with a fresh plan. Only
        if it still cannot make headway does the leg fail, which hands the task
        back to the scheduler - the failure path that was already there.
        """
        start = Pose2D(robot.pose.x, robot.pose.y, robot.pose.yaw)
        straight = start.distance_to(target)
        # Distance the recoveries themselves add; folded into the leg's report
        # so the fleet energy/distance totals stay honest.
        extra_dist = 0.0
        recoveries = 0

        while True:
            handle, err = self._send_goal(robot, target, tag)
            if err == "unavailable":
                if recoveries == 0:
                    self.get_logger().warn(
                        f"{robot.id}: navigate_to_pose not available for {tag}"
                    )
                    # Without Nav2 we still want the *scheduling* experiment to
                    # run, so fall back to a kinematic estimate rather than
                    # failing the task.
                    return self._simulate_leg(robot, target)
                return False, 0.0, 0.0
            if err is not None:
                # A rejection means Nav2 is running but would not take the goal
                # (usually the stack is still activating, or the target is not
                # reachable). Faking the leg here would silently "deliver" cargo
                # the robot never carried, so fail honestly and let the task be
                # requeued instead.
                self.get_logger().warn(
                    f"{robot.id}: goal not accepted for {tag} ({err}) - "
                    f"task will be requeued"
                )
                return False, 0.0, 0.0

            with self._lock:
                self.active_goal[robot.id] = handle
            status, stalled, distance = self._wait_for_result(
                robot, target, handle.get_result_async()
            )
            with self._lock:
                self.active_goal.pop(robot.id, None)

            if stalled and distance <= self.stall_arrive_radius_m:
                # The robot is standing on the goal; what it has not got is a
                # navigation stack willing to say so. Reversing here would make
                # it drive away from a pad it is already parked on, so cancel
                # the goal and accept the arrival. This is not a faked
                # success: the decision is taken from the ground-truth
                # odometry, and only inside stall_arrive_radius_m, which is
                # twice Nav2's own xy_goal_tolerance.
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001 - cancellation is best effort
                    pass
                self.get_logger().info(
                    f"{robot.id}: {tag} within {distance:.2f} m of the goal "
                    f"after {self.stall_timeout_sec:.0f}s without progress - "
                    f"accepting the arrival"
                )
                robot.pose = Pose2D(target.x, target.y, target.yaw)
                driven = straight + extra_dist
                return True, driven, robot.energy_for(
                    driven, robot.current_payload_kg
                )

            if stalled:
                recoveries += 1
                xy = self._live_xy(robot.id) or (target.x, target.y, 0.0)
                self.get_logger().warn(
                    f"{robot.id}: no progress for {self.stall_timeout_sec:.0f}s "
                    f"on {tag} at ({xy[0]:.2f},{xy[1]:.2f}), {distance:.2f} m "
                    f"from the goal - recovery {recoveries}/"
                    f"{self.stall_max_recoveries}"
                )
                self._cancel_navigation(handle)
                if recoveries > self.stall_max_recoveries:
                    self.get_logger().error(
                        f"{robot.id}: still wedged after "
                        f"{self.stall_max_recoveries} recoveries - failing {tag}"
                    )
                    return False, 0.0, 0.0
                extra_dist += self._back_off(robot, recoveries)
                continue

            # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
            ok = status == 4
            if not ok:
                self.get_logger().warn(
                    f"{robot.id}: {tag} finished with status {status}"
                )
            # Only believe the robot reached the target when Nav2 said so.
            # Forcing the pose to the goal unconditionally made a failed leg
            # look like a successful one to every later decision: the cost
            # function then measured travel from a place the robot never
            # reached, and /fleet_status advertised a position (e.g. a pickup
            # point 11 m away) that the ground-truth odometry flatly
            # contradicted. The odometry-driven correction in _on_odom
            # supplies the real pose on the next idle tick either way.
            if not ok:
                return False, 0.0, 0.0
            robot.pose = Pose2D(target.x, target.y, target.yaw)
            driven = straight + extra_dist
            return True, driven, robot.energy_for(driven, robot.current_payload_kg)

    def _simulate_leg(self, robot: RobotState, target: Pose2D):
        """Kinematic stand-in for a real navigation leg.

        Used only when the Nav2 action server is unreachable, so that a
        scheduling experiment still produces meaningful numbers on a machine
        where the navigation stack is not running.
        """
        start = Pose2D(robot.pose.x, robot.pose.y, robot.pose.yaw)
        dist = start.distance_to(target)
        travel = dist / max(robot.max_speed, 0.05)
        self.get_logger().info(
            f"{robot.id}: simulating leg to ({target.x:.1f},{target.y:.1f}) "
            f"~{dist:.1f}m / {travel:.1f}s (Nav2 unavailable)"
        )
        self._dwell(travel)
        robot.pose = Pose2D(target.x, target.y, target.yaw)
        energy = robot.energy_for(dist, robot.current_payload_kg)
        return True, dist, energy

    def _fail(self, robot: RobotState, task: Task, reason: str) -> None:
        """Mark a task failed. One requeue attempt, then give up."""
        with self._lock:
            task.requeue_count += 1
            if task.requeue_count <= 1:
                task.state = TaskState.REQUEUED
                task.assigned_robot = ""
                task.assigned_at_sec = None
                # Only send it back if some other robot could plausibly do it.
                task.state = TaskState.PENDING
                self.pending.append(task.id)
                self.get_logger().warn(
                    f"task {task.id} requeued after: {reason}"
                )
            else:
                task.state = TaskState.FAILED
                self.metrics.note_failed(task, reason)
                self.get_logger().error(
                    f"task {task.id} FAILED permanently: {reason}"
                )
            robot.current_payload_kg = 0.0
            # Same as the success path: IDLE immediately, so a requeued task
            # can go straight back to this robot if it is still the best
            # choice. The trip home is a separate, end-of-work decision.
            robot.status = RobotStatus.IDLE
            robot.current_task_id = ""
            robot.busy_until_sec = 0.0

    def _apply_battery_drain(self, robot: RobotState, distance_m: float) -> None:
        """Recompute battery_pct from the energy the robot has spent so far.

        Derived from cumulative Wh out of the pack rather than decremented per
        task, so it cannot drift out of sync with the energy metric. A
        per-metre floor is also applied, which lets the simulated drain be
        exaggerated for experiments without touching the energy model.
        """
        pct_from_energy = 0.0
        if robot.battery_capacity_wh > 0:
            pct_from_energy = (
                100.0 * robot.energy_consumed_wh / robot.battery_capacity_wh
            )

        drain_per_m = float(
            self.get_parameter("battery_drain_per_meter_pct").value
        )
        pct_from_distance = 100.0 - drain_per_m * robot.odom_distance_m

        # Take whichever estimate says the pack is emptier, but never below 0.
        robot.battery_pct = max(0.0, 100.0 - max(pct_from_energy, 100.0 - pct_from_distance))

    def _dwell(self, seconds: float) -> None:
        """Sleep for a simulated dwell period (loading / unloading).

        Sleeping in small steps rather than one long call keeps the node
        responsive to Ctrl-C during a long load.
        """
        if seconds <= 0:
            return
        remaining = seconds
        step = 0.2
        while remaining > 0 and rclpy.ok():
            time.sleep(min(step, remaining))
            remaining -= step

    # ---------------------------------------------------------------- housekeeping

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self) -> None:
        """Low-frequency safety net.

        The event-driven path handles the normal cases; this catches anything
        that slipped through (for example a robot whose action result was lost)
        and keeps the published status fresh. Re-running the assignment pass
        here also covers the case where a robot freed up without its worker
        thread triggering a rescan.
        """
        # Safety-net pass only: it must not inflate the reschedule metric, and
        # it must not re-log unservable tasks on every tick.
        self._try_assign_all(count_reschedule=False)
        self._update_idle_timers()
        self._publish_status()
        self._publish_markers()

    def _update_idle_timers(self) -> None:
        """Track idle time per robot and send it home when it runs out of work.

        This is the only place that starts a trip to the charger. A robot goes
        home when all of these hold:

          * THIS robot has nothing it could serve - see _work_for, and the note
            there about why the fleet-wide version of that question was the
            thing parking idle robots in the middle of corridors;
          * it has been idle long enough: `dock_when_idle_sec` in general, but
            only `clear_task_point_sec` if it is sitting on a pickup or dropoff
            point. Finishing a job leaves the robot standing on the pad it just
            used, and that is the one place it must not loiter - the next task
            for that pad would have to queue behind it, which is exactly what
            the zone approach slots exist to prevent;
          * it is not already standing on its bay.

        Once the task stream is finished this is what parks the fleet: nothing
        is left to serve, so every robot walks home a few seconds later instead
        of standing around on the work pads.

        Robots are deliberately left IDLE rather than given some new "parked"
        state once they arrive: IDLE-on-the-bay is exactly what the start of a
        run looks like, and any task that arrives can be given to them without
        waiting for a docking trip to unwind first.
        """
        now = self._now()
        for name, r in self.robots.items():
            if r.status == RobotStatus.IDLE:
                if self._idle_since.get(name) is None:
                    self._idle_since[name] = now
            else:
                self._idle_since[name] = None

        for name, r in list(self.robots.items()):
            started = self._idle_since.get(name)
            if r.status != RobotStatus.IDLE or started is None:
                continue
            threshold = self.dock_when_idle_sec
            if self._on_task_point(r, self.clear_task_point_radius_m):
                threshold = min(threshold, self.clear_task_point_sec)
            if now - started < threshold:
                continue
            if not self.return_to_charger or self._work_for(r):
                continue
            home = self.charger_pose.get(name)
            if home is None or self._at_charger(r, home):
                continue
            self._start_docking(r)

    def _on_task_point(self, robot: RobotState, radius: float) -> bool:
        """True when the robot is standing on a pickup or dropoff pad.

        Any point it has ever been given counts, so a robot that has just
        delivered is caught by the spot it is standing on - which is the case
        that matters, and the only one the scheduler can know about.
        """
        xy = self._live_xy(robot.id)
        if xy is None:
            return False
        for task in self.tasks.values():
            for point in (task.pickup, task.dropoff):
                if math.hypot(xy[0] - point.x, xy[1] - point.y) <= radius:
                    return True
        return False

    def _publish_status(self) -> None:
        msg = FleetStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for r in self.robots.values():
            m = RobotStateMsg()
            m.id = r.id
            m.type = r.type
            m.pose.x = r.pose.x
            m.pose.y = r.pose.y
            m.pose.theta = r.pose.yaw
            m.status = r.status.value
            m.battery_pct = float(r.battery_pct)
            m.battery_capacity_wh = float(r.battery_capacity_wh)
            m.payload_capacity_kg = float(r.payload_capacity_kg)
            m.current_payload_kg = float(r.current_payload_kg)
            m.max_speed = float(r.max_speed)
            m.capabilities = list(r.capabilities)
            m.estimated_idle_time_sec = float(
                max(0.0, r.busy_until_sec - self._now())
            )
            m.current_task_id = r.current_task_id
            m.odom_distance_m = float(r.odom_distance_m)
            m.energy_consumed_wh = float(r.energy_consumed_wh)
            m.tasks_completed = int(r.tasks_completed)
            msg.robots.append(m)

        for tid in self.pending:
            t = self.tasks[tid]
            tm = TaskMsg()
            tm.id = t.id
            tm.type = t.type
            tm.pickup.x, tm.pickup.y, tm.pickup.theta = t.pickup.x, t.pickup.y, t.pickup.yaw
            tm.dropoff.x, tm.dropoff.y, tm.dropoff.theta = t.dropoff.x, t.dropoff.y, t.dropoff.yaw
            tm.payload_kg = float(t.payload_kg)
            tm.required_capability = t.required_capability
            tm.priority = int(t.priority)
            tm.deadline_sec = float(t.deadline_sec)
            tm.created_at_sec = float(t.created_at_sec)
            tm.assigned_robot = t.assigned_robot
            tm.state = t.state.value
            msg.queue.append(tm)

        msg.queue_length = len(self.pending)
        msg.tasks_pending = len(self.pending)
        msg.tasks_active = sum(
            1 for r in self.robots.values()
            if r.status not in (RobotStatus.IDLE, RobotStatus.FAILED)
        )
        msg.tasks_completed = self.metrics.m.tasks_completed
        msg.tasks_failed = self.metrics.m.tasks_failed
        msg.makespan_sec = float(self.metrics.m.makespan())
        msg.reschedule_count = int(self.metrics.m.reschedule_count)
        msg.total_distance_m = float(sum(r.odom_distance_m for r in self.robots.values()))
        msg.total_energy_wh = float(sum(r.energy_consumed_wh for r in self.robots.values()))

        self.status_pub.publish(msg)

    def _publish_markers(self) -> None:
        """Draw the zones, the task endpoints and each robot's label.

        This is the "simple panel" the design asks for, done as RViz markers so
        it needs no custom plugin.
        """
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # --- task markers -------------------------------------------------
        for i, (tid, t) in enumerate(sorted(self.tasks.items())):
            colour = {
                TaskState.PENDING: (1.0, 0.8, 0.0),
                TaskState.ASSIGNED: (1.0, 0.5, 0.0),
                TaskState.TO_PICKUP: (0.2, 0.6, 1.0),
                TaskState.LOADING: (0.6, 0.2, 1.0),
                TaskState.TO_DROPOFF: (0.1, 0.9, 0.9),
                TaskState.COMPLETED: (0.1, 0.9, 0.1),
                TaskState.FAILED: (1.0, 0.1, 0.1),
                TaskState.REQUEUED: (1.0, 0.4, 0.4),
            }.get(t.state, (1.0, 1.0, 1.0))

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "tasks"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = *colour, 0.9
            p0 = PoseStamped().pose.position
            p0.x, p0.y, p0.z = t.pickup.x, t.pickup.y, 0.15
            p1 = PoseStamped().pose.position
            p1.x, p1.y, p1.z = t.dropoff.x, t.dropoff.y, 0.15
            m.points = [p0, p1]
            arr.markers.append(m)

            # Text label showing task id and state.
            tm = Marker()
            tm.header.frame_id = "map"
            tm.header.stamp = now
            tm.ns = "task_labels"
            tm.id = i
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = (t.pickup.x + t.dropoff.x) / 2.0
            tm.pose.position.y = (t.pickup.y + t.dropoff.y) / 2.0
            tm.pose.position.z = 0.8
            tm.scale.z = 0.35
            tm.color.r, tm.color.g, tm.color.b, tm.color.a = 1.0, 1.0, 1.0, 1.0
            tm.text = f"{t.id} [{t.state.value}]"
            if t.assigned_robot:
                tm.text += f" @{t.assigned_robot}"
            arr.markers.append(tm)

        # --- per-robot status labels, one per corner of the map -------------
        # The status text is anchored in FIXED map positions instead of being
        # pinned to each robot: labels that follow the robots pile up into an
        # unreadable heap the moment two of them work near each other, and they
        # hide the map underneath. Each robot gets its own corner, so the four
        # blocks are tens of metres apart and can never overlap no matter how
        # the fleet moves (or how the RViz camera is orbited - TEXT_VIEW_FACING
        # always faces the camera).
        #
        # status_panel_*_min/max define the rectangle whose corners are used;
        # set it once to match your typical RViz view. A fleet larger than four
        # robots wraps around: robot 5 is placed under robot 1, and so on.
        x_min = float(self.get_parameter("status_panel_x_min").value)
        x_max = float(self.get_parameter("status_panel_x_max").value)
        y_min = float(self.get_parameter("status_panel_y_min").value)
        y_max = float(self.get_parameter("status_panel_y_max").value)
        panel_z = float(self.get_parameter("status_panel_z").value)
        text_h = float(self.get_parameter("status_panel_text_height").value)
        corners = [(x_min, y_max), (x_max, y_max), (x_min, y_min), (x_max, y_min)]
        centre = (0.5 * (x_min + x_max), 0.5 * (y_min + y_max))
        base_inset = float(self.get_parameter("status_panel_inset_m").value)
        overrides = {}
        for item in self.get_parameter("status_panel_pos_overrides").value:
            if not item:
                continue
            name, _, value = str(item).partition(":")
            try:
                ox, oy = (float(v) for v in value.split(","))
                overrides[name.strip()] = (ox, oy)
            except ValueError:
                self.get_logger().warn(
                    f"status_panel_pos_overrides: cannot read {item!r} "
                    f"(expected 'name:x,y')")

        def panel_position(corner, inset):
            """Corner pulled `inset` metres along the diagonal to the centre."""
            dx, dy = centre[0] - corner[0], centre[1] - corner[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-6 or inset <= 0.0:
                return corner
            f = min(1.0, inset / dist)
            return corner[0] + dx * f, corner[1] + dy * f

        def text_marker(ns: str, mid: int, x: float, y: float, z: float,
                        height: float, colour, text: str) -> Marker:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = ns
            m.id = mid
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.scale.z = height
            m.color.r, m.color.g, m.color.b, m.color.a = *colour, 1.0
            m.text = text
            return m

        # Colours match the robot materials in Gazebo, so a corner can be
        # matched to the vehicle on the map at a glance.
        type_colour = {
            "light_diff_amr": (0.98, 0.72, 0.10),
            "heavy_diff_amr": (0.35, 0.60, 0.98),
            "omni_mecanum_amr": (0.85, 0.20, 0.22),
        }
        for j, r in enumerate(self.robots.values()):
            if r.id in overrides:
                cx, cy = overrides[r.id]
            else:
                cx, cy = panel_position(corners[j % len(corners)], base_inset)
            cy -= (j // len(corners)) * text_h * 3.4   # wrap for bigger fleets
            arr.markers.append(text_marker(
                "fleet_status", 10 + j, cx, cy, panel_z,
                text_h, type_colour.get(r.type, (1.0, 1.0, 0.4)),
                f"{r.id}\n"
                f"{r.status.value}   bat {r.battery_pct:3.0f}%\n"
                f"load {r.current_payload_kg:.0f}/{r.payload_capacity_kg:.0f}kg"
                f"   task {r.current_task_id or '-'}",
            ))

        self.marker_pub.publish(arr)

    def _report(self) -> None:
        line = self.metrics.log_line(list(self.robots.values()))
        self.get_logger().info(f"[metrics] {line}")

        # Persist on every report so a crash still leaves usable data.
        outdir = self.metrics_dir
        if outdir:
            try:
                self.metrics.write_json(
                    os.path.join(outdir, "metrics.json"),
                    list(self.robots.values()),
                )
                self.metrics.write_csv(os.path.join(outdir, "tasks.csv"))
            except OSError as exc:
                self.get_logger().warn(f"could not write metrics: {exc}")

    def destroy_node(self):
        # Final report on the way out.
        try:
            self._report()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

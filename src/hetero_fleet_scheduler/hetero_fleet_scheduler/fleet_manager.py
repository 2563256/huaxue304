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
import copy
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, Spin, NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from hetero_fleet_msgs.msg import FleetStatus, RobotState as RobotStateMsg, Task as TaskMsg

from .cost import CostWeights, evaluate, select_robot
from .geometry import rectangle_is_free, rectangles_overlap
from .heading_recovery import heading_sweep_clear
from .reservations import intersects
from .metrics import MetricsRecorder
from .traffic import Encounter, TrafficCoordinator
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
        self.use_sim_clock = bool(self.get_parameter("use_sim_time").value)

        # --- parameters ---------------------------------------------------
        self.declare_parameter("profiles_file", "")
        self.declare_parameter("tasks_file", "")
        self.declare_parameter("autostart_tasks", False)
        self.simulate_when_nav_unavailable = bool(
            self.declare_parameter("simulate_when_nav_unavailable", False).value)
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
        self.declare_parameter("channel_wait_timeout_sec", 300.0)
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

        self.scheduling_method = str(self.declare_parameter('scheduling_method', 'cp_sat_rhcr').value)
        if self.scheduling_method not in ('cp_sat_rhcr', 'greedy'):
            raise ValueError('scheduling_method must be cp_sat_rhcr or greedy')
        for key, default in [('rolling_period_sec', 5.), ('rolling_commit_sec', 2.),
                             ('rolling_horizon_sec', 1800.), ('rolling_max_tasks', 20),
                             ('cp_sat_budget_sec', .5), ('schedule_drift_sec', 15.),
                             ('travel_time_factor', 1.5), ('schedule_channels_file', '')]:
            self.declare_parameter(key, default)
        self.cb_group = ReentrantCallbackGroup()

        # --- profiles -----------------------------------------------------
        profiles = self._load_profiles()
        self.types = profiles["robot_types"]
        self.fleet_cfg = profiles["fleet"]
        sched = profiles.get("scheduler", {})
        self.weights = CostWeights.from_dict(sched.get("weights", {}))
        self.min_battery_pct = float(sched.get("min_battery_pct", 15.0))
        self.load_time = float(sched.get("load_time_sec", 3.0))
        self.unload_time = float(sched.get("unload_time_sec", 3.0))
        self.live_pose_stamps = {}
        # Optional forecourt admission order. Keep one wheel platform admitted
        # until its queued work is finished AND it has physically parked.
        # G1 does not participate in this admission or corridor ownership.
        self.wheel_dispatch_order = list(sched.get('wheel_dispatch_order', []))
        self.wheel_batch_collect_sec = float(sched.get('wheel_batch_collect_sec', 0.))
        self._wheel_batch_robot = None
        self._wheel_batch_started = None
        self.wheel_yield_to_g1 = bool(sched.get('wheel_yield_to_g1', False))
        if self.scheduling_method == 'cp_sat_rhcr':
            # CP-SAT replaces the old fixed batch/type-priority policy.
            self.wheel_dispatch_order = []
            self.wheel_batch_collect_sec = 0.
            self.wheel_yield_to_g1 = False
        self._dispatch_path_cache = {}
        self._dispatch_probe_busy = set()
        self.startup_clearance_robots = {r['name'] for r in self.fleet_cfg
                                        if r.get('startup_clearance', False)}

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
        self.charger_route: dict[str, list[Pose2D]] = {}
        self._docking_owner = None
        self._docking_retry_at = {}
        self.idle_clearance_pose: dict[str, Pose2D] = {}
        self.clearance_arrive_radius: dict[str, float] = {}
        self.clearance_release_after_types: dict[str, set[str]] = {}
        self.charger_exit: dict[str, Pose2D] = {}
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
            if entry.get("charger_exit"):
                self.charger_exit[entry["name"]] = Pose2D(*map(float, entry["charger_exit"]))
            home = entry.get("charger") or entry["spawn"]
            hx, hy = float(home[0]), float(home[1])
            hyaw = float(home[2]) if len(home) > 2 else 0.0
            self.charger_pose[entry["name"]] = Pose2D(hx, hy, hyaw)
            self.charger_route[entry["name"]] = [
                Pose2D(float(p[0]), float(p[1]), float(p[2]))
                for p in entry.get("charger_route", [])
            ]
            if entry.get('idle_clearance'):
                self.idle_clearance_pose[entry['name']] = Pose2D(*map(float, entry['idle_clearance']))
                self.clearance_arrive_radius[entry['name']] = float(entry.get('clearance_arrive_radius_m', .20))
                self.clearance_release_after_types[entry['name']] = set(entry.get('clearance_release_after_types', []))
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

        self.locomotion_health = {entry["name"]: (False, 0.0)
                                  for entry in self.fleet_cfg
                                  if self.types[entry["type"]]["kinematics"] == "legged"}
        for name in self.locomotion_health:
            self.create_subscription(
                Bool, f"/{name}/locomotion_ready",
                lambda msg, n=name: self.locomotion_health.__setitem__(n, (msg.data, self._now())),
                10, callback_group=self.cb_group)

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
        self._nav_targets = {}
        self.backup_clients: dict[str, ActionClient] = {}
        self.spin_clients: dict[str, ActionClient] = {}
        self.planner_clients: dict[str, ActionClient] = {}
        self._recovery_lock = threading.Lock()
        self.traffic = TrafficCoordinator()
        self.channel_wait_timeout = float(self.get_parameter("channel_wait_timeout_sec").value)
        self._channel_wait_states = {}
        self._channel_snapshot = ({}, -math.inf)
        self._channel_counts = {}
        self._channel_yield_waits = {}
        self._channel_lock = threading.Lock()
        self._channel_intent_publishers = {}
        self._channel_transport_publishers = {}
        self._task_hold_publishers = {}
        intent_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/channel_reservations', self._on_channel_snapshot,
                                 intent_qos, callback_group=self.cb_group)
        for name in self.robots:
            self.planner_clients[name] = ActionClient(
                self, ComputePathToPose, f"/{name}/compute_path_to_pose", callback_group=self.cb_group)
            self._channel_transport_publishers[name] = self.create_publisher(
                Bool, f"/{name}/reservation_transport_active", intent_qos)
            self._channel_intent_publishers[name] = self.create_publisher(
                Bool, f"/{name}/reservation_active", intent_qos)
            self._task_hold_publishers[name] = self.create_publisher(
                Bool, f"/{name}/task_hold", intent_qos)
            self._task_hold_publishers[name].publish(Bool(data=False))
            self.create_subscription(Bool, f"/{name}/channel_wait",
                                     lambda msg, n=name: self._on_channel_wait(n, msg), 1,
                                     callback_group=self.cb_group)
            self.spin_clients[name] = ActionClient(
                self, Spin, f"/{name}/spin", callback_group=self.cb_group)
            self.backup_clients[name] = ActionClient(
                self, BackUp, f"/{name}/backup", callback_group=self.cb_group)
            self.nav_clients[name] = ActionClient(
                self, NavigateToPose, f"/{name}/navigate_to_pose",
                callback_group=self.cb_group,
            )

        # Guards the assignment path: several callbacks can try to schedule at
        # once (task arrival + robot idle), and double-assignment is the classic
        # bug in this kind of node.
        self._lock = threading.RLock()
        if self.scheduling_method == 'cp_sat_rhcr':
            self._init_joint_schedule(profiles)

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

        # Keep queued intent alive during a cancelled Nav2 goal and while ROS
        # time pauses. A crashed manager must not leave a phantom request.
        self._channel_wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self._publish_channel_intents, clock=self._channel_wall_clock,
                          callback_group=self.cb_group)
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
                "free_thresh": float(meta.get("free_thresh", 0.196)),
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
                occupancy = getattr(self, '_occupancy', None)
                profile = getattr(self, 'types', {}).get(robot.type)
                if occupancy is not None and profile is not None:
                    # Slot feasibility is heading-dependent, like State Lattice.
                    # An inscribed task marker may fit the complete rectangle
                    # while its all-orientations circumcircle intersects a wall.
                    if not rectangle_is_free(occupancy, (x, y, point.yaw),
                            profile['footprint'], profile.get('footprint_padding', 0.)):
                        continue
                elif not self._slot_is_free(x, y, clearance):
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
        if not hasattr(self, 'live_pose_stamps'):
            self.live_pose_stamps = {}
        self.live_pose_stamps[name] = time.monotonic()
        traffic = getattr(self, "traffic", None)
        if traffic is not None:
            traffic.observe(name, self.live_pose[name], self._now())
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
        robot = self.robots.get(ignore_robot)
        if robot is not None and self._exit_target_occupied(robot, point):
            return False
        radius = float(self.get_parameter("zone_dispatch_clearance_m").value)
        for (x, y), claims in getattr(self, "zone_claims", {}).items():
            if any(owner != ignore_robot for owner in claims.values()) and \
                    math.hypot(point.x-x, point.y-y) < radius:
                return False
        for other in self.robots.values():
            if other.id == ignore_robot:
                continue
            # Reserve a pickup as soon as dispatch changes the robot state.
            # Its worker may still be leaving the charger before claiming the
            # physical pad; waiting for that claim permits duplicate dispatch.
            if other.status in (RobotStatus.ASSIGNED, RobotStatus.TO_PICKUP):
                task = getattr(self, "tasks", {}).get(other.current_task_id)
                if task is not None and point.distance_to(task.pickup) < radius:
                    return False
            if other.status not in (RobotStatus.ASSIGNED, RobotStatus.TO_PICKUP,
                    RobotStatus.LOADING, RobotStatus.TO_DROPOFF, RobotStatus.CHARGING):
                continue
            pose = self._live_xy(other.id)
            if pose is not None and math.hypot(pose[0]-point.x, pose[1]-point.y) < 1.2:
                return False
        return True

    def _dispatch_scene(self):
        now = time.monotonic()
        stamps = getattr(self, 'live_pose_stamps', {})
        if any(not 0 <= now-stamps.get(n, -math.inf) <= .5 for n in self.robots):
            return None
        return dict(self.live_pose)

    @staticmethod
    def _same_dispatch_scene(a, b):
        return (a is not None and b is not None and a.keys() == b.keys()
                and all(math.dist(a[n][:2], b[n][:2]) < .08
                        and abs(math.atan2(math.sin(a[n][2]-b[n][2]),
                                           math.cos(a[n][2]-b[n][2]))) < .15 for n in a))

    def _task_preflight_valid(self, cached, scene):
        if cached is None:
            return False
        stamp, old_scene, ok = cached
        # Keep a stationary blocked pair excluded long enough for CP-SAT to
        # admit another robot. Moving peers invalidate it immediately.
        return (time.monotonic()-stamp < (8. if ok else 60.)
                and self._same_dispatch_scene(scene, old_scene))

    def _dispatch_path_ready(self, robot, task):
        if not self._pickup_ready(task.pickup, robot.id):
            return False
        scene = self._dispatch_scene()
        if scene is None:
            return None
        key = (robot.id, task.id)
        cached = self._dispatch_path_cache.get(key)
        if (cached is not None and time.monotonic()-cached[0] < 8.
                and self._same_dispatch_scene(scene, cached[1])):
            return cached[2]
        if key not in self._dispatch_probe_busy:
            self._dispatch_probe_busy.add(key)
            def probe():
                ok = False
                try:
                    ok = self._probe_target_path(robot, task.pickup, 'TaskGridBased')
                except Exception as exc:
                    self.get_logger().warn(f'{robot.id}: dispatch path probe failed: {exc!r}')
                finally:
                    self._dispatch_path_cache[key] = (time.monotonic(), scene, ok)
                    self._dispatch_probe_busy.discard(key)
            threading.Thread(target=probe, daemon=True).start()
        return None

    def _task_route_ready(self, robot, task):
        scene = self._dispatch_scene()
        if scene is None:
            return None
        pair = robot.id, task.id
        cached = self._task_preflight_cache.get(pair)
        if self._task_preflight_valid(cached, scene):
            return cached[2]
        # At most one real-planner request per idle robot at a time.
        if robot.id in self._task_preflight_busy:
            return None
        self._task_preflight_busy.add(robot.id)
        def probe():
            ok = False
            try:
                start = None
                home = self.charger_pose.get(robot.id)
                exit_pose = self.charger_exit.get(robot.id)
                if home is not None and exit_pose is not None and self._at_charger(robot,home):
                    poses = self._request_target_path(robot,exit_pose,'GridBased')
                    if not poses:
                        return
                    start = self._end_pose(poses)
                poses = self._request_target_path(robot,task.pickup,'TaskGridBased',start)
                if not poses:
                    return
                ok = bool(self._request_target_path(robot,task.dropoff,'TaskGridBased',self._end_pose(poses)))
            except Exception as exc:
                self.get_logger().warn(f'{robot.id}: task preflight failed: {exc!r}')
            finally:
                with self._lock:
                    # Do not reuse a proof if a peer moved while it was computed.
                    if robot.status == RobotStatus.IDLE and self._same_dispatch_scene(scene,self._dispatch_scene()):
                        self._task_preflight_cache[pair] = time.monotonic(),scene,ok
                        if not ok:
                            self._rolling.invalidate('physical_route_blocked')
                            self.get_logger().info(f'{task.id}: State Lattice preflight blocked; remaining PENDING')
                    self._task_preflight_busy.discard(robot.id)
        threading.Thread(target=probe,daemon=True).start()
        return None

    @staticmethod
    def _end_pose(poses):
        p = poses[-1].pose
        return Pose2D(p.position.x,p.position.y,quaternion_to_yaw(p.orientation))

    def _g1_dispatch_priority(self):
        """Wheel admission yields to G1; only physical paths admit G1 jobs."""
        if not getattr(self, 'wheel_yield_to_g1', False):
            return None, False
        g1s = [r for r in self.robots.values() if r.type == 'g1_humanoid']
        if any(r.current_task_id or r.status == RobotStatus.CHARGING for r in g1s):
            return set(), True
        # Finish an already started wheel transport / parking move. G1's next
        # job stays pending rather than starting a route known to be cut off.
        if any(r.type != 'g1_humanoid' and (r.current_task_id
               or r.status == RobotStatus.CHARGING) for r in self.robots.values()):
            return set(), False
        allowed, pending_probe = set(), False
        for robot in g1s:
            if robot.status != RobotStatus.IDLE:
                continue
            for tid in self.pending:
                task = self.tasks.get(tid)
                if task is None or not self._can_serve(robot, task):
                    continue
                ready = self._dispatch_path_ready(robot, task)
                if ready:
                    allowed.add((robot.id, tid))
                elif ready is None:
                    pending_probe = True
            # Before giving wheels another turn, physically clear G1 from its
            # completed work pad. Its normal docking worker performs the move.
            park = self.idle_clearance_pose.get(robot.id, self.charger_pose.get(robot.id))
            if robot.tasks_completed and park is not None and not self._at_charger(robot, park):
                pending_probe = True
        return allowed, bool(allowed) or pending_probe

    def _wheel_admission_owner(self, now):
        order = getattr(self, 'wheel_dispatch_order', [])
        if not order:
            return None
        if self._wheel_batch_started is None:
            self._wheel_batch_started = now
        if now - self._wheel_batch_started < self.wheel_batch_collect_sec:
            return ''
        def has_work(robot):
            return bool(robot.current_task_id) or any(
                tid in self.tasks and self.tasks[tid].state == TaskState.PENDING
                and self._can_serve(robot, self.tasks[tid]) for tid in self.pending)
        owner = self.robots.get(self._wheel_batch_robot)
        if owner is not None:
            park = self.idle_clearance_pose.get(owner.id, self.charger_pose.get(owner.id))
            if (has_work(owner) or owner.status != RobotStatus.IDLE or park is None
                    or not self._at_charger(owner, park)):
                return owner.id
        self._wheel_batch_robot = next((name for name in order
            if name in self.robots and has_work(self.robots[name])), None)
        return self._wheel_batch_robot

    def _init_joint_schedule(self, profiles):
        from pathlib import Path
        from .joint_schedule import JointScheduler
        from .route_estimate import RouteEstimator
        from .rolling_schedule import RollingSchedule
        pkg = Path(get_package_share_directory('hetero_fleet_scheduler'))
        map_path = Path(get_package_share_directory('hetero_fleet_bringup'))/'maps/geomap.yaml'
        channel_path = str(self.get_parameter('schedule_channels_file').value) or str(pkg/'config/channels.yaml')
        config = yaml.safe_load(Path(channel_path).read_text())
        self._joint_estimator = RouteEstimator(map_path, profiles, config,
            float(self.get_parameter('travel_time_factor').value))
        solver = JointScheduler(self._joint_estimator,
            budget=float(self.get_parameter('cp_sat_budget_sec').value),
            min_battery=self.min_battery_pct, service=self.load_time+self.unload_time,
            horizon=float(self.get_parameter('rolling_horizon_sec').value),
            max_tasks=int(self.get_parameter('rolling_max_tasks').value))
        self._rolling = RollingSchedule(solver,
            period=float(self.get_parameter('rolling_period_sec').value),
            commit=float(self.get_parameter('rolling_commit_sec').value),
            drift=float(self.get_parameter('schedule_drift_sec').value))
        self._task_preflight_cache = {}
        self._task_preflight_busy = set()
        self._joint_worker_busy = False
        self._joint_failure_at = -math.inf
        self._schedule_pub = self.create_publisher(String, '/fleet_schedule', 1)

    def _try_assign_joint(self, count_reschedule=True):
        """Snapshot under the lock; solve outside callbacks; commit live heads only."""
        assignments = []
        with self._lock:
            now = self._now()
            if not self.pending or (getattr(self, 'use_sim_clock', False) and now <= 0):
                return
            for tid in self.pending:
                if self.tasks[tid].created_at_sec == 0.:
                    self.tasks[tid].created_at_sec = now
            if self.metrics.m.first_task_at_sec == 0.:
                self.metrics.m.first_task_at_sec = now
            tasks = [self.tasks[tid] for tid in self.pending if self.tasks[tid].state == TaskState.PENDING]
            scene = self._dispatch_scene()
            blocked = tuple(sorted(pair for pair, (stamp, old_scene, ok) in self._task_preflight_cache.items()
                if not ok and self._task_preflight_valid((stamp,old_scene,ok),scene)))
            key = (tuple(sorted(t.id for t in tasks)), tuple(sorted(
                (r.id, r.status.value, r.current_task_id) for r in self.robots.values())), blocked)
            if self._rolling.due(now, key) and not self._joint_worker_busy and time.monotonic()-self._joint_failure_at >= 1.:
                self._joint_worker_busy = True
                task_copy, robot_copy = copy.deepcopy(tasks), copy.deepcopy(list(self.robots.values()))
                for robot in robot_copy:
                    pose = self.live_pose.get(robot.id)
                    if pose is not None:
                        robot.pose = Pose2D(*pose)
                active = copy.deepcopy(self.tasks)
                channel, stamp = self._channel_snapshot
                channel = copy.deepcopy(channel) if 0 <= time.monotonic()-stamp <= 1.5 else {}
                def plan():
                    try:
                        ready = {}
                        for robot in robot_copy:
                            task = active.get(robot.current_task_id)
                            if task is not None:
                                # Running tasks remain committed. Forecast only
                                # the earliest availability after the current job.
                                if task.state in (TaskState.TO_PICKUP, TaskState.ASSIGNED):
                                    legs = [self._joint_estimator(robot,robot.pose,task.pickup),
                                            self._joint_estimator(robot,task.pickup,task.dropoff)]
                                else:
                                    legs = [self._joint_estimator(robot,robot.pose,task.dropoff)]
                                ready[robot.id] = sum(v.seconds for v in legs if v is not None)+self.load_time+self.unload_time
                                if any(v is None for v in legs):
                                    ready[robot.id] = self._rolling.solver.horizon
                                spent = sum(robot.energy_for(v.distance,task.payload_kg) for v in legs if v is not None)
                                if robot.battery_capacity_wh > 0:
                                    robot.battery_pct -= 100*spent/robot.battery_capacity_wh
                                robot.pose = task.dropoff
                            elif robot.status != RobotStatus.IDLE:
                                ready[robot.id] = self._rolling.solver.horizon
                        feedback = {}
                        for name, windows in channel.get('scheduler',{}).get('windows',{}).items():
                            if not windows:
                                continue
                            first = min(windows, key=lambda w:w['start'])
                            feedback[name,first['group']] = max(now,first['start'])
                        # Physical occupants (including G1) forecast a hold for
                        # peers; actual release remains the arbiter's decision.
                        for group,state in channel.get('groups',{}).items():
                            owners = set(state.get('occupants',[]))
                            if state.get('owner'):
                                owners.add(state['owner'])
                            for owner in owners:
                                delay = ready.get(owner, self._rolling.period)
                                for robot in robot_copy:
                                    if robot.id != owner:
                                        feedback[robot.id,group] = max(feedback.get((robot.id,group),now),now+delay)
                        self._rolling.update(now,key,task_copy,robot_copy,ready,feedback,blocked)
                        if count_reschedule:
                            self.metrics.note_reschedule()
                        self._schedule_pub.publish(String(data=json.dumps(self._rolling.hints())))
                        if self._rolling.plan.status not in ('OPTIMAL','FEASIBLE','EMPTY'):
                            self.get_logger().warn('CP-SAT holds dispatch: '+self._rolling.plan.status)
                    except Exception as exc:
                        self._rolling.plan = None
                        self._joint_failure_at = time.monotonic()
                        self.get_logger().error(f'Joint scheduling failed; holding dispatch: {exc!r}')
                    finally:
                        self._joint_worker_busy = False
                threading.Thread(target=plan,daemon=True).start()
            # A changed task/robot set invalidates the old uncommitted plan.
            if self._rolling.key != key:
                return
            snapshot, stamp = self._channel_snapshot
            if not 0 <= time.monotonic()-stamp <= 1.5 or not snapshot.get('pose_ready',False):
                return
            points=[]
            for row in self._rolling.heads(now):
                robot=self.robots.get(row['robot_id']); task=self.tasks.get(row['task_id'])
                if (robot is None or task is None or task.state != TaskState.PENDING
                        or robot.status != RobotStatus.IDLE or task.id not in self.pending
                        or not self._can_serve(robot,task)
                        or not self._locomotion_available(robot.id,now)
                        or not self._navigation_available(robot.id)
                        or self._should_clear_channel(robot.id)
                        or not self._pickup_ready(task.pickup,robot.id)):
                    continue
                clearance=float(self.get_parameter('zone_dispatch_clearance_m').value)
                if any(task.pickup.distance_to(p)<clearance for p in points):
                    continue
                if (self._wheel_clearance_required(robot) and
                        not self._at_clearance(robot,self.idle_clearance_pose[robot.id])):
                    continue
                if self._task_route_ready(robot,task) is not True:
                    continue
                points.append(task.pickup)
                robot.status=RobotStatus.ASSIGNED; robot.current_task_id=task.id
                task.assigned_robot=robot.id; task.state=TaskState.ASSIGNED
                task.assigned_at_sec=now; self.pending.remove(task.id)
                assignments.append((robot,task))
            if assignments:
                self._rolling.invalidate('dispatch_commit')
        for robot,task in assignments:
            self.get_logger().info(f'CP-SAT ASSIGN {task.id} -> {robot.id}')
            threading.Thread(target=self._execute_task,args=(robot,task),daemon=True).start()

    def _try_assign_all(self, count_reschedule: bool = True) -> None:
        """Greedy pass over the pending queue.

        Idle robots are the scarce resource, so we walk the queue ordered by
        priority (then by age) and let the most urgent task pick first. That is
        still greedy - it just makes the greedy order defensible.

        `count_reschedule` is False for the periodic safety-net pass, so the
        reschedule metric reflects genuine re-planning events (task arrival,
        robot completion) rather than 2 Hz timer ticks.
        """
        if getattr(self, 'scheduling_method', 'greedy') == 'cp_sat_rhcr':
            return self._try_assign_joint(count_reschedule)
        with self._lock:
            if not self.pending:
                return
            now = self._now()
            if getattr(self, "use_sim_clock", False):
                if now <= 0.0:
                    return  # DDS has not delivered the first /clock sample.
                for tid in self.pending:
                    if self.tasks[tid].created_at_sec == 0.0:
                        self.tasks[tid].created_at_sec = now
                if self.metrics.m.first_task_at_sec == 0.0:
                    self.metrics.m.first_task_at_sec = now

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

            admitted_wheel = self._wheel_admission_owner(now)
            g1_allowed, yield_to_g1 = self._g1_dispatch_priority()
            assignments = []
            dispatch_points = []
            clearance = float(self.get_parameter("zone_dispatch_clearance_m").value)
            for tid in order:
                task = self.tasks[tid]
                if task.state != TaskState.PENDING:
                    continue

                free = [
                    r for r in self.robots.values()
                    if r.status == RobotStatus.IDLE
                    and (not getattr(self, 'wheel_dispatch_order', [])
                         or r.type == 'g1_humanoid' or r.id == admitted_wheel)
                    and (r.type == 'g1_humanoid' or not yield_to_g1)
                    and (r.type != 'g1_humanoid' or g1_allowed is None
                         or (r.id, tid) in g1_allowed)
                    and self._locomotion_available(r.id, now)
                    and self._navigation_available(r.id)
                    and not self._should_clear_channel(r.id)
                ]
                if not free:
                    # Admission is task-specific: an earlier wheel job must
                    # not hide a later eligible G1 job in this queue scan.
                    continue

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

        getattr(self,'_temporary_pickup_blocks',{}).pop(robot.id,None)
        try:
            # --- leg 1: go to pickup ------------------------------------
            task.state = TaskState.TO_PICKUP
            robot.status = RobotStatus.TO_PICKUP
            home = self.charger_pose.get(robot.id)
            exit_pose = getattr(self, "charger_exit", {}).get(robot.id)
            if home is not None and exit_pose is not None and self._at_charger(robot, home):
                # Clear the parking row before following the transport route;
                # a direct route along the row crosses other robots' bays.
                ok, dist, energy = self._navigate(robot, exit_pose, f"{task.id}:leave_charger")
                d_total += dist
                e_total += energy
                if not ok:
                    reason = getattr(self,'_temporary_pickup_blocks',{}).pop(robot.id,None)
                    if reason:
                        self._defer_task(robot,task,reason,d_total,e_total)
                    else:
                        self._fail(robot, task, "navigation out of charging bay failed")
                    return
            # Where to actually stop: the marker if this robot has the zone to
            # itself, otherwise a free approach slot beside it.
            pickup_goal = self._claim_zone_slot(robot, task.pickup)
            ok, dist, energy = self._navigate(robot, pickup_goal, f"{task.id}:pickup")
            d_total += dist
            e_total += energy
            if not ok:
                reason = getattr(self,'_temporary_pickup_blocks',{}).pop(robot.id,None)
                if reason:
                    self._defer_task(robot,task,reason,d_total,e_total)
                else:
                    self._fail(robot, task, "navigation to pickup failed")
                return

            # --- load ---------------------------------------------------
            task.state = TaskState.LOADING
            robot.status = RobotStatus.LOADING
            if not self._task_point_dwell(robot, self.load_time):
                self._fail(robot, task, "pickup service interrupted")
                return
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
            if not self._task_point_dwell(robot, self.unload_time):
                self._fail(robot, task, "dropoff service interrupted")
                return
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

        An occupied pickup is still pending work for this robot. Keep it
        available for dispatch here; temporary occupancy must not trigger docking.
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
                if self._can_serve(robot, task):
                    return True
        return False

    def _wheel_clearance_required(self, robot):
        if robot.id not in getattr(self,'idle_clearance_pose',{}):
            return False
        startup = (robot.id in getattr(self,'startup_clearance_robots',set())
                   and robot.tasks_completed == 0)
        if getattr(self,'scheduling_method','greedy') == 'cp_sat_rhcr':
            if robot.type == 'g1_humanoid':
                return startup and self._work_remains()
            dependencies = getattr(self,'clearance_release_after_types',{}).get(robot.id,set())
            if (dependencies and not self._work_for(robot)
                    and not any(r.id != robot.id and r.type in dependencies
                                and (r.current_task_id or self._work_for(r)) for r in self.robots.values())):
                # The west refuge clears the charging row but blocks a light
                # robot leaving its own west refuge. Return east once the
                # platforms needing the charging-row clearance have finished.
                return False
            return self._work_remains() and (startup or not self._work_for(robot))
        if robot.type == 'g1_humanoid':
            return False
        return bool(getattr(self,'wheel_dispatch_order',[])) and (robot.tasks_completed>0 or startup)

    def _should_clear_idle_g1(self, robot):
        """Pending but undispatchable work must not park G1 on a work pad."""
        if (robot.type != 'g1_humanoid' or robot.tasks_completed < 1
                or robot.id not in getattr(self, 'idle_clearance_pose', {})):
            return False
        with self._lock:
            if not self._on_task_point(robot, self.clear_task_point_radius_m):
                return False
            eligible = [self.tasks[tid] for tid in self.pending if tid in self.tasks
                        and self.tasks[tid].state == TaskState.PENDING
                        and self._can_serve(robot, self.tasks[tid])]
            busy = any(r.id != robot.id and r.status not in (RobotStatus.IDLE, RobotStatus.CHARGING)
                       for r in self.robots.values())
            pending_work = any(tid in self.tasks and self.tasks[tid].state == TaskState.PENDING
                               and any(self._can_serve(r, self.tasks[tid]) for r in self.robots.values())
                               for tid in self.pending)
            return (busy or pending_work) and not any(self._pickup_ready(t.pickup, robot.id) for t in eligible)

    def _start_docking(self, robot: RobotState) -> None:
        """Claim an idle robot for its charger and drive it there off-thread.

        Status goes to CHARGING inside the lock before the thread is started,
        so the 2 Hz assignment pass can never hand the robot a task while it is
        on its way to the bay.
        """
        with self._lock:
            if (robot.status != RobotStatus.IDLE
                    or getattr(self, '_docking_owner', None) is not None
                    or time.monotonic() < getattr(self, '_docking_retry_at', {}).get(robot.id, 0.)):
                return
            self._docking_owner = robot.id
            self._idle_since[robot.id] = None
            robot.status = RobotStatus.CHARGING
        threading.Thread(
            target=self._drive_home, args=(robot,), daemon=True
        ).start()

    def _plan_docking_route(self, robot, home, waypoints):
        """Prefer configured approaches, but allow a verified direct return.

        A historical approach waypoint may be occupied by an already docked
        peer. It is guidance, not a reason to reject a reachable charging bay.
        """
        if getattr(self, 'planner_clients', {}).get(robot.id) is None:
            return [*waypoints, home]
        candidates = [[*waypoints, home]]
        if waypoints:
            candidates.append([home])
        for route in candidates:
            scene = self._dispatch_scene()
            if scene is None:
                return None
            start = None
            for target in route:
                poses = self._request_target_path(robot, target, 'GridBased', start)
                if not poses:
                    break
                start = self._end_pose(poses)
            else:
                if self._same_dispatch_scene(scene, self._dispatch_scene()):
                    return route
        return None

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
        clearing_idle = self._should_clear_idle_g1(robot) or self._wheel_clearance_required(robot)
        if clearing_idle:
            home = self.idle_clearance_pose[robot.id]
        ok = True
        drove = False
        dist = 0.0
        energy = 0.0
        try:
            if home is None or (self._at_clearance(robot, home) if clearing_idle else self._at_charger(robot, home)):
                # Either there is no bay configured, or the robot is already
                # standing on it - re-issuing a centimetre goal is how a robot
                # spends a whole recovery cycle going nowhere.
                return
            if not self.return_to_charger or (self._work_for(robot) and
                    not self._should_clear_channel(robot.id) and not clearing_idle):
                # Work arrived between the tick that decided to dock and this
                # thread starting. Stay put and take it.
                return
            robot.status = RobotStatus.CHARGING
            drove = True
            self.get_logger().info(
                f"{robot.id}: {'clearing task point' if clearing_idle else 'returning to charger'} "
                f"({home.x:.2f},{home.y:.2f})"
            )
            # Approach the charging row from its open side; driving straight
            # down the row can trap a biped behind an already parked AMR.
            route = [] if clearing_idle else getattr(self, "charger_route", {}).get(robot.id, [])
            if self._should_clear_channel(robot.id):
                # A charger may be blocked by G1 even while an exit waypoint
                # is reachable. Leave the resource before attempting docking.
                if getattr(self, 'planner_clients', {}).get(robot.id) is not None:
                    cleared, leg_dist, leg_energy = self._drive_to_clearance_exit(robot)
                    dist += leg_dist
                    energy += leg_energy
                    if not cleared:
                        ok = False
                        self.get_logger().warn(f'{robot.id}: no reachable clearance exit; retrying when space changes')
                        return
                else:
                    exit_pose = self._channel_maneuver(robot, 'exit_waypoints')
                    if exit_pose is not None:
                        route = [exit_pose, *route]
            planned = self._plan_docking_route(robot, home, route)
            if planned is None:
                ok = False
                self.get_logger().info(
                    f'{robot.id}: docking pending; no clear full-body route, allowing another robot to return first')
                return
            for index, target in enumerate(planned):
                ok, leg_dist, leg_energy = self._navigate(
                    robot, target, f"{robot.id}:charge:{index}")
                dist += leg_dist
                energy += leg_energy
                if not ok:
                    break
            if ok and clearing_idle and not self._at_clearance(robot, home):
                ok = False
                self.get_logger().info(f'{robot.id}: clearance target not reached within its dedicated tolerance')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{robot.id}: charger leg crashed: {exc!r}")
            ok = False
        finally:
            with self._lock:
                robot.odom_distance_m += dist
                robot.energy_consumed_wh += energy
                self._apply_battery_drain(robot, dist)
                robot.status = RobotStatus.IDLE
                if getattr(self, '_docking_owner', None) == robot.id:
                    self._docking_owner = None
                if not ok:
                    if not hasattr(self, '_docking_retry_at'):
                        self._docking_retry_at = {}
                    self._docking_retry_at[robot.id] = time.monotonic() + 8.

        if not drove:
            return
        if ok:
            self.get_logger().info(
                f"{robot.id}: {'parked at idle clearance' if clearing_idle else 'docked at charger'}, "
                f"battery {robot.battery_pct:.1f}%"
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

    def _at_clearance(self, robot, target):
        """Clearance is a physical maneuver, not the looser charging radius."""
        stamps = getattr(self, 'live_pose_stamps', None)
        if stamps is not None and not 0 <= time.monotonic()-stamps.get(robot.id,-math.inf) <= .5:
            return False
        pose = self._live_xy(robot.id)
        radius = getattr(self, 'clearance_arrive_radius', {}).get(robot.id, .20)
        return pose is not None and math.dist(pose[:2], (target.x,target.y)) <= radius

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

    @staticmethod
    def _is_task_point(tag):
        return tag.rsplit(':', 1)[-1] in ('pickup', 'dropoff')

    def _task_arrival_pose(self, robot, target):
        pose = self.live_pose.get(robot.id)
        stamps = getattr(self, 'live_pose_stamps', None)
        if stamps is not None and not 0 <= time.monotonic() - stamps.get(robot.id, -math.inf) <= .5:
            return None
        if (pose is not None and all(math.isfinite(v) for v in pose)
                and math.hypot(pose[0] - target.x, pose[1] - target.y) <= .15):
            return pose
        return None

    def _locomotion_available(self, robot_id: str, now: float) -> bool:
        """Legged robots must have fresh balance readiness; wheel robots pass."""
        health = getattr(self, "locomotion_health", {})
        if robot_id not in health:
            return True
        ready, stamp = health[robot_id]
        return ready and 0.0 <= now - stamp < 1.0

    def _navigation_available(self, robot_id: str) -> bool:
        if getattr(self, "simulate_when_nav_unavailable", False):
            return True
        client = getattr(self, "nav_clients", {}).get(robot_id)
        return client is not None and client.server_is_ready()

    def _send_goal(self, robot: RobotState, target: Pose2D, tag: str):
        """Hand one NavigateToPose goal to Nav2.

        Returns (handle, None) on success and (None, reason) on any failure -
        "goal_occupied", "unavailable", "timeout" or "rejected". The reasons are kept apart
        because they mean different things to the caller: only "unavailable"
        (no Nav2 at all) justifies the kinematic stand-in, while a rejected or
        timed-out goal has to fail the leg honestly instead of silently
        "delivering" cargo the robot never carried.
        """
        occupancy = getattr(self, "_occupancy", None)
        profile = getattr(self, "types", {}).get(robot.type)
        task_point = self._is_task_point(tag)
        headings = [target.yaw]
        if task_point:
            model = (profile or {}).get('kinematics', 'legged')
            if model in ('diff', 'mecanum'):
                if not hasattr(self, '_arrival_heading_cache'):
                    self._arrival_heading_cache = {}
                lattice = 'omni' if model == 'mecanum' else 'diff'
                if lattice not in self._arrival_heading_cache:
                    filename = os.path.join(get_package_share_directory('hetero_fleet_bringup'),
                                            'config', 'lattice', f'{lattice}_64.json')
                    with open(filename) as stream:
                        self._arrival_heading_cache[lattice] = json.load(stream)['lattice_metadata']['heading_angles']
                headings = self._arrival_heading_cache[lattice]
            else:
                headings = [2 * math.pi * k / 64 for k in range(64)]
        if occupancy is not None and profile is not None and not any(rectangle_is_free(
                occupancy, (target.x, target.y, yaw), profile["footprint"],
                profile.get("footprint_padding", 0.0)) for yaw in headings):
            self.get_logger().warn(
                f"{robot.id}: target body intersects the static map for {tag}")
            return None, "goal_occupied"
        if not self._locomotion_available(robot.id, self._now()):
            return None, "locomotion_unavailable"
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
        goal = self._make_goal(target)
        if task_point:
            tree = 'g1_task_navigation.xml' if robot.type == 'g1_humanoid' else 'task_navigation.xml'
            goal.behavior_tree = os.path.join(
                get_package_share_directory('hetero_fleet_bringup'), 'config', tree)
        elif tag.endswith(':channel_queue_retreat'):
            goal.behavior_tree = os.path.join(
                get_package_share_directory('hetero_fleet_bringup'), 'config', 'retreat_navigation.xml')
        send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
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

    def _wait_for_result(self, robot: RobotState, target: Pose2D, result_future,
                         allow_traffic=True, cancel_when=None, position_only=False):
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
            last_distance = math.hypot(anchor[0] - target.x, anchor[1] - target.y)
            best = last_distance if position_only else self._goal_error(anchor, target)
        started = last_progress = self._now()
        # Bound a leg even if a moving robot keeps wandering. This is simulation
        # time, so a slow / paused Gazebo does not consume the navigation budget.
        leg_budget = max(180.0, 60.0 + last_distance / 0.10)

        while not result_future.done():
            if not self._locomotion_available(robot.id, self._now()):
                self.get_logger().error(f"{robot.id}: balance readiness lost during navigation")
                return None, False, last_distance
            time.sleep(0.1)
            if cancel_when is not None and cancel_when():
                return "traffic_timeout", False, last_distance
            if position_only and self._task_arrival_pose(robot, target) is not None:
                return 'task_arrived', False, 0.
            if self._channel_blocked(robot.id):
                # The command gate already stopped the vehicle. Cancel before
                # Nav2's progress checker initiates recovery motions in a queue.
                return "channel_wait", False, last_distance
            if not self.stall_enabled:
                continue
            xy = self._live_xy(robot.id)
            if xy is None:
                continue
            now = self._now()
            traffic = getattr(self, "traffic", None)
            if allow_traffic and traffic is not None:
                traffic.observe(robot.id, xy, now)
                with self._lock:
                    idle = {name for name, state in self.robots.items()
                            if state.status == RobotStatus.IDLE
                            and self._locomotion_available(name, now)
                            and name in self.charger_pose
                            and not self._at_charger(state, self.charger_pose[name])}
                    request = traffic.request(robot.id, self.robots,
                                              dict(self.live_pose), set(self.active_goal), now, idle=idle,
                                              targets=getattr(self, '_nav_targets', {}),
                                              corridor_clear=lambda a,b: all(self._slot_is_free(
                                                  a[0]+(b[0]-a[0])*s/20,
                                                  a[1]+(b[1]-a[1])*s/20, .05)
                                                  for s in range(1,20)))
                    if request is not None and request.loser_idle and not request.yield_started.is_set():
                        request.yield_started.set()
                        self.robots[request.loser].status = RobotStatus.CHARGING
                        threading.Thread(target=self._yield_idle_peer,
                                         args=(self.robots[request.loser], request), daemon=True).start()
                if request is not None and (robot.id == request.loser or not request.parked.is_set()):
                    return request, False, math.hypot(xy[0]-target.x, xy[1]-target.y)
            if now < started or now < last_progress:
                started = last_progress = now
                best_path = None
            distance = math.hypot(xy[0] - target.x, xy[1] - target.y)
            last_distance = distance
            error = distance if position_only else self._goal_error(xy, target)
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
            displacement = math.hypot(xy[0] - anchor[0], xy[1] - anchor[1]) \
                if anchor is not None else 0.0
            # Changing the global route can shorten feedback distance without
            # moving the robot. Only physical displacement earns more time.
            path_progress = path_progress and displacement > self.stall_move_eps_m
            moved = displacement > self.stall_wander_m
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
        if not hasattr(self, '_nav_result_errors'):
            self._nav_result_errors = {}
        self._nav_result_errors[robot.id] = getattr(getattr(result, 'result', None), 'error_code', 0)
        return status, False, last_distance

    def _has_nearby_route_peer(self, robot, target):
        """A fresh physical peer can temporarily disconnect an otherwise free pose."""
        pose = self.live_pose.get(robot.id)
        profile = getattr(self, 'types', {}).get(robot.type)
        occupancy = getattr(self, '_occupancy', None)
        if (pose is None or profile is None or occupancy is None or
                not rectangle_is_free(occupancy, pose, profile['footprint'],
                                      profile.get('footprint_padding', 0.))):
            return False
        stamps = getattr(self, 'live_pose_stamps', {})
        now = time.monotonic()
        if not 0 <= now - stamps.get(robot.id, -math.inf) <= .5:
            return False
        return any(name != robot.id and 0 <= now - stamps.get(name, -math.inf) <= .5
                   and (math.dist(p[:2], pose[:2]) <= 3. or
                        math.dist(p[:2], (target.x, target.y)) <= 2.)
                   for name, p in self.live_pose.items())

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

    def _yield_idle_peer(self, robot: RobotState, request: Encounter) -> None:
        try:
            self._yield_to_peer(robot, request)
        finally:
            with self._lock:
                robot.status = RobotStatus.IDLE
                self._idle_since[robot.id] = self._now()

    def _yield_to_peer(self, robot: RobotState, request: Encounter) -> float:
        """Pull clear while the peer holds, then let it pass before retrying."""
        driven = 0.0
        try:
            hold_deadline = self._now()+10.0
            while not request.held.is_set() and rclpy.ok() and self._now() < hold_deadline:
                time.sleep(.1)
            if not request.held.is_set() or request.done.is_set():
                return driven
            initial = self._live_xy(robot.id)
            if initial is None:
                return driven
            toward_peer = math.atan2(request.origin[1]-initial[1], request.origin[0]-initial[0])
            reverse_away = math.cos(toward_peer-initial[2]) > 0.0
            if not request.loser_idle and reverse_away:
                with self._recovery_lock:
                    driven += self._back_off(robot)
            pose = self._live_xy(robot.id)
            if pose is None or request.done.is_set():
                return driven
            x, y, yaw = pose
            clearance = self.slot_clearance.get(robot.id, .45)
            candidates = []
            bearing = math.atan2(y-request.origin[1], x-request.origin[0])
            nx, ny = -math.sin(bearing), math.cos(bearing)
            winner = self.robots.get(request.winner)
            profile = getattr(self, 'types', {}).get(winner.type, {}) if winner else {}
            lane_clearance = profile.get('footprint', [0.6,0.52])[1]/2+.06
            # Search behind and to either side. Check the whole connection,
            # so a free point across a wall is not mistaken for a pullout.
            offsets = (.5, 1.0, 1.5, 2.0) if reverse_away else (-.5, -1.0, -1.5, -2.0)
            if request.loser_idle:
                offsets = (.5, 1.0, 1.5, 2.0, -.5, -1.0, -1.5, -2.0)
            for behind in offsets:
                for side in (-.4, .4, -.45, .45, -.48, .48, -.5, .5, -.6, .6, -.8, .8, -1., 1.):
                    tx = x-behind*math.cos(yaw)-side*math.sin(yaw)
                    ty = y-behind*math.sin(yaw)+side*math.cos(yaw)
                    if math.dist((tx,ty), request.origin[:2]) < 1.6:
                        continue
                    if any(not self._slot_is_free(x+(tx-x)*s/12,
                                                  y+(ty-y)*s/12, clearance)
                           for s in range(1,13)):
                        continue
                    if self._nearest_teammate_m(robot.id,tx,ty) < 1.3:
                        continue
                    # A nearby parking point can still occupy the middle of
                    # the aisle. Require room for the peer to pass beside it.
                    if not any(all(self._slot_is_free(
                            tx+sign*.80*nx+along*math.cos(bearing),
                            ty+sign*.80*ny+along*math.sin(bearing), lane_clearance)
                            for along in (-.85,-.425,0.,.425,.85)) for sign in (-1,1)):
                        continue
                    final = getattr(self, '_nav_targets', {}).get(request.winner)
                    if final and math.dist((tx,ty), final) < clearance+self.slot_clearance.get(request.winner,.45)+.15:
                        continue
                    candidates.append((math.hypot(tx-x,ty-y), Pose2D(tx,ty,yaw)))
            # Do not send a working robot across the map to its charger.
            # Release the reservation if the nearby aisle has no safe pullout.
            if not candidates:
                self.get_logger().warn(f"{robot.id}: no nearby safe pullout; releasing traffic hold")
                return driven
            refuge = min(candidates, key=lambda item:item[0])[1]
            self.get_logger().info(f"{robot.id}: yielding to {request.winner} at ({refuge.x:.2f},{refuge.y:.2f})")
            park_deadline = self._now()+30.0
            ok, distance, _ = self._navigate(
                robot, refuge, 'traffic_pullout', allow_traffic=False,
                cancel_when=lambda: request.done.is_set() or self._now() > park_deadline)
            driven += distance
            actual = self._live_xy(robot.id)
            if not ok or actual is None or math.hypot(actual[0]-refuge.x,actual[1]-refuge.y) > .35:
                return driven
            request.parked.set()
            # Keep the refuge until the peer has moved past the encounter.
            # Re-sending both transport goals immediately recreates the jam.
            deadline = self._now()+90.0
            if request.loser_origin:
                dx = request.loser_origin[0]-request.origin[0]
                dy = request.loser_origin[1]-request.origin[1]
                norm = max(math.hypot(dx,dy), .01)
                forward = (dx/norm,dy/norm)
            else:
                forward = (math.cos(request.origin[2]),math.sin(request.origin[2]))
            clear_distance = clearance+self.slot_clearance.get(request.winner,.45)+.15
            while rclpy.ok() and self._now() < deadline and not request.done.is_set():
                peer = self._live_xy(request.winner)
                if peer is not None and (peer[0]-actual[0])*forward[0] + (peer[1]-actual[1])*forward[1] > .8 \
                        and math.dist(peer[:2], actual[:2]) > clear_distance:
                    break
                if request.winner_finished.is_set() and peer is not None \
                        and math.dist(peer[:2], actual[:2]) > clear_distance:
                    break
                time.sleep(.1)
            return driven
        finally:
            self.traffic.finish(request, self._now())

    def _recover_heading(self, robot):
        """Native collision-checked Spin only after an actual full-body sweep.

        This helps a wheeled body leave a tight aisle at a skew angle without
        lowering MPPI obstacle costs or accepting an unvisited transport goal.
        The normal navigation intent, CBS planner and velocity gate stay active.
        """
        if robot.type not in ('omni_mecanum_amr','light_diff_amr','heavy_diff_amr'):
            return False
        pose = self._live_xy(robot.id)
        profile = getattr(self,'types',{}).get(robot.type)
        if profile is None:
            return False
        stamps = getattr(self,'live_pose_stamps',{})
        wall = time.monotonic()
        if pose is None or not 0 <= wall-stamps.get(robot.id,-math.inf) <= .5:
            return False
        target = min((0.,math.pi/2,math.pi,-math.pi/2),key=lambda yaw:
                     abs(math.atan2(math.sin(yaw-pose[2]),math.cos(yaw-pose[2]))))
        turn = math.atan2(math.sin(target-pose[2]),math.cos(target-pose[2]))
        if abs(turn) < .12:
            return False
        peers = []
        for name,peer in self.robots.items():
            if name == robot.id:
                continue
            other = self._live_xy(name)
            if other is None or not 0 <= wall-stamps.get(name,-math.inf) <= .5:
                return False
            shape = self.types[peer.type]
            size = [v+2*(shape.get('footprint_padding',0.)+.12) for v in shape['footprint']]
            peers.append((other,size))
        if not heading_sweep_clear(getattr(self,'_occupancy',None),pose,target,
                profile['footprint'],profile.get('footprint_padding',0.),peers):
            return False
        client = getattr(self,'spin_clients',{}).get(robot.id)
        if client is None or not client.wait_for_server(timeout_sec=1.):
            return False
        # Commit through the actual CBS/EECBS planner. The resulting /plan
        # also supplies the channel gate with a checked recovery path.
        if not self._request_target_path(robot,Pose2D(pose[0],pose[1],target),'GridBased',commit=True):
            return False
        goal = Spin.Goal(); goal.target_yaw = turn; goal.time_allowance.sec = 12
        future = client.send_goal_async(goal)
        limit = time.monotonic()+8.
        while not future.done() and rclpy.ok() and time.monotonic()<limit:
            time.sleep(.05)
        if not future.done():
            future.add_done_callback(lambda f: f.result().cancel_goal_async()
                                     if f.result() and f.result().accepted else None)
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        self.get_logger().info(f'{robot.id}: full-body checked Spin recovery {turn:.3f} rad')
        result = handle.get_result_async(); limit = time.monotonic()+35.
        deadline = self._now()+14.
        while not result.done() and rclpy.ok() and time.monotonic()<limit and self._now()<deadline:
            time.sleep(.05)
        if not result.done():
            self._cancel_navigation(handle)
            return False
        actual = self._live_xy(robot.id)
        return bool(result.result().status == 4 and actual is not None and
                    abs(math.atan2(math.sin(target-actual[2]),math.cos(target-actual[2])))<.15)

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

        # NavigateToPose may turn and drive forwards toward a point behind it;
        # that cannot release a robot whose turning envelope is already blocked.
        # Nav2 BackUp reverses without turning and retains collision checking.
        client = getattr(self, "backup_clients", {}).get(robot.id)
        if client is not None:
            reverse = min(back, math.hypot(tx - x, ty - y))
            # BackUp follows the current heading, so check that whole straight
            # segment rather than just the angled candidate's endpoint.
            for step in range(1, 11):
                d = reverse * step / 10
                if not self._slot_is_free(x-d*math.cos(yaw), y-d*math.sin(yaw), clearance):
                    return 0.0
            if not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn(f"{robot.id}: BackUp unavailable; holding instead of turning into traffic")
                return 0.0
            goal = BackUp.Goal()
            goal.target.x = -reverse
            goal.speed = min(0.12, robot.max_speed)
            goal.time_allowance.sec = max(1, int(self.stall_backoff_sec))
            sent = client.send_goal_async(goal)
            deadline = self._now() + self.stall_backoff_sec
            while not sent.done() and self._now() < deadline:
                time.sleep(0.1)
            if not sent.done():
                sent.add_done_callback(lambda f: f.result().cancel_goal_async()
                                       if f.result() and f.result().accepted else None)
                return 0.0
            handle = sent.result()
            if handle is None or not handle.accepted:
                return 0.0
            future = handle.get_result_async()
            while not future.done() and self._now() < deadline:
                time.sleep(0.1)
            if not future.done():
                self._cancel_navigation(handle)
            final = self._live_xy(robot.id) or (x, y, yaw)
            moved = math.hypot(final[0]-x, final[1]-y)
            self.get_logger().info(f"{robot.id}: straight BackUp moved {moved:.2f} m")
            return moved

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

    def _on_channel_wait(self, name, message):
        self._channel_wait_states[name] = message.data, time.monotonic()

    def _on_channel_snapshot(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('channel status must be an object')
            self._channel_snapshot = (data, time.monotonic())
            rolling = getattr(self, '_rolling', None)
            if rolling is not None and rolling.plan is not None:
                epoch = float(data.get('sim_time', rolling.plan.epoch))
                if epoch < rolling.plan.epoch:
                    rolling.invalidate('clock_reset')
                elif epoch-rolling.last >= rolling.period:
                    for row in rolling.plan.rows:
                        windows = data.get('scheduler',{}).get('windows',{}).get(row['robot_id'],[])
                        if windows and min(w['start'] for w in windows)-row['start'] > rolling.drift:
                            rolling.invalidate('traffic_delay')
                            break
        except (ValueError, TypeError):
            self.get_logger().warn('Ignoring invalid channel status')

    def _should_clear_channel(self, name):
        """Yield an occupied bottleneck between jobs when another robot queues.

        Physically navigate out before releasing the token. Taking a new job
        inside the same group used to let one robot monopolize it indefinitely.
        """
        robot = self.robots.get(name)
        if robot is None or (robot.type == 'g1_humanoid' and getattr(self, 'scheduling_method', 'greedy') != 'cp_sat_rhcr') or not getattr(self, 'return_to_charger', True):
            return False
        snapshot, stamp = getattr(self, '_channel_snapshot', ({}, -math.inf))
        if not 0 <= time.monotonic() - stamp <= 1.5:
            return False
        return any(group.get('owner') == name and group.get('entered') and
                   (any(peer != name for peer in group.get('queue', [])) or
                    name in getattr(self, '_channel_yield_waits', {}).copy().values())
                   for group in snapshot.get('groups', {}).values())

    def _channel_blocked(self, name):
        robot = getattr(self, 'robots', {}).get(name)
        if robot is not None and robot.type == 'g1_humanoid' and getattr(self, 'scheduling_method', 'greedy') != 'cp_sat_rhcr':
            return False
        if name in getattr(self, '_channel_retreating', set()):
            # A replacement route OUT of the queue must get a chance to reach
            # the arbiter. Its command gate still validates every command.
            return False
        return getattr(self, '_channel_wait_states', {}).get(name, (False, 0))[0]

    def _channel_exit_owner(self, name):
        """Queued wheels clear approaches for every current owner.

        An active task can leave the resource to reach a forecourt pickup;
        limiting this handshake to charging owners deadlocks that pickup
        behind a queued vehicle. Full-body checks still select the refuge.
        """
        snapshot, stamp = getattr(self, '_channel_snapshot', ({}, -math.inf))
        if not 0 <= time.monotonic() - stamp <= 1.5:
            return None
        for group in snapshot.get('groups', {}).values():
            owner = group.get('owner')
            peer = self.robots.get(owner)
            if (owner != name and name in group.get('queue', [])
                    and peer is not None and (peer.type != 'g1_humanoid' or getattr(self, 'scheduling_method', 'greedy') == 'cp_sat_rhcr')):
                return owner
        return None

    def _channel_owner_holds_turn(self, owner):
        snapshot, stamp = getattr(self, '_channel_snapshot', ({}, -math.inf))
        if not 0 <= time.monotonic() - stamp <= 1.5:
            return True  # An unknown physical exit cannot release the waiter.
        return any(g.get('owner') == owner
                   for g in snapshot.get('groups', {}).values())

    def _retreat_from_channel_queue(self, robot, owner, cancel_when):
        snapshot = getattr(self, '_channel_snapshot', ({}, 0))[0]
        configured = bool(snapshot.get('maneuvers', {}).get('retreat_waypoints'))
        home = (self._channel_maneuver(robot, 'retreat_waypoints') if configured
                else self.charger_pose.get(robot.id))
        if home is None or (not configured and self._at_charger(robot, home)):
            return 0.
        if not hasattr(self, '_channel_retreating'):
            self._channel_retreating = set()
        self._channel_retreating.add(robot.id)
        deadline = self._now() + 120.
        self.get_logger().info(f'{robot.id}: clearing entrance for {owner}; retreating to ({home.x:.2f},{home.y:.2f})')
        try:
            _, distance, _ = self._navigate(
                robot, home, f'{robot.id}:channel_queue_retreat', allow_traffic=False,
                cancel_when=lambda: self._now() > deadline or
                (configured and self._parking_blocks_committed_task(robot,(home.x,home.y,home.yaw))) or
                (cancel_when is not None and cancel_when()))
            return distance
        finally:
            self._channel_retreating.discard(robot.id)

    def _exit_target_occupied(self, robot, target):
        profile = getattr(self, 'types', {}).get(robot.type)
        if profile is None:
            return False
        own_size = [v + 2*profile.get('footprint_padding', 0.) for v in profile['footprint']]
        now = time.monotonic()
        stamps = getattr(self, 'live_pose_stamps', {})
        for name, peer in self.robots.items():
            if name == robot.id or not 0 <= now-stamps.get(name, -math.inf) <= .5:
                continue
            pose = self.live_pose.get(name)
            peer_profile = self.types.get(peer.type)
            if pose is not None and peer_profile is not None and rectangles_overlap(
                    (target.x, target.y, target.yaw), own_size, pose,
                    [v + .24 for v in peer_profile['footprint']], margin=.025):
                return True
        return False

    def _drive_to_clearance_exit(self, robot):
        excluded, distance, energy = [], 0., 0.
        while True:
            target = self._reachable_channel_exit(robot, excluded=excluded)
            if target is None:
                return False, distance, energy
            excluded.append(target)
            occupied_since = [None]
            def blocked():
                now = self._now()
                if not self._exit_target_occupied(robot, target):
                    occupied_since[0] = None
                    return False
                if occupied_since[0] is None or now < occupied_since[0]:
                    occupied_since[0] = now
                return now - occupied_since[0] >= 2.
            ok, moved, used = self._navigate(
                robot, target, f'{robot.id}:charge:exit:{len(excluded)}', cancel_when=blocked)
            distance += moved
            energy += used
            if ok:
                return True, distance, energy
            self.get_logger().info(f'{robot.id}: clearance exit changed; selecting another route')

    def _reachable_channel_exit(self, robot, excluded=()):
        """Verify candidates with the actual Nav2 body planner before choosing."""
        client = getattr(self, 'planner_clients', {}).get(robot.id)
        if client is None:
            return self._channel_maneuver(robot, 'exit_waypoints')
        if not client.wait_for_server(timeout_sec=1.):
            return None
        excluded = list(excluded)
        while True:
            target = self._channel_maneuver(robot, 'exit_waypoints', excluded=excluded)
            if target is None:
                return None
            excluded.append(target)
            if self._probe_target_path(robot, target):
                return target
            self.get_logger().info(
                f'{robot.id}: exit ({target.x:.2f},{target.y:.2f}) has no route; trying another exit')

    def _probe_target_path(self, robot, target, planner_id='GridBased'):
        return bool(self._request_target_path(robot,target,planner_id))

    def _request_target_path(self, robot, target, planner_id='GridBased', start=None, commit=False):
        client = getattr(self, 'planner_clients', {}).get(robot.id)
        if client is None or not client.wait_for_server(timeout_sec=1.):
            return []
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_goal(target).pose
        goal.planner_id = planner_id if commit else "Probe" + planner_id
        goal.use_start = start is not None
        if start is not None:
            goal.start = self._make_goal(start).pose
        sent = client.send_goal_async(goal)
        deadline = time.monotonic() + 15.
        while not sent.done() and rclpy.ok() and time.monotonic() < deadline:
            time.sleep(.05)
        if not sent.done():
            sent.add_done_callback(lambda f: f.result().cancel_goal_async()
                                   if f.result() and f.result().accepted else None)
            return []
        handle = sent.result()
        if handle is None or not handle.accepted:
            return []
        future = handle.get_result_async()
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            time.sleep(.05)
        if not future.done():
            handle.cancel_goal_async()
            return []
        result = future.result()
        poses = result.result.path.poses if result is not None else []
        if result is None or result.status != 4 or not poses:
            return []
        end = poses[-1].pose.position
        return poses if (math.hypot(end.x-target.x, end.y-target.y) <= .15
                         and self._path_clears_peers(robot, poses)) else []

    def _path_clears_peers(self, robot, poses):
        types = getattr(self, 'types', {})
        if robot.type not in types:
            return False
        scene = self._dispatch_scene()
        if scene is None:
            return False
        profile = types[robot.type]
        own_size = [v+2*profile.get('footprint_padding', 0.) for v in profile['footprint']]
        previous = None
        radius = math.hypot(*own_size)/2
        for stamped in poses:
            p = stamped.pose
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            current = (p.position.x,p.position.y,yaw)
            start = previous or current
            turn = math.atan2(math.sin(yaw-start[2]),math.cos(yaw-start[2]))
            steps = max(1, math.ceil((math.dist(start[:2],current[:2])+radius*abs(turn))/.025))
            for index in range(steps+1):
                t = index/steps
                sample = (start[0]+t*(current[0]-start[0]),
                          start[1]+t*(current[1]-start[1]),start[2]+t*turn)
                for name, peer in self.robots.items():
                    if name == robot.id:
                        continue
                    if peer.type not in types or name not in scene:
                        return False
                    if rectangles_overlap(sample, own_size, scene[name],
                            [v+.24 for v in types[peer.type]['footprint']], .025):
                        return False
            previous = current
        return True

    def _parking_blocks_committed_task(self, robot, pose):
        """Refuges must leave both endpoints of committed transports usable."""
        types = getattr(self, 'types', {})
        profile = types.get(robot.type)
        if profile is None:
            return True
        radius = math.hypot(*(v/2+profile.get('footprint_padding',0.) for v in profile['footprint']))
        for name, peer in self.robots.items():
            if name == robot.id:
                continue
            task = getattr(self,'tasks',{}).get(getattr(peer,'current_task_id',''))
            if task is None or task.state in TERMINAL_STATES:
                continue
            other = types.get(peer.type)
            if other is None:
                return True
            clearance = radius + math.hypot(*(v/2+other.get('footprint_padding',0.) for v in other['footprint'])) + .12
            if any(math.dist(pose[:2],(p.x,p.y)) < clearance for p in (task.pickup,task.dropoff)):
                return True
        return False

    def _channel_maneuver(self, robot, kind, excluded=()):
        """Choose a real Nav2 waypoint outside the complete shared resource.

        Retreats also require a free approach-side segment and increased
        entrance distance. This is a destination filter; Nav2 and the command
        arbiter still check the actual kinematic route and all live obstacles.
        """
        snapshot, stamp = getattr(self, '_channel_snapshot', ({}, -math.inf))
        occupancy = getattr(self, '_occupancy', None)
        types = getattr(self, 'types', {})
        pose = self._live_xy(robot.id)
        if (not 0 <= time.monotonic() - stamp <= 1.5 or occupancy is None
                or robot.type not in types or pose is None):
            return None
        bounds = [c['bounds'] for c in snapshot.get('channels', [])]
        if not bounds:
            return None
        profile = types[robot.type]
        size = profile['footprint']
        padding = profile.get('footprint_padding', 0.)
        margin = snapshot.get('clearance_margin', .15)
        queued = {n for g in snapshot.get('groups', {}).values()
                  if g.get('owner') == robot.id and g.get('entered')
                  for n in g.get('queue', [])}
        queued.update(name for name, owner in getattr(self, '_channel_yield_waits', {}).copy().items()
                      if owner == robot.id)
        retreats = {}
        if kind == 'exit_waypoints':
            for name in queued:
                peer = self.robots.get(name)
                if peer is not None and (peer.type != 'g1_humanoid' or getattr(self, 'scheduling_method', 'greedy') == 'cp_sat_rhcr'):
                    target = self._channel_maneuver(peer, 'retreat_waypoints')
                    if target is not None:
                        retreats[name] = (target.x, target.y, target.yaw)

        def distance_to(p, b):
            return math.hypot(max(b[0]-p[0], p[0]-b[2], 0.),
                              max(b[1]-p[1], p[1]-b[3], 0.))
        entrance = min(bounds, key=lambda b: distance_to(pose, b))

        def free(p):
            if (any(intersects(p, size, padding + margin, b) for b in bounds)
                    or not rectangle_is_free(occupancy, p, size, padding)):
                return False
            for name, peer in self.robots.items():
                if name == robot.id:
                    continue
                # Account for where a queued peer will actually retreat. The
                # nearest exit can itself be blocked by that destination.
                q = retreats.get(name, self._live_xy(name))
                if q is None or peer.type not in types:
                    return False
                stamps = getattr(self, 'live_pose_stamps', None)
                if stamps is not None and not 0 <= time.monotonic() - stamps.get(name, -math.inf) <= .5:
                    return False
                if rectangles_overlap(p, [v + 2*padding for v in size],
                                      q, types[peer.type]['footprint'], .24):
                    return False
            return True

        candidates = sorted(snapshot.get('maneuvers', {}).get(kind, []),
                            key=lambda p: math.dist(p[:2], pose[:2]))
        if kind == 'retreat_waypoints':
            if (free(pose) and not self._parking_blocks_committed_task(robot,pose)
                    and any(math.dist(p[:2], pose[:2]) <= .35 for p in candidates)):
                return Pose2D(*pose)
            # A short retreat clears space without imposing a new parking yaw.
            candidates = [(p[0], p[1], pose[2]) for p in candidates]
        for point in candidates:
            if any(math.hypot(point[0]-p.x, point[1]-p.y) < .05 for p in excluded):
                continue
            if not free(point):
                continue
            if self._parking_blocks_committed_task(robot,point):
                continue
            if kind == 'retreat_waypoints':
                # Moving away from this entrance may bring a body nearer a
                # different entrance. All-channel exclusion is checked above;
                # minimizing distance over every entrance rejected safe short
                # retreats and sent vehicles across the owner's return route.
                if distance_to(point, entrance) < distance_to(pose, entrance) + .3:
                    continue
                turn = math.atan2(math.sin(point[2]-pose[2]), math.cos(point[2]-pose[2]))
                steps = max(1, math.ceil((math.dist(point[:2], pose[:2]) + abs(turn)) / .025))
                if not all(free((pose[0]+(point[0]-pose[0])*i/steps,
                                 pose[1]+(point[1]-pose[1])*i/steps, pose[2]+turn*i/steps))
                           for i in range(steps+1)):
                    continue
                direction = math.atan2(point[1]-pose[1], point[0]-pose[0])
                alignment = math.cos(direction-pose[2])
                if abs(alignment) < .9:
                    # A lateral displacement cannot end at the original yaw
                    # in every pocket. Choose the nearer forward/reverse
                    # travel heading; aligned short reverses keep their yaw.
                    yaw = direction if alignment >= 0 else direction + math.pi
                    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                    point = (point[0], point[1], yaw)
                    if not free(point):
                        continue
            return Pose2D(*point)
        return None

    def _publish_channel_intents(self):
        with self._channel_lock:
            for name, publisher in self._channel_intent_publishers.items():
                publisher.publish(Bool(data=self._channel_counts.get(name, 0) > 0))
            for name, publisher in getattr(self, '_channel_transport_publishers', {}).items():
                robot = self.robots[name]
                carrying_out_task = bool(robot.current_task_id) and (robot.type != 'g1_humanoid' or getattr(self, 'scheduling_method', 'greedy') == 'cp_sat_rhcr') and robot.status in (
                    RobotStatus.TO_PICKUP, RobotStatus.LOADING, RobotStatus.TO_DROPOFF)
                publisher.publish(Bool(data=carrying_out_task))

    def _set_channel_active(self, name, active):
        if not getattr(self, '_channel_intent_publishers', {}):
            return  # Non-ROS scheduler fixtures and legacy direct use.
        with self._channel_lock:
            self._channel_counts[name] = max(0, self._channel_counts.get(name, 0) + (1 if active else -1))
            self._channel_intent_publishers[name].publish(Bool(data=self._channel_counts[name] > 0))

    def _wait_channel_turn(self, robot, cancel_when):
        started = self._now()
        attempted = set()
        retreat_attempted_at = {}
        yielding_owner = None
        retreat_distance = 0.
        with self._lock:
            docking = (getattr(self,'_docking_owner',None) == robot.id and not robot.current_task_id)
            if docking:
                # A waiting return cannot hold the docking turn needed by
                # the corridor's physical owner to evacuate that corridor.
                self._docking_owner = None
        if not hasattr(self, '_channel_yield_waits'):
            self._channel_yield_waits = {}
        try:
            while rclpy.ok():
                if cancel_when is not None and cancel_when():
                    return False, retreat_distance
                if not self._locomotion_available(robot.id, self._now()):
                    return False, retreat_distance
                owner = self._channel_exit_owner(robot.id)
                now = self._now()
                if (owner is not None and owner not in attempted and
                        (now < retreat_attempted_at.get(owner, -math.inf) or
                         now - retreat_attempted_at.get(owner, -math.inf) >= 10.)):
                    self._channel_yield_waits[robot.id] = owner
                    retreat_attempted_at[owner] = now
                    yielding_owner = owner
                    retreat_distance += self._retreat_from_channel_queue(robot, owner, cancel_when)
                    snapshot = getattr(self, '_channel_snapshot', ({}, 0))[0]
                    configured = bool(snapshot.get('maneuvers', {}).get('retreat_waypoints'))
                    refuge = (self._channel_maneuver(robot, 'retreat_waypoints') if configured
                              else self.charger_pose.get(robot.id))
                    pose = self._live_xy(robot.id)
                    if refuge is not None and pose is not None and math.hypot(
                            pose[0] - refuge.x, pose[1] - refuge.y) <= .35:
                        attempted.add(owner)
                held, stamp = self._channel_wait_states.get(robot.id, (True, -float('inf')))
                if (not held and 0 <= time.monotonic() - stamp <= .5 and
                        (yielding_owner is None or not self._channel_owner_holds_turn(yielding_owner))):
                    with self._lock:
                        if not docking or getattr(self,'_docking_owner',None) in (None,robot.id):
                            if docking:
                                self._docking_owner = robot.id
                            return True, retreat_distance
                now = self._now()
                if now < started:
                    started = now
                if now - started > self.channel_wait_timeout:
                    self.get_logger().warn(f"{robot.id}: corridor reservation wait timed out")
                    return False, retreat_distance
                time.sleep(.05)
            return False, retreat_distance
        finally:
            self._channel_yield_waits.pop(robot.id, None)

    def _navigate(self, robot: RobotState, target: Pose2D, tag: str,
                  allow_traffic=True, cancel_when=None):
        self._set_channel_active(robot.id, True)
        try:
            result = self._navigate_impl(robot, target, tag, allow_traffic, cancel_when)
            if result[0] and robot.type == 'g1_humanoid' and not self._is_task_point(tag):
                # A new path starts from the actual settled biped pose, rather
                # than inheriting the previous leg's walking/turning transient.
                if not self._task_point_dwell(robot, 3.):
                    return False, result[1], result[2]
                pose = self._live_xy(robot.id)
                if pose is not None:
                    robot.pose = Pose2D(*pose)
            return result
        finally:
            self._set_channel_active(robot.id, False)
            request = getattr(getattr(self, 'traffic', None), 'encounter', None)
            if request is not None and request.winner == robot.id:
                request.winner_finished.set()

    def _navigate_impl(self, robot: RobotState, target: Pose2D, tag: str,
                       allow_traffic=True, cancel_when=None):
        """Drive to `target`, breaking standoffs along the way.

        Returns (success, distance_travelled_m, energy_wh).

        A leg is retried up to stall_max_recoveries times: each time the robot
        stops making progress the current goal is preempted, the robot reverses
        a short distance, and the same goal is re-sent with a fresh plan. Only
        if it still cannot make headway does the leg fail, which hands the task
        back to the scheduler - the failure path that was already there.
        """
        start = Pose2D(robot.pose.x, robot.pose.y, robot.pose.yaw)
        position_only = self._is_task_point(tag)
        straight = start.distance_to(target)
        # Distance the recoveries themselves add; folded into the leg's report
        # so the fleet energy/distance totals stay honest.
        extra_dist = 0.0
        recoveries = 0
        traffic_retries = 0
        peer_wait_started = None
        if not hasattr(self, '_nav_targets'):
            self._nav_targets = {}

        while True:
            self._nav_targets[robot.id] = (target.x, target.y)
            if cancel_when is not None and cancel_when():
                return False, extra_dist, robot.energy_for(extra_dist, robot.current_payload_kg)
            handle, err = self._send_goal(robot, target, tag)
            if err == "unavailable":
                if robot.type == "g1_humanoid" or not getattr(self, "simulate_when_nav_unavailable", False):
                    self.get_logger().error(f"{robot.id}: Nav2 unavailable; physical navigation required")
                    return False, 0.0, 0.0
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
                request = getattr(getattr(self, "traffic", None), "encounter", None)
                if request is not None and request.winner == robot.id and request.parked.is_set():
                    request.resumed.set()
            getattr(self, '_nav_result_errors', {}).pop(robot.id, None)
            status, stalled, distance = self._wait_for_result(
                robot, target, handle.get_result_async(),
                allow_traffic=allow_traffic and traffic_retries < 2,
                cancel_when=cancel_when,
                position_only=position_only,
            )
            planner_error = getattr(self, '_nav_result_errors', {}).pop(robot.id, 0)
            with self._lock:
                self.active_goal.pop(robot.id, None)
            request = getattr(getattr(self, "traffic", None), "encounter", None)
            if request is not None and request.winner == robot.id and not isinstance(status, Encounter):
                request.held.set()

            if status == "channel_wait":
                self._cancel_navigation(handle)
                self.get_logger().info(f"{robot.id}: waiting for corridor reservation")
                cleared, retreat_distance = self._wait_channel_turn(robot, cancel_when)
                extra_dist += retreat_distance
                if not cleared:
                    return False, extra_dist, robot.energy_for(extra_dist, robot.current_payload_kg)
                # Waiting is not a stall/recovery and consumes no recovery retry.
                continue

            if status == 'task_arrived':
                self._cancel_navigation(handle)
                pose = self._task_arrival_pose(robot, target)
                if pose is None:
                    return False, extra_dist, robot.energy_for(extra_dist, robot.current_payload_kg)
                robot.pose = Pose2D(*pose)
                driven = straight + extra_dist
                return True, driven, robot.energy_for(driven, robot.current_payload_kg)

            if status == "traffic_timeout":
                self._cancel_navigation(handle)
                return False, extra_dist, robot.energy_for(extra_dist, robot.current_payload_kg)

            if isinstance(status, Encounter):
                self._cancel_navigation(handle)
                traffic_retries += 1
                if robot.id == status.winner:
                    status.held.set()
                    self.get_logger().info(f"{robot.id}: holding for {status.loser} to pull aside")
                    deadline = self._now()+45.0
                    while not status.parked.is_set() and self._now() < deadline and rclpy.ok():
                        time.sleep(.1)
                    if not status.parked.is_set():
                        self.traffic.finish(status, self._now())
                else:
                    extra_dist += self._yield_to_peer(robot, status)
                continue

            # Occupied targets, search timeouts and disconnected paths can be
            # temporary while a nearby vehicle traverses the only passage.
            # Keep the real goal pending instead of reversing into the wall.
            # This wait has a finite ROS-time budget and never accepts arrival.
            if (status == 6 and planner_error in (205, 206, 207, 208)
                    and self._has_nearby_route_peer(robot, target)):
                now = self._now()
                if peer_wait_started is None or now < peer_wait_started:
                    peer_wait_started = now
                    self.get_logger().info(
                        f'{robot.id}: waiting for physical route clearance on {tag} '
                        f'(Nav2 {planner_error}, at most 180 ROS seconds)')
                if now - peer_wait_started < 180.:
                    self._cancel_navigation(handle)
                    if not self._dwell(5.):
                        return False, extra_dist, robot.energy_for(extra_dist, robot.current_payload_kg)
                    continue
                if (tag.endswith(':pickup') or tag.endswith(':leave_charger')) and robot.current_payload_kg == 0.:
                    if not hasattr(self,'_temporary_pickup_blocks'):
                        self._temporary_pickup_blocks = {}
                    self._temporary_pickup_blocks[robot.id] = 'peer blocks a full-body pickup route'
                    self._cancel_navigation(handle)
                    return False,extra_dist,robot.energy_for(extra_dist,robot.current_payload_kg)

            # Apply the existing general-goal arrival policy to Nav2 aborts
            # before deciding to reverse. Task points retain their 15 cm check.
            if status == 6 and self._locomotion_available(robot.id, self._now()):
                stalled = True
            if stalled and distance <= self.stall_arrive_radius_m and not position_only:
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

            # STATUS_ABORTED is also recoverable; otherwise an MPPI abort
            # immediately burns every task requeue while the same jam remains.
            if status == 6 and self._locomotion_available(robot.id, self._now()):
                stalled = True
            if stalled:
                recoveries += 1
                xy = self._live_xy(robot.id) or (target.x, target.y, 0.0)
                self.get_logger().warn(
                    (f"{robot.id}: Nav2 aborted; " if status == 6 else
                     f"{robot.id}: no progress for {self.stall_timeout_sec:.0f}s ")
                    + f"on {tag} at ({xy[0]:.2f},{xy[1]:.2f}), {distance:.2f} m "
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
                # One fleet recovery at a time prevents simultaneous retreat
                # into the same pocket. Waiting robots have cancelled their goals.
                lock = getattr(self, "_recovery_lock", None)
                if lock is None:
                    extra_dist += self._back_off(robot, recoveries)
                else:
                    with lock:
                        current = self._live_xy(robot.id)
                        if not self._recover_heading(robot):
                            if current is not None and self._nearest_teammate_m(robot.id, current[0], current[1]) < 2.0:
                                extra_dist += self._back_off(robot, recoveries)
                        # Give the previous retreat a short clearance window.
                        self._dwell(5.0 if status == 6 else 2.0)
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
            if position_only:
                pose = self._task_arrival_pose(robot, target)
                if pose is None:
                    return False, 0.0, 0.0
                robot.pose = Pose2D(*pose)
            else:
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

    def _defer_task(self, robot, task, reason, distance_m=0., energy_wh=0.):
        """An unladen robot blocked by a peer waits without spending failure retries."""
        if robot.current_payload_kg > 0. or task.state not in (TaskState.ASSIGNED,TaskState.TO_PICKUP):
            raise ValueError('Only an unladen pickup attempt can be deferred')
        if not all(math.isfinite(v) and v >= 0. for v in (distance_m,energy_wh)):
            raise ValueError('Invalid deferred motion accounting')
        with self._lock:
            robot.odom_distance_m += distance_m
            robot.energy_consumed_wh += energy_wh
            task.state = TaskState.PENDING
            task.assigned_robot = ''
            task.assigned_at_sec = None
            if task.id not in self.pending:
                self.pending.append(task.id)
            robot.current_task_id = ''
            robot.status = RobotStatus.IDLE
            self._task_preflight_cache.pop((robot.id,task.id),None)
            self._rolling.invalidate('temporary_peer_block')
        self.get_logger().info(f'{task.id}: deferred without failure: {reason}')

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

    def _task_point_dwell(self, robot, seconds):
        publisher = getattr(self, '_task_hold_publishers', {}).get(robot.id)
        if publisher is not None:
            publisher.publish(Bool(data=True))
        try:
            return self._dwell(seconds)
        finally:
            if publisher is not None:
                publisher.publish(Bool(data=False))

    def _dwell(self, seconds: float) -> bool:
        """Sleep for a simulated dwell period (loading / unloading).

        Sleeping in small steps rather than one long call keeps the node
        responsive to Ctrl-C during a long load.
        """
        if seconds <= 0:
            return True
        remaining, previous = seconds, self._now()
        while remaining > 0 and rclpy.ok():
            time.sleep(.05)
            now = self._now()
            remaining = seconds if now < previous else remaining - (now - previous)
            previous = now
        return remaining <= 0

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
            if not self._locomotion_available(name, now):
                continue
            standby = getattr(self, 'idle_clearance_pose', {}).get(name)
            if (standby is not None
                    and self._at_clearance(r, standby)
                    and (self._wheel_clearance_required(r) or self._should_clear_idle_g1(r)
                         or (r.type == 'g1_humanoid' and self._work_remains())
                         or getattr(self, 'wheel_dispatch_order', []))):
                continue
            threshold = self.dock_when_idle_sec
            clearing = self._should_clear_channel(name)
            clearing_idle = self._should_clear_idle_g1(r) or self._wheel_clearance_required(r)
            if clearing or clearing_idle:
                threshold = min(threshold, .5)
            if self._on_task_point(r, self.clear_task_point_radius_m):
                threshold = min(threshold, self.clear_task_point_sec)
            if now - started < threshold:
                continue
            if not self.return_to_charger or (self._work_for(r) and not clearing and not clearing_idle):
                continue
            home = self.charger_pose.get(name)
            if self._wheel_clearance_required(r):
                home = standby
            if home is None or (self._at_clearance(r,home) if clearing_idle else self._at_charger(r,home)):
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
            1 for task in self.tasks.values()
            if task.state in (TaskState.ASSIGNED, TaskState.TO_PICKUP,
                              TaskState.LOADING, TaskState.TO_DROPOFF)
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
            "g1_humanoid": (0.25, 0.90, 0.65),
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

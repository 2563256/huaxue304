"""
Tests for the two behaviours layered on top of the scheduler: breaking
standoffs, and going home to the charger after every task.

These run without a simulation and almost without ROS. FleetManager is built
with __new__ and hand-populated with just the attributes the methods under test
touch, so nothing here needs a node, a parameter server or an executor - only
the module import, which does need rclpy present.

Run with:
    cd src/hetero_fleet_scheduler && /usr/bin/python3 -m pytest test/ -v
"""

import os
import sys
import threading
import time
import math
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from hetero_fleet_scheduler.fleet_manager import FleetManager  # noqa: E402
from hetero_fleet_scheduler.task_model import (  # noqa: E402
    Pose2D,
    RobotState,
    RobotStatus,
    Task,
    TaskState,
)


# --------------------------------------------------------------------- fakes


class FakeFuture:
    def __init__(self):
        self._done = False
        self._result = None

    def done(self):
        return self._done

    def set_result(self, value):
        self._result = value
        self._done = True

    def result(self):
        return self._result


def resolved(value):
    f = FakeFuture()
    f.set_result(value)
    return f


class _Logger:
    """Collects the manager's log lines instead of printing them."""

    lines = []

    def info(self, msg):
        self.lines.append(("info", str(msg)))

    def warn(self, msg):
        self.lines.append(("warn", str(msg)))

    def error(self, msg):
        self.lines.append(("error", str(msg)))


class FakeGoalHandle:
    """One goal handed to the scripted action server."""

    def __init__(self, kind, goal):
        self.kind = kind
        self.goal = goal
        self.accepted = True
        self.canceled = False
        self.result = FakeFuture()

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.canceled = True
        return resolved(None)


class ScriptedNav:
    """Nav2 action-client stand-in driven by one word per accepted goal.

      "drive"          the robot makes progress and the goal SUCCEEDS
      "wedged"         the robot does not move and the goal never completes
      "arrive_silent"  the robot reaches the goal and the goal never completes
      "wander"         the robot drives around in a big circle, never closer,
                       and the goal never completes
      "align"          the robot arrives, then turns on the spot to line up
                       with the goal heading, and the goal never completes

    "wedged" is the deadlock the watchdog exists for - two vehicles nose to
    nose, or one pinned by the collision monitor. The result future is simply
    never resolved, so the only thing that can end that goal is the manager
    giving up on it.

    "arrive_silent" is the second hang it exists for, and the one that cost the
    most time on the measured runs: the robot is sitting on the pad and the
    navigation stack simply never declares success.
    """

    STEP_SEC = 0.03
    STEPS = 5           # a "drive" goal finishes in ~0.15 s

    def __init__(self, manager, robot_id, script, pose):
        self.manager = manager
        self.robot_id = robot_id
        self.script = list(script)
        self.pose = [float(pose[0]), float(pose[1]), float(pose[2])]
        self.goals = []
        self._lock = threading.Lock()
        manager.live_pose[robot_id] = tuple(self.pose)

    # --- the action client interface Nav2 exposes --------------------------
    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal):
        with self._lock:
            kind = self.script.pop(0) if self.script else "drive"
        handle = FakeGoalHandle(kind, goal)
        self.goals.append(handle)
        if kind == "drive":
            threading.Thread(
                target=self._run, args=(handle,), daemon=True
            ).start()
        elif kind == "arrive_silent":
            threading.Thread(
                target=self._teleport_to_goal, args=(handle,), daemon=True
            ).start()
        elif kind == "wander":
            threading.Thread(
                target=self._wander, args=(handle,), daemon=True
            ).start()
        elif kind == "align":
            threading.Thread(
                target=self._align, args=(handle,), daemon=True
            ).start()
        return resolved(handle)

    # --- the fake robot ----------------------------------------------------
    def _run(self, handle):
        target = handle.goal.pose.pose.position
        x, y, yaw = self.manager.live_pose[self.robot_id]
        for _ in range(self.STEPS):
            time.sleep(self.STEP_SEC)
            if handle.canceled:
                return
            x += (target.x - x) * 0.6
            y += (target.y - y) * 0.6
            self.pose[0], self.pose[1] = x, y
            self.manager.live_pose[self.robot_id] = (x, y, yaw)
        self.manager.live_pose[self.robot_id] = (target.x, target.y, yaw)
        handle.result.set_result(SimpleNamespace(status=4))

    def _teleport_to_goal(self, handle):
        """Arrive, then never let the goal complete."""
        target = handle.goal.pose.pose.position
        time.sleep(self.STEP_SEC)
        _, _, yaw = self.manager.live_pose[self.robot_id]
        self.manager.live_pose[self.robot_id] = (target.x, target.y, yaw)

    def _wander(self, handle):  # noqa: ARG002 - the goal never completes
        """Drive a wide circle: lots of motion, no progress towards anything."""
        cx, cy, yaw = self.manager.live_pose[self.robot_id]
        radius, theta = 2.5, 0.0
        for _ in range(300):          # ~9 s, long enough to outlast a timeout
            time.sleep(self.STEP_SEC)
            theta += 0.6
            self.manager.live_pose[self.robot_id] = (
                cx + radius * math.cos(theta),
                cy + radius * math.sin(theta),
                yaw,
            )

    def _align(self, handle):
        """Sit on the goal position and rotate towards the goal heading."""
        target = handle.goal.pose.pose.position
        x, y, yaw = self.manager.live_pose[self.robot_id]
        self.manager.live_pose[self.robot_id] = (target.x, target.y, yaw)
        for _ in range(60):
            time.sleep(self.STEP_SEC)
            yaw -= 0.08
            self.manager.live_pose[self.robot_id] = (target.x, target.y, yaw)


# ------------------------------------------------------------- manager build


def make_manager(nav=None, return_to_charger=True, charger=(5.0, 5.0, 0.0)):
    """A FleetManager stripped down to what the standoff/charger code needs."""
    m = FleetManager.__new__(FleetManager)
    m._lock = threading.Lock()
    m._occupancy = None                      # every slot counts as free floor
    m.get_logger = _Logger
    # _apply_battery_drain reads one declared parameter; stand in for the
    # node's parameter server so these tests never need rclpy.init().
    m.get_parameter = lambda name: SimpleNamespace(
        value={"battery_drain_per_meter_pct": 0.25}[name]
    )
    m.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: None)
    )
    m.slot_clearance = {"r1": 0.25}
    m.charger_pose = {"r1": Pose2D(*charger)}
    m.live_pose = {}
    m.active_goal = {}
    m.pending = []
    m.tasks = {}
    m.min_battery_pct = 15.0
    m.return_to_charger = return_to_charger
    m.dock_when_idle_sec = 20.0
    m.clear_task_point_sec = 5.0
    m.clear_task_point_radius_m = 1.2
    m._idle_since = {"r1": None}
    m.charger_arrive_radius_m = 0.35
    m.stall_timeout_sec = 0.25
    m.stall_move_eps_m = 0.05
    m.stall_yaw_weight = 0.30
    m.stall_wander_m = 1.20
    m.stall_arrive_radius_m = 0.30
    m.stall_backoff_m = 0.50
    m.stall_backoff_growth = 0.6
    m.stall_backoff_max_m = 1.30
    m.stall_backoff_sec = 0.60
    m.stall_max_recoveries = 2
    m.stall_enabled = True
    m.robots = {
        "r1": RobotState(
            id="r1", type="light_diff_amr", pose=Pose2D(*charger),
            status=RobotStatus.CHARGING,
            battery_capacity_wh=480.0, max_speed=0.70,
        )
    }
    m.nav_clients = {"r1": nav} if nav is not None else {}
    return m


def place(m, pose):
    """Put the fake robot, and both of its poses, at the same place.

    robot.pose is the manager's leg-by-leg bookkeeping and live_pose is the
    odometry-relayed ground truth; a test that moves one without the other is
    testing a robot that does not exist.
    """
    m.live_pose["r1"] = tuple(float(v) for v in pose)
    m.robots["r1"].pose = Pose2D(*pose)


def messages_containing(text):
    return [msg for _, msg in _Logger.lines if text in msg]


def setup_function(_fn):
    _Logger.lines = []


# ------------------------------------------------------------------- tests


def make_task(tid, payload_kg, capability="", state=TaskState.PENDING):
    return Task(id=tid, pickup=Pose2D(), dropoff=Pose2D(),
                payload_kg=payload_kg, required_capability=capability,
                state=state)


def test_work_remains_ignores_a_task_no_robot_can_serve():
    """demo_10's 200 kg job sits in the queue for ever; it must not keep the
    fleet out of its chargers."""
    m = make_manager()
    robot = m.robots["r1"]
    robot.capabilities = ["heavy_payload"]
    robot.payload_capacity_kg = 150.0
    m.tasks["t_impossible"] = make_task("t_impossible", 200.0, "heavy_payload")
    m.pending = ["t_impossible"]
    assert m._work_remains() is False
    # ...but a job it *can* take does count.
    m.tasks["t_ok"] = make_task("t_ok", 10.0, "heavy_payload")
    m.pending = ["t_ok"]
    assert m._work_remains() is True


def test_a_robot_standing_on_a_pad_goes_home_before_the_general_timeout():
    """Finishing a job leaves the robot ON the pad it just used; that is the
    one place it must not loiter, so it clears out far sooner."""
    m = make_manager()
    robot = m.robots["r1"]
    robot.status = RobotStatus.IDLE
    robot.payload_capacity_kg = 150.0
    m.tasks["t1"] = make_task("t1", 10.0)
    m.tasks["t1"].pickup = Pose2D(7.0, 7.0)          # a pad, nowhere near the bay
    place(m, (7.0, 7.0, 0.0))
    assert m._on_task_point(robot, m.clear_task_point_radius_m) is True
    docked = []
    m._start_docking = lambda r: docked.append(r.id)

    m._now = lambda: 0.0
    m._update_idle_timers()
    m._now = lambda: m.clear_task_point_sec + 1.0    # just past the pad timeout
    m._update_idle_timers()
    assert docked == ["r1"]


def test_a_robot_parked_away_from_the_pads_waits_the_full_timeout():
    m = make_manager()
    robot = m.robots["r1"]
    robot.status = RobotStatus.IDLE
    m.tasks["t1"] = make_task("t1", 10.0)
    m.tasks["t1"].pickup = Pose2D(7.0, 7.0)
    place(m, (-6.0, -6.0, 0.0))                      # off in a corner
    assert m._on_task_point(robot, m.clear_task_point_radius_m) is False
    docked = []
    m._start_docking = lambda r: docked.append(r.id)
    m._now = lambda: 0.0
    m._update_idle_timers()
    m._now = lambda: m.clear_task_point_sec + 1.0
    m._update_idle_timers()
    assert docked == []                              # too soon
    m._now = lambda: m.dock_when_idle_sec + 1.0
    m._update_idle_timers()
    assert docked == ["r1"]


def test_work_remains_counts_a_robot_that_is_out_on_a_leg():
    m = make_manager()
    m.robots["r1"].status = RobotStatus.TO_PICKUP
    assert m._work_remains() is True


def test_work_remains_does_not_count_a_robot_that_is_already_docking():
    """Otherwise the first robot to dock would strand the other three."""
    m = make_manager()
    m.robots["r1"].status = RobotStatus.CHARGING
    assert m._work_remains() is False


def test_idle_timer_docks_only_after_the_threshold():
    m = make_manager()
    place(m, (7.0, 7.0, 0.0))          # well away from the bay at (5,5)
    m.robots["r1"].status = RobotStatus.IDLE
    docked = []
    m._start_docking = lambda robot: docked.append(robot.id)

    m._now = lambda: 0.0
    m._update_idle_timers()            # first tick only starts the clock
    assert docked == []
    m._now = lambda: m.dock_when_idle_sec - 1.0
    m._update_idle_timers()
    assert docked == []
    m._now = lambda: m.dock_when_idle_sec + 1.0
    m._update_idle_timers()
    assert docked == ["r1"]


def test_no_docking_while_there_is_still_work_to_do():
    m = make_manager()
    place(m, (7.0, 7.0, 0.0))
    robot = m.robots["r1"]
    robot.status = RobotStatus.IDLE
    robot.payload_capacity_kg = 150.0
    m.tasks["t_ok"] = make_task("t_ok", 10.0)
    m.pending = ["t_ok"]
    docked = []
    m._start_docking = lambda r: docked.append(r.id)
    m._now = lambda: 10_000.0          # idle for ever
    m._update_idle_timers()
    assert docked == []


def test_a_robot_already_on_its_bay_is_not_sent_there_again():
    m = make_manager()
    place(m, (5.0, 5.0, 0.0))          # exactly on the bay
    m.robots["r1"].status = RobotStatus.IDLE
    docked = []
    m._start_docking = lambda r: docked.append(r.id)
    m._now = lambda: 10_000.0
    m._update_idle_timers()
    assert docked == []


def test_wait_for_result_detects_a_robot_that_stops_moving():
    m = make_manager()
    m.live_pose["r1"] = (0.0, 0.0, 0.0)
    robot = m.robots["r1"]
    never = FakeFuture()
    t0 = time.time()
    status, stalled, distance = m._wait_for_result(robot, Pose2D(9.0, 0.0), never)
    elapsed = time.time() - t0
    assert stalled is True
    assert status is None
    assert distance == pytest.approx(9.0, abs=0.05)
    # It must fire on the stall timeout, not hang waiting for a goal that will
    # never finish.
    assert elapsed < 3.0


def test_wait_for_result_returns_normally_while_the_robot_is_driving():
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"], pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    goal = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=2.0, y=0.0))
        )
    )
    handle = nav.send_goal_async(goal).result()
    status, stalled, _ = m._wait_for_result(
        m.robots["r1"], Pose2D(2.0, 0.0), handle.get_result_async()
    )
    assert stalled is False
    assert status == 4


def test_goal_error_treats_heading_as_progress():
    """The last metre of a docking manoeuvre is mostly rotation."""
    m = make_manager()
    target = Pose2D(1.0, 0.0, 0.0)
    assert m._goal_error((1.0, 0.0, 0.0), target) == pytest.approx(0.0)
    # On the goal position but 90 degrees out: worth more than the threshold.
    off = m._goal_error((1.0, 0.0, math.pi / 2), target)
    assert off > m.stall_move_eps_m
    # ...and monotone in the heading error, so turning towards the goal always
    # registers as progress.
    assert m._goal_error((1.0, 0.0, 0.8), target) \
        > m._goal_error((1.0, 0.0, 0.4), target)


def test_aligning_on_the_spot_is_not_a_stall():
    """A robot turning to line up with the pad must not be reversed off it."""
    m = make_manager()
    # On the pad, well out of alignment, turning towards the goal yaw.
    nav = ScriptedNav(m, "r1", ["align"], pose=(0.0, 0.0, 2.8))
    place(m, (0.0, 0.0, 2.8))
    goal = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))
        )
    )
    handle = nav.send_goal_async(goal).result()
    outcome = {}

    def wait():
        outcome["r"] = m._wait_for_result(
            m.robots["r1"], Pose2D(0.0, 0.0, 0.0), handle.get_result_async()
        )

    thread = threading.Thread(target=wait, daemon=True)
    thread.start()
    thread.join(3 * m.stall_timeout_sec)
    assert thread.is_alive(), (
        "a robot lining itself up on the pad was called wedged"
    )


def test_wait_for_result_lets_a_genuine_detour_run():
    """Motion alone must not be penalised - only lack of motion towards the
    goal counts, and a robot far from its last progress point is driving."""
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["wander"], pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    goal = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=50.0, y=50.0))
        )
    )
    handle = nav.send_goal_async(goal).result()
    outcome = {}

    def wait():
        outcome["r"] = m._wait_for_result(
            m.robots["r1"], Pose2D(50.0, 50.0), handle.get_result_async()
        )

    thread = threading.Thread(target=wait, daemon=True)
    thread.start()
    thread.join(4 * m.stall_timeout_sec)
    # Several stall timeouts of driving in circles, and still not called wedged.
    assert thread.is_alive(), (
        "the watchdog false-stalled on a robot that was genuinely driving"
    )

def test_navigate_accepts_an_arrival_the_navigator_never_confirmed():
    """The measured killer: parked ON the pad, no success ever reported."""
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["arrive_silent"], pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    robot.status = RobotStatus.TO_PICKUP
    target = Pose2D(4.0, 0.0, 0.0)
    ok, dist, _ = m._navigate(robot, target, "t1:pickup")
    assert ok is True
    # It accepted the arrival instead of reversing away from the pad.
    assert len(nav.goals) == 1
    assert nav.goals[0].canceled is True
    assert messages_containing("accepting the arrival")


def test_navigate_does_not_accept_an_arrival_it_has_not_reached():
    """A robot wedged 2 m short must still be reversed, not declared arrived."""
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["wedged", "drive", "drive"], pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    robot.status = RobotStatus.TO_PICKUP
    ok, _, _ = m._navigate(robot, Pose2D(4.0, 0.0, 0.0), "t1:pickup")
    assert ok is True
    assert len(nav.goals) == 3                 # 1 wedged + reverse + retry
    assert messages_containing("reversing")


def test_back_off_reverses_and_reports_the_distance_driven():
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"], pose=(3.0, 1.0, 0.0))
    place(m, (3.0, 1.0, 0.0))
    m.nav_clients["r1"] = nav
    moved = m._back_off(m.robots["r1"])
    assert moved > 0.0
    goal = nav.goals[-1].goal.pose.pose.position
    # Reversing from (3,1) facing +x must go to smaller x on the same y.
    assert goal.x < 3.0
    assert abs(goal.y - 1.0) < 1e-6


def test_back_off_avoids_reversing_into_a_teammate():
    """The jam this map produces is a cluster, so retreating must spread it."""
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"], pose=(3.0, 1.0, 0.0))
    place(m, (3.0, 1.0, 0.0))
    # A teammate sitting exactly where the straight reverse would end up.
    m.live_pose["r2"] = (2.5, 1.0, 0.0)
    m.nav_clients["r1"] = nav
    moved = m._back_off(m.robots["r1"])
    goal = nav.goals[-1].goal.pose.pose.position
    assert moved > 0.0
    # It must not drop the robot on top of r2.
    assert ((goal.x - 2.5) ** 2 + (goal.y - 1.0) ** 2) ** 0.5 > 0.35


def test_back_off_retreats_further_on_each_retry():
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"] * 4, pose=(3.0, 1.0, 0.0))
    place(m, (3.0, 1.0, 0.0))
    m.nav_clients["r1"] = nav
    first = m._back_off(m.robots["r1"], attempt=1)
    second = m._back_off(m.robots["r1"], attempt=2)
    third = m._back_off(m.robots["r1"], attempt=3)
    assert first < second < third
    # ...but never past the configured ceiling, however many times we retry.
    assert m._back_off(m.robots["r1"], attempt=50) <= m.stall_backoff_max_m


def test_navigate_recovers_from_one_stall_and_still_succeeds():
    m = make_manager()
    #            goal 1 (pickup)   goal 2 (reverse)  goal 3 (retry, arrives)
    nav = ScriptedNav(m, "r1", ["wedged", "drive", "drive"], pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    robot.status = RobotStatus.TO_PICKUP
    ok, dist, energy = m._navigate(robot, Pose2D(4.0, 0.0, 0.0), "t1:pickup")
    assert ok is True
    assert nav.goals[0].kind == "wedged"
    assert nav.goals[0].canceled is True      # preempted before reversing
    assert len(nav.goals) == 3
    # The leg's reported distance covers the detour, not just the straight line.
    assert dist > 4.0
    assert energy > 0.0
    assert robot.pose.x == 4.0


def test_navigate_gives_up_after_the_recovery_budget_is_spent():
    m = make_manager()
    # Every attempt wedges: 1 initial try plus stall_max_recoveries rounds of
    # (reverse + retry), and then the leg fails.
    nav = ScriptedNav(m, "r1", ["wedged"] * 8, pose=(0.0, 0.0, 0.0))
    place(m, (0.0, 0.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    robot.status = RobotStatus.TO_PICKUP
    ok, dist, energy = m._navigate(robot, Pose2D(4.0, 0.0, 0.0), "t1:pickup")
    assert ok is False
    assert (dist, energy) == (0.0, 0.0)
    assert len(nav.goals) == 1 + 2 * m.stall_max_recoveries
    assert messages_containing("still wedged")


def test_drive_home_skips_the_trip_when_the_robot_is_already_on_the_bay():
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"], pose=(5.10, 5.0, 0.0))
    place(m, (5.10, 5.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    m._drive_home(robot)
    assert nav.goals == []                     # nothing was sent
    assert robot.status == RobotStatus.IDLE    # ...and it is still schedulable


def test_drive_home_moves_the_robot_to_the_bay_and_leaves_it_idle():
    m = make_manager()
    nav = ScriptedNav(m, "r1", ["drive"], pose=(1.0, 1.0, 0.0))
    place(m, (1.0, 1.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    m._drive_home(robot)
    assert robot.status == RobotStatus.IDLE
    assert robot.odom_distance_m > 0.0
    goal = nav.goals[-1].goal.pose.pose.position
    assert (goal.x, goal.y) == (5.0, 5.0)


def test_drive_home_never_leaves_a_robot_stuck_in_charging():
    """The finally block is what stops a crash from freezing the fleet."""
    m = make_manager()

    class ExplodingNav(ScriptedNav):
        def send_goal_async(self, goal):
            raise RuntimeError("nav2 died")

    nav = ExplodingNav(m, "r1", [], pose=(1.0, 1.0, 0.0))
    place(m, (1.0, 1.0, 0.0))
    m.nav_clients["r1"] = nav
    robot = m.robots["r1"]
    m._drive_home(robot)                       # must not raise
    assert robot.status == RobotStatus.IDLE
    assert messages_containing("charger leg crashed")

"""
Tests for the greedy cost function.

These run without ROS, which is the whole reason cost.py is kept free of ROS
imports: the interesting logic (who gets which task, and why) can be verified
in a plain pytest process.

Run with:
    cd src/hetero_fleet_scheduler && python3 -m pytest test/ -v
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hetero_fleet_scheduler.cost import (  # noqa: E402
    CostWeights,
    evaluate,
    select_robot,
)
from hetero_fleet_scheduler.task_model import (  # noqa: E402
    Pose2D,
    RobotState,
    RobotStatus,
    Task,
)


def make_robot(rid, caps, payload_kg, speed, pose=(0.0, 0.0), battery=100.0,
               rtype="light_diff_amr"):
    return RobotState(
        id=rid,
        type=rtype,
        pose=Pose2D(*pose),
        status=RobotStatus.IDLE,
        battery_pct=battery,
        battery_capacity_wh=480.0,
        payload_capacity_kg=payload_kg,
        max_speed=speed,
        capabilities=list(caps),
        wh_per_meter_empty=0.85,
        wh_per_meter_kg=0.012,
    )


def make_task(tid="task_001", payload=5.0, cap="light_payload",
              pickup=(10.0, 0.0), dropoff=(10.0, 10.0), deadline=0.0,
              created_at=0.0, priority=1):
    return Task(
        id=tid,
        pickup=Pose2D(*pickup),
        dropoff=Pose2D(*dropoff),
        payload_kg=payload,
        required_capability=cap,
        priority=priority,
        deadline_sec=deadline,
        created_at_sec=created_at,
    )


W = CostWeights()


# --- feasibility ---------------------------------------------------------

def test_capability_mismatch_is_infeasible():
    """A robot without the required capability must never be chosen.

    The payload is kept within capacity on purpose so this test isolates the
    capability check - otherwise both constraints fire and the penalty
    compounds (which test_penalties_accumulate covers).
    """
    light = make_robot("light_1", ["light_payload"], 20.0, 1.0)
    task = make_task(cap="heavy_payload", payload=5.0)

    b = evaluate(light, task, 0.0, W, 15.0)
    assert not b.feasible
    assert b.mismatch == W.capability_mismatch
    assert any("capability" in r for r in b.reasons)


def test_penalties_accumulate_for_multiple_violations():
    """Violating two constraints costs more than violating one."""
    light = make_robot("light_1", ["light_payload"], 20.0, 1.0)

    only_cap = evaluate(light, make_task(cap="heavy_payload", payload=5.0),
                        0.0, W, 15.0)
    both = evaluate(light, make_task(cap="heavy_payload", payload=100.0),
                    0.0, W, 15.0)

    assert both.mismatch == 2 * only_cap.mismatch
    assert len(both.reasons) == 2


def test_payload_over_capacity_is_infeasible():
    """Payload above capacity is rejected even if the capability matches."""
    light = make_robot("light_1", ["light_payload"], 20.0, 1.0)
    task = make_task(payload=35.0, cap="light_payload")

    b = evaluate(light, task, 0.0, W, 15.0)
    assert not b.feasible
    assert any("payload" in r for r in b.reasons)


def test_failed_robot_is_infeasible():
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0)
    r.status = RobotStatus.FAILED
    b = evaluate(r, make_task(), 0.0, W, 15.0)
    assert not b.feasible
    assert any("FAILED" in x for x in b.reasons)


def test_low_battery_excluded_by_select_robot():
    """A robot under the battery floor must not be selected."""
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0, battery=5.0)
    task = make_task()

    chosen, why = select_robot([r], task, 0.0, W, min_battery_pct=15.0)
    assert chosen is None
    assert why is not None
    assert any("battery" in x for x in why.reasons)
    # And the same robot IS chosen once the floor is lowered.
    chosen2, _ = select_robot([r], task, 0.0, W, min_battery_pct=1.0)
    assert chosen2 is not None


# --- selection behaviour -------------------------------------------------

def test_nearest_robot_wins():
    """With everything else equal, the closer robot should be chosen."""
    near = make_robot("near", ["light_payload"], 20.0, 1.0, pose=(9.0, 0.0))
    far = make_robot("far", ["light_payload"], 20.0, 1.0, pose=(-50.0, 0.0))
    task = make_task(pickup=(10.0, 0.0), dropoff=(10.0, 10.0))

    chosen, _ = select_robot([near, far], task, 0.0, W, 15.0)
    assert chosen.id == "near"


def test_faster_robot_wins_when_equidistant():
    """Same distance, higher speed -> lower travel time -> chosen."""
    fast = make_robot("fast", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    slow = make_robot("slow", ["light_payload"], 20.0, 0.25, pose=(0.0, 0.0))
    task = make_task(pickup=(10.0, 0.0), dropoff=(10.0, 10.0))

    chosen, _ = select_robot([fast, slow], task, 0.0, W, 15.0)
    assert chosen.id == "fast"


def test_capability_filters_then_distance_decides():
    """The heavy task must go to the heavy robot, even if it is further away."""
    light = make_robot("light_1", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    heavy = make_robot(
        "heavy_1", ["heavy_payload"], 150.0, 0.6, pose=(50.0, 50.0),
        rtype="heavy_diff_amr",
    )
    task = make_task(payload=120.0, cap="heavy_payload")

    chosen, _ = select_robot([light, heavy], task, 0.0, W, 15.0)
    assert chosen.id == "heavy_1"


def test_busy_robot_is_penalised_by_wait_term():
    """An idle robot should beat an identical one that is busy for a while."""
    idle = make_robot("idle", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    busy = make_robot("busy", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    busy.busy_until_sec = 100.0
    task = make_task(pickup=(1.0, 0.0), dropoff=(1.0, 5.0))

    chosen, _ = select_robot([idle, busy], task, 0.0, W, 15.0)
    assert chosen.id == "idle"

    b_busy = evaluate(busy, task, 0.0, W, 15.0)
    assert b_busy.queue_wait == 100.0


def test_no_robot_available_returns_reason():
    """When nothing can serve the task we must get an explanation, not None."""
    light = make_robot("light_1", ["light_payload"], 20.0, 1.0)
    task = make_task(payload=200.0, cap="heavy_payload")

    chosen, why = select_robot([light], task, 0.0, W, 15.0)
    assert chosen is None
    assert why is not None
    assert why.reasons


# --- cost internals ------------------------------------------------------

def test_travel_time_and_energy_are_positive():
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    task = make_task(pickup=(10.0, 0.0), dropoff=(10.0, 10.0))
    b = evaluate(r, task, 0.0, W, 15.0)

    # distance to pickup 10, loaded leg 10, speed 1.0 -> 20 s
    assert abs(b.travel_time - 20.0) < 1e-6
    assert b.energy > 0.0
    assert b.total > 0.0


def test_lateness_grows_when_deadline_is_tight():
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    loose = make_task(pickup=(10.0, 0.0), dropoff=(10.0, 10.0), deadline=1000.0)
    tight = make_task(pickup=(10.0, 0.0), dropoff=(10.0, 10.0), deadline=1.0)

    assert evaluate(r, loose, 0.0, W, 15.0).lateness == 0.0
    assert evaluate(r, tight, 0.0, W, 15.0).lateness > 0.0


def test_priority_amplifies_lateness():
    """A high-priority late task must be penalised harder than a low one."""
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0, pose=(0.0, 0.0))
    low = make_task(deadline=1.0, priority=1)
    high = make_task(deadline=1.0, priority=5)

    b_low = evaluate(r, low, 0.0, W, 15.0)
    b_high = evaluate(r, high, 0.0, W, 15.0)
    assert b_high.lateness > b_low.lateness


def test_heavy_robot_consumes_more_energy_per_metre():
    """The energy model must reflect that the heavy platform is thirstier."""
    light = make_robot("light_1", ["heavy_payload"], 150.0, 1.0)
    heavy = make_robot("heavy_1", ["heavy_payload"], 150.0, 1.0,
                       rtype="heavy_diff_amr")
    # Give them the heavy AMR's real coefficients.
    heavy.wh_per_meter_empty = 1.80
    heavy.wh_per_meter_kg = 0.020

    task = make_task(payload=100.0, cap="heavy_payload")
    b_light = evaluate(light, task, 0.0, W, 15.0)
    b_heavy = evaluate(heavy, task, 0.0, W, 15.0)
    assert b_heavy.energy > b_light.energy


def test_explain_is_human_readable():
    r = make_robot("light_1", ["light_payload"], 20.0, 1.0)
    ok = evaluate(r, make_task(), 0.0, W, 15.0).explain()
    bad = evaluate(r, make_task(payload=999.0), 0.0, W, 15.0).explain()
    assert "light_1" in ok and "total=" in ok
    assert "INFEASIBLE" in bad


def test_infeasible_total_is_not_diluted_by_soft_terms():
    """An infeasible pairing must not be made cheaper by being nearby."""
    near = make_robot("near", ["light_payload"], 20.0, 1.0, pose=(10.0, 0.0))
    task = make_task(payload=999.0, cap="light_payload")
    b = evaluate(near, task, 0.0, W, 15.0)
    assert b.total == W.capability_mismatch

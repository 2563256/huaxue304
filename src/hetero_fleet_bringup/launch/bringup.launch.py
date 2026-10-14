"""
Top-level bringup for the heterogeneous fleet simulation.

Brings up, in order:
  1. Gazebo + the scanned-map world + the four robots and their controllers
  2. the ros_gz bridge (clock, scans, ground-truth poses)
  3. one Nav2 stack per robot, in per-robot namespaces
  4. RViz2 with the fleet layout
  5. the fleet_manager scheduler

Everything downstream waits on the clock, so we stagger the stages rather than
starting them simultaneously and letting TF lookups race.

This workspace runs on a scanned indoor map (geomap_colour_noroof) rather than
the warehouse world - see MAP_SWAP.md for how the map, the robot sizes and the
zone coordinates were derived. The timings below are unchanged: this map is
smaller, but the same number of nodes still has to come up.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_gz_launch_module(gz_share):
    """Load gz_sim.launch.py as a module so we can reuse controller_spawners().

    launch/ directories are not importable packages, so the file is loaded by
    path. Reusing the function (rather than duplicating it) keeps the mapping
    from kinematics to controller name in exactly one place.
    """
    path = os.path.join(gz_share, "launch", "gz_sim.launch.py")
    spec = importlib.util.spec_from_file_location("fleet_gz_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    gz_share = get_package_share_directory("hetero_fleet_gz")
    bringup_share = get_package_share_directory("hetero_fleet_bringup")
    desc_share = get_package_share_directory("hetero_fleet_description")

    gz_launch = _load_gz_launch_module(gz_share)
    controller_spawners = gz_launch.controller_spawners

    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_scheduler = LaunchConfiguration("scheduler")

    map_yaml = os.path.join(bringup_share, "maps", "geomap.yaml")
    rviz_cfg = os.path.join(bringup_share, "rviz", "fleet.rviz")

    # --- 1. Gazebo, world, robots, controllers ----------------------------
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # --- 2. ros_gz bridge -------------------------------------------------
    # Reuse the complete bridge launch so clock, scans, scan_fixed relay and
    # Gazebo-pose TF relay are all started together.  Starting only
    # parameter_bridge leaves RViz without map->base TF and the fixed scan
    # topics used by Nav2/RViz.
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bridge.launch.py")
        )
    )

    # --- 3. Nav2 per robot ------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # --- 4. RViz2 ---------------------------------------------------------
    rviz_node = Node(
        package="hetero_fleet_bringup",
        executable="rviz_compat.py",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    # --- 5. Scheduler -----------------------------------------------------
    scheduler = Node(
        package="hetero_fleet_scheduler",
        executable="fleet_manager",
        name="fleet_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "metrics_dir": ParameterValue(LaunchConfiguration("metrics_dir"), value_type=str)}],
        condition=IfCondition(use_scheduler),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("metrics_dir", default_value="",
                                  description="Optional result directory; empty uses workspace metrics/"),
            DeclareLaunchArgument(
                "scheduler", default_value="true",
                description="Start fleet_manager alongside the simulation",
            ),

            gz,

            # Bridge once the clock exists.
            TimerAction(period=8.0, actions=[bridge]),

            # Controller spawners go AFTER the bridge. gz_ros2_control runs its
            # update loop off the ROS /clock, which only exists once the bridge
            # is up; starting the spawners earlier makes the controller switch
            # time out with "Switch controller timed out after 5 seconds".
            TimerAction(period=13.0, actions=controller_spawners(desc_share)),

            # Nav2 needs /clock and TF before it will activate.
            TimerAction(period=30.0, actions=[nav2]),

            # Nav2 starts at 30 s and each of the four stacks is staggered by
            # 30 s inside nav2.launch.py (see the note there: activating them in
            # parallel makes the lifecycle managers' get_state calls time out
            # and aborts the bringup, which is what leaves a robot with no
            # global costmap). The last stack is therefore active around 140 s,
            # so a full bringup takes roughly three minutes on this machine.
            #
            # RViz joins after that: connecting while dozens of Nav2 endpoints
            # are still being created can leave it with an incomplete TF
            # publisher match, and its costmap displays would latch nothing.
            TimerAction(period=160.0, actions=[rviz_node]),

            # The scheduler waits for the same point: even though it only needs
            # the action servers to exist, goals sent before the stacks finish
            # activating are rejected.
            TimerAction(period=150.0, actions=[scheduler]),
        ]
    )

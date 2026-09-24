"""
Start fleet_manager on its own, for use against an already-running simulation.

Useful when you want to restart just the scheduler without tearing down Gazebo
and Nav2, or when you want to replay a different scenario against the same
running world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("hetero_fleet_scheduler")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tasks_file", default_value="",
                description="Task scenario YAML; empty means wait for /task_request",
            ),
            DeclareLaunchArgument(
                "autostart_tasks", default_value="false",
                description="Begin replaying tasks_file immediately",
            ),
            DeclareLaunchArgument("task_interval_sec", default_value="8.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "metrics_dir",
                default_value="",
                description=(
                    "Where metrics.json / tasks.csv are written. Empty means "
                    "the node default, <workspace>/metrics."
                ),
            ),

            Node(
                package="hetero_fleet_scheduler",
                executable="fleet_manager",
                name="fleet_manager",
                output="screen",
                parameters=[
                    {
                        "tasks_file": LaunchConfiguration("tasks_file"),
                        "autostart_tasks": LaunchConfiguration("autostart_tasks"),
                        "task_interval_sec": LaunchConfiguration("task_interval_sec"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "metrics_dir": LaunchConfiguration("metrics_dir"),
                    }
                ],
            ),
        ]
    )

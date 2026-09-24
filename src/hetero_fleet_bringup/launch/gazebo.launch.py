from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    share = FindPackageShare('hetero_fleet_gz').find('hetero_fleet_gz')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'headless': 'false', 'start_controllers': 'true', 'use_sim_time': 'true'}.items())])

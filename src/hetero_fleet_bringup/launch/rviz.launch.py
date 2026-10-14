import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('hetero_fleet_bringup'), 'rviz', 'fleet.rviz')
    actions = []
    transports = os.environ.get('HETERO_FLEET_DDS_TRANSPORTS', '')
    if transports:
        actions.append(SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', transports))
    actions.append(
        Node(package='hetero_fleet_bringup', executable='rviz_compat.py', name='rviz2', output='screen',
             arguments=['-d', cfg], parameters=[{'use_sim_time': True}]))
    return LaunchDescription(actions)

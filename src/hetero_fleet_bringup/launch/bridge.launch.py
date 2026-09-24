import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Sensors and debug topics that exist once per robot. bridge.yaml only carries
# the fleet-wide entries (clock, the ground-truth pose array); these are
# generated from robot_profiles.yaml so the bridge can never fall behind the
# fleet list again.
PER_ROBOT_TOPICS = (
    ("scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
    ("gz_pose", "geometry_msgs/msg/PoseStamped", "gz.msgs.Pose"),
    ("gz_odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry"),
)


def render_bridge_config(cfg_path, profiles):
    """bridge.yaml + one scan/pose/odom block per robot actually in the fleet.

    WHY THIS IS GENERATED. These entries used to be hand-written in
    bridge.yaml, and when light_2 joined the fleet nobody added them for it:
    /light_2/scan was never bridged into ROS at all. The consequences were
    invisible for a while and then fatal - scan_frame_relay had no input, so
    /light_2/scan_fixed and /light_2/robot_obstacles stayed silent, the local
    costmap had no obstacle source (light_2 drove blind), and the collision
    monitor, which refuses to drive while one of its sources has no data,
    pinned the robot in place with "Robot to stop due to invalid source".

    It stayed hidden because the cmd_vel chain was a loop at the time: the
    controller was also publishing straight onto cmd_vel, so the stamper kept
    receiving a live command and the monitor's stop was accidentally
    overridden. Fixing that loop (controller -> cmd_vel_nav) made the monitor
    authoritative and the missing bridge entry immediately visible as a robot
    that would not move.
    """
    with open(cfg_path) as f:
        entries = yaml.safe_load(f) or []
    with open(profiles) as f:
        fleet = yaml.safe_load(f)["fleet"]

    for robot in fleet:
        name = robot["name"]
        for suffix, ros_type, gz_type in PER_ROBOT_TOPICS:
            topic = f"/{name}/{suffix}"
            entries.append({
                "ros_topic_name": topic,
                "gz_topic_name": topic,
                "ros_type_name": ros_type,
                "gz_type_name": gz_type,
                "direction": "GZ_TO_ROS",
            })

    fd, out_path = tempfile.mkstemp(prefix="hetero_fleet_bridge_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(entries, f)
    return out_path


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('hetero_fleet_bringup'), 'config', 'bridge.yaml')
    desc_share = get_package_share_directory('hetero_fleet_description')
    with open(os.path.join(desc_share, 'config', 'robot_profiles.yaml')) as f:
        profiles = yaml.safe_load(f)
    cfg = render_bridge_config(
        cfg, os.path.join(desc_share, 'config', 'robot_profiles.yaml')
    )
    # DDS transport: leave it at the Fast DDS default (shared memory + UDPv4).
    # This fleet runs ~90 nodes on one host, and shared memory is what keeps
    # their service calls and lifecycle bonds from timing out - forcing UDPv4
    # (as these launch files used to) makes the Nav2 bringup abort with
    # "get_state ... async_send_request failed" / "unable to be reached by
    # bond", leaving stacks inactive and their costmaps unpublished.
    # Only set HETERO_FLEET_DDS_TRANSPORTS=UDPv4 if a previous run left stale
    # segments in /dev/shm that you cannot clean up.
    transports = os.environ.get('HETERO_FLEET_DDS_TRANSPORTS', '')
    pre_actions = []
    if transports:
        pre_actions.append(SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', transports))
    actions = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen', parameters=[{'config_file': cfg}]),
        Node(package='hetero_fleet_bringup', executable='scan_frame_relay.py', name='scan_frame_relay', output='screen'),
    ]
    actions = pre_actions + actions

    # --- ground-truth odometry, one relay per robot ------------------------
    # The ros2_control drive controllers run with open_loop: true, i.e. their
    # odometry is the integral of the *commanded* velocity. As soon as a robot
    # is blocked, slips, or turns against an obstacle, that estimate drifts -
    # and Nav2 projects every laser scan through it. The costmap then fills
    # with phantom obstacles at the wrong place, the planner reports
    # "no valid path found" in an aisle that is actually clear, and the robot
    # looks stuck for no visible reason.
    #
    # So Gazebo's own model pose is the single odometry source for the fleet:
    # each robot gets its own relay publishing /<name>/odom plus the
    # odom -> base_footprint transform. The drive controllers must keep
    # enable_odom_tf: false or they will fight this relay over the transform.
    for robot in profiles['fleet']:
        name = robot['name']
        sx, sy, syaw = robot['spawn']
        actions.append(Node(
            package='hetero_fleet_bringup',
            executable='omni_odom_relay.py',
            name=f'{name}_odom_relay',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': name,
                'spawn_x': float(sx),
                'spawn_y': float(sy),
                'spawn_yaw': float(syaw),
            }],
        ))

    for robot in profiles['fleet']:
        name = robot['name']
        x, y, yaw = robot['spawn']
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name=f'map_to_odom_{name}', output='screen',
            arguments=[str(x), str(y), '0', '0', '0', str(yaw), 'map', f'{name}/odom']))
    return LaunchDescription(actions)

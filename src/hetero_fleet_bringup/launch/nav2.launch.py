"""
Start one independent Nav2 stack per robot, each in its own namespace.

Every robot gets its own controller_server, planner_server, bt_navigator,
behavior_server and costmaps, with the parameters selected by its *type*
(light / heavy / omni). That is what makes `heavy_1` inflate obstacles
differently and drive slower than `light_1` while sharing one map.

Map sharing: all stacks point at the same map yaml and the same `map` frame.
Only odom, base_footprint and the costmaps are per-robot.
"""

import os
import json
import math

import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Nav2 servers that need an explicit lifecycle transition. Costmap nodes are
# created by the servers themselves, so they are not listed here.
LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    # Safety layer: sits between the velocity smoother and the wheel
    # controllers and refuses to pass a command that would drive the base into
    # something the lidar can see. It was configured in every params file but
    # never launched, so the fleet had no last-line-of-defence at all.
    "collision_monitor",
]

# The type -> parameter file mapping. This is the only place the three-way
# split is expressed on the launch side.
NAV2_PARAM_FILE = {
    "light_diff_amr": "nav2_params_light.yaml",
    "heavy_diff_amr": "nav2_params_heavy.yaml",
    "omni_mecanum_amr": "nav2_params_omni.yaml",
    "g1_humanoid": "nav2_params_g1.yaml",
}


class ParameterDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        # ROS 2's C parameter parser rejects aliases. Source YAML may share
        # controller/planner defaults, but deployed files must expand them.
        return True


def render_params(src_path, robot_name, dst_dir, kind):
    """Write a per-robot Nav2 params file keyed by the robot's namespace.

    KEY DETAIL: ROS 2 matches a parameter file's top-level key against the
    node's *fully qualified* name. These nodes run under a namespace, so the
    node is /light_1/controller_server and a section named merely
    `controller_server` does NOT match it. The whole file is then silently
    ignored, the node falls back to its built-in defaults, and the symptom is
    bizarre: controller_server loads dwb_core::DWBLocalPlanner (the Nav2
    default) instead of the MPPI controller this file asks for, then fails
    with "No critics defined for FollowPath".

    Nav2's own bringup avoids this with RewrittenYaml(root_key=namespace),
    which prefixes every section. We do the same thing, in plain Python, so
    the mapping stays visible.
    """
    with open(src_path) as f:
        text = f.read()
    text = text.replace("<robot_name>", robot_name)
    text = text.replace("<bringup_share>", get_package_share_directory("hetero_fleet_bringup"))
    body = yaml.safe_load(text)
    validate_lattice_params(body)

    body["bt_navigator"]["ros__parameters"]["mapf_rolling_period_sec"] = 45.0

    # Re-key every top-level section as /<robot_name>/<section>.
    namespaced = {f"/{robot_name}/{section}": params for section, params in body.items()}

    os.makedirs(dst_dir, exist_ok=True)
    # Native preflight planners stay independent from execution commitments.
    planners = body["planner_server"]["ros__parameters"]
    for planner_name in list(planners["planner_plugins"]):
        if planner_name.startswith("Probe"):
            continue
        probe_name = "Probe" + planner_name
        planners[probe_name] = dict(planners[planner_name], mapf_enabled=False)
        planners[planner_name]["mapf_enabled"] = True
        planners["planner_plugins"].append(probe_name)

    dst = os.path.join(dst_dir, f"{robot_name}_{kind}.yaml")
    with open(dst, "w") as f:
        yaml.dump(namespaced, f, Dumper=ParameterDumper,
                  default_flow_style=False, sort_keys=False)
    return dst


def validate_lattice_params(body):
    """Fail before starting servers when control sets or body checks disagree."""
    planners = body["planner_server"]["ros__parameters"]
    for name in planners["planner_plugins"]:
        controller = "TaskFollowPath" if name in ("TaskGridBased", "RetreatGridBased") else "FollowPath"
        _validate_lattice_planner(body, planners[name], controller)


def _validate_lattice_planner(body, planner, controller_name):
    if planner["plugin"] != "hetero_fleet_navigation::SweptLattice":
        return  # Other planner plugins manage their own configuration.
    with open(planner["lattice_filepath"]) as stream:
        metadata = json.load(stream)["lattice_metadata"]
    costmap = body["global_costmap"]["global_costmap"]["ros__parameters"]
    local = body["local_costmap"]["local_costmap"]["ros__parameters"]
    controller = body["controller_server"]["ros__parameters"][controller_name]
    expected = "omni" if controller["motion_model"] == "Omni" else "diff"
    if metadata["motion_model"] != expected:
        raise ValueError("State Lattice control set does not match the controller motion model")
    if not math.isclose(metadata["grid_resolution"], costmap["resolution"], abs_tol=1e-9):
        raise ValueError("State Lattice resolution does not match the global costmap")
    if (yaml.safe_load(costmap["footprint"]) != yaml.safe_load(local["footprint"]) or
            costmap["footprint_padding"] != local["footprint_padding"]):
        raise ValueError("State Lattice global/local footprints disagree")
    if (yaml.safe_load(planner["swept_footprint"]) != yaml.safe_load(local["footprint"]) or
            planner["swept_footprint_padding"] != local["footprint_padding"]):
        raise ValueError("State Lattice swept body does not match the local physical footprint")
    if planner["allow_reverse_expansion"] != (expected == "diff"):
        raise ValueError("Diff lattices need reverse expansion; omni lattices contain reverse moves")


def workspace_dir(package: str) -> str:
    """Absolute path of the overlay's workspace root.

    get_package_share_directory gives <ws>/install/<pkg>/share/<pkg>, so the
    root is four levels up: the per-robot Nav2 params written below then land in
    <ws>/generated/params, next to the controllers rendered by
    hetero_fleet_gz/launch/gz_sim.launch.py and the metrics written by
    fleet_manager, instead of in /tmp. See the note in that launch file for why
    these four lines are duplicated rather than shared.
    """
    share = get_package_share_directory(package)
    return os.path.abspath(os.path.join(share, *([".."] * 4)))


def navigation_library_dir():
    """Resolve the plugin even from a terminal sourced before it was added."""
    try:
        prefix = get_package_prefix("hetero_fleet_navigation")
    except PackageNotFoundError:
        prefix = os.path.join(os.path.dirname(get_package_prefix("hetero_fleet_bringup")),
                              "hetero_fleet_navigation")
    directory = os.path.join(prefix, "lib")
    for library in ["libhetero_stable_route_bt.so", "libhetero_swept_lattice.so", "libhetero_snapshot_obstacle_layer.so"]:
        if not os.path.isfile(os.path.join(directory, library)):
            raise RuntimeError("Build hetero_fleet_navigation before starting Nav2: missing " + library)
    return directory


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("hetero_fleet_bringup")
    desc_share = get_package_share_directory("hetero_fleet_description")

    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_dir = LaunchConfiguration("params_dir").perform(context)

    with open(os.path.join(desc_share, "config", "robot_profiles.yaml")) as f:
        profiles = yaml.safe_load(f)

    types = profiles["robot_types"]
    actions = [Node(package="hetero_fleet_scheduler", executable="mapf_router", output="screen",
                    parameters=[{"use_sim_time": use_sim_time,
                                 "channels_file": LaunchConfiguration("channels_file")}]), Node(
        package="hetero_fleet_scheduler", executable="channel_reservations",
        name="channel_reservations", output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "enabled": LaunchConfiguration("reserve_channels"),
                     "map_file": map_yaml,
                     "channels_file": LaunchConfiguration("channels_file")}])]
    delay = 0.0
    navigator_env = {"LD_LIBRARY_PATH": navigation_library_dir() + os.pathsep +
                     os.environ.get("LD_LIBRARY_PATH", "")}

    # --- one GLOBAL map server, in the root namespace ---------------------
    # All robots share a single map, but the per-robot map_servers below live
    # in their own namespaces, so they publish /light_1/map, /heavy_1/map and
    # so on - nothing ever publishes a plain /map. RViz's Map display watches
    # /map, which is why it showed an empty grid.
    #
    # This extra server publishes the same file at /map purely for
    # visualisation and introspection (ros2 topic echo /map, map_saver, etc.).
    # Each robot's static_layer still uses its own namespaced copy, so
    # navigation behaviour is unchanged.
    actions.append(
        TimerAction(
            # Deliberately late: this root-namespace map server exists only so
            # RViz and `ros2 topic echo /map` have something to look at (each
            # robot has its own map_server for navigation). Starting it with
            # the first Nav2 stack made its lifecycle bond drop and re-form in
            # a loop while the other stacks were activating, and the manager
            # gave up with "Server map_server was unable to be reached ... by
            # bond". It now comes up in the quiet window after the four stacks.
            period=delay + 150.0,
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    namespace="",
                    output="screen",
                    parameters=[
                        {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
                    ],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_map",
                    namespace="",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "autostart": autostart,
                            # The 4 s default is too tight while four Nav2
                            # stacks, four controller spawners and Gazebo are
                            # all starting: the manager gives up with
                            # "Server map_server was unable to be reached after
                            # 4.00s by bond" and the bringup aborts.
                            "bond_timeout": 20.0,
                            "node_names": ["map_server"],
                        }
                    ],
                ),
            ],
        )
    )

    # Safety net for the visualisation map server. Its lifecycle service
    # response is occasionally dropped by DDS on this machine, which leaves the
    # manager waiting forever and the server `inactive` - and an inactive map
    # server publishes nothing, so RViz reports an empty map even though the
    # yaml and pgm are fine. The watchdog is a no-op when the manager worked.
    actions.append(
        TimerAction(
            period=delay + 175.0,
            actions=[
                Node(
                    package="hetero_fleet_bringup",
                    executable="map_server_watchdog.py",
                    name="map_server_watchdog",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ],
        )
    )

    # Same idea for the four navigation stacks. A dropped lifecycle service
    # call makes one robot's manager abort its whole bringup, leaving all nine
    # of its nodes inactive - the stack looks alive but bt_navigator rejects
    # every goal, so every task that robot is given fails. Runs a little later
    # than the map watchdog so the managers have had their chance.
    actions.append(
        TimerAction(
            period=delay + 185.0,
            actions=[
                Node(
                    package="hetero_fleet_bringup",
                    executable="nav2_stack_watchdog.py",
                    name="nav2_stack_watchdog",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ],
        )
    )

    for robot in profiles["fleet"]:
        name = robot["name"]
        rtype = types[robot["type"]]
        sx, sy, syaw = robot["spawn"]
        param_file = NAV2_PARAM_FILE[robot["type"]]

        params = render_params(
            os.path.join(bringup_share, "config", param_file),
            name, params_dir, "nav2",
        )

        ns = f"/{name}"

        # --- per-robot map server -----------------------------------------
        # Each namespace needs its own map_server so that stack's static_layer
        # has a map to subscribe to. These publish /<robot>/map; the global
        # one above publishes /map for RViz.
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        namespace=name,
                        output="screen",
                        parameters=[
                            params,
                            {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
                        ],
                    ),
                ],
            )
        )

        # --- AMCL: each robot localises independently against the shared map.
        #     Without this every robot would need a manually published
        #     map->odom transform.
        actions.append(
            TimerAction(
                period=delay + 1.0,
                actions=[
                    Node(
                        package="nav2_amcl",
                        executable="amcl",
                        name="amcl",
                        namespace=name,
                        output="screen",
                        parameters=[
                            params,
                            {"use_sim_time": use_sim_time,
                             "set_initial_pose": True,
                             "tf_broadcast": False,
                             "initial_pose.x": float(sx),
                             "initial_pose.y": float(sy),
                             "initial_pose.yaw": float(syaw)},
                        ],
                        remappings=[("scan", f"{ns}/scan_fixed")],
                    ),
                ],
            )
        )

        # --- core Nav2 servers -------------------------------------------
        # THE cmd_vel CHAIN MUST BE A CHAIN, NOT A LOOP.
        #
        # As this stack was wired, controller_server and behavior_server both
        # published on `cmd_vel`, velocity_smoother READ `cmd_vel` and wrote
        # `cmd_vel_smoothed`, and collision_monitor read `cmd_vel_smoothed` and
        # wrote back onto `cmd_vel`. That is a closed loop: the collision
        # monitor's own output is fed straight back into the smoother and round
        # again, 20 times a second.
        #
        # The damage is that the collision monitor's slowdown is applied once
        # per lap. While a robot sits inside a slow polygon its command is
        # multiplied by slowdown_ratio (0.4) again and again, so a 0.5 m/s
        # request collapses to a few millimetres per second within a handful of
        # laps. Measured with tools/drive_trace.py on omni_1: cmd_vx between
        # 0.001 and 0.09 m/s for minutes at a time, the robot crawling 3.2 m in
        # 55 s, and the fleet as a whole running at 0.06-0.16 m/s against
        # 0.45-0.70 m/s ratings. omni_1 suffered most because its slowdown
        # polygon is the largest of the three types - which is the "it keeps
        # getting stuck in place" symptom exactly.
        #
        # The fix is Nav2's own convention: the raw command goes onto
        # `cmd_vel_nav`, the smoother owns `cmd_vel_smoothed`, and the
        # channel arbiter filters cmd_vel_smoothed -> cmd_vel_channel, its
        # local watchdog writes cmd_vel_reserved, then collision_monitor owns
        # cmd_vel, the last topic before the wheels.
        # Jazzy's velocity_smoother has no input/output topic parameters, so
        # the split is done with remappings on the three nodes.
        actions.append(TimerAction(period=delay + 2.0, actions=[Node(
            package="hetero_fleet_bringup", executable="channel_gate.py",
            name="channel_gate", namespace=name, output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_name": name}])]))
        raw_cmd = [("cmd_vel", "cmd_vel_nav")]
        servers = [
            ("nav2_controller", "controller_server", "controller_server",
             raw_cmd),
            ("nav2_smoother", "smoother_server", "smoother_server", []),
            ("nav2_planner", "planner_server", "planner_server", []),
            ("nav2_behaviors", "behavior_server", "behavior_server", raw_cmd),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator", []),
            # Reads cmd_vel_nav (its own `cmd_vel` remapped away from the
            # collision monitor's output), writes cmd_vel_smoothed.
            ("nav2_velocity_smoother", "velocity_smoother",
             "velocity_smoother", raw_cmd),
            # Activated after velocity_smoother: it consumes that node's
            # filtered output (cmd_vel_reserved) and republishes on cmd_vel.
            ("nav2_collision_monitor", "collision_monitor",
             "collision_monitor", []),
        ]
        for pkg, exe, node_name, remaps in servers:
            actions.append(
                TimerAction(
                    period=delay + 2.0,
                    actions=[
                        Node(
                            package=pkg,
                            executable=exe,
                            name=node_name,
                            namespace=name,
                            output="screen",
                            parameters=[params, {"use_sim_time": use_sim_time}],
                            remappings=remaps,
                            additional_env=navigator_env if node_name == "bt_navigator" else None,
                        )
                    ],
                )
            )

        # Nav2's velocity_smoother publishes geometry_msgs/Twist. Jazzy's
        # ros2_control drive controllers consume TwistStamped, so adapt the
        # message at the boundary instead of leaving a disconnected cmd_vel
        # chain that makes NavigateToPose abort without moving the robot.
        controller_name = (
            "diff_drive_controller"
            if rtype["kinematics"] == "diff"
            else "mecanum_drive_controller"
        )
        # mecanum_drive_controller subscribes to <controller>/reference, not
        # <controller>/cmd_vel: without this the omni robot has no subscriber
        # on its velocity command and never moves.
        stamper_output = (
            f"/{name}/{controller_name}/reference"
            if rtype["kinematics"] == "mecanum"
            else f"/{name}/{controller_name}/cmd_vel"
        )
        if rtype["kinematics"] != "legged":
            actions.append(
                TimerAction(
                    period=delay + 2.0,
                    actions=[
                        Node(
                            package="hetero_fleet_bringup",
                            executable="cmd_vel_stamper.py",
                            name="cmd_vel_stamper",
                            namespace=name,
                            output="screen",
                            parameters=[
                                {
                                    "use_sim_time": use_sim_time,
                                    "robot_name": name,
                                    "controller_name": controller_name,
                                    "output_topic": stamper_output,
                                    # Take the command from the collision
                                    # monitor's output rather than straight from
                                    # the smoother, so the safety layer is really
                                    # in the chain.
                                    "input_topic": f"/{name}/cmd_vel",
                                    # Physical X-arranged passive rollers use the standard twist signs.
                                    "angular_z_scale": 0.95 if rtype["kinematics"] == "mecanum" else 1.0,
                                    "mirror_y": False,
                                }
                            ],
                        )
                    ],
                )
            )

        # --- lifecycle manager: activates everything in the namespace -----
        actions.append(
            TimerAction(
                period=delay + 3.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        namespace=name,
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                # See the map lifecycle manager above: give the
                                # bond and state changes room to complete while
                                # the whole fleet is starting up.
                                "bond_timeout": 20.0,
                                "node_names": ["map_server", "amcl"] + LIFECYCLE_NODES,
                            }
                        ],
                    )
                ],
            )
        )

        # One stack every 15 s, not every 4 s. Each stack drives ~14 lifecycle
        # nodes through configure/activate, and the lifecycle manager calls
        # <node>/get_state for every one of them. Starting four stacks back to
        # back (56 activations in ~12 s, on top of Gazebo, the bridge and five
        # Python relays) overloads DDS service discovery on this machine: the
        # calls fail with "async_send_request failed", the manager aborts the
        # bringup, and the stack is left with planner_server unconfigured - so
        # the global costmap it owns never activates and publishes nothing,
        # which RViz shows as an empty "global costmap" display. Spreading the
        # stacks out costs ~45 s of startup and makes every bringup succeed.
        delay += 30.0

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("hetero_fleet_bringup")
    scheduler_share = get_package_share_directory("hetero_fleet_scheduler")
    default_map = os.path.join(bringup_share, "maps", "geomap.yaml")

    # See bridge.launch.py: the DDS default (shared memory + UDPv4) is what
    # keeps this many nodes on one host talking; forcing UDPv4 makes the
    # lifecycle bringup time out. Override only via HETERO_FLEET_DDS_TRANSPORTS.
    actions = [
        DeclareLaunchArgument("reserve_channels", default_value="true",
                              description="Enable exclusive corridor reservations"),
        DeclareLaunchArgument("channels_file", default_value=os.path.join(scheduler_share, "config", "channels.yaml")),
    ]
    transports = os.environ.get("HETERO_FLEET_DDS_TRANSPORTS", "")
    if transports:
        actions.append(SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", transports))

    return LaunchDescription(
        actions
        + [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "params_dir",
                default_value=os.path.join(
                    workspace_dir("hetero_fleet_bringup"), "generated", "params"
                ),
                description=(
                    "Where per-robot Nav2 params are rendered. Defaults to "
                    "<workspace>/generated/params."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

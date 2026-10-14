"""
Bring up Gazebo with the scanned-map fleet world and spawn every robot.

This launch file does three things:
  1. starts Gazebo on worlds/geomap_fleet.sdf (via ros_gz_sim's own launch,
     so GZ_SIM_SYSTEM_PLUGIN_PATH is set correctly),
  2. converts each robot's xacro to URDF and spawns it at its configured pose,
  3. starts a robot_state_publisher per robot so the URDF frames exist in TF.

Controllers are intentionally NOT started here - see controller_spawners()
and the note at the bottom of launch_setup(). Use bringup.launch.py for the
full stack, or pass start_controllers:=true to run this file standalone.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_profiles(desc_share):
    """Read the single source of truth for robot specs and fleet composition."""
    path = os.path.join(desc_share, "config", "robot_profiles.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def render_controllers(desc_share, robot_name, controllers_file, dst_dir):
    """Materialise a per-robot controller YAML with the namespace substituted.

    The templates contain a literal <robot_name> placeholder. Each instance gets
    its own rendered copy on disk because controller_manager resolves the
    `parameters` path from inside Gazebo, where we cannot substitute at runtime.
    """
    src = os.path.join(desc_share, "config", controllers_file)
    with open(src) as f:
        text = f.read()
    text = text.replace("<robot_name>", robot_name)

    os.makedirs(dst_dir, exist_ok=True)
    dst = os.path.join(dst_dir, f"{robot_name}_controllers.yaml")
    with open(dst, "w") as f:
        f.write(text)
    return dst


def workspace_dir(package: str) -> str:
    """Absolute path of the overlay's workspace root.

    get_package_share_directory gives <ws>/install/<pkg>/share/<pkg>, so the
    root is four levels up. Rendered artefacts (the controller YAMLs below, and
    the Nav2 params rendered by nav2.launch.py) are written under
    <ws>/generated so a run's files stay with the experiment - next to
    metrics/ - instead of being scattered through /tmp and shared with every
    other user on the machine.

    The same few lines exist in hetero_fleet_bringup/launch/nav2.launch.py:
    launch/ directories are not importable packages, and duplicating four lines
    beats adding a module that every launch file has to load by path.
    """
    share = get_package_share_directory(package)
    return os.path.abspath(os.path.join(share, *([".."] * 4)))


def launch_setup(context, *args, **kwargs):
    desc_share = get_package_share_directory("hetero_fleet_description")
    gz_share = get_package_share_directory("hetero_fleet_gz")

    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_dir = LaunchConfiguration("controllers_dir").perform(context)

    profiles = load_profiles(desc_share)
    types = profiles["robot_types"]
    fleet = profiles["fleet"]

    actions = []
    # DDS transport: default (shared memory + UDPv4). Shared memory is what
    # keeps ~90 nodes on one host from timing out on service calls and
    # lifecycle bonds; forcing UDPv4 (as this file used to, to dodge stale
    # /dev/shm segments) makes the Nav2 bringup abort. If you ever hit real
    # shared-memory collisions after an unclean shutdown, clear /dev/shm or
    # export HETERO_FLEET_DDS_TRANSPORTS=UDPv4.
    transports = os.environ.get("HETERO_FLEET_DDS_TRANSPORTS", "")
    if transports:
        actions.append(SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", transports))

    # Gazebo resolves package:// mesh URIs as model://<package>/... , so both
    # this package's `models` directory (the converted scan) and the parent of
    # the description package's share (the robot shells) have to be on
    # GZ_SIM_RESOURCE_PATH.
    #
    # The mechanism that actually works is the `<gazebo_ros
    # gazebo_model_path=...>` export in each package.xml: ros_gz_sim assembles
    # GZ_SIM_RESOURCE_PATH itself from those exports (plus LD_LIBRARY_PATH) and
    # passes it to the gz process as additional_env, which overrides whatever a
    # launch file sets with SetEnvironmentVariable. The SetEnvironmentVariable
    # below is kept for the case where gz is started some other way (e.g. by
    # hand), and because it makes the intended search path explicit.
    resource_parent = os.path.dirname(desc_share)
    old_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = (os.path.join(gz_share, "models") + os.pathsep + resource_parent
                     + (os.pathsep + old_resource_path if old_resource_path else ""))
    actions.append(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path))

    # --- 1. Gazebo ---------------------------------------------------------
    # Use ros_gz_sim's own launch rather than spawning `gz sim` ourselves.
    # This matters: ros_gz_sim sets GZ_SIM_SYSTEM_PLUGIN_PATH (and
    # GZ_SIM_RESOURCE_PATH) from LD_LIBRARY_PATH plus every package's exported
    # plugin paths. Launching gz directly skips that, and Gazebo then fails
    # with "Failed to load system plugin [gz_ros2_control-system] : Could not
    # find shared library" - the controllers never come up.
    #
    # -r runs the sim immediately, -v 3 keeps the log readable.
    # --render-engine ogre: ogre2 in this install ignores mesh materials and
    # draws the coloured scan as a plain white shell. One has to match the
    # <render_engine> in the world's sensors-system plugin (make_geomap_world.py)
    # or the GUI and the GPU lidars disagree about the engine.
    gz_args = ["-r", "-v", "3", "--render-engine", "ogre", world]
    if headless:
        # Headless still needs a rendering context for the GPU lidars; on a
        # machine with no X display that means EGL.
        gz_args.insert(0, "-s")

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch", "gz_sim.launch.py",
                )
            ),
            launch_arguments={"gz_args": " ".join(gz_args)}.items(),
        )
    )

    # --- 2 + 3. Spawn each robot and start its state publisher -------------
    for robot in fleet:
        name = robot["name"]
        rtype = types[robot["type"]]
        xacro_path = os.path.join(desc_share, "urdf", rtype["urdf"])

        # Render this instance's controller config.
        xml_path = (render_controllers(
            desc_share, name, rtype["controllers"], controllers_dir
        ) if rtype["kinematics"] != "legged" else "")

        robot_desc = Command(
            [
                "xacro ",
                xacro_path,
                " robot_name:=", name,
                " controllers_file:=", xml_path,
            ]
        )

        # robot_state_publisher owns the static frames; the moving
        # odom -> base_footprint transform comes from the drive controller.
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=name,
                output="screen",
                parameters=[
                    {
                        # The xacro output is XML, which is not valid YAML.
                        # Without value_type=str launch_ros tries to parse it
                        # as YAML and aborts with "Unable to parse the value of
                        # parameter robot_description as yaml".
                        "robot_description": ParameterValue(
                            robot_desc, value_type=str
                        ),
                        "use_sim_time": use_sim_time,
                        "frame_prefix": f"{name}/",
                        # Gazebo time restarts at zero on every run. Accept
                        # joint states after that reset instead of retaining a
                        # newer stamp from an earlier simulation instance.
                        "ignore_timestamp": True,
                    }
                ],
            )
        )

        sx, sy, syaw = robot["spawn"]
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{name}",
            output="screen",
            arguments=[
                "-name", name,
                "-topic", f"/{name}/robot_description",
                # 0.05 m: with the rescaled robots (wheel radius 0.068-0.13 m)
                # a 0.15 m drop makes them land hard enough to end up resting
                # on a bumper/guard edge, where the wheels spin without
                # traction and the vehicle will not yaw. A short drop lets them
                # settle straight onto wheels + caster.
                "-x", str(sx), "-y", str(sy), "-z", str(robot.get("spawn_z", 0.05)),
                "-Y", str(syaw),
            ],
        )
        actions.append(spawn)

        # --- 4. Controllers ------------------------------------------------
        # Deliberately NOT started here.
        #
        # gz_ros2_control drives its update loop off the ROS /clock, and
        # controller switches time out ("Switch controller timed out after 5
        # seconds") if /clock is not already flowing. /clock only starts once
        # the ros_gz bridge is up, which happens in bringup.launch.py *after*
        # this file returns. So the spawners are launched by the caller, once
        # the bridge is known to be running.
        #
        # controller_spawners() below builds them; if this file is used
        # standalone (no bridge), pass start_controllers:=true and they are
        # started on a timer instead.

    return actions


def controller_spawners(desc_share, stagger: float = 2.0) -> list:
    """Build one joint_state_broadcaster + drive spawner per robot.

    Called by bringup.launch.py once the bridge is up. Kept here so the
    knowledge of which controller name goes with which kinematics stays in
    one place.

    The spawners are staggered by `stagger` seconds. Firing all of them at
    once races the freshly-created controller_manager services and produces
    "Could not successfully call service .../list_controllers after 3
    attempts" on whichever robot loses the race, so each robot gets its own
    slot instead.

    Each robot gets joint_state_broadcaster first, then its drive controller
    one step later, since the broadcaster should exist before anything
    consumes joint states.
    """
    profiles = load_profiles(desc_share)
    types = profiles["robot_types"]

    actions = []
    slot = 0.0

    for robot in profiles["fleet"]:
        name = robot["name"]
        rtype = types[robot["type"]]
        if rtype["kinematics"] == "legged":
            # G1 plugin owns leg torques and joint states; no wheel controllers.
            continue
        ctrl_name = (
            "diff_drive_controller"
            if rtype["kinematics"] == "diff"
            else "mecanum_drive_controller"
        )

        jsb = Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawner_jsb_{name}",
            namespace=name,
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "controller_manager",
                "--controller-manager-timeout", "90",
            ],
        )
        drive = Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawner_drive_{name}",
            namespace=name,
            output="screen",
            arguments=[
                ctrl_name,
                "--controller-manager", "controller_manager",
                "--controller-manager-timeout", "90",
            ],
        )

        actions.append(TimerAction(period=slot, actions=[jsb]))
        actions.append(TimerAction(period=slot + stagger / 2.0, actions=[drive]))
        slot += stagger

    return actions


def generate_launch_description():
    gz_share = get_package_share_directory("hetero_fleet_gz")
    default_world = os.path.join(gz_share, "worlds", "geomap_fleet.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value=default_world,
                description="SDF world file to load",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false",
                description="Run Gazebo server only (no GUI)",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
                description="Use Gazebo clock as ROS time",
            ),
            DeclareLaunchArgument(
                "controllers_dir",
                default_value=os.path.join(
                    workspace_dir("hetero_fleet_gz"), "generated", "controllers"
                ),
                description=(
                    "Where per-robot controller YAMLs are rendered. Defaults to "
                    "<workspace>/generated/controllers."
                ),
            ),
            DeclareLaunchArgument(
                "start_controllers", default_value="false",
                description=(
                    "Also start the controller spawners on a timer. Leave "
                    "false when driven by bringup.launch.py, which starts "
                    "them after the ros_gz bridge is up."
                ),
            ),
            OpaqueFunction(function=launch_setup),
            OpaqueFunction(function=standalone_controllers),
        ]
    )


def standalone_controllers(context, *args, **kwargs):
    """Optionally start the controllers when this file is used on its own.

    Without the bridge there is no /clock, so give it enough time for the
    models to spawn and the plugins to register before asking for a switch.
    """
    if LaunchConfiguration("start_controllers").perform(context).lower() != "true":
        return []

    desc_share = get_package_share_directory("hetero_fleet_description")
    return [
        TimerAction(period=12.0, actions=controller_spawners(desc_share)),
    ]

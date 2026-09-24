#!/usr/bin/env bash
# Kill every process this workspace's simulations leave behind.
#
# Why this exists: the launch tree does not always take its children down with
# it. The damage is not the leaked CPU, it is the leaked TF. A surviving
# `static_transform_publisher map <robot>/odom` from an earlier run keeps
# publishing the OLD spawn offsets on the same ROS_DOMAIN_ID, so the next run
# has two contradictory map->odom trees at once. Nav2 then resolves the robot
# start pose from whichever publisher it hears last and every goal dies in
# under a second with
#
#     controller_server: Resulting plan has 0 poses in it.
#
# which looks like a planner bug and is not one. The same applies to a second
# `gz sim` still running: the controller spawners attach to whichever instance
# they find.
#
# Usage:  tools/cleanup_fleet.sh          (graceful, then force)

set -u

# Kill every process whose command line matches $1, EXCEPT this script, its
# parent and its grandparent shell.
#
# The exclusions matter more than they look. Run
# `tools/cleanup_fleet.sh && ros2 launch ... bringup.launch.py` and the whole
# chain is one command line: it contains "bringup.launch.py", so the script
# matches the shell that invoked it and SIGKILLs it before the launch ever
# runs (the calling tool reports this as exit 137 with no output at all).
# In a pipeline the caller is two levels up, hence the grandparent.
kill_matching() {
  local pattern="$1" signal="$2" pid
  local skip=" $$ $PPID $(ps -o ppid= -p "$PPID" 2>/dev/null | tr -d ' ') "
  for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    if [[ "$skip" == *" $pid "* ]]; then
      continue
    fi
    kill "-$signal" "$pid" 2>/dev/null
  done
}

echo "==> never run this script in the same command line as the launch it is"
echo "    meant to precede: the pattern would match that command line itself"

echo "==> stopping the launch trees"
kill_matching "bringup.launch.p[y]" INT
kill_matching "gz_sim.launch.p[y]" INT
sleep 6

# The bracket in "…[y]" is deliberate: it keeps the pattern from matching this
# script's own command line, which pkill -f would otherwise kill first.
PATTERNS=(
  "gz sim"
  "static_transform_publishe[r]"
  "_odom_rela[y]"
  "scan_frame_rela[y]"
  "gz_pose_tf_rela[y]"
  "robot_state_publishe[r]"
  "cmd_vel_stampe[r]"
  "random_joint_playe[r]"
  "lifecycle_manage[r]"
  "velocity_smoothe[r]"
  "collision_monito[r]"
  "smoother_serve[r]"
  "behavior_serve[r]"
  "bt_navigato[r]"
  "planner_serve[r]"
  "controller_serve[r]"
  "map_serve[r]"
  "amc[l]"
  "watchdo[g]"
  "fleet_manage[r]"
  "task_publishe[r]"
)

echo "==> stopping the fleet nodes"
for p in "${PATTERNS[@]}"; do
  kill_matching "$p" 9
done
# Gazebo's simulator process, if the launch tree did not reap it.
kill_matching "^ruby$" 9
kill_matching "^gz$" 9

sleep 3

# FastRTPS leaves shared-memory segments behind after an unclean shutdown; a
# stale one makes the next bringup fail to create nodes at all.
/usr/bin/python3 - <<'PY'
import glob
import os

removed = 0
for pattern in ("/dev/shm/sem.fastrtps_*", "/dev/shm/fastrtps_*",
                "/dev/shm/sem.*_shm_*"):
    for path in glob.glob(pattern):
        try:
            os.unlink(path)
            removed += 1
        except OSError:
            pass
print(f"    removed {removed} stale FastRTPS shared-memory segment(s)")
PY

left=$(pgrep -f "hetero_fleet_geomap_ws/instal[l]" | wc -l)
gz_left=$(pgrep -x ruby | wc -l)
echo "==> remaining workspace processes: $left, gz sim instances: $gz_left"
echo "==> done"

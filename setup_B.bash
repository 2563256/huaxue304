# Source this file from a clean terminal before building/running copy B.
_B_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$_B_ROOT/install/setup.bash" ]; then
    source "$_B_ROOT/install/setup.bash"
fi
export PYTHONPATH="$_B_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=44
export GZ_PARTITION=huaxue304_B
unset HETERO_FLEET_DDS_TRANSPORTS
unset _B_ROOT

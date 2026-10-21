# Source this file from a clean terminal before building/running copy C.
_C_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$_C_ROOT/install/setup.bash" ]; then
    source "$_C_ROOT/install/setup.bash"
fi
export PYTHONPATH="$_C_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=68
export GZ_PARTITION=huaxue304_C
unset HETERO_FLEET_DDS_TRANSPORTS
unset _C_ROOT

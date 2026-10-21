# Source this file from a clean terminal before building/running copy A.
_A_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$_A_ROOT/install/setup.bash" ]; then
    source "$_A_ROOT/install/setup.bash"
fi
export PYTHONPATH="$_A_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=43
export GZ_PARTITION=huaxue304_A
unset HETERO_FLEET_DDS_TRANSPORTS
unset _A_ROOT

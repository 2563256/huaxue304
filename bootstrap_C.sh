#!/usr/bin/env bash
set -euo pipefail
C_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$C_WORKSPACE"
/usr/bin/python3 -m venv --without-pip --system-site-packages .venv
mkdir -p work
touch .venv/COLCON_IGNORE work/COLCON_IGNORE
if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o work/get-pip.py
    .venv/bin/python work/get-pip.py
fi
.venv/bin/python -m pip install -r requirements-C.txt
# Build with system Python and system C++ libraries, not the dependency overlay.
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install

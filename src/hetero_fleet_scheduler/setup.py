import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hetero_fleet_scheduler"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "tasks"), glob("config/tasks/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fleet dev",
    maintainer_email="dev@example.com",
    description="Greedy task scheduler and fleet state machine for the heterogeneous AMR fleet.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # The node bringup.launch.py expects.
            "fleet_manager = hetero_fleet_scheduler.fleet_manager:main",
            # Standalone task injector, useful without the full launch.
            "task_publisher = hetero_fleet_scheduler.task_publisher:main",
        ],
    },
)

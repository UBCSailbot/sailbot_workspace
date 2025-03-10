import os
from glob import glob

from setuptools import setup

package_name = "local_pathfinding"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Patrick Creighton",
    maintainer_email="software@ubcsailbot.org",
    description="UBC Sailbot's local pathfinding ROS package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigate = local_pathfinding.node_navigate:main",
            "mock_global_path = local_pathfinding.node_mock_global_path:main",
            "mock_wind_sensor = local_pathfinding.mock_wind_sensor:main",
        ],
    },
)

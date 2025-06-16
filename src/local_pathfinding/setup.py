import os
from glob import glob

from setuptools import setup

package_name = "local_pathfinding"

# List of land subdirectories to include
land_subdirs = ["cache", "csv", "mock", "pkl", "shp"]

# Collect top-level land files (geojson, notebooks, excluding scripts)
top_land_patterns = ["*.geojson"]  # Removed *.py to avoid copying launch scripts
top_land_files = []
for pattern in top_land_patterns:
    top_land_files.extend(glob(os.path.join("land", pattern)))

# Prepare data_files entries
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    (os.path.join("share", package_name), ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
]

# Add top-level land files
if top_land_files:
    data_files.append((os.path.join("share", package_name, "land"), top_land_files))

# Add each land subdirectory
for subdir in land_subdirs:
    src = os.path.join("land", subdir)
    dst = os.path.join("share", package_name, "land", subdir)
    files = glob(os.path.join(src, "*"))
    if files:
        data_files.append((dst, files))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
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
            "mock_wind_sensor = local_pathfinding.node_mock_wind_sensor:main",
            "mock_ais = local_pathfinding.node_mock_ais:main",
            "mock_gps = local_pathfinding.node_mock_gps:main",
            "navigate_observer = local_pathfinding.node_navigate_observer:main",
        ],
    },
)

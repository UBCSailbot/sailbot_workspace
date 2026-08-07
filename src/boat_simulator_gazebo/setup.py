from glob import glob
from os.path import join

from setuptools import find_packages, setup

PACKAGE_NAME = "boat_simulator_gazebo"
REQUIRED_MODULES = ["setuptools", "numpy", "pyproj"]

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    install_requires=REQUIRED_MODULES,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (join("share", PACKAGE_NAME), glob("launch/*_launch.py")),
        (join("share", PACKAGE_NAME, "worlds"), glob("worlds/*.sdf")),
        # Both boat models, meshes included. gazebo_launch.py resolves boat_model_sdf out
        # of the source tree
        (
            join("share", PACKAGE_NAME, "models", "polaris"),
            glob(join("models", "polaris", "model.*")),
        ),
        (
            join("share", PACKAGE_NAME, "models", "polaris", "meshes"),
            glob(join("models", "polaris", "meshes", "*.STL")),
        ),
        (
            join("share", PACKAGE_NAME, "models", "polaris_basic"),
            glob(join("models", "polaris_basic", "model.*")),
        ),
    ],
    zip_safe=True,
    maintainer="Akshanjay Kompelli",
    maintainer_email="software@ubcsailbot.org",
    description="Gazebo (gz-sim) physics backend for UBC Sailbot's Boat Simulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gazebo_physics_bridge_node = "
            + "boat_simulator_gazebo.gazebo_physics_bridge_node:main",
        ],
    },
)

from glob import glob
from os.path import join

from setuptools import find_packages, setup

PACKAGE_NAME = "controller"
REQUIRED_MODULES = ["setuptools", "numpy", "scipy"]

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    install_requires=REQUIRED_MODULES,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (join("share", PACKAGE_NAME), glob("launch/*_launch.py")),
    ],
    zip_safe=True,
    maintainer="Devon Friend",
    maintainer_email="software@ubcsailbot.org",
    description="UBC Sailbot's Controller",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wingsail_ctrl_node = controller.wingsail.wingsail_ctrl_node:main",
        ],
    },
)

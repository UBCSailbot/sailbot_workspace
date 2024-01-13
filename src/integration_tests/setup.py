import os
from glob import glob

from setuptools import setup

package_name = "integration_tests"

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
    maintainer="Henry Huang",
    maintainer_email="henryhuang2001@gmail.com",
    description="UBC Sailbot's integration tests ROS package",
    license="MIT",
    tests_require=[],
    entry_points={
        "console_scripts": ["run = integration_tests.integration_test_node:main"],
    },
)

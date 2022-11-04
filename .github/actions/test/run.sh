#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
./setup.sh
./build.sh OFF  # Do not run static analysis or linting
./test.sh

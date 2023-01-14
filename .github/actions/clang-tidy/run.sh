#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
./setup.sh
./build.sh RelWithDebInfo OFF
./run_clang-tidy.sh

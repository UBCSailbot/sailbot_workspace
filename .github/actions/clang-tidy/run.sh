#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
./setup.sh
./build.sh Debug OFF
./run_clang-tidy.sh

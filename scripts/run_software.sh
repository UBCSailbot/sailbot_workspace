#!/bin/bash
set -e

cd $ROS_WORKSPACE
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup.sh
./scripts/build.sh
source ./install/local_setup.bash
ros2 launch src/global_launch/main_launch.py

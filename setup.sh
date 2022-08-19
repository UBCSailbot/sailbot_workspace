#!/bin/bash
set -e

vcs import < src/ros2.repos src
sudo apt-get update
source /opt/ros/${ROS2_DISTRO}/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y

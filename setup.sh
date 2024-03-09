#!/bin/bash
set -e

sudo apt-get update
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

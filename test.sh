#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
colcon test --merge-install
colcon test-result

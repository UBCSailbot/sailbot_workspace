#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
./setup.sh
cd src
# exclude virtual_iridium because it is a legacy library
VALID_SRC_DIRS=$(ls | grep -v -e virtual_iridium -e new_project.repos)
ament_${LINTER} $VALID_SRC_DIRS

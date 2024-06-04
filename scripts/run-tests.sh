#!/bin/bash
set -e

# give user permissions, required for GitHub Actions
sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE

source /opt/ros/${ROS_DISTRO}/setup.bash
./scripts/setup.sh
./scripts/build.sh
./scripts/test.sh

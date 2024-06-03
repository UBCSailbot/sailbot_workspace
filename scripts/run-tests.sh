#!/bin/bash
set -e

# give user permissions
ls -la $ROS_WORKSPACE
sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE
ls -la $ROS_WORKSPACE

./scripts/setup.sh
./scripts/build.sh
./scripts/test.sh

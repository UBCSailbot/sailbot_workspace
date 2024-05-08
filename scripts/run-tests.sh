#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
./scripts/setup.sh
./scripts/build.sh RelWithDebInfo OFF  # Do not run static analysis or linting
./scripts/test.sh

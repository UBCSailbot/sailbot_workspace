#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
./setup.sh DISABLE_VCS
if [[ "$LINTER" == "clang-tidy" ]]
then
    ./build.sh RelWithDebInfo OFF
    ./run_clang-tidy.sh
else
    ament_${LINTER} src/
fi

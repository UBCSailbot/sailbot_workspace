#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
if [[ $DISABLE_VCS == "true" ]]
then
    ./setup.sh DISABLE_VCS
else
    ./setup.sh
fi
if [[ "$LINTER" == "clang-tidy" ]]
then
    ./build.sh RelWithDebInfo OFF
    ./run_clang-tidy.sh
else
    ament_${LINTER} src/
fi

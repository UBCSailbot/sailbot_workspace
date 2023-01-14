#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ $DISABLE_VCS == "true" ]]
then
    ./setup.sh DISABLE_VCS
else
    ./setup.sh
fi
./build.sh RelWithDebInfo OFF  # Do not run static analysis or linting
./test.sh

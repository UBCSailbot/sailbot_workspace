#!/bin/bash
set -e

if [[ $LOCAL_RUN != "true" ]]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./setup.sh
    ./build.sh RelWithDebInfo OFF
fi

python3 .github/actions/run-in-container/ament_clang_tidy.py build/compile_commands.json --jobs 8

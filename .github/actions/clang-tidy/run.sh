#!/bin/bash
set -e

if [[ $LOCAL_RUN != "true" ]]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./setup.sh
    ./build.sh RelWithDebInfo OFF
fi

python3 .github/actions/clang-tidy/ament_clang_tidy.py compile_commands.json --jobs 8

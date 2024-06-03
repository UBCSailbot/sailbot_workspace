#!/bin/bash
set -e

if [[ $LOCAL_RUN != "true" ]]; then
    # give user permissions
    ls -la $ROS_WORKSPACE
    sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE
    ls -la $ROS_WORKSPACE

    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./scripts/setup.sh
    ./scripts/build.sh
fi

python3 scripts/ament_clang_tidy.py build/compile_commands.json --jobs 8

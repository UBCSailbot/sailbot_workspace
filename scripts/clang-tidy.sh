#!/bin/bash
set -e

if [[ $LOCAL_RUN != "true" ]]; then
    # give user permissions, required for GitHub Actions
    sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE

    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./scripts/setup.sh
    ./scripts/build.sh
fi

python3 scripts/ament_clang_tidy.py build/compile_commands.json --jobs 8

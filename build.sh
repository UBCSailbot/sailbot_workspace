#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        -Wall -Wextra -Wpedantic

# For Clangd
if [ ! -f /workspaces/sailbot_workspace/compile_commands.json ]; then
    ln -s /workspaces/sailbot_workspace/build/compile_commands.json /workspaces/sailbot_workspace/compile_commands.json
fi

#!/bin/bash
set -e

# Set the build type
BUILD_TYPE=${1:-RelWithDebInfo}
STATIC_ANALYSIS=${2:-ON}
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DSTATIC_ANALYSIS=$STATIC_ANALYSIS" "--no-warn-unused-cli"

# For Clangd
if [ ! -f /workspaces/sailbot_workspace/compile_commands.json ]
then
    ln -s /workspaces/sailbot_workspace/build/compile_commands.json
fi

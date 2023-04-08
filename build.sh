#!/bin/bash
set -e

# Set the build type
BUILD_TYPE=${1:-RelWithDebInfo}
STATIC_ANALYSIS=${2:-ON}
UNIT_TEST=${3:-ON}
colcon build \
        --packages-ignore virtual_iridium \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DSTATIC_ANALYSIS=$STATIC_ANALYSIS" "-DUNIT_TEST=$UNIT_TEST" "--no-warn-unused-cli"

# For Clangd
if [[ -f $ROS_WORKSPACE/build/compile_commands.json && ! -f $ROS_WORKSPACE/compile_commands.json ]]
then
    ln -s $ROS_WORKSPACE/build/compile_commands.json $ROS_WORKSPACE
fi

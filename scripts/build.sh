#!/bin/bash
set -e

# Display a help message for using this script
function helpMessage() {
    echo -e "Build ROS package(s) in the Sailbot Workspace."
    echo -e "Usage: ./build.sh [OPTION] ..."
    echo -e "Example: ./build.sh -q -p local_pathfinding\n"
    echo -e "Options (All Optional):"
    echo -e "\t-q: Do a quick build. Specifying this argument will skip static analysis and unit testing when building."
    echo -e "\t-p <PACKAGE_NAME>: Build a ROS package. Can only specify one package. If argument not included, all packages are built."
    echo -e "\t-h: Display this message."
}

# Change to workspace root directory to ensure consistent build location
cd "$(dirname "$0")/.." || exit 1

# Package to build if selecting an individual package
# If still empty after argument parsing, all ROS packages are built
PACKAGE=""

# Parse command-line options (all are optional arguments)
while getopts "hp:q" flag; do
    case ${flag} in
        p )     PACKAGE="${OPTARG}" ;;
        h )     helpMessage; exit 0 ;;
        \? )    echo "Invalid option: -${flag}"; helpMessage; exit 1 ;;
        : )     echo "Option -${flag} requires an argument"; helpMessage; exit 1 ;;
    esac
done

# Assign build type debug
BUILD_TYPE="Debug"

# Whether to run clang-tidy during build (unnecessary since we have separate CI and task)
STATIC_ANALYSIS="OFF"

UNIT_TEST="ON"

# Build ROS packages in src directory
colcon build \
        --base-paths $ROS_WORKSPACE/src \
        ${PACKAGE:+--packages-select $PACKAGE} \
        --packages-ignore virtual_iridium \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DSTATIC_ANALYSIS=$STATIC_ANALYSIS" "-DUNIT_TEST=$UNIT_TEST" "--no-warn-unused-cli"

if [[ "$PACKAGE" == "local_pathfinding" || "$PACKAGE" == "" ]]; then
    if pip3 show ompl 2>&1 | grep -q "not found"; then
        pip3 install ompl==1.7.0
    fi
fi

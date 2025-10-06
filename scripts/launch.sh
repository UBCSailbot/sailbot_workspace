#!/bin/bash
set -e

# Display a help message for using this script
function helpMessage() {
    echo -e "Launch ROS package(s) in the Sailbot Workspace."
    echo -e "Usage: ./launch.sh [OPTION] ..."
    echo -e "Example: ./launch.sh -p local_pathfinding\n"
    echo -e "Options (All Optional):"
    echo -e "\t-p <PACKAGE_NAME>: Launch a ROS package. Can only specify one package. If argument not included, all packages are launched."
    echo -e "\t-f <ADDITIONAL_FLAGS>: Pass in additonal flags for the specified packag . Use -s to see all availible flags for the package."
    echo -e "\t-h: Display this message."
}


# Package to launch if selecting an individual package
# If still empty after argument parsing, all ROS packages are launched.
PACKAGE=""

# Additional arguments for the specified pacakge.
# If still empty after argument parsing, no additional arguments are given.
flags=""

# Parse command-line options (all are optional arguments)
while getopts "hp:f:" flag; do
    case ${flag} in
        p ) PACKAGE="${OPTARG}" ;;
        h ) helpMessage exit 0 ;;
        f ) flags="${OPTARG}";;
        \? )    echo "Invalid option: -${flag}"; helpMessage; exit 1 ;;
        : )     echo "Option -${flag} requires an argument"; helpMessage; exit 1 ;;
    esac
done

source ./install/local_setup.bash

case ${PACKAGE} in
    boat_simulator) ros2 launch boat_simulator main_launch.py $flags ;;
    controller) ros2 launch controller main_launch.py $flags ;;
    local_pathfinding) ros2 launch local_pathfinding main_launch.py $flags ;;
    network_systems) ros2 launch network_systems main_launch.py $flags ;;
    "") ros2 launch src/global_launch/main_launch.py ;;
    *) echo "No ROS launch command found" ;;
esac

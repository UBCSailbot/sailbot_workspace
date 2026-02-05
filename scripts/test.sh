#!/bin/bash
set -e

# Display a help message for using this script
function helpMessage() {
    echo -e "Test ROS package(s) in the Sailbot Workspace."
    echo -e "Usage: ./test.sh [OPTION] ..."
    echo -e "Example: ./test.sh -p local_pathfinding\n"
    echo -e "Options (All Optional):"
    echo -e "\t-p <PACKAGE_NAME>: Test a ROS package. Can only specify one package. If argument not included, all packages are tested."
    echo -e "\t-h: Display this message."
}

function signal_handler() {
    if [ -n "$(pgrep -f virtual_iridium)" ]; then
        pkill -f virtual_iridium
    fi
}

# Package to test if selecting an individual package
# If still empty after argument parsing, all ROS packages are tested
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

# Test failures terminate the script
# Catch the signal so child processes can be cleaned up
trap 'exit' INT TERM
trap 'signal_handler' EXIT

if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi

if [[ "$PACKAGE" == "network_systems" || "$PACKAGE" == "" ]]; then
    # Change MONGODB_PASSWORD password
    export MONGODB_PASSWORD="YE5aQ61K1qnIYJCm"

    NET_DIR=src/network_systems
    if [ -d $NET_DIR ]; then
        ./scripts/run_virtual_iridium.sh &> /dev/null &
        pushd $NET_DIR
        ./scripts/sailbot_db sailbot_db --clear
        ./scripts/sailbot_db sailbot_db --populate
        python3 scripts/rockblock_web_server.py &
        ROCKBLOCK_SERVER_PID=$!
        popd
    fi
    # Set the environment variables
    export ROS_LOG_DIR="/src/network_systems/scripts/can_transceiver.log"
    export RCUTILS_COLORIZED_OUTPUT="1"
    echo "ROS_LOG_DIR is set to: $ROS_LOG_DIR"
fi

colcon test \
    ${PACKAGE:+--packages-select $PACKAGE} \
    --packages-ignore virtual_iridium \
    --merge-install \
    --event-handlers console_cohesion+
colcon test-result
exit 0

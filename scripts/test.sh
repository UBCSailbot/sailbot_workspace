#!/bin/bash
set -e
export LD_LIBRARY_PATH=/usr/share:$LD_LIBRARY_PATH
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

function signal_handler() {
    if [ -n "$(pgrep -f virtual_iridium)" ]; then
        pkill -f virtual_iridium
    fi
}

# Test failures terminate the script
# Catch the signal so child processes can be cleaned up
trap 'exit' INT TERM
trap 'signal_handler' EXIT

if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi

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

colcon test --packages-ignore virtual_iridium --merge-install --event-handlers console_cohesion+
colcon test-result
exit 0

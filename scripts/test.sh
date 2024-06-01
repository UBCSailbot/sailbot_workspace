#!/bin/bash
set -e

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
    popd
fi

colcon test --packages-ignore virtual_iridium --merge-install --event-handlers console_cohesion+
colcon test-result
exit 0

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
echo 'testing'
pwd
ls
ls src
ls src/network_systems/scripts
if [ -d src/network_systems ]; then src/network_systems/scripts/run_virtual_iridium.sh &> /dev/null & fi
colcon test --packages-ignore virtual_iridium --merge-install --event-handlers console_cohesion+
colcon test-result
exit 0

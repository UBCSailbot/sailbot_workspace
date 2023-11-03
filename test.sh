#!/bin/bash
set -e

# Test failures terminate the script
# Catch the signal so child processes can be cleaned up
trap 'exit' INT TERM
trap 'pkill -f virtual_iridium' EXIT

if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
if [ -d src/network_systems ]; then ./src/network_systems/scripts/run_virtual_iridium.sh &> /dev/null & fi
colcon test --packages-ignore virtual_iridium --merge-install --event-handlers console_cohesion+
colcon test-result
exit 0 # explicitly exit to trigger cleanup

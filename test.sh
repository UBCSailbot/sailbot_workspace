#!/bin/bash
set -e

if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
colcon test --packages-ignore virtual_iridium --merge-install --event-handlers console_cohesion+
colcon test-result

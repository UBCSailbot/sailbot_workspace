#!/bin/bash
set -e

if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
colcon test --merge-install
colcon test-result

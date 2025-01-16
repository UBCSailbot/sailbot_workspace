#!/bin/bash
set -e

# give user permissions, required for GitHub Actions
ls -la
sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE

echo "Integration tests started"

for FILE in $ROS_WORKSPACE/src/integration_tests/testplans/*; do
    if [ -f $FILE ]; then
        echo "Running $FILE test plan"
        ros2 run integration_tests run --ros-args -p testplan:=$FILE
    fi
done

echo "Integration tests finished"

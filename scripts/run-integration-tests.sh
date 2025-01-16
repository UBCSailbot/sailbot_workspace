#!/bin/bash
set -e

if [[ $LOCAL_RUN != "true" ]]; then
    # give user permissions, required for GitHub Actions
    sudo chown -R $(whoami):$(whoami) $ROS_WORKSPACE

    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./scripts/setup.sh
    ./scripts/build.sh
    source $ROS_WORKSPACE/install/setup.bash 
fi

echo "Integration tests started"
cd $ROS_WORKSPACE/src/integration_tests

for FILE in $ROS_WORKSPACE/src/integration_tests/testplans/*; do
    if [ -f $FILE ]; then
        echo "Running $FILE test plan"
        ros2 run integration_tests run --ros-args -p testplan:=$FILE
    fi
done

echo "Integration tests finished"

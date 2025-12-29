#!/bin/bash
set -e

EXPECTED_RUNNING_NODES=(
  /cached_fib_node
  /can_transceiver_node
  /local_transceiver_node
  /low_level_control_node
  /navigate_main
  /navigate_observer
  /remote_transceiver_node
  /wingsail_ctrl_node
)

cd $ROS_WORKSPACE
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup.sh
./scripts/build.sh
source ./install/local_setup.bash

ros2 launch src/global_launch/main_launch.py &
LAUNCH_PID=$!

sleep 10

NODES=$(ros2 node list)

# Verify each expected node exists
for node in "${EXPECTED_RUNNING_NODES[@]}"; do
  if ! echo "$NODES" | grep -qx "$node"; then
    echo "ERROR: Missing node $node"
    kill $LAUNCH_PID
    exit 1
  fi
done

echo "All expected nodes are running"

kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null || true

echo "System shut down successfully"

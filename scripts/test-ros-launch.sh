#!/bin/bash
set -e

# the normal directories that colcon would use are not writeable in the github actions env where
# this script is meant to run
COLCON_BUILD_BASE="/tmp/colcon-build"
COLCON_INSTALL_BASE="/tmp/colcon-install"
COLCON_LOG_PATH="/tmp/colcon-log"

export COLCON_BUILD_BASE=$COLCON_BUILD_BASE
export COLCON_INSTALL_BASE=$COLCON_INSTALL_BASE
export COLCON_LOG_PATH=$COLCON_LOG_PATH

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

BUILD_TYPE="Debug"
STATIC_ANALYSIS="OFF"
UNIT_TEST="ON"

cd $ROS_WORKSPACE
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup.sh

colcon build \
        --build-base $COLCON_BUILD_BASE \
        --install-base $COLCON_INSTALL_BASE \
        --log-base $COLCON_LOG_PATH \
        --packages-ignore virtual_iridium \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DSTATIC_ANALYSIS=$STATIC_ANALYSIS" "-DUNIT_TEST=$UNIT_TEST" "--no-warn-unused-cli"


source "$COLCON_INSTALL_BASE/setup.bash"

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

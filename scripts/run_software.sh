#!/bin/bash
set -e

cd $ROS_WORKSPACE
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup.sh
./scripts/build.sh
source ./install/local_setup.bash

# Software restart support (ticket #1099), see LocalTransceiver::restartSoftware().
# A node that wants the whole software system restarted creates the file at this path and then stops `ros2 launch`.
# The loop below relaunches the system when it sees that file. Nodes find the path through the environment
# variable, and refuse to restart if it is not set, i.e. when the system was not started by this script.
# The marker lives in a private directory: a fixed name in the shared /tmp could be pre-created (e.g. as a symlink).
RESTART_DIR="$(mktemp -d)"
export SAILBOT_RESTART_SENTINEL="$RESTART_DIR/restart_requested"
RESTART_DELAY_SEC=2
trap 'rm -rf "$RESTART_DIR"' EXIT

while true; do
    launch_status=0
    # `ros2 launch` only shuts down gracefully on SIGINT, which is ignored if this script was itself started as a
    # background job (`&`). env resets it to the default so a restart request always works.
    env --default-signal=INT ros2 launch src/global_launch/main_launch.py || launch_status=$?

    # Anything other than a restart request ends the script, with the exit code ros2 launch had
    if [[ ! -e "$SAILBOT_RESTART_SENTINEL" ]]; then
        exit "$launch_status"
    fi

    rm -f "$SAILBOT_RESTART_SENTINEL"
    echo "[run_software.sh] Software restart requested, relaunching in ${RESTART_DELAY_SEC}s"
    sleep "$RESTART_DELAY_SEC"
done

#!/bin/bash
set -e

function helpMessage() {
    echo -e "Manually test the controller wingsail trim tab output."
    echo -e "Usage: ./scripts/test_ctrl_manual.sh <speed_kmph> <wind_direction_deg> <desired_heading_deg>"
    echo -e "Example: ./scripts/test_ctrl_manual.sh 10 45 90\n"
    echo -e "Arguments:"
    echo -e "\tspeed_kmph: Apparent wind speed in kmph. Must be >= 0."
    echo -e "\twind_direction_deg: Apparent wind direction in degrees. Must be in [-180, 180]."
    echo -e "\tdesired_heading_deg: Desired heading in degrees. Must be in [-180, 180]."
}

function is_number() {
    [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

function is_greater_than_or_equal() {
    awk -v value="$1" -v min="$2" 'BEGIN { exit !(value >= min) }'
}

function is_in_range() {
    awk -v value="$1" -v min="$2" -v max="$3" 'BEGIN { exit !(value >= min && value <= max) }'
}

function cleanup() {
    trap - INT TERM EXIT
    for pid in "$WIND_PUB_PID" "$HEADING_PUB_PID" "$CONTROLLER_PID"; do
        if [ -n "$pid" ] && ps -p "$pid" > /dev/null; then
            kill "$pid" 2> /dev/null || true
        fi
    done
    wait "$WIND_PUB_PID" "$HEADING_PUB_PID" "$CONTROLLER_PID" 2> /dev/null || true
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    helpMessage
    exit 0
fi

if [ "$#" -ne 3 ]; then
    echo "Error: expected exactly 3 arguments."
    helpMessage
    exit 1
fi

SPEED_KMPH="$1"
WIND_DIRECTION_DEG="$2"
DESIRED_HEADING_DEG="$3"

for value in "$SPEED_KMPH" "$WIND_DIRECTION_DEG" "$DESIRED_HEADING_DEG"; do
    if ! is_number "$value"; then
        echo "Error: all arguments must be numeric."
        helpMessage
        exit 1
    fi
done

if ! is_greater_than_or_equal "$SPEED_KMPH" 0; then
    echo "Error: speed_kmph must be >= 0."
    exit 1
fi

if ! is_in_range "$WIND_DIRECTION_DEG" -180 180; then
    echo "Error: wind_direction_deg must be in [-180, 180]."
    exit 1
fi

if ! is_in_range "$DESIRED_HEADING_DEG" -180 180; then
    echo "Error: desired_heading_deg must be in [-180, 180]."
    exit 1
fi

if [ -f install/local_setup.bash ]; then
    source install/local_setup.bash
fi

# ROS launch writes log files on startup. Default to /tmp when ROS_LOG_DIR is
# unset because /home/ros/.ros/log may be read-only in dev containers.
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "$ROS_LOG_DIR"

trap 'cleanup' INT TERM EXIT

ros2 launch controller main_launch.py &
CONTROLLER_PID=$!

sleep 2

ros2 topic pub /filtered_wind_sensor custom_interfaces/msg/WindSensor \
    "{speed: {speed: ${SPEED_KMPH}}, direction: ${WIND_DIRECTION_DEG}}" -r 1 &
WIND_PUB_PID=$!

ros2 topic pub /desired_heading custom_interfaces/msg/DesiredHeading \
    "{heading: {heading: ${DESIRED_HEADING_DEG}}, steering: 0, sail: true}" -r 1 &
HEADING_PUB_PID=$!

ros2 topic echo --no-daemon /sail_cmd

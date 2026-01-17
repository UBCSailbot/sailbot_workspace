#!/bin/bash
# Load true wind parameters into mock GPS and wind sensor nodes.
#
# Prerequisites:
# 1. Edit wind_params.yaml with desired true wind values.
# 2. Ensure both /mock_gps and /mock_wind_sensor nodes are running.
# 3. Run this script from any directory.
#
# WARNING: This script must be used to set tw_speed_kmph and tw_dir_deg.
# Using ros2 param set directly will cause parameter mismatch and break calculations.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM_FILE="$SCRIPT_DIR/wind_params.yaml"
GPS="/mock_gps"
WIND_SENSOR="/mock_wind_sensor"

echo "Loading parameters from $PARAM_FILE into $GPS and $WIND_SENSOR"

ros2 param load "$GPS" "$PARAM_FILE"
ros2 param load "$WIND_SENSOR" "$PARAM_FILE"

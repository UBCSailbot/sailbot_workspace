#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAM_FILE="$SCRIPT_DIR/wind_params.yaml"
GPS="/mock_gps"
WIND_SENSOR="/mock_wind_sensor"

echo "Loading parameters from $PARAM_FILE into $GPS and $WIND_SENSOR"

ros2 param load "$GPS" "$PARAM_FILE"
ros2 param load "$WIND_SENSOR" "$PARAM_FILE"

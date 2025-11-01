#!/bin/bash

# Check arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <wind_speed_kmh> <angle_degrees>"
    echo "Example: $0 25.5 180"
    exit 1
fi

wind_speed_kmh=$1
angle_deg=$2

# Convert km/h to knots (1 knot = 1.852 km/h)
wind_speed_knots=$(awk "BEGIN {printf \"%.2f\", $wind_speed_kmh / 1.852}")

# Multiply by 10 to preserve 1 decimal place as integer
# e.g., 0.54 knots becomes 5, 14.23 knots becomes 142
wind_speed_scaled=$(awk "BEGIN {printf \"%.0f\", $wind_speed_knots * 10}")
angle_int=$(awk "BEGIN {printf \"%.0f\", $angle_deg}")

# Clamp values to valid ranges (0-65535)
if [ "$wind_speed_scaled" -lt 0 ]; then wind_speed_scaled=0; fi
if [ "$wind_speed_scaled" -gt 65535 ]; then wind_speed_scaled=65535; fi
if [ "$angle_int" -lt 0 ]; then angle_int=0; fi
if [ "$angle_int" -gt 65535 ]; then angle_int=65535; fi

# Convert to 4-digit hexadecimal, then swap bytes for little-endian
speed_hex=$(printf "%04X" "$wind_speed_scaled")
angle_hex=$(printf "%04X" "$angle_int")

# Swap bytes: ABCD -> CDAB (little-endian)
speed_le="${speed_hex:2:2}${speed_hex:0:2}"
angle_le="${angle_hex:2:2}${angle_hex:0:2}"

# Build the CAN message - ANGLE THEN SPEED
can_message="cansend can1 040##0${angle_le}${speed_le}"

echo "Wind Speed: $wind_speed_kmh km/h = $wind_speed_knots knots (scaled x10: $wind_speed_scaled)"
echo "Angle: $angle_deg degrees"
echo "CAN Message: $can_message"
echo ""

# Send the message
eval "$can_message"

if [ $? -eq 0 ]; then
    echo "✓ Message sent successfully"
else
    echo "✗ Failed to send message (is can1 interface up?)"
fi

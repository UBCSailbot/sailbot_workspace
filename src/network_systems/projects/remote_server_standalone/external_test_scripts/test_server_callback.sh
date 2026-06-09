# Call this from an external device

#!/bin/bash
# test_server_callback.sh
# Usage: ./test_server_callback.sh <server_ip> [port]
# Example: ./test_server_callback.sh 203.0.113.42 8081

SERVER_IP=${1:-137.184.35.110}
PORT=${2:-8081}

# Sample RockBLOCK POST data (hex payload)
# Take from a past message received from console
DATA_HEX="120012070d6ff0a53f107e"

# Sample values for other RockBLOCK fields
IMEI="300434065264590"
MOMSN="1234"
TRANSMIT_TIME="2026-03-28T12:00:00Z"
IRIDIUM_LATITUDE="37.7749"
IRIDIUM_LONGITUDE="-122.4194"
IRIDIUM_CEP="9"
SERIAL="5678"

POST_BODY="imei=$IMEI&serial=$SERIAL&momsn=$MOMSN&transmit_time=$TRANSMIT_TIME&iridium_latitude=$IRIDIUM_LATITUDE&iridium_longitude=$IRIDIUM_LONGITUDE&iridium_cep=$IRIDIUM_CEP&data=$DATA_HEX"

curl -v -X POST \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "$POST_BODY" \
  "http://$SERVER_IP:$PORT/sensors"

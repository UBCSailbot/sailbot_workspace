# Call this from an external device

#!/bin/bash
# test_server_callback.sh
# Usage: ./test_server_callback.sh <server_ip> [port]
# Example: ./test_server_callback.sh 203.0.113.42 8081

SERVER_IP=${1:-137.184.35.110}
PORT=${2:-8081}

# Sample RockBLOCK POST data (hex payload)
# Sample payload (old, from Dry land test): DATA_HEX="120012070d6ff0a53f107e"
# Sample payload:
DATA_HEX="0a140d3d0a4542150080f6c21d00002040250000344312070d0000a040105a12080d0000f04010c8011a0a0d66666641159a9999bf222608b96015000044421d0000f6c225000040402d0000344235000000003d00000040450000c0402d0000a441359a9901413d00000c424a180a0a0d9a994342153333f6c20a0a0d66664442159a99f6c2"

# Sample values for other RockBLOCK fields
IMEI="300434065264590"
MOMSN="1234"
TRANSMIT_TIME=$(date -u +%Y-%m-%dT%H:%M:%SZ)
IRIDIUM_LATITUDE="37.7749"
IRIDIUM_LONGITUDE="-122.4194"
IRIDIUM_CEP="9"
SERIAL="5678"

POST_BODY="imei=$IMEI&serial=$SERIAL&momsn=$MOMSN&transmit_time=$TRANSMIT_TIME&iridium_latitude=$IRIDIUM_LATITUDE&iridium_longitude=$IRIDIUM_LONGITUDE&iridium_cep=$IRIDIUM_CEP&data=$DATA_HEX"

curl -v -X POST \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "$POST_BODY" \
  "http://$SERVER_IP:$PORT/sensors"

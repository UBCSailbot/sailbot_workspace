#!/bin/bash
# README under sailbot_workspace/src/network_systems/scripts/

WEBHOOK_SERVER_ENDPOINT=${1:-"http://127.0.0.1:8081"}
VIRTUAL_IRIDIUM_HTTP_SERVER_PORT=${2:-8080}

# Make sure everything is killed on exit
trap 'exit' INT TERM
trap 'kill %; exit 0' EXIT

# Port environment variables are defined in $ROS_WORKSPACE/.devcontainer/base-dev/base-dev.Dockerfile
touch $LOCAL_TRANSCEIVER_TEST_PORT
touch $VIRTUAL_IRIDIUM_PORT

# Setup socat relay pair
socat -d -d -t 0 pty,raw,echo=0,link=$LOCAL_TRANSCEIVER_TEST_PORT pty,raw,echo=0,link=$VIRTUAL_IRIDIUM_PORT &

# Run Virtual Iridium
python2 $ROS_WORKSPACE/src/virtual_iridium/python/Iridium9602.py --webhook_server_endpoint $WEBHOOK_SERVER_ENDPOINT \
    --http_server_port $VIRTUAL_IRIDIUM_HTTP_SERVER_PORT -d $VIRTUAL_IRIDIUM_PORT -m HTTP_POST

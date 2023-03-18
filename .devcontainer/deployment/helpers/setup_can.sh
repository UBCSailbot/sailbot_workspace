#!/bin/bash
# Sets up vcan0 and can0

modprobe vcan
ip link add dev vcan0 type vcan
ip link set up vcan0

# TODO: setup physical CAN
# ip link add dev can0 type can bitrate 12500 # TODO: Don't know what bitrate should be set to
# ip link set up can0

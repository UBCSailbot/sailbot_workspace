#!/bin/bash
# Sets up vcan0 and can0

modprobe vcan # enable vcan driver
ip link add dev vcan0 type vcan
ip link set up vcan0

# Enable CAN interfaces
ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
ip link set can1 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
# Set buffer size
ifconfig can0 txqueuelen 65536
ifconfig can1 txqueuelen 65536

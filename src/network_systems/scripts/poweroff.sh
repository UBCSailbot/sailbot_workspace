#!/bin/bash
# get pid,command of processes started with 'ros2 run' or 'ros2 launch'
# excluding grep command, extract only pids
pids=$(ps -eo pid,command | grep -E 'ros2 run|ros2 launch' | grep -v grep | awk '{print $1}')

for pid in $pids; do
    echo "Sending SIGINT to process $pid"
    kill -2 "$pid"
done
shutdown

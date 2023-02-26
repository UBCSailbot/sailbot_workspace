#!/bin/bash
# build without any extra linting or static analysis

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."

# colcon doesn't like to be run outside the root dir so cd into it
cd $HOST_WORKSPACE_ROOT
./build.sh RelWithDebInfo OFF OFF
cd - > /dev/null

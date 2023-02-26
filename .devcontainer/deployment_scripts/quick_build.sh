#!/bin/bash
# build without any extra linting or static analysis

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."

$HOST_WORKSPACE_ROOT/build.sh RelWithDebInfo OFF OFF

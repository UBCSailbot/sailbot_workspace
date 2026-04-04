#!/bin/bash
set -e

# Example script to run the standalone remote server
DIR="$(dirname "$0")/.."
"$DIR/build/remote_server" 2>&1 | tee "$DIR/server.log"
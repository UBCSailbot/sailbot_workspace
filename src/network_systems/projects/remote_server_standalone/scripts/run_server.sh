#!/bin/bash
set -e

# Example script to run the standalone remote server
DIR="$(dirname "$0")/.."
"$DIR/build/remote_server"

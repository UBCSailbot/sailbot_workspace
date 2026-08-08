#!/bin/bash
set -e

# Run the standalone server test suite
DIR="$(dirname "$0")/.."

# Set MongoDB password for runtime (export for later use)
if [ -z "$MONGODB_PASSWORD" ]; then
  read -sp "Enter MongoDB password (will be used at runtime): " MONGODB_PASSWORD
  echo
fi
export MONGODB_PASSWORD

"$DIR/build/remote_server_tests"

#!/bin/bash
set -e

# Build script for remote_server_standalone
# 1. Install dependencies (if not already installed)
# 2. Generate protobuf files
# 3. Build with CMake

# Step 0: Set MongoDB password for runtime (export for later use)
if [ -z "$MONGODB_PASSWORD" ]; then
  read -sp "Enter MongoDB password (will be used at runtime): " MONGODB_PASSWORD
  echo
fi
export MONGODB_PASSWORD

# Step 1: Install dependencies (optional, comment out if not needed)
# ./scripts/setup_boost.sh
# ./scripts/setup_mongo.sh
# ./scripts/setup_protobuf.sh
# ./scripts/setup_curl.sh

# Step 2: Generate protobuf files
PROTO_DIR="$(dirname "$0")/proto"
SRC_DIR="$(dirname "$0")/src"

protoc -I"$PROTO_DIR" --cpp_out="$SRC_DIR" "$PROTO_DIR"/*.proto
# Only move protobuf-generated headers, not all .h files
mv "$SRC_DIR"/*.pb.h inc/

# Step 3: Build with CMake
BUILD_DIR="$(dirname "$0")/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake ..
make -j$(nproc)

cd ..
echo "Build complete. Run ./scripts/run_server.sh to start the server."

#!/bin/bash
set -e

# Build script for remote_server_standalone
# 1. Install dependencies (if not already installed)
# 2. Generate protobuf files
# 3. Build with CMake

DIR="$(dirname "$0")/.."

# Step 0: Set MongoDB password for runtime (export for later use)
if [ -z "$MONGODB_PASSWORD" ]; then
  read -sp "Enter MongoDB password (will be used at runtime): " MONGODB_PASSWORD
  echo
fi
export MONGODB_PASSWORD

# Step 1: Install dependencies (optional, comment out if not needed)
# "$DIR/scripts/setup_boost.sh"
# "$DIR/scripts/setup_mongo.sh"
# "$DIR/scripts/setup_protobuf.sh"
# "$DIR/scripts/setup_curl.sh"

# Step 2: Generate protobuf files
PROTO_DIR="$DIR/proto"
SRC_DIR="$DIR/src"

protoc -I"$PROTO_DIR" --cpp_out="$SRC_DIR" "$PROTO_DIR"/*.proto
# Only move protobuf-generated headers, not all .h files
mv "$SRC_DIR"/*.pb.h "$DIR/inc/"

# Step 3: Build with CMake
BUILD_DIR="$DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake ..
make -j$(nproc)

cd "$DIR"
echo "Build complete. Run ./scripts/run_server.sh to start the server."

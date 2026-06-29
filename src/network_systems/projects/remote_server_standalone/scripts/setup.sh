#!/bin/bash
set -e

# 1. Update and install basic build tools
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config wget curl gnupg

# 2. Install Boost, Protobuf, and Curl
sudo apt-get install -y libboost-all-dev protobuf-compiler libprotobuf-dev libcurl4-openssl-dev

# 3. Install MongoDB C driver (libmongoc, libbson) from source
MONGOC_VERSION=1.24.4
cd /tmp
curl -LO https://github.com/mongodb/mongo-c-driver/releases/download/${MONGOC_VERSION}/mongo-c-driver-${MONGOC_VERSION}.tar.gz
rm -rf mongo-c-driver-${MONGOC_VERSION}
tar -xzf mongo-c-driver-${MONGOC_VERSION}.tar.gz
cd mongo-c-driver-${MONGOC_VERSION}
mkdir -p cmake-build
cd cmake-build
cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig

# 4. Install MongoDB C++ driver (libmongocxx, libbsoncxx) from source
CXX_VERSION=r4.1.4
cd /tmp
curl -LO https://github.com/mongodb/mongo-cxx-driver/releases/download/${CXX_VERSION}/mongo-cxx-driver-${CXX_VERSION}.tar.gz
rm -rf mongo-cxx-driver-${CXX_VERSION}
tar -xzf mongo-cxx-driver-${CXX_VERSION}.tar.gz
cd mongo-cxx-driver-${CXX_VERSION}
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
make -j$(nproc)
sudo make install
sudo ldconfig

# 5. (Optional) Clean up downloaded files
cd /tmp
rm -rf mongo-c-driver-${MONGOC_VERSION} mongo-c-driver-${MONGOC_VERSION}.tar.gz
rm -rf mongo-cxx-driver-${CXX_VERSION} mongo-cxx-driver-${CXX_VERSION}.tar.gz

# 6. Export PKG_CONFIG_PATH for this session (add to ~/.bashrc for persistence)
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

echo "\nAll dependencies for the standalone server are installed!"
echo "If you want PKG_CONFIG_PATH to persist, add this line to your ~/.bashrc:"
echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH"

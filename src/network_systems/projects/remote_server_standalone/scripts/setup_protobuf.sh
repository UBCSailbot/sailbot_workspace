#!/bin/bash
set -e

# Install protobuf compiler and libraries
sudo apt-get update
sudo apt-get install -y protobuf-compiler libprotobuf-dev

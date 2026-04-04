#!/bin/bash
set -e

# Install Boost (system-wide, or adapt for local if needed)
sudo apt-get update
sudo apt-get install -y libboost-all-dev

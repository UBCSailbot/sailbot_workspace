#!/bin/bash
set -e

# Install GoogleTest (needed to build the test suite)
sudo apt-get update
sudo apt-get install -y libgtest-dev

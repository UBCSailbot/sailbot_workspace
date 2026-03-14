#!/bin/bash
set -e

# Install MongoDB C++ driver dependencies
sudo apt-get update
sudo apt-get install -y libmongocxx-dev libbsoncxx-dev

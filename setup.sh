#!/bin/bash
set -e

vcs import < src/new_project.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

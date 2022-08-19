#!/bin/bash
set -e

echo $USER
echo $HOME
id -u $USER
id -g $USER
vcs import < src/ros2.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

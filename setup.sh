#!/bin/bash
set -e

echo $USER
echo $HOME
id -u $USER
id -g $USER
git clone https://github.com/UBCSailbot/py_pubsub src/py_pubsub
ls -lah src
vcs --version
vcs import < src/ros2.repos src --debug
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

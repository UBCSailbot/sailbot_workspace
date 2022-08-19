#!/bin/bash
set -e

echo $USER
echo $HOME
id -u $USER
id -g $USER
GIT_CURL_VERBOSE=1 GIT_TRACE=1 git ls-remote https://github.com/UBCSailbot/py_pubsub main
ls -lah src
vcs --version
vcs import < src/ros2.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

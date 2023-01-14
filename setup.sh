#!/bin/bash
set -e

if [[ $DISABLE_VCS != "true" ]]
then
    vcs import < src/new_project.repos src
fi
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

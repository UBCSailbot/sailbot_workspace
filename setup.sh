#!/bin/bash
set -e

# Display warning message
function warn() {
    message=$1
    echo -e "\e[1;33m${message}\e[0m"
}

# Import all project repositories
if [[ $DISABLE_VCS != "true" ]]; then
    echo "Importing project repositories..."
    vcs import < src/new_project.repos src
else
    warn "VCS disabled. Skipping project repository imports..."
fi

sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

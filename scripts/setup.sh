#!/bin/bash
#!/usr/bin/env bash
set -e

# Usage:
#   ./install_deps.sh exec   # installs only exec_depend deps
#   ./install_deps.sh        # installs all deps (default)

# Choose dependency mode based on argument
if [ "$1" = "exec" ]; then
    DEP_FLAGS="--dependency-types exec -y"
    echo "Installing only runtime (exec_depend) dependencies..."
else
    DEP_FLAGS="-y"
    echo "Installing all dependency types (build, exec, test, etc.)..."
fi

# Create/overwrite the custom rosdep list file
CUSTOM_ROSDEP_LIST="/etc/ros/rosdep/sources.list.d/20-sailbot.list"
CUSTOM_ROSDEP_FILE="custom-rosdep.yaml"
echo "# sailbot" | sudo tee "$CUSTOM_ROSDEP_LIST" > /dev/null

for DIR in "$ROS_WORKSPACE"/src/*; do
    if [ -d "$DIR" ]; then
        FILE="$DIR/$CUSTOM_ROSDEP_FILE"
        if [ -f "$FILE" ]; then
            echo "Adding $FILE to $CUSTOM_ROSDEP_LIST"
            echo "yaml file://$(realpath "$FILE")" | sudo tee --append "$CUSTOM_ROSDEP_LIST" > /dev/null
        fi
    fi
done

sudo apt-get update
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" $DEP_FLAGS

# Create logging folder for all ROS logs.
LOG_PATH="$ROS_WORKSPACE/src/global_launch/voyage_log"
if [ ! -d "$LOG_PATH" ]; then
    sudo mkdir -p "$LOG_PATH"
fi

# Generate land obstacle data so it is always set up on launch for either on-water testing and offshore launch.
LAND_SCRIPT="$ROS_WORKSPACE/src/local_pathfinding/land/pickle_land_data.py"
PKL_DIR="$ROS_WORKSPACE/src/local_pathfinding/land/pkl"

# Ensure the output dir (and any existing root-owned .pkl files from a prior run)
# are writable by the current user so regeneration does not fail with PermissionError.
sudo mkdir -p "$PKL_DIR"
sudo chown -R "$(id -u):$(id -g)" "$PKL_DIR"

# Before launch (full offshore coastline):
python3 "$LAND_SCRIPT" --source offshore

# Before on-water testing at Jericho Beach (cut to the pier reference):
python3 "$LAND_SCRIPT" --source on_water --cut

source "$HOME/.bashrc"

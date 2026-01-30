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


# Add test_plans to PYTHONPATH if not already present
TEST_PLANS_PATH="/workspaces/sailbot_workspace/src/local_pathfinding/test_plans"
case ":$PYTHONPATH:" in
  *":$TEST_PLANS_PATH:"*) ;;
  *) export PYTHONPATH="$TEST_PLANS_PATH${PYTHONPATH:+:$PYTHONPATH}" ;;
esac


source "$HOME/.bashrc"

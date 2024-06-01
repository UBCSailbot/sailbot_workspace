#!/bin/bash
set -e

# Create/overwrite the custom rosdep list file
CUSTOM_ROSDEP_LIST="/etc/ros/rosdep/sources.list.d/20-sailbot.list"
CUSTOM_ROSDEP_FILE="custom-rosdep.yaml"
echo "# sailbot" | sudo tee $CUSTOM_ROSDEP_LIST > /dev/null
for DIR in $ROS_WORKSPACE/src/*; do
    if [ -d "$DIR" ]; then
        FILE="$DIR/$CUSTOM_ROSDEP_FILE"
        if [ -f $FILE ]; then
            echo "Adding $FILE to $CUSTOM_ROSDEP_LIST"
            echo "yaml file://$FILE" | sudo tee --append $CUSTOM_ROSDEP_LIST > /dev/null
        fi
    fi
done

sudo apt-get update
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

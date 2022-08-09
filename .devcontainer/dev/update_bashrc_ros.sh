echo "" >> ~/.bashrc
echo "# Setup ROS" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/workspaces/sailbot_ros2_workspace" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "# setup ROS 1 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "export ROS_WORKSPACE=/workspaces/catkin_ws" >> ~/.bashrc
echo "source /workspaces/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "# setup ROS 2 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "source /workspaces/sailbot_ros2_workspace/install/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "# Set up X11 forwarding for GUI apps" >> ~/.bashrc
echo "export DISPLAY=:0" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "# setup ROS" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/workspaces/vscode_ros2_workspace" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "# setup ROS 1 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "export ROS_WORKSPACE=/workspaces/catkin_ws" >> ~/.bashrc
echo "source /workspaces/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "# setup ROS 2 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc
echo "if [ -f /workspaces/vscode_ros2_workspace/install/local_setup.bash ]" >> ~/.bashrc
echo "then" >> ~/.bashrc
echo "    source /workspaces/vscode_ros2_workspace/install/local_setup.bash" >> ~/.bashrc
echo "else" >> ~/.bashrc
echo "    echo 'ROS 2 workspace local_setup.bash not found; run the Build task then source ~/.bashrc'" >> ~/.bashrc
echo "fi" >> ~/.bashrc
echo "# place ROS 2 Python site packages first in PYTHONPATH" >> ~/.bashrc
echo "export PYTHONPATH=/opt/ros/${ROS2_DISTRO}/lib/python3.6/site-packages:${PYTHONPATH}" >> ~/.bashrc

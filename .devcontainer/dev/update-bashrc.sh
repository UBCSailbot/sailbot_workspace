echo "" >> ~/.bashrc
echo "# setup ROS 1 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "export ROS_WORKSPACE=$ROS1_WORKSPACE" >> ~/.bashrc
echo "source /opt/ros/$ROS1_DISTRO/setup.bash" >> ~/.bashrc
echo "alias srcraye='unset ROS_DISTRO; source $ROS1_WORKSPACE/devel/setup.bash'" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "# setup ROS 2 environment" >> ~/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> ~/.bashrc
echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> ~/.bashrc
echo "if [ -f $ROS2_WORKSPACE/install/local_setup.bash ]" >> ~/.bashrc
echo "then" >> ~/.bashrc
echo "    source $ROS2_WORKSPACE/install/local_setup.bash" >> ~/.bashrc
echo "    export PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:$PYTHONPATH" >> ~/.bashrc
echo "else" >> ~/.bashrc
echo "    echo 'ROS 2 workspace local_setup.bash not found; run the Build task then run srcnew'" >> ~/.bashrc
echo "fi" >> ~/.bashrc
echo "alias srcnew='unset ROS_DISTRO; source /workspaces/vscode_ros2_workspace/install/local_setup.bash; PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:$PYTHONPATH'" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=$ROS2_WORKSPACE" >> ~/.bashrc

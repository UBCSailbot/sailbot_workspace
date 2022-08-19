echo "" >> /home/$USERNAME/.bashrc
echo "# setup ROS 1 environment" >> /home/$USERNAME/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> /home/$USERNAME/.bashrc
echo "export ROS_WORKSPAcE=$ROS1_WORKSPACE" >> /home/$USERNAME/.bashrc
echo "source /opt/ros/$ROS1_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
echo "alias srcraye='unset ROS_DISTRO; source $ROS1_WORKSPACE/devel/setup.bash'" >> /home/$USERNAME/.bashrc

echo "" >> /home/$USERNAME/.bashrc
echo "# setup ROS 2 environment" >> /home/$USERNAME/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> /home/$USERNAME/.bashrc
echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
echo "if [ -f $ROS2_WORKSPACE/install/local_setup.bash ]" >> /home/$USERNAME/.bashrc
echo "then" >> /home/$USERNAME/.bashrc
echo "    source $ROS2_WORKSPACE/install/local_setup.bash" >> /home/$USERNAME/.bashrc
echo "    export PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:\$PYTHONPATH" >> /home/$USERNAME/.bashrc
echo "else" >> /home/$USERNAME/.bashrc
echo "    echo 'ROS 2 workspace local_setup.bash not found; run the Build task then run srcnew'" >> /home/$USERNAME/.bashrc
echo "fi" >> /home/$USERNAME/.bashrc
echo "alias srcnew='unset ROS_DISTRO; source /workspaces/vscode_ros2_workspace/install/local_setup.bash; PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:\$PYTHONPATH'" >> /home/$USERNAME/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc
echo "export _colcon_cd_root=$ROS2_WORKSPACE" >> /home/$USERNAME/.bashrc

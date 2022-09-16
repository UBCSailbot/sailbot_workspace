echo "" >> $HOME/.bashrc
echo "# source aliases and functions, if they exist" >> $HOME/.bashrc
echo "if [ -f $HOME/.aliases.bash ]; then . $HOME/.aliases.bash; fi" >> $HOME/.bashrc
echo "if [ -f $HOME/.functions.bash ]; then . $HOME/.functions.bash; fi" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# setup ROS 1 environment" >> $HOME/.bashrc
echo "export ROS_WORKSPACE=$ROS1_WORKSPACE" >> $HOME/.bashrc
echo "alias srcraye='unset ROS_DISTRO; source $ROS1_WORKSPACE/devel/setup.bash; echo \"sourcing ROS 1 setup\"'" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# setup ROS 2 environment" >> $HOME/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> $HOME/.bashrc
echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> $HOME/.bashrc
echo "if [ -f $ROS2_WORKSPACE/install/local_setup.bash ]" >> $HOME/.bashrc
echo "then" >> $HOME/.bashrc
echo "    source $ROS2_WORKSPACE/install/local_setup.bash" >> $HOME/.bashrc
echo "    export PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:\$PYTHONPATH" >> $HOME/.bashrc
echo "else" >> $HOME/.bashrc
echo "    echo 'ROS 2 workspace local_setup.bash not found; run the Build task then run srcnew'" >> $HOME/.bashrc
echo "fi" >> $HOME/.bashrc
echo "alias srcnew='unset ROS_DISTRO; source $ROS2_WORKSPACE/install/local_setup.bash; PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:\$PYTHONPATH; echo \"source ROS 2 setup\"'" >> $HOME/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> $HOME/.bashrc
echo "export _colcon_cd_root=$ROS2_WORKSPACE" >> $HOME/.bashrc

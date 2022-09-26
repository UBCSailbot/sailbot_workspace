echo "" >> $HOME/.bashrc
echo "# source aliases and functions, if they exist" >> $HOME/.bashrc
echo "if [ -f $HOME/.aliases.bash ]; then . $HOME/.aliases.bash; fi" >> $HOME/.bashrc
echo "if [ -f $HOME/.functions.bash ]; then . $HOME/.functions.bash; fi" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# setup ROS 1 environment" >> $HOME/.bashrc
echo "export ROS_WORKSPACE=$ROS1_WORKSPACE" >> $HOME/.bashrc
echo "alias srcraye='source $ROS1_WORKSPACE/devel/setup.bash; echo \"sourcing the ROS 1 underlay and overlay of $ROS1_WORKSPACE\"'" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# setup ROS 2 environment" >> $HOME/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> $HOME/.bashrc
echo "export _colcon_cd_root=$ROS2_WORKSPACE" >> $HOME/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> $HOME/.bashrc
echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> $HOME/.bashrc
# the commented out version of srcnew would enable srcnew to work properly when called in a terminal that previously called srcraye
# however, best practice is to source different workspaces in different terminals
echo "alias srcnew=\"if [ -f $ROS2_WORKSPACE/install/local_setup.bash ]; then source $ROS2_WORKSPACE/install/local_setup.bash; echo 'sourcing the ROS 2 overlay of $ROS2_WORKSPACE'; else echo '$ROS2_WORKSPACE/install/local_setup.bash not found: run the Build task (CTRL+SHIFT+B) then srcnew (in the terminal)'; fi\"" >> $HOME/.bashrc
# echo "alias srcnew='unset ROS_DISTRO; source $ROS2_WORKSPACE/install/local_setup.bash; export PYTHONPATH=/opt/ros/$ROS2_DISTRO/lib/python3.6/site-packages:\$PYTHONPATH; echo \"sourcing the overlay in $ROS2_WORKSPACE\"'" >> $HOME/.bashrc

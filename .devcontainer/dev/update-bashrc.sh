echo "" >> $HOME/.bashrc
echo "# source aliases and functions, if they exist" >> $HOME/.bashrc
echo "if [ -f $HOME/.aliases.bash ]; then . $HOME/.aliases.bash; fi" >> $HOME/.bashrc
echo "if [ -f $HOME/.functions.bash ]; then . $HOME/.functions.bash; fi" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# setup ROS environment" >> $HOME/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> $HOME/.bashrc
echo "export _colcon_cd_root=$ROS_WORKSPACE" >> $HOME/.bashrc
echo "unset ROS_DISTRO  # unsetting ROS_DISTRO to silence ROS_DISTRO override warning" >> $HOME/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc
echo "alias srcros=\"if [ -f $ROS_WORKSPACE/install/local_setup.bash ]; then source $ROS_WORKSPACE/install/local_setup.bash; echo 'sourcing the ROS overlay of $ROS_WORKSPACE'; else echo '$ROS_WORKSPACE/install/local_setup.bash not found: run the Build task (CTRL+SHIFT+B) then srcnew (in the terminal)'; fi\"" >> $HOME/.bashrc

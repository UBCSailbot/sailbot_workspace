echo "" >> $HOME/.bashrc
echo "# source aliases and functions, if they exist" >> $HOME/.bashrc
echo "if [ -f $HOME/.aliases.bash ]; then . $HOME/.aliases.bash; fi" >> $HOME/.bashrc
echo "if [ -f $HOME/.functions.bash ]; then . $HOME/.functions.bash; fi" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# add Python bin to PATH" >> $HOME/.bashrc
echo "export PATH=$PATH:$HOME/.local/bin" >> $HOME/.bashrc

echo "" >> $HOME/.bashrc
echo "# set up ROS environment" >> $HOME/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> $HOME/.bashrc
echo "export _colcon_cd_root=$ROS_WORKSPACE" >> $HOME/.bashrc
echo "alias srcros=\"if [ -f $ROS_WORKSPACE/install/local_setup.bash ]; then source $ROS_WORKSPACE/install/local_setup.bash; echo 'sourcing the ROS overlay of $ROS_WORKSPACE'; else echo '$ROS_WORKSPACE/install/local_setup.bash not found: run the Build task (CTRL+SHIFT+B) then srcros (in the terminal)'; fi\"" >> $HOME/.bashrc


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
echo "# customize ROS log format: https://docs.ros.org/en/humble/Concepts/About-Logging.html#environment-variables" >> $HOME/.bashrc
echo "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}:{line_number}]: {message}'" >> $HOME/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc
echo "if [ -f $ROS_WORKSPACE/install/local_setup.bash ]" >> $HOME/.bashrc
echo "then" >> $HOME/.bashrc
echo "    source $ROS_WORKSPACE/install/local_setup.bash" >> $HOME/.bashrc
echo "else" >> $HOME/.bashrc
echo "    echo \"WARNING: Can't find the ROS workspace overlay: build then run 'source $ROS_WORKSPACE/install/local_setup.bash'\"" >> $HOME/.bashrc
echo "fi" >> $HOME/.bashrc

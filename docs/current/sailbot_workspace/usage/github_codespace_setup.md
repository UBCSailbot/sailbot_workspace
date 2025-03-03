# How to run sailbot_workspace on Github Codespaces

Github code spaces is very useful for working in sailbot_workspaces as it removed the overhead of setting up the environment.

1. go to [sailbot_workspace](https://github.com/UBCSailbot/sailbot_workspace) repo on GitHub.

2. click **Code → Codespaces → +**, a new tab will open, wait as the browser IDE sets up.

3. Open `sailbot.code-workspace` and click **Open Workspace**,
wait a moment as the work space sets up, the IDE should have a blue border.

4. press `ctlr + shift + p` and type **Run Build Tasks** then select **Build All**.

5. if any bugs occur with building custom_interfaces package run following command in the terminal:

```
colcon build --packages-select custom_interfaces --merge-install
```

6.. run command in the terminal:

```
source install/setup.sh
```

7. then run command to launch entire system:

```
ros2 launch $ROS_WORKSPACE/src/global_launch/main_launch.py
```

8. if things are working you are done! you can use `ctrl + c` to stop the system.

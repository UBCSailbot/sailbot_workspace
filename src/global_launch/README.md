# Description

- Running `ros2 launch $ROS_WORKSPACE/src/global_launch/main_launch.py` in the terminal will begin the entire system.
- You can utilize different launch arguments to perform different tasks when launching.

# Recording a rosbag file for future analysis

- When utilizing the launch argument `ros2 launch $ROS_WORKSPACE/src/global_launch/main_launch.py record:=true`, a rosbag file is automatically created
- This file records all messages published to the local_pathfinding for future analysis. Note that this setting is defaulted to false, and the `record:=true` function is case sensitive, which means that you must exactly type "true" to create a rosbag.
- By default, files are saved to `workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recordings/`. If you would like to specify where it should be saved, you can use the `save_path` launch argument. This launch argument only works if you also pass in the `record:=true` argument as well.
- If you do specify which path to save it to, note that it already saves to `workspaces/sailbot_workspace/` so all you have to do is specify where inside of the workspace you would like it to be saved. For example, to save to a new test_recordings folder inside of the notebook, you would run `ros2 launch $ROS_WORKSPACE/src/global_launch/main_launch.py record:=true save_path:=notebooks/test_recordings`. In this case, the folder would automatically be created for you as well.
- Usually, you wouldn't want to push rosbag recordings to main, as they take up a large amount of storage. Thus, the session_recordings folder is ignored when pushing. If you create a new folder, make sure you add it to `gitignore` unless you do want to push to main.

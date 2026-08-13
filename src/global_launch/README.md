# Description

<!-- markdownlint-disable-next-line MD013 -->
- Running `ros2 launch global_launch main_launch.py` in the terminal will begin the entire system.
- You can utilize different launch arguments to perform different tasks when
  launching.
  <!-- markdownlint-disable MD013 -->
    - For example, we can look at all the debug statements for all the nodes when we use `log_level:=debug` as our launch argument.
  <!-- markdownlint-enable MD013 -->
  <!-- markdownlint-disable-next-line MD013 -->
- To select the mode, we need use `mode:="development"` or `mode:="production"` or `mode:="sim"` or `mode:="can"` launch arguments
  <!-- markdownlint-disable MD013 -->
    - `development` (default): `local_pathfinding` generates the `filtered_wind_sensor` and `gps` data internally (mock/simulated data) for local testing without hardware. `network_systems` runs for the sake of completeness as `controller` uses one of the publishers in `network_systems's` `can_transciever_node`.
    - `production`: `network_systems` provides the `filtered_wind_sensor` and `gps` data from the real sensors over the network, and `controller` relies on `network_systems`' control publishers/subscribers to drive the boat. Used for on-water operation.
    - `sim`: launches the additional `boat_simulator` package, which simulates the boat (physics, GPS, wind sensors) to run the full software stack end-to-end against a simulated boat on pathfinding's visualizer.
    - `can`: launches the same packages as `production` (`controller`, `local_pathfinding`, `network_systems`), but `network_systems`' `can_transceiver_node` replays a recorded candump-style CSV log instead of talking to real or mocked CAN hardware. Useful for running the full stack against realistic, previously-recorded CAN traffic without a boat. Requires the `can_replay_file` ROS parameter to be set **and** to point at a log that exists; the node fails to start otherwise. Logs are not checked into this repo, so when the default configs are used and the log is not there yet, the launch downloads it with `./scripts/get_mock_can_msg.sh` before starting the node — the first `mode:="can"` run therefore needs network access and pulls a ~32 MB file (see [network_systems' README](../network_systems/README.md#can-log-replay)). way to run the script is to run the equivalent VS
    Code Task: `CTRL + SHIFT + P > Tasks: Run Task > Pull Mock CAN Log > Choose an owt date > csv is pulled, enjoy 😊`
  <!-- markdownlint-enable MD013 -->
- To select the config file, use the `config` launch argument (e.g. `config:=globals.yaml`).
  <!-- markdownlint-disable MD013 -->
    - This is the ROS parameter config filename located in `src/global_launch/config`, and it controls the ROS parameters passed into the ROS nodes. There are three different scenario configurations:
    - `globals.yaml` (default): A default ROS parameter setting.
    - `on_water_globals.yaml`: For on-water testing across `production`, `development`, and `sim`. This must be used when doing on-water testing.
    - `launch_globals.yaml`: For launch in the open ocean for `production`, but can also run in `development` and `sim` for testing.
- To control the pathfinding visualizer, use the `visualizer_mode` launch argument (e.g. `visualizer_mode:=true`).
    - Leave empty (default) to use the value in the config file (`navigate_main.ros__parameters.visualizer_mode` in `globals.yaml`).
    - `true`: Enables the pathfinding visualizer during on-water testing.
    - `false`: Disables the pathfinding visualizer.
- To use mock AIS data during on-water testing, use the `on_water_mock_ais` launch argument (e.g. `on_water_mock_ais:=true`).
    - Leave empty (default) to use the value in the config file (`mock_ais.ros__parameters.on_water_mock_ais` in `globals.yaml`).
    - `true`: Uses mock AIS data during production mode on-water testing.
    - `false`: Uses real AIS data.
- To control whether network system's CAN_transceiver runs in manual mode or not, use the `manual_mode` launch argument. If manual mode is enabled CAN communications between the software and elec are disabled.
    - Leave empty (default) to use the value defined in a config file. If undefined in `globals.yaml` or similar, the value defaults to what is declared in `network_systems/../can_transceiver_ros_intf.cpp` that being `false`.
    - `true`: Enables manual_mode
    - `false`: Disables manual_mode
  <!-- markdownlint-enable MD013 -->

## Example launch commands

<!-- markdownlint-disable MD013 -->
- Default development run (mock data, no recording):
  `ros2 launch global_launch main_launch.py`
- Development run with debug logging:
  `ros2 launch global_launch main_launch.py mode:=development log_level:=debug`
- Simulation run against the boat simulator:
  `ros2 launch global_launch main_launch.py mode:=sim config:=globals.yaml`
- CAN log replay run (effectively runs on-water test in development):
  `ros2 launch global_launch main_launch.py mode:=can config:=on_water_globals.yaml`
- On-water testing (must use the on-water config):
  `ros2 launch global_launch main_launch.py mode:=production config:=on_water_globals.yaml record:=true`
- Open-ocean launch run for production:
  `ros2 launch global_launch main_launch.py mode:=production config:=launch_globals.yaml record:=true`
- Recording a rosbag to a custom folder:
  `ros2 launch global_launch main_launch.py record:=true save_path:=notebooks/test_recordings`
- Full on-water run with a timestamped log file with ros2bag:
  `ros2 launch global_launch main_launch.py record:=true mode:=production config:=on_water_globals.yaml 2>&1 | tee src/global_launch/voyage_log/combined_log_$( date +%F_%T).txt`
- Full Launch run with a timestamped log file with ros2bag:
  `ros2 launch global_launch main_launch.py record:=true mode:=production config:=launch_globals.yaml 2>&1 | tee src/global_launch/voyage_log/combined_log_$( date +%F_%T).txt`
<!-- markdownlint-enable MD013 -->

## Recording a rosbag file for future analysis

<!-- markdownlint-disable MD013 -->
- When utilizing the launch argument `ros2 launch global_launch main_launch.py record:=true`, a rosbag file is automatically created
- This file records all messages published to the local_pathfinding for future analysis. Note that this setting is defaulted to false, and the `record:=true` function is case sensitive, which means that you must exactly type "true" to create a rosbag.
- By default, files are saved to `workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recordings/`. If you would like to specify where it should be saved, you can use the `save_path` launch argument. This launch argument only works if you also pass in the `record:=true` argument as well.
- If you do specify which path to save it to, note that it already saves to `workspaces/sailbot_workspace/` so all you have to do is specify where inside of the workspace you would like it to be saved. For example, to save to a new test_recordings folder inside of the notebook, you would run `ros2 launch global_launch main_launch.py record:=true save_path:=notebooks/test_recordings`. In this case, the folder would automatically be created for you as well.
- Usually, you wouldn't want to push rosbag recordings to main, as they take up a large amount of storage. Thus, the session_recordings folder is ignored when pushing. If you create a new folder, make sure you add it to `gitignore` unless you do want to push to main.
<!-- markdownlint-enable MD013 -->

## Recording the logs for future analysis

<!-- markdownlint-disable MD013 -->
- When we want to store our logs in a separate file, then we can use the following add command,
`2>&1 | tee src/global_launch/voyage_log/combined_log.txt`. The logs will be stored in the `combined_log*.txt` file. Where * can be any suffix like `combined_log_1.txt`, `combined_log_clean.txt`, etc.
- The `esc[32m` is present in the `[DEBUG]` statement, as that is used for colouring that line green.
- A on-water testing launch would look something like this, `ros2 launch global_launch main_launch.py record:=true mode:="production" 2>&1 | tee src/global_launch/voyage_log/combined_log.txt`
- To automatically timestamp each log file (so successive runs don't overwrite each other), append `$( date +%F_%T )` to the filename. For example, `ros2 launch global_launch main_launch.py record:=true mode:=development config:=globals.yaml 2>&1 | tee src/global_launch/voyage_log/combined_log_$( date +%F_%T).txt`
<!-- markdownlint-enable MD013 -->

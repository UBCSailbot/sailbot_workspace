# Local Pathfinding

UBC Sailbot's local pathfinding ROS package

## Run

Using main launch file: `ros2 launch local_pathfinding main_launch.py`

## Wind Tracking

Local pathfinding keeps a rolling apparent-wind history in `WindTracker` so wind
history survives `LocalPathState` replacement during path regeneration. Once the
history reaches `WIND_HISTORY_LEN` readings, `WindTracker.aw_avg` is used as the
smoothed apparent wind estimate.

Each generated local path stores `path_generated_wind`, the apparent wind value
used by OMPL for that path. This is the rolling average when available; before
the rolling average is populated, it falls back to the current filtered apparent
wind reading. Wind-based path switching compares the current rolling average
against `path_generated_wind`, so a path is regenerated only when the smoothed
wind has drifted significantly from the wind used to generate the current path.

### Launch Parameters

Launch arguments are added to the run command in the format `<name>:=<value>`.

<!-- markdownlint-disable MD013 -->
| name        | description   | value                                                 |
| ----------- | ------------- | ----------------------------------------------------- |
| `log_level` | Logging level (default: `{INFO}`) | A [severity level](https://docs.ros.org/en/humble/Concepts/About-Logging.html#severity-level) (case insensitive) `{DEBUG, INFO, WARN, ERROR, and FATAL}` |
|`mode` | Mode (default: `development`) | `{development, production}`|
|`test_plan` | Test Plan is not required when `mode=production` | any test definition yaml file in `local_pathfinding/test_plans`. (eg. `test_plan:=basic.yaml`) |
<!-- markdownlint-enable MD013 -->

## Some other important commands

<!-- markdownlint-disable MD013 -->
- `ros2 topic list`: Lists all the topics ([ROS 2 topic list documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-list))
- `ros2 node list`: Lists all the nodes that are running
- `ros2 topic echo <topic name>`, e.g.: `ros2 topic echo /filtered_wind_sensor`: ([ROS2 topic echo documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-echo:~:text=to%20avoid%20confusion.-,4%20ros2%20topic%20echo,-%EF%83%81))
- You can also do `ros2 topic echo <topic name> --field <field name>` to isolate
  a field in a topic (eg. `speed` in `\filtered_wind_sensor`)
<!-- markdownlint-enable MD013 -->
- `ros2 <command> -h` can be used to get more info about any command.


## Setting up the right land mass shape for testing and deployment

<!-- markdownlint-disable MD013 -->
We have different landmass types depending on whether Polaris is voyaging on the open ocean or near Jericho Beach (near UBC). Review the `src/local_pathfinding/land/README.md` for details on how to set up the land masses for testing/deployment. Rebuild the colcon packages `Tasks -> Build all` to ensure we have the pickle file in the build folder.

- Use the on water testing land pickle file when running the `jericho_on_water_test.yaml`
- Use the production land pickle file when running the `launch.yaml`
<!-- markdownlint-enable MD013 -->

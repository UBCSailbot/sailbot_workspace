# Local Pathfinding

UBC Sailbot's local pathfinding ROS package

## Run

Using main launch file: `ros2 launch local_pathfinding main_launch.py`

### Launch Parameters

Launch arguments are added to the run command in the format `<name>:=<value>`.

| name        | description   | value                                                 |
| ----------- | ------------- | ----------------------------------------------------- |
| `log_level` | Logging level (default: `{INFO}`) | A [severity level][severity level] (case insensitive) `{DEBUG, INFO, WARN, ERROR, and FATAL}` |
|`mode` | Mode (default: `development`) | `{development, production}`|
|`test_plan` | Test Plan is not required when `mode=production` | any test definition yaml file in `local_pathfinding/test_plans`. (eg. `test_plan:=basic.yaml`) |

## Some other important commands

- `ros2 topic list`: Lists all the topics ([ROS 2 topic list documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-list))
- `ros2 node list`: Lists all the nodes that are running
- `ros2 topic echo <topic name>`, e.g.: `ros2 topic echo /filtered_wind_sensor`: ([ROS2 topic echo documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-echo:~:text=to%20avoid%20confusion.-,4%20ros2%20topic%20echo,-%EF%83%81))
[severity level]: <https://docs.ros.org/en/humble/Concepts/About-Logging.html#severity-level>.
- You can also do `ros2 topic echo <topic name> --field <field name>` to isolate a field in a topic (eg. `speed` in `\filtered_wind_sensor`)
- `ros2 <command> -h` can be used to get more info about any command.

# Local Pathfinding

UBC Sailbot's local pathfinding ROS package

## Run

Using main launch file: `ros2 launch local_pathfinding main_launch.py`

### Launch Parameters

Launch arguments are added to the run command in the format `<name>:=<value>`.

| name        | description   | value                                                 |
| ----------- | ------------- | ----------------------------------------------------- |
| `log_level` | Logging level | A [severity level][severity level] (case insensitive) |

[severity level]: <https://docs.ros.org/en/humble/Concepts/About-Logging.html#severity-level>

### Server Files

The server files: `get_server.py` and `post_server.py` are basic http server files which are used for testing the
global_path module's GET and POST methods.

# Local Pathfinding

[![Tests](https://github.com/UBCSailbot/local_pathfinding/actions/workflows/tests.yml/badge.svg)](https://github.com/UBCSailbot/local_pathfinding/actions/workflows/tests.yml)

UBC Sailbot's local pathfinding ROS package

## Run

Using main launch file: `ros2 launch local_pathfinding main_launch.py`

### Launch Parameters

Launch arguments are added to the run command in the format `<name>:=<value>`.

| name        | description   | value                                                 |
| ----------- | ------------- | ----------------------------------------------------- |
| `log_level` | Logging level | A [severity level][severity level] (case insensitive) |

[severity level]: <https://docs.ros.org/en/humble/Concepts/About-Logging.html#severity-level>

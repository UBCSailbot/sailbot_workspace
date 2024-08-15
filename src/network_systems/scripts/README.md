# Scripts

## Generate ROS Info

```shell
python3 gen_ros_info.py
```

Takes [ros_info.yaml](../ros_info.yaml) and generates a C++ header file and Python file with constants defined within.

## Sailbot DB

```shell
./sailbot_db <db-name> [COMMAND]
./sailbot_db --help
```

Wrapper for the [SailbotDB Utility DB tool](../lib/sailbot_db/src/main.cpp).

- Requires network_systems to be built
- Run with `--help` for full details on how to run
- Can clear, populate, and dump data from a DB

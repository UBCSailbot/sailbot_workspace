# Integration Tests

The Integration Tests package defines Polaris' software verification suite. With it, we can specify testplans that
encompass the entire software project to verify the integration of our various software projects.

## Defining Tests

Tests are defined using `.yaml` files found under the `testplan/` directory. See the
[template](testplans/testplan_template.yaml) and [example](testplans/example.yaml) testplans for details.

## Invoking with ros2 run

To directly run a testplan, we can use a `ros2 run` command as follows:

```shell
ros2 run integration_tests run --ros-args -p testplan:=<path/to/testplan.yaml>
```

For example:

```shell
ros2 run integration_tests run --ros-args -p testplan:=testplans/example.yaml
```

## Invoking with ros2 launch

Since the custom testplan `.yaml` files are not valid ROS parameter config files, using them with `ros2 launch` requires
a wrapper. To create the wrappers, run:

```shell
./config/generate_configs.sh
```

This command must be run whenever the `testplans/` directory gets a new or renamed `.yaml` file. We can then run:

```shell
ros2 launch integration_tests main_launch.py config:=<path/to/config.yaml>
```

For example:

```shell
ros2 launch integration_tests main_launch.py config:=config/example.yaml
```

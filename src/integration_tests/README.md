# Integration Tests

The Integration Tests package defines Polaris' software verification suite. With it, we can specify testplans that
encompass the entire software project to verify the integration of our various software projects.

## Defining Tests

Tests are defined using `.yaml` files found under the `testplan/` directory.
Each testplan should implement the following template:

```yaml
timeout_sec: # Integer (Optional): Number of seconds allowed for the test to run (default 3 seconds)

required_packages: # List of packages and configuration files
  - name: # String: one of (boat_simulator, controller, local_pathfinding, network_systems, virtual_iridium, website)
    configs: # List[String] (Optional): config files relative to the package's config/ folder

inputs: # List of inputs to drive the test
  - type: # String: one of (ROS, HTTP<TODO>)
    name: # String: Name of ROS topic or HTTP<TODO> target
    data: # Varies: Define the type and value of the input
      dtype: # One of (HTTP type<TODO>, custom_interfaces type, std ROS message type)
      field_name: # dtype: field_val (both field_name and field_val depend on the dtype)
                  # if dtype is a builtin type (ex. int32), then set "field_name: <field_val>" as "val: <int val>"

expected_outputs: # List of expected outputs
  - type: # String: one of (ROS, HTTP<TODO>)
    name: # String: Name of ROS topic or HTTP<TODO> target
    data: # Varies: Define the type and value of the output
      dtype: # One of (HTTP type<TODO>, custom_interfaces type, std ROS message type)
      DONT_CARE: # Boolean (Optional): Specify True if output data does not matter. Default is False.
      field_name: # dtype (Optional): field_val (both field_name and field_val depend on the dtype)
                  # Required if DONT_CARE is False
```

See [example.yaml](testplans/example.yaml) for an example implementation.

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

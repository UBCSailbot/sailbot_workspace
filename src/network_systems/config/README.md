# Launch Configs

`.yaml` files defining ROS node parameters for different network_systems modules. Each `.yaml` file should have a short
description at the top describing its purpose.

## How to Run

To run with pure default defined in the code, run:

```shell
ros2 launch network_systems main_launch.py
```

To run with config files in this folder:

```shell
ros2 launch network_systems main_launch.py config:=<comma separated list of config files>
```

For example:

```shell
ros2 launch network_systems main_launch.py config:=default_prod_en.yaml
```

launches network_systems with the parameters specified in `default_prod_en.yaml`.

```shell
ros2 launch network_systems main_launch.py config:=default_prod_en.yaml,example/example_en.yaml
```

launches network_systems with the parameters specified in `default_prod_en.yaml` *and* `example/example_en.yaml`. Since
`example_en.yaml` is specified after `default_prod_en.yaml`, it overrides any duplicate parameters.

**NOTE**: Instead of defining a `mode` parameter for each node, a global ROS launch argument is used.

```shell
ros2 launch network_systems main_launch.py config:=<...> mode:=production
ros2 launch network_systems main_launch.py config:=<...> mode:=development
```

## Sub-folders

Each sub-folder contains configs specific to each module found under [network_systems/projects](../projects/). Aside
from the trivial `example/` config, there should be a `<module>_template.yaml` file with the available parameters.
However, this template file is not automatically synchronized with parameter declaration in the code, which are what
actually matter. Hence, the most up-to-date parameter declarations are found under
`network_systems/projects/<module>/src/<module>_ros_intf.cpp`.

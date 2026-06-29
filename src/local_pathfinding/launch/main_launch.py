"""Launch file that runs all nodes for the local pathfinding ROS package."""

import importlib
import os
from typing import List, Tuple

import yaml
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.logging import get_logger

_LOGGER = get_logger("local_pathfinding_launch")
# Local launch arguments and constants
PACKAGE_NAME = "local_pathfinding"


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `local_pathfinding`
    package.

    Returns:
        LaunchDescription: The launch description.
    """
    global_launch_arguments, global_environment_vars = get_global_launch_arguments()
    return LaunchDescription(
        [
            *global_launch_arguments,
            *global_environment_vars,
            OpaqueFunction(function=setup_launch),
        ]
    )


def get_global_launch_arguments() -> Tuple:
    """Gets the global launch arguments and environment variables from the global launch file.

    Returns:
        Tuple: The global launch arguments and environment variables.
    """
    ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
    global_main_launch = os.path.join(ros_workspace, "src", "global_launch", "main_launch.py")
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    if spec is None:
        raise ImportError(f"Couldn't import global_launch module from {global_main_launch}")
    module = importlib.util.module_from_spec(spec)  # type: ignore[arg-type] # spec is not None
    spec.loader.exec_module(module)  # type: ignore[union-attr] # spec is not None
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    global_environment_vars = module.ENVIRONMENT_VARIABLES
    return global_launch_arguments, global_environment_vars


def setup_launch(context: LaunchContext) -> List[Node]:
    """Collects launch descriptions that describe the system behavior in the `local_pathfinding`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Node]: Nodes to launch.
    """
    config = LaunchConfiguration("config").perform(context)
    if not os.path.isabs(config):
        ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
        config = os.path.join(ros_workspace, "src", "global_launch", "config", config)
        context.launch_configurations["config"] = config

    mode = LaunchConfiguration("mode").perform(context)
    on_water_mock_ais = get_launch_bool_or_config(context, "on_water_mock_ais", config, "mock_ais")
    visualizer_mode = get_launch_bool_or_config(
        context, "visualizer_mode", config, "navigate_main"
    )
    launch_description_entities = []
    launch_description_entities.append(get_navigate_node_description(context))
    if mode == "production":
        if on_water_mock_ais:
            _LOGGER.warn("Mock AIS is ON in production")
            launch_description_entities.append(get_mock_ais_node_description(context))
        else:
            _LOGGER.warn("Mock AIS is OFF in production")

        if visualizer_mode:
            _LOGGER.warn("Visualizer is ON in production")
            launch_description_entities.append(get_navigate_observer_node_description(context))
        else:
            _LOGGER.warn("Visualizer is OFF in production")

    elif mode == "development":
        launch_description_entities.append(get_mock_global_path_node_description(context))
        launch_description_entities.append(get_mock_wind_sensor_node_description(context))
        launch_description_entities.append(get_mock_ais_node_description(context))
        launch_description_entities.append(get_mock_gps_node_description(context))
        launch_description_entities.append(get_navigate_observer_node_description(context))
    elif mode == "sim":
        launch_description_entities.append(get_mock_global_path_node_description(context))
        launch_description_entities.append(get_mock_ais_node_description(context))
        launch_description_entities.append(get_navigate_observer_node_description(context))
    return launch_description_entities


def get_launch_bool_or_config(
    context: LaunchContext, argument: str, config_path: str, node_name: str
) -> bool:
    """Return an explicit launch boolean, or inherit the node parameter from YAML."""
    # Launch value can be an empty string when nothing is specified and this will become false
    launch_value = LaunchConfiguration(argument).perform(context).strip().lower()
    if launch_value:
        return launch_value == "true"

    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file) or {}

    node_parameters = config.get(node_name, {}).get("ros__parameters", {})
    global_parameters = config.get("/**", {}).get("ros__parameters", {})
    global_value = global_parameters.get(argument, False)
    node_value = node_parameters.get(argument, global_value)
    return bool(node_value)


def get_navigate_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_main node.
    """
    node_name = "navigate_main"
    mode = LaunchConfiguration("mode").perform(context)
    inline_params = {"mode": mode}

    if mode == "development":
        test_plan = LaunchConfiguration("test_plan").perform(context)
        # Only override the config's test_plan when one is explicitly provided.
        if test_plan:
            inline_params["test_plan"] = test_plan

    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        inline_params,
    ]

    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="navigate",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_global_path_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mgp_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the mgp_main node.
    """
    node_name = "mock_global_path"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    # Pass the shared params file, then inline the test_plan override only when one is
    # explicitly provided so the config file's test_plan is used by default.
    ros_parameters: list = [LaunchConfiguration("config").perform(context)]
    if test_plan:
        ros_parameters.append({"test_plan": test_plan})
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_global_path",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_ais_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock ais node."""
    node_name = "mock_ais"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    ros_parameters: list = [LaunchConfiguration("config").perform(context)]
    inline_params = {}
    if test_plan:
        inline_params["test_plan"] = test_plan
    on_water_mock_ais = LaunchConfiguration("on_water_mock_ais").perform(context).strip()
    if on_water_mock_ais:
        inline_params["on_water_mock_ais"] = on_water_mock_ais.lower() == "true"
    on_water_test_plan = LaunchConfiguration("on_water_test_plan").perform(context).strip()
    if on_water_test_plan:
        inline_params["on_water_test_plan"] = on_water_test_plan
    if inline_params:
        ros_parameters.append(inline_params)
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_ais",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_wind_sensor_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock wind sensor node"""
    node_name = "mock_wind_sensor"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    ros_parameters: list = [LaunchConfiguration("config").perform(context)]
    if test_plan:
        ros_parameters.append({"test_plan": test_plan})
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_wind_sensor",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_gps_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock gps node"""
    node_name = "mock_gps"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    inline_params = {}

    # Only override the config's test_plan when one is explicitly provided.
    if test_plan:
        inline_params["test_plan"] = test_plan
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        inline_params,
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_gps",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_navigate_observer_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_observer node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_observer node.
    """
    node_name = "navigate_observer"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="navigate_observer",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node

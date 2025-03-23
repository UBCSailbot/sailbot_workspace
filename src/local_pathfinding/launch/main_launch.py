"""Launch file that runs all nodes for the local pathfinding ROS package."""

import importlib
import os
from typing import List, Tuple

from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration

# Local launch arguments and constants
PACKAGE_NAME = "local_pathfinding"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = []


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
            *LOCAL_LAUNCH_ARGUMENTS,
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
    mode = LaunchConfiguration("mode").perform(context)
    launch_description_entities = []
    launch_description_entities.append(get_navigate_node_description(context))
    if mode == "development":
        launch_description_entities.append(get_mock_global_path_node_description(context))
        launch_description_entities.append(get_mock_wind_sensor_node_description(context))
        launch_description_entities.append(get_mock_ais_node_description(context))
        launch_description_entities.append(get_mock_gps_node_description(context))
    return launch_description_entities


def get_navigate_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_main node.
    """
    node_name = "navigate_main"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
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
    ros_parameters = [LaunchConfiguration("config").perform(context)]
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
    """Gets the launch description for the navigate_main node.
    Node: The node object that launches the navigate_main node.
    """
    node_name = "mock_ais"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
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
    """Gets the launch description for the mgp_main node.
    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the mgp_main node.
    """
    node_name = "mock_wind_sensor"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
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
    """Gets the launch description for the mgp_main node.
    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the mgp_main node.
    """
    node_name = "mock_gps"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
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

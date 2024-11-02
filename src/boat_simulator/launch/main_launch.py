"""Launch file that runs all nodes for the boat simulator ROS package."""

import importlib
import os
from typing import List, Tuple

from launch_ros.actions import Node

import boat_simulator.common.constants as Constants
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration

# Local launch arguments and constants
PACKAGE_NAME = "boat_simulator"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="enable_sim_multithreading",
        default_value="false",
        choices=["true", "false"],
        description="Enable multithreaded execution of callbacks in the boat simulator",
    ),
    DeclareLaunchArgument(
        name="enable-data-collection",
        default_value="false",
        choices=["true", "false"],
        description="Enable data collection in the boat simulator",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `boat_simulator`
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
    """Collects launch descriptions that describe the system behavior in the `boat_simulator`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Node]: Nodes to launch.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_physics_engine_description(context))
    launch_description_entities.append(get_low_level_control_description(context))
    launch_description_entities.append(get_data_collection_description(context))
    launch_description_entities.append(get_mock_data_description(context))
    return launch_description_entities


def get_physics_engine_description(context: LaunchContext) -> Node:
    """Gets the launch description for the physics engine node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the physics engine node.
    """
    node_name = "physics_engine_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    local_arguments: List[SomeSubstitutionsType] = [
        Constants.MULTITHREADING_CLI_ARG_NAME,
        [LaunchConfiguration("enable_sim_multithreading")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node


def get_low_level_control_description(context: LaunchContext) -> Node:
    """Gets the launch description for the low level control node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the low level control node.
    """
    node_name = "low_level_control_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    local_arguments: List[SomeSubstitutionsType] = [
        Constants.MULTITHREADING_CLI_ARG_NAME,
        [LaunchConfiguration("enable_sim_multithreading")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node


def get_data_collection_description(context: LaunchContext) -> Node:
    """Gets the launch description for the data collection node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the data collection node.
    """
    node_name = "data_collection_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    local_arguments: List[SomeSubstitutionsType] = [
        Constants.DATA_COLLECTION_CLI_ARG_NAME,
        [LaunchConfiguration("enable-data-collection")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node


def get_mock_data_description(context: LaunchContext) -> Node:
    """Gets the launch description for the data collection node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the data collection node.
    """
    node_name = "mock_data_node"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    # may not need local arguments.
    local_arguments: List[SomeSubstitutionsType] = [
        Constants.MULTITHREADING_CLI_ARG_NAME,
        [LaunchConfiguration("enable_sim_multithreading")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
        arguments=local_arguments,
    )

    return node

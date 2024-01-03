"""Launch file that runs all nodes for the  integration tests ROS package."""

import os
from importlib.util import module_from_spec, spec_from_file_location
from typing import List, Tuple

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Local launch arguments and constants
PACKAGE_NAME = "integration_tests"
NAMESPACE = ""

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = []


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `integration_tests`
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
    spec = spec_from_file_location("global_launch", global_main_launch)
    if spec is None:
        raise ImportError(f"Couldn't import global_launch module from {global_main_launch}")
    module = module_from_spec(spec)  # type: ignore[arg-type] # spec is not None
    spec.loader.exec_module(module)  # type: ignore[union-attr] # spec is not None
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    global_environment_vars = module.ENVIRONMENT_VARIABLES

    return global_launch_arguments, global_environment_vars


def setup_launch(context: LaunchContext) -> List[Node]:
    """Collects launch descriptions that describe the system behavior in the `integration_tests`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Nodes]: Nodes to launch.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_integration_test_description(context))
    return launch_description_entities


def get_integration_test_description(context: LaunchContext) -> Node:
    """Gets the launch description for the integration_test node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the integration_test node.
    """
    node_name = "integration_test_node"
    ros_parameters = [
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="run",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node

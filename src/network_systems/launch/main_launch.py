"""Launch file that runs all nodes for the network systems ROS package."""

import os
import sys
from importlib.util import module_from_spec, spec_from_file_location
from typing import List, Tuple

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration

# Deal with Python import paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(SCRIPT_DIR)
from ros_info import (  # noqa: E402
    CACHED_FIB_NODE,
    CAN_TRANSCEIVER_NODE,
    MOCK_AIS_NODE,
    REMOTE_TRANSCEIVER_NODE,
)

# Local launch arguments and constants
PACKAGE_NAME = "network_systems"
NAMESPACE = ""
global_launch_config = ""

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = []


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `network_systems`
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
    global global_launch_config
    global_launch_config = module.GLOBAL_LAUNCH_CONFIG

    return global_launch_arguments, global_environment_vars


def setup_launch(context: LaunchContext) -> List[Node]:
    """Collects launch descriptions that describe the system behavior in the `network_systems`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Nodes]: Nodes to launch.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_cached_fib_description(context))
    launch_description_entities.append(get_mock_ais_description(context))
    launch_description_entities.append(get_can_transceiver_description(context))
    launch_description_entities.append(get_remote_transceiver_description(context))
    launch_description_entities.append(get_local_transceiver_description(context))
    return launch_description_entities


def get_cached_fib_description(context: LaunchContext) -> Node:
    """Gets the launch description for the cached_fib_node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the cached_fib_node.
    """
    node_name = CACHED_FIB_NODE
    ros_parameters = [
        global_launch_config,
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="example",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_ais_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock_ais_node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the mock_ais_node.
    """
    node_name = MOCK_AIS_NODE
    ros_parameters = [
        global_launch_config,
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="mock_ais",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_can_transceiver_description(context: LaunchContext) -> Node:
    """Gets the launch description for the can_transceiver_node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the can_transceiver_node.
    """
    node_name = CAN_TRANSCEIVER_NODE
    ros_parameters = [
        global_launch_config,
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="can_transceiver",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_remote_transceiver_description(context: LaunchContext) -> Node:
    """Gets the launch description for the remote_transceiver_node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the remote_transceiver_node.
    """
    node_name = REMOTE_TRANSCEIVER_NODE
    ros_parameters = [
        global_launch_config,
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="remote_transceiver",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node

def get_local_transceiver_description(context: LaunchContext) -> Node:
    """Gets the launch description for the local_transceiver_node.

        Args:
            context (LaunchContext): The current launch context.

        Returns:
            Node: The node object that launches the local_transceiver_node.
        """
    node_name = "local_transceiver_node"
    ros_parameters = [
        global_launch_config,
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]
    node = Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="local_transceiver",
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node

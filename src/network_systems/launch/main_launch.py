"""Launch file that runs all nodes for the network systems ROS package."""

import os
import subprocess
import sys
from importlib.util import module_from_spec, spec_from_file_location
from typing import List, Optional, Tuple

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Deal with Python import paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(SCRIPT_DIR)
from ros_info import (  # noqa: E402
    CAN_TRANSCEIVER_NODE,
    REMOTE_TRANSCEIVER_NODE,
)

# Local launch arguments and constants
PACKAGE_NAME = "network_systems"
NAMESPACE = ""

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="rudder_debug",
        default_value="false",
        choices=["true", "false"],
        description="Force rudder heading data to use RUDDER_DEBUG_DATA_FRAME.",
    )
]


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

    return global_launch_arguments, global_environment_vars


def setup_launch(context: LaunchContext) -> List[Node]:
    """Collects launch descriptions that describe the system behavior in the `network_systems`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Nodes]: Nodes to launch.
    """
    config = LaunchConfiguration("config").perform(context)
    if not os.path.isabs(config):
        ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
        config = os.path.join(ros_workspace, "src", "global_launch", "config", config)
        context.launch_configurations["config"] = config

    mode = LaunchConfiguration("mode").perform(context)
    if mode in ("development", "can"):
        SetEnvironmentVariable(
            name="ROS_LOG_DIR", value="/workspaces/sailbot_workspace/log"
        ).visit(context)
    if mode == "can":
        ensure_can_replay_log(context)
    launch_description_entities = list()
    launch_description_entities.append(get_can_transceiver_description(context))
    launch_description_entities.append(get_remote_transceiver_description(context))
    launch_description_entities.append(get_local_transceiver_description(context))
    return launch_description_entities


def get_configured_can_replay_file(context: LaunchContext) -> Optional[str]:
    """Reads the can_replay_file parameter out of the config yaml(s) passed to the launch.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Optional[str]: The configured replay log path, or None if no config sets one.
    """
    replay_file = None
    # Later config files override earlier ones, matching how ROS merges parameter files
    for config in LaunchConfiguration("config").perform(context).split(","):
        try:
            with open(config, "r") as config_file:
                params = yaml.safe_load(config_file) or {}
        except (OSError, yaml.YAMLError):
            continue
        node_params = params.get(CAN_TRANSCEIVER_NODE, {}).get("ros__parameters", {})
        if "can_replay_file" in node_params:
            replay_file = node_params["can_replay_file"]
    return replay_file


def ensure_can_replay_log(context: LaunchContext) -> None:
    """Downloads the default CAN replay log if mode:=can is set to use it and it isn't present.

    CAN logs are versioned in the OWT-data repository rather than checked into this repo, so a
    fresh workspace has the default path configured but no file behind it. Only the default path
    is fetched: a custom can_replay_file is the user's own log, and the node reports it itself if
    it is missing.

    Args:
        context (LaunchContext): The current launch context.
    """
    replay_file = get_configured_can_replay_file(context)
    if not replay_file or os.path.exists(replay_file):
        return

    ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
    default_replay_file = os.path.join(
        ros_workspace, "src", "network_systems", "lib", "can_log", "combined_can_frames.csv"
    )
    if os.path.abspath(replay_file) != default_replay_file:
        return

    script = os.path.join(ros_workspace, "scripts", "get_mock_can_msg.sh")
    print(f"[network_systems] CAN replay log missing, downloading it with {script}")
    try:
        subprocess.run([script], check=True)
    except (OSError, subprocess.CalledProcessError) as err:
        # Let the launch continue: can_transceiver_node reports the missing log and how to get one
        print(f"[network_systems] Failed to download the CAN replay log: {err}")


def get_can_transceiver_description(context: LaunchContext) -> Node:
    """Gets the launch description for the can_transceiver_node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the can_transceiver_node.
    """
    node_name = CAN_TRANSCEIVER_NODE
    ros_parameters: list = [
        {"mode": LaunchConfiguration("mode")},
        *LaunchConfiguration("config").perform(context).split(","),
        {"rudder_debug": LaunchConfiguration("rudder_debug")},
    ]
    # Appended after the config files so that the launch argument overrides them when it is set
    heading_source = LaunchConfiguration("can_replay_heading_source").perform(context).strip()
    if heading_source:
        ros_parameters.append({"can_replay_heading_source": heading_source})
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

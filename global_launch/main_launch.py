"""Global launch file that starts the entire system."""

import os
from typing import List, Tuple

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration

# TODO: Add the controller package when it is ready
PRODUCTION_ROS_PACKAGES = ["local_pathfinding", "network_systems"]
DEVELOPMENT_ROS_PACKAGES = ["boat_simulator", "local_pathfinding", "network_systems"]

# Global launch arguments and constants.
# To add global arguments, update the GLOBAL_LAUNCH_ARGUMENTS list and then add it to the
# format_global_launch_arguments function in this file.
ROS_WORKSPACE = os.getenv("ROS_WORKSPACE")
GLOBAL_LAUNCH_ARGUMENTS = [
    DeclareLaunchArgument(
        name="config",
        default_value=os.path.join(ROS_WORKSPACE, "global_launch", "config", "globals.yaml"),
        description="Path to ROS parameter config file. Controls ROS parameters passed into"
        + " ROS nodes",
    ),
    # Reference: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
    DeclareLaunchArgument(
        name="log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging severity level. A logger will only process log messages with"
        + " severity levels at or higher than the specified severity",
    ),
    DeclareLaunchArgument(
        name="mode",
        default_value="development",
        choices=["production", "development"],
        description="System mode. Decides whether the system is ran with development or production"
        + " interfaces",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """Entry point for the launch file. Invokes launch files from all workspace ROS packages.

    Returns:
        LaunchDescription: A collection of entities representing the behavior of the system.
    """
    launch_description = LaunchDescription(
        [
            *GLOBAL_LAUNCH_ARGUMENTS,
            OpaqueFunction(function=set_log_dir),
            OpaqueFunction(function=setup_launch),
        ]
    )
    return launch_description


def set_log_dir(context: LaunchContext, *args, **kwargs) -> List[LaunchDescriptionEntity]:
    """Set the log directory of nodes to the log directory of launches so that all logs are in the
    same directory.

    Args:
        context (LaunchContext): Runtime context used by launch entities.

    Returns:
        Optional[List[LaunchDescriptionEntity]]: An entity that sets the log directory of nodes.

    Reference: https://github.com/ros2/launch/issues/551#issuecomment-982146452
    """
    return [SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir)]


def setup_launch(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Collects launch descriptions from all local launch files while passing the global launch
    arguments to each of them.

    Args:
        context (LaunchContext): The current context of the launch.

    Returns:
        List[LaunchDescriptionEntity]: Launch file descriptions.
    """
    mode = LaunchConfiguration("mode").perform(context)
    ros_packages = get_running_ros_packages(mode)
    global_arguments = [
        (arg.name, LaunchConfiguration(arg.name).perform(context))
        for arg in GLOBAL_LAUNCH_ARGUMENTS
    ]
    return get_include_launch_descriptions(ros_packages, global_arguments)


def get_running_ros_packages(mode: str) -> List[str]:
    """Get the names of the ROS packages to be launched depending on an indicated mode.

    Args:
        mode (str): The system mode upon launch.

    Raises:
        ValueError: Raised when an invalid mode is passed.

    Returns:
        List[str]: List of ROS package names to be launched.
    """
    match mode:
        case "production":
            return PRODUCTION_ROS_PACKAGES
        case "development":
            return DEVELOPMENT_ROS_PACKAGES
        case _:
            raise ValueError("Invalid launch mode. Must be one of 'production', 'development'.")


def get_include_launch_descriptions(
    ros_package_list: List[str],
    global_arguments: List[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]],
) -> List[IncludeLaunchDescription]:
    """Get the launch descriptions for each ROS package.

    Args:
        ros_package_list (List[str]): The names of the packages to be launched.
        global_arguments(List[Tuple[SomeSubstitutionType, SomeSubstitutionType]]): The global
            arguments common across all ROS packages.

    Returns:
        List[IncludeLaunchDescriptions]: The launch descriptions.
    """
    include_launch_descriptions = []
    for pkg in ros_package_list:
        pkg_main_launch = os.path.join(ROS_WORKSPACE, "src", pkg, "launch", "main_launch.py")
        launch_description = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=pkg_main_launch
            ),
            launch_arguments=global_arguments,
        )
        include_launch_descriptions.append(launch_description)
    return include_launch_descriptions

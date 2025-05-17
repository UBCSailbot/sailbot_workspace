"""Global launch file that starts the entire system."""

import os
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration

PRODUCTION_ROS_PACKAGES = ["controller", "local_pathfinding", "network_systems"]
DEVELOPMENT_ROS_PACKAGES = ["controller", "boat_simulator", "local_pathfinding", "network_systems"]

# Global launch arguments and constants.
ROS_PACKAGES_DIR = os.path.join(
    os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace"), "src"
)
GLOBAL_LAUNCH_CONFIG = os.path.join(ROS_PACKAGES_DIR, "global_launch", "config", "globals.yaml")
GLOBAL_LAUNCH_ARGUMENTS = [
    DeclareLaunchArgument(
        name="config",
        default_value=GLOBAL_LAUNCH_CONFIG,
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
ENVIRONMENT_VARIABLES = [
    SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
    SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
]


def generate_launch_description() -> LaunchDescription:
    """Entry point for the launch file. Invokes launch files from all workspace ROS packages.

    Returns:
        LaunchDescription: A collection of entities representing the behavior of the system.
    """
    launch_description = LaunchDescription(
        [
            *GLOBAL_LAUNCH_ARGUMENTS,
            OpaqueFunction(function=setup_launch),
        ]
    )
    return launch_description


def setup_launch(context: LaunchContext) -> List[IncludeLaunchDescription]:
    """Collects launch descriptions from all local launch files.

    Args:
        context (LaunchContext): The current context of the launch.

    Returns:
        List[IncludeLaunchDescription]: Launch file descriptions.
    """
    mode = LaunchConfiguration("mode").perform(context)
    ros_packages = get_running_ros_packages(mode)
    return get_include_launch_descriptions(ros_packages)


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


def get_include_launch_descriptions(ros_package_list: List[str]) -> List[IncludeLaunchDescription]:
    """Get the launch descriptions for each ROS package.

    Args:
        ros_package_list (List[str]): The names of the packages to be launched.

    Returns:
        List[IncludeLaunchDescriptions]: The launch descriptions.
    """
    include_launch_descriptions = []
    for pkg in ros_package_list:
        pkg_main_launch = os.path.join(ROS_PACKAGES_DIR, pkg, "launch", "main_launch.py")

        # Note: Normally, arguments would be passed by setting the `launch_arguments` input.
        # However, since we load global arguments in package launch files already, this is not
        # necessary. Only the launch description source is required.
        launch_description = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=pkg_main_launch
            )
        )
        include_launch_descriptions.append(launch_description) 
    return include_launch_descriptions

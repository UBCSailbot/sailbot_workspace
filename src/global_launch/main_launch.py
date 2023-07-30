import importlib
import os
from types import ModuleType
from typing import List, Optional

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration

# TODO: Add the controller package when it is ready
PRODUCTION_ROS_PACKAGES = ["local_pathfinding", "network_systems"]
# SIMULATION_ROS_PACKAGES = ["boat_simulator", "local_pathfinding", "network_systems"]
SIMULATION_ROS_PACKAGES = ["local_pathfinding"]
ROS_NODES = ["navigate_main"]
ROS_PACKAGES_DIR = os.path.join(os.getenv("ROS_WORKSPACE"), "src")

GLOBAL_CONFIG = os.path.join(ROS_PACKAGES_DIR, "global_launch", "config", "globals.yaml")
GLOBAL_LAUNCH_ARGUMENTS = [
    # Reference: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
    DeclareLaunchArgument(
        name="log_level",
        default_value=["info"],
        description="Logging level",
    ),
    DeclareLaunchArgument(
        name="mode",
        default_value="simulation",
        choices=["production", "simulation"],
        description="System mode.",
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
            OpaqueFunction(function=get_nodes),
        ]
    )
    return launch_description


def set_log_dir(
    context: LaunchContext, *args, **kwargs
) -> Optional[List[LaunchDescriptionEntity]]:
    """Set the log directory of nodes to the log directory of launches so that all logs are in the
    same directory.

    Args:
        context (LaunchContext): Runtime context used by launch entities.

    Returns:
        Optional[List[LaunchDescriptionEntity]]: An entity that sets the log directory of nodes.

    Reference: https://github.com/ros2/launch/issues/551#issuecomment-982146452
    """

    return [SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir)]


def get_nodes(context: LaunchContext, *args, **kwargs) -> Optional[List[LaunchDescriptionEntity]]:
    """Determine which ROS nodes to launch, initializing and returning them.

    Args:
        context (LaunchContext): Runtime context used by launch entities.

    Returns:
        Optional[List[LaunchDescriptionEntity]]: The initialized nodes.
    """
    # get_nodes() arguments for the global launch
    common_params = [GLOBAL_CONFIG]
    common_ros_args = [*get_log_ros_arguments()]
    mode = LaunchConfiguration("mode").perform(context)

    ros_package_list = get_running_ros_packages(mode=mode)
    modules = get_launch_modules(ros_package_list=ros_package_list)
    return [
        node
        for module in modules
        for node in module.get_nodes(
            common_parameters=common_params, common_ros_arguments=common_ros_args, mode=mode
        )
    ]


def get_log_ros_arguments() -> List:
    """Create ROS node arguments to set their log levels to the launch argument.

    Returns:
        List[str]: ROS arguments that set the log levels of all ROS nodes.

    Reference: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
    """
    ros_arguments = []
    log_level = LaunchConfiguration("log_level")
    for node in ROS_NODES:
        ros_arguments.extend(["--log-level", [f"{node}:=", log_level]])
    return ros_arguments


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
        case "simulation":
            return SIMULATION_ROS_PACKAGES
        case _:
            raise ValueError("Invalid launch mode. Must be one of 'production', 'simulation'.")


def get_launch_modules(ros_package_list: List[str]) -> List[ModuleType]:
    """Execute and return the main launch files of the ROS packages to be launched.

    Args:
        ros_package_list (List[str]): The names of the packages to be launched.

    Returns:
        List[ModuleType]: The executed modules.
    """
    modules = []
    for pkg in ros_package_list:
        pkg_main_launch = os.path.join(ROS_PACKAGES_DIR, pkg, "launch", "main_launch.py")
        spec = importlib.util.spec_from_file_location(f"{pkg}_launch", pkg_main_launch)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        modules.append(module)
    return modules
